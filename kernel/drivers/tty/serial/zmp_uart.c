/*
 * ZMP110x uart driver
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/module.h>
#include <linux/ioport.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/of_gpio.h>
#include <linux/console.h>
#include <linux/tty.h>
#include <linux/tty_flip.h>
#include <linux/serial_core.h>
#include <linux/serial.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/major.h>

#include <mach/irqs.h>
#include <mach/map.h>
#include "zmp_uart.h"

struct zmp_uart_port {
	char			name[24];
	struct uart_port	port;
    unsigned long baud;
	unsigned char __iomem   *rxfifo_base;
	unsigned char __iomem   *txfifo_base;

	unsigned int		rxfifo_offset;
	unsigned int		nbr_to_read;
	unsigned int		timeout_cnt;

	unsigned char		claimed;
	struct clk		*clk;

	unsigned int  rs485_flag;
	unsigned int rs485_dir;

};

static struct zmp_uart_port *zmp_serial_ports[NR_PORTS];
static struct uart_driver zmp_uart_drv;


/* macros to change one thing to another */
#define tx_enabled(port)	((port)->unused[0])
#define rx_enabled(port)	((port)->unused[1])

static int uart_intevent_decode(unsigned long status, unsigned int maskbit, unsigned int statusbit)
{
	if ((status & 1<<maskbit) && (status & 1<<statusbit))
		return 1;
	else
		return 0;
}

static inline void uart_subint_disable(struct zmp_uart_port *ourport, unsigned long mask)
{
	unsigned long uart_reg;

	/* disable tx_end interrupt */
	uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
	uart_reg &= ~mask;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
}

static inline void uart_subint_enable(struct zmp_uart_port *ourport, unsigned long unmask)
{
	unsigned long uart_reg;

	/* enable tx_end interrupt */
	uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
	uart_reg |= unmask;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
}

static void uart_subint_clear(struct zmp_uart_port *ourport, unsigned int subint)
{
	unsigned long uart_reg;

	switch (subint) {

	case TX_THR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
	break;

	case RX_THR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg &= ZMPUART_INT_MASK;
		uart_reg |= (1<<subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RECVDATA_ERR_INT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg &= ZMPUART_INT_MASK;
		uart_reg |= (0x1 << subint);
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
		break;

	case RX_TIMEOUT:
		uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
		uart_reg |= (0x1 << subint);
		uart_reg &= ~( 0x1<<3 );
		__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);

		/* start to receive data */
		uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
		uart_reg |= (1 << 31);
		__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);
		break;

	default:
		printk(KERN_ERR "zmp 9xx 9xx 9xx 9xx 9xx 9xx 9xx 9xx kown subint type: %d\n", subint);
		break;
	}

	return;
}

static inline struct zmp_uart_port *to_ourport(struct uart_port *port)
{
	return container_of(port, struct zmp_uart_port, port);
}

static inline void uart_txend_interrupt(struct zmp_uart_port *ourport, unsigned short status)
{
	unsigned long uart_reg;

	/*handle Tx_end end interrupt */
	uart_reg = __raw_readl(ourport->port.membase + UART_CONF2);
	switch(status)
	{
		case ENABLE:
			uart_reg |= (UARTN_CONFIG2_TX_END_INT_EN);
			break;

		case DISABLE:
			uart_reg &= ~(UARTN_CONFIG2_TX_END_INT_EN);
			break;

		default:
			break;
	}
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);
}

/* clear a UARTn buffer status flag */
static inline void clear_uart_buf_status(struct zmp_uart_port *ourport, unsigned short status)
{
    unsigned long regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = __raw_readl(ZMP_VA_L2CTRL + 0x8C);
	switch(status)
	{
		case RX_STATUS:
			regval |= (0x1 << (17 + ourport->port.line * 2));
			break;

		case TX_STATUS:
			regval |= (0x1 << (16 + ourport->port.line * 2));
			break;

		default:
			break;
    }
	__raw_writel(regval,  ZMP_VA_L2CTRL + 0x8C);

	local_irq_restore(flags);
}

/* clear TX and RX internal status */
static inline void clear_internal_status(struct zmp_uart_port *ourport, unsigned short status)
{
    unsigned long regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = __raw_readl(ourport->port.membase + UART_CONF1);
    switch(status)
	{
		case RX_STATUS:
			__raw_writel(regval | (0x1 << 29), ourport->port.membase + UART_CONF1);
			break;

		case TX_STATUS:
			__raw_writel(regval | (0x1 << 28), ourport->port.membase + UART_CONF1);
    		break;

		default:
			break;
    }

	local_irq_restore(flags);
}

/* clear TX_th and RX_th count interrupt */
static inline void clear_Int_status(struct zmp_uart_port *ourport, unsigned short status)
{
    unsigned long regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
    switch(status)
	{
		case RX_STATUS:
			__raw_writel(regval | (0x1 << 5), ourport->port.membase + BUF_THRESHOLD);
			break;

		case TX_STATUS:
			__raw_writel(regval | (0x1 << 11), ourport->port.membase + BUF_THRESHOLD);
    		break;

		default:
			break;
    }

	local_irq_restore(flags);
}

/* enable/disable interrupt of  RX_th */
static inline void uart_Rx_interrupt(struct zmp_uart_port *ourport, unsigned short status)
{
    unsigned long regval;
	unsigned long flags;

	local_irq_save(flags);

	regval = __raw_readl(ourport->port.membase + UART_CONF2);
	if(status)
		__raw_writel(regval | (0x1 << RX_INTTERUPT), ourport->port.membase + UART_CONF2);
	else
		__raw_writel(regval & ~(0x1 << RX_INTTERUPT), ourport->port.membase + UART_CONF2);

	local_irq_restore(flags);
}

/* power management control */
static void zmp_serial_pm(struct uart_port *port, unsigned int level, unsigned int old)
{
	switch (level) {
	case 3: /* disable */
		//pr_debug("%s: enterring pm level: %d\n", __FUNCTION__, level);
		break;

	case 0:	/* enable  */
		//pr_debug("%s: enterring pm level: %d\n", __FUNCTION__, level);
		break;

	default:
		pr_debug(KERN_ERR "zmp serial: unknown pm %d\n", level);
		break;
	}
}

/* is tx fifo empty */
static unsigned int zmp_serial_tx_empty(struct uart_port *port)
{
	unsigned long uart_reg;

	uart_reg = __raw_readl(port->membase + UART_CONF2);

	if (uart_reg & (1 << TXFIFO_EMPTY))
		return 1;

	return 0;
}

/* no modem control lines */
static unsigned int zmp_serial_get_mctrl(struct uart_port *port)
{
	/* FIXME */
	pr_debug("%s\n", __FUNCTION__);
	return 0;
}

static void zmp_serial_set_mctrl(struct uart_port *port, unsigned int mctrl)
{
	/* todo - possibly remove AFC and do manual CTS */
	pr_debug("%s\n", __FUNCTION__);
}


static void zmp_serial_start_tx(struct uart_port *port)
{
	struct zmp_uart_port *ourport = to_ourport(port);

	pr_debug("%s\n", __FUNCTION__);



	if (!tx_enabled(port))
	{

        if(ourport->rs485_flag  == 1){ //&&  ourport->port.line == 1){

            gpio_set_value(ourport->rs485_dir, 1);
//			 usleep_range(1000,1100);
			mdelay(1);
	    }

		uart_txend_interrupt(ourport, ENABLE);
		tx_enabled(port) = 1;
	}
}




static void zmp_serial_stop_tx(struct uart_port *port)
{
	struct zmp_uart_port *ourport = to_ourport(port);

	pr_debug("%s\n", __FUNCTION__);

    if (tx_enabled(port))
   	{
   		uart_txend_interrupt(ourport, DISABLE);
	    tx_enabled(port) = 0;
    }

    if(ourport->rs485_flag  == 1){// &&  ourport->port.line == 1){

//			while(!zmp_serial_tx_empty(port));
//			udelay(50);
		mdelay(1);
//		printk("send complete\n");
		 gpio_set_value(ourport->rs485_dir, 0);
//       tasklet_schedule(&rs485_tasklet);
//       schedule_work(&(ourport->rs485_wq));
	}
}

static void zmp_serial_stop_rx(struct uart_port *port)
{
	struct zmp_uart_port *ourport = to_ourport(port);

	pr_debug("%s\n", __FUNCTION__);

	if (rx_enabled(port))
   	{
		uart_Rx_interrupt(ourport, DISABLE);
		rx_enabled(port) = 0;
	}
}

static void zmp_serial_enable_ms(struct uart_port *port)
{
	pr_debug("%s\n", __FUNCTION__);
}

static void zmp_serial_break_ctl(struct uart_port *port, int break_state)
{
	pr_debug("%s\n", __FUNCTION__);
}

static irqreturn_t zmp_uart_irqhandler(int irq, void *dev_id)
{
	struct zmp_uart_port	*ourport = dev_id;
	struct uart_port *port = &ourport->port;
	struct circ_buf *xmit = &port->state->xmit;	    //&port->info->xmit;
	//struct tty_struct *tty = port->state->port.tty;	//port->info->tty;
	struct tty_port *tty = &(port->state->port);	//port->info->tty;

	unsigned int flag= TTY_NORMAL;

	unsigned char __iomem   *pbuf;
	unsigned char *pxmitbuf;
	unsigned long uart_status;
	unsigned int rxcount = 0;
	unsigned char ch = 0;
	unsigned int i;
	int txcount , tx_tail;
	unsigned int l2_offset = 0;
	unsigned long regval;

	uart_status = __raw_readl(ourport->port.membase + UART_CONF2);

	/* clear R_err interrupt */
	if ( uart_intevent_decode(uart_status, RECVDATA_ERR_INT_ENABLE, RECVDATA_ERR_INT) )
	{
		uart_subint_clear(ourport, RECVDATA_ERR_INT);
	}

	if ( uart_intevent_decode(uart_status, TX_END_INTERRUPT, TX_END_STATUS) )
	{
		/* if there is not anything more to transmit, or the uart is now
		 * stopped, disable the uart and exit
		 */

		if (uart_circ_empty(xmit) || uart_tx_stopped(port))
		{
			zmp_serial_stop_tx(port);
			goto rx_irq;
		}

		txcount = uart_circ_chars_pending(xmit);

		if(txcount > 32)
			txcount = 32;
		pbuf = ourport->txfifo_base;
		pxmitbuf = xmit->buf;

		/* clear a uartx buffer status */
		clear_uart_buf_status(ourport, TX_STATUS);

		/* clear the tx internal status */
		clear_internal_status(ourport, TX_STATUS);

		__raw_writel(0x0, ourport->txfifo_base + 0x3C);

		l2_offset = 0;
		tx_tail = xmit->tail;
		regval = 0;
		for(i = 0; i < txcount; i++)
		{
			regval |= pxmitbuf[tx_tail]<<((i & 3) * 8 );
			if((i & 3) == 3)
			{
				__raw_writel(regval, pbuf + l2_offset);
				l2_offset = l2_offset + 4;
				regval = 0;
			}
			tx_tail = (tx_tail + 1) & (UART_XMIT_SIZE - 1);
			port->icount.tx += 1;
		}
		if(i & 3)
		{
			__raw_writel(regval, pbuf + l2_offset);
		}

		regval = (__raw_readl(ourport->port.membase + UART_CONF2)&(~UARTN_CONFIG2_TX_BYT_CNT_MASK)) | (UARTN_CONFIG2_TX_BYT_CNT(txcount))  | (UARTN_CONFIG2_TX_BYT_CNT_VLD);
		__raw_writel(regval, ourport->port.membase + UART_CONF2);
		xmit->tail = tx_tail;

		if (uart_circ_chars_pending(xmit) < WAKEUP_CHARS)
			uart_write_wakeup(port);

//		if (uart_circ_empty(xmit))
//			zmp_serial_stop_tx(port);
	}

rx_irq:

	/* rx threshold interrupt */
	if ( uart_intevent_decode(uart_status, RX_THR_INT_ENABLE, RX_THR_INT) ||
		uart_intevent_decode(uart_status, RX_TIMEOUT_INT_ENABLE, RX_TIMEOUT))
	{

		if ( uart_intevent_decode(uart_status, RX_THR_INT_ENABLE, RX_THR_INT))
			uart_subint_clear(ourport, RX_THR_INT);
		else {
			uart_subint_clear(ourport, RX_TIMEOUT);
		}

		//while(__raw_readl(ourport->port.membase + UART_CONF2) & (UARTN_CONFIG2_MEM_RDY));

		ourport->nbr_to_read = (__raw_readl(ourport->port.membase + DATA_CONF)>>13) & 0x7f;

		if (ourport->nbr_to_read  != ourport->rxfifo_offset) {
			l2_offset = ourport->rxfifo_offset;
			pbuf = ourport->rxfifo_base + l2_offset;

			/* copy data */
			if (ourport->nbr_to_read > l2_offset) {
            			rxcount = ourport->nbr_to_read - l2_offset;
				for (i=0; i<rxcount; i++) {
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}
		        } else {
				rxcount = (UART_RX_FIFO_SIZE - l2_offset);
				for (i=0; i<rxcount; i++) {
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}

				pbuf = ourport->rxfifo_base;
				for (i=0; i < ourport->nbr_to_read; i++) {
					ch = __raw_readb(pbuf + i);
					uart_insert_char(port, 0, 0, ch, flag);
				}
			}
			ourport->rxfifo_offset = ourport->nbr_to_read;
		}

		tty_flip_buffer_push(tty);
	} else if (uart_intevent_decode(uart_status, MEM_RDY_INT_ENABLE, MEM_RDY_INT)) {
		ourport->nbr_to_read = (__raw_readl(ourport->port.membase + DATA_CONF)>>13) & 0x7f;

		l2_offset = ourport->rxfifo_offset;
		pbuf = ourport->rxfifo_base + l2_offset;

		printk(KERN_ERR"NOTICE:UART%d 0x%lx %u/%u\n", ourport->port.line, uart_status, ourport->nbr_to_read, ourport->rxfifo_offset);
		/* copy data */
		if (ourport->nbr_to_read > l2_offset) {
			rxcount = ourport->nbr_to_read - l2_offset;
			for (i=0; i<rxcount; i++) {
				ch = __raw_readb(pbuf + i);
				uart_insert_char(port, 0, 0, ch, flag);
			}
		} else {
			rxcount = (UART_RX_FIFO_SIZE - l2_offset);
			for (i=0; i<rxcount; i++) {
				ch = __raw_readb(pbuf + i);
				uart_insert_char(port, 0, 0, ch, flag);
			}
			pbuf = ourport->rxfifo_base;
			for (i=0; i < ourport->nbr_to_read; i++) {
				ch = __raw_readb(pbuf + i);
				uart_insert_char(port, 0, 0, ch, flag);
			}
		}
		ourport->rxfifo_offset = ourport->nbr_to_read;
		tty_flip_buffer_push(tty);
	}

	return IRQ_HANDLED;

}

static void zmp_serial_shutdown(struct uart_port *port)
{
	struct zmp_uart_port *ourport = to_ourport(port);
	unsigned int uart_reg = 0;
	u32 regval;

	/*
	 * 1st, free irq.
	 * 2nd, disable/mask hw uart setting.
	 * 3rd, close uart clock.
	 */

	/* mask all interrupt */
	__raw_writel(0, ourport->port.membase + UART_CONF2);

	uart_reg = __raw_readl(ourport->port.membase + 0xc);
	uart_reg  &= (~(1<<5));
	__raw_writel(uart_reg, ourport->port.membase + 0xc);
	uart_reg = __raw_readl(ourport->port.membase + 0x0);
	uart_reg  &= (~(1<<29));
	__raw_writel(uart_reg, ourport->port.membase + 0x0);
	/* clear uartx interrupt */
	uart_reg  &= (~1<<21);
	__raw_writel(uart_reg, ourport->port.membase + 0x0);

	/* clear uartn TX/RX buf status flag and call zmp_setpin_as_gpio() */
	switch(ourport->port.line) {
		case 0:
			regval = __raw_readl(rL2_CONBUF8_15);
			regval |= (0x3<<16);
			__raw_writel(regval, rL2_CONBUF8_15);
			break;
		case 1:
			regval = __raw_readl(rL2_CONBUF8_15);
			regval |= (0x3<<18);
			__raw_writel(regval, rL2_CONBUF8_15);
			break;
	}
	free_irq(port->irq, ourport);
	clk_disable_unprepare(ourport->clk);
}

/*
 * 1, setup gpio.
 * 2, enable clock.
 * 3, request irq and setting up uart control.
 * 4, enable subirq.
 */
static int zmp_serial_startup(struct uart_port *port)
{
	struct zmp_uart_port *ourport = to_ourport(port);
	unsigned long uart_reg;
	int ret;

	if ( rx_enabled(port) && tx_enabled(port))
		return 0;

	/* enable uart clock */
    ret = clk_prepare_enable(ourport->clk);
	if (ret) {
		pr_err( "failed to enable uart clock source.\n");
		goto startup_err;
	}

	//clear L2 Buffer
	clear_uart_buf_status(ourport, RX_STATUS);

	uart_reg = UARTN_CONFIG1_RTS_EN_BY_CIRCUIT | UARTN_CONFIG1_EN |UARTN_CONFIG1_RX_STA_CLR|UARTN_CONFIG1_TX_STA_CLR|UARTN_CONFIG1_TIMEOUT_EN;
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF1);

	/* mask all interrupt */
	__raw_writel(0, ourport->port.membase + UART_CONF2);

	/*
	 * config stop bit and timeout value
	 * set timeout = 32, stop bit = 1;
	 */
	uart_reg = (0x1f << 16);
	__raw_writel(uart_reg, ourport->port.membase + UART_STOPBIT_TIMEOUT);


   /*
      * set threshold to 32bytes
      * set RX_th_cfg_h = 0, set RX_th_cfg_l = 31
      */
	uart_reg = __raw_readl(ourport->port.membase + DATA_CONF);
	uart_reg &= ~UARTN_RX_TH_CFG_H_MASK;
	__raw_writel(uart_reg, ourport->port.membase + DATA_CONF);
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
    uart_reg &= ~(UARTN_RX_TH_CFG_L_MASK);
	uart_reg |= UARTN_RX_TH_CFG_L(0x1F);	/* 32 Bytes */
	//uart_reg |= (1 << 11);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);

	clear_internal_status(ourport, RX_STATUS);

	/* ourport->rxfifo_offset = 0; */
	ourport->rxfifo_offset = 0;

   	/* to clear  RX_th count interrupt */
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	uart_reg |= (UARTN_RX_TH_CLR);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);
	udelay(10);
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	uart_reg &= ~(UARTN_RX_TH_CLR);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);
		udelay(10);
	uart_reg = __raw_readl(ourport->port.membase + BUF_THRESHOLD);
	uart_reg |= (UARTN_RX_START);
	__raw_writel(uart_reg, ourport->port.membase + BUF_THRESHOLD);


    /*
	 *  enable timeout, rx mem_rdy and rx_th tx_end interrupt
	 */
	 uart_reg = (UARTN_CONFIG2_RX_TH_INT_EN|UARTN_CONFIG2_RX_BUF_FULL_INT_EN|UARTN_CONFIG2_TIMEOUT_INT_EN|UARTN_CONFIG2_R_ERR_INT_EN);
	__raw_writel(uart_reg, ourport->port.membase + UART_CONF2);

	rx_enabled(port) = 1;
	tx_enabled(port) = 0;

	/* register interrupt */
	ret = request_irq(port->irq, zmp_uart_irqhandler, 0/*IRQF_DISABLED*/, ourport->name, ourport);
	if (ret) {
		printk(KERN_ERR "can't request irq %d for %s\n", port->irq, ourport->name);
		goto startup_err;
	}
	return 0;

startup_err:
	zmp_serial_shutdown(port);
	return ret;
}

static void zmp_serial_set_termios(struct uart_port *port,
				struct ktermios *termios, struct ktermios *old)
{
	struct zmp_uart_port *ourport = to_ourport(port);
	unsigned int baud;
	unsigned long flags;
	unsigned long regval;
    unsigned long asic_clk = ourport->port.uartclk;

	termios->c_cflag &= ~(HUPCL | CMSPAR);
	termios->c_cflag |= CLOCAL;

	/*
	 * Ask the core to calculate the divisor for us.
	 * min: 2.4kbps, max: 2.4Mbps
	 */
#ifdef CONFIG_AEC_DUMP_DEBUG
    baud = 921600;
#else
	baud = uart_get_baud_rate(port, termios, old, 0, port->uartclk/32);
#endif

	spin_lock_irqsave(&port->lock, flags);

	ourport->baud = baud;

	/* baudrate setting */
	regval = __raw_readl(port->membase + UART_CONF1);
	regval &= ~(0xffff);
	regval &= ~(0x1 << 22);
	regval |= ((asic_clk / baud - 1) & 0xffff);
	regval |= (1 << 28) | (1 << 29);

	if (asic_clk % baud)
		regval |= (0x1 << 22);

	ourport->rxfifo_offset = 0;

	/* flow control setting */
	if(port->line != 0)
	{
		if((termios->c_cflag & CRTSCTS)) /* directly */
		{
			switch (port->line) {
			case 1:
 				printk(KERN_WARNING "%s: Hardware flow control is open\n", __func__);
				break;
			}
			regval &= ~(1<<18|1<<19);
		}
		else  /* inversly */
		{
			switch(port->line) {
			case 1:
				break;
			}
			regval &= ~(1<<18|1<<19);
		}
	}

	/* parity setting */
	if (termios->c_cflag & PARENB) {
		if (termios->c_cflag & PARODD)
			regval |= (0x2 << 25);  /* odd parity */
		else
			regval |= (0x3 << 25);  /* evnt parity*/
	}

    __raw_writel(regval, port->membase + UART_CONF1);

	/*
	 * Update the per-port timeout.
	 */
	uart_update_timeout(port, termios->c_cflag, baud);

	/*
	 * Which character status flags should we ignore?
	 */
	port->ignore_status_mask = 0;

	spin_unlock_irqrestore(&port->lock, flags);
}

static const char *zmp_serial_type(struct uart_port *port)
{
	switch (port->type)
	{
		case PORT_ZMP:
			return "ZMP";
		default:
			return NULL;
	}
}

static void zmp_serial_release_port(struct uart_port *port)
{
	pr_debug("%s\n", __FUNCTION__);
}

static int zmp_serial_request_port(struct uart_port *port)
{
	pr_debug("%s\n", __FUNCTION__);
	return 0;
}

static void zmp_serial_config_port(struct uart_port *port, int flags)
{
	struct zmp_uart_port *ourport = to_ourport(port);

	port->type = PORT_ZMP;
	ourport->rxfifo_offset = 0;
}

/*
 * verify the new serial_struct (for TIOCSSERIAL).
 */
static int
zmp_serial_verify_port(struct uart_port *port, struct serial_struct *ser)
{
	pr_debug("%s\n", __FUNCTION__);

	return 0;
}

static struct uart_ops zmp_serial_ops = {
	.pm             = zmp_serial_pm,
	.tx_empty       = zmp_serial_tx_empty,
	.get_mctrl      = zmp_serial_get_mctrl,
	.set_mctrl      = zmp_serial_set_mctrl,
	.stop_tx        = zmp_serial_stop_tx,
	.start_tx       = zmp_serial_start_tx,
	.stop_rx        = zmp_serial_stop_rx,
	.enable_ms      = zmp_serial_enable_ms,
	.break_ctl      = zmp_serial_break_ctl,
	.startup        = zmp_serial_startup,
	.shutdown       = zmp_serial_shutdown,
	.set_termios    = zmp_serial_set_termios,
	.type           = zmp_serial_type,
	.release_port   = zmp_serial_release_port,
	.request_port   = zmp_serial_request_port,
	.config_port    = zmp_serial_config_port,
	.verify_port    = zmp_serial_verify_port,
};

static void zmp_uart_add_console_port(struct zmp_uart_port *up)
{
	zmp_serial_ports[up->port.line] = up;
}

static int zmp_serial_probe(struct platform_device *dev)
{
	struct zmp_uart_port *ourport;
    struct resource *resource;
	struct device_node *np = dev->dev.of_node;
	int ret = 0,err;
    unsigned int speed;

    dev->id = of_alias_get_id(np, "uart");
    ourport = devm_kzalloc(&dev->dev, sizeof(*ourport), GFP_KERNEL);
	if (!ourport) {
		dev_err(&dev->dev, "Failed to allocate memory for tup\n");
		return -ENOMEM;
	}

    resource = platform_get_resource(dev, IORESOURCE_MEM, 0);
	if (!resource) {
		dev_err(&dev->dev, "No IO memory resource\n");
		return -ENODEV;
	}

    strcpy(ourport->name,dev->name);

    ourport->rxfifo_base	= ZMP_UART0_RXBUF_BASE + (dev->id)*ZMP_UART_REG_OFFSET;
	ourport->txfifo_base	= ZMP_UART0_TXBUF_BASE + (dev->id)*ZMP_UART_REG_OFFSET;


	ourport->port.ops = &zmp_serial_ops;
	ourport->port.iotype	= UPIO_MEM;
	ourport->port.mapbase	= resource->start;
	ourport->port.membase	= devm_ioremap_resource(&(dev->dev), resource);
	ourport->port.irq		= platform_get_irq(dev, 0);

	ourport->port.flags		= UPF_BOOT_AUTOCONF;
	ourport->port.line      = dev->id;

    ourport->clk = devm_clk_get(&dev->dev, NULL);
	if (IS_ERR(ourport->clk)) {
		dev_err(&dev->dev, "failed to find clock source.\n");
		ret = PTR_ERR(ourport->clk);
		goto probe_err;
	}

	/* uart clock is divided by asic clock */
    ourport->port.uartclk = clk_get_rate(ourport->clk);

    if (of_property_read_u32(dev->dev.of_node,
			"fifosize",&ourport->port.fifosize)) {
		dev_err(&dev->dev,"Unable to find fifosize in uart node.\n");
		ret = -EFAULT;
		goto probe_err;
	}
	/* If current-speed was set, then try not to change it. */
	if (of_property_read_u32(dev->dev.of_node, "current-speed", &speed)){
		dev_err(&dev->dev, "current-speed property NOT set\n");
		return -EINVAL;
	}

	ourport->rs485_flag = 0;

    if (of_find_property(dev->dev.of_node, "dir-gpios", NULL)){

	    ourport->rs485_dir = of_get_named_gpio(dev->dev.of_node, "dir-gpios", 0);

        if(gpio_is_valid(ourport->rs485_dir)){
	        err = devm_gpio_request_one(&dev->dev,ourport->rs485_dir,GPIOF_OUT_INIT_LOW, "rs485_dir_gpio");
            if (err)
                dev_err(&dev->dev, "unable to request rs485 dir gpio");
        	else
            	ourport->rs485_flag = 1;
    	}
	}


	dev_info(&dev->dev, "device id is %d,rs485 flag is %d\n",ourport->port.line,ourport->rs485_flag);
	ourport->baud = speed;

    zmp_uart_add_console_port(ourport);

	uart_add_one_port(&zmp_uart_drv, &ourport->port);

	platform_set_drvdata(dev, &ourport->port);

	return 0;

probe_err:
	return ret;
}

static int zmp_serial_remove(struct platform_device *dev)
{
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(&dev->dev);

	if (port) {
		uart_remove_one_port(&zmp_uart_drv, port);
	}
	return 0;
}

/* UART power management code */

#ifdef CONFIG_PM

static int zmp_serial_suspend(struct platform_device *dev, pm_message_t state)
{
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(&dev->dev);

	if (port)
		uart_suspend_port(&zmp_uart_drv, port);
	return 0;
}

static int zmp_serial_resume(struct platform_device *dev)
{
	struct uart_port *port = (struct uart_port *)dev_get_drvdata(&dev->dev);

	if (port)
		uart_resume_port(&zmp_uart_drv, port);
	return 0;
}
#else
#define zmp_serial_suspend NULL
#define zmp_serial_resume	NULL
#endif

static const struct of_device_id zmp_uart_of_ids[] = {
	{ .compatible = "zlgmcu,zmp110x-uart0" },
	{ .compatible = "zlgmcu,zmp110x-uart1" },
	{ .compatible = "zlgmcu,zmp110x-uart2" },
	{ .compatible = "zlgmcu,zmp110x-uart3" },
	{},
};
MODULE_DEVICE_TABLE(of, zmp_uart_of_ids);

#ifdef CONFIG_SERIAL_ZMP_CONSOLE
static inline void zmp_uart_putchar(struct zmp_uart_port *ourport, unsigned char ch)
{
	unsigned long regval;

	/* clear the tx internal status */
	clear_internal_status(ourport, TX_STATUS);

	/* clear a uartx buffer status */
	clear_uart_buf_status(ourport, TX_STATUS);

	/*to inform the buf is full*/
	__raw_writel(ch, ourport->txfifo_base);
	__raw_writel(0x0, ourport->txfifo_base + 0x3C);

   	/* to clear  TX_th count interrupt */
	clear_Int_status(ourport, TX_STATUS);

	/* start to transmit */
	regval = __raw_readl(ourport->port.membase + UART_CONF2);
	regval &= ZMPUART_INT_MASK;
	regval |= (0x1<<4) | (0x1<<16);
	__raw_writel(regval, ourport->port.membase + UART_CONF2);


	/* wait for tx end */
	while (!(__raw_readl(ourport->port.membase + UART_CONF2) & (1 << TX_END_STATUS)))
		;
}

static inline void zmp_uart_send(struct zmp_uart_port *ourport, const char *s, unsigned int count)
{
	unsigned long regval;
    unsigned int left, trans_size;
    int offset;

	/* clear the tx internal status */
	clear_internal_status(ourport, TX_STATUS);

	/* clear a uartx buffer status */
	clear_uart_buf_status(ourport, TX_STATUS);

    left = count;

    while (left)
    {
        trans_size = 128; // L2 buffer for UART is 128 bytes
        if (left < trans_size)
            trans_size = left;

        if ((int)s & 0x3) // this should not happen
            return;

        // copy
        for (offset=0; offset<trans_size; offset+=4)
        {
            __raw_writel(*(int *)(s+offset), ourport->txfifo_base + offset);
        }
        left -= trans_size;
        s += trans_size;

        if ((offset&0x3f) != 0)
        {// when an L2 block is not filled, write last dword to inform the buf is full
            __raw_writel(0x0, ourport->txfifo_base + offset - (offset&0x3f) + 0x3C);
        }

        /* to clear  TX_th count interrupt */
        clear_Int_status(ourport, TX_STATUS);

        /* start to transmit */
        regval = __raw_readl(ourport->port.membase + UART_CONF2);
        regval &= ZMPUART_INT_MASK;
        regval |= (trans_size<<4) | (0x1<<16);
        __raw_writel(regval, ourport->port.membase + UART_CONF2);

        /* wait for tx end */
        while (!(__raw_readl(ourport->port.membase + UART_CONF2) & (1 << TX_END_STATUS)))
            ;
    }
}

static inline void zmp_wait_for_txend(struct zmp_uart_port *ourport)
{
	unsigned int timeout = 10000;

	/*
	 * Wait up to 10ms for the character(s) to be sent
	 */
    while (!(__raw_readl(ourport->port.membase + UART_CONF2) & (1 << TX_END_STATUS))) {
        if (--timeout == 0)
            break;
        udelay(1);
    }
}

static void
zmp_serial_console_putchar(struct uart_port *port, int ch)
{
	struct zmp_uart_port *ourport = to_ourport(port);

	zmp_wait_for_txend(ourport);

	zmp_uart_putchar(ourport, ch);
}

static void
zmp_serial_console_write(struct console *co, const char *s, unsigned int count)
{
    static struct zmp_uart_port * up;

    up =zmp_serial_ports[co->index];
	uart_console_write(&up->port, s, count, zmp_serial_console_putchar);
}

void
zmp_serial_console_send(struct uart_port *port, const char *s, unsigned int count)
{
	struct zmp_uart_port *ourport = to_ourport(port);
	zmp_wait_for_txend(ourport);
    zmp_uart_send(ourport, s, count);
}
EXPORT_SYMBOL_GPL(zmp_serial_console_send);


static int __init
zmp_serial_console_setup(struct console *co, char *options)
{
	struct zmp_uart_port *uap;
	int baud = 115200;
	int bits = 8;
	int parity = 'n';
	int flow = 'n';
    int ret;

	uap = zmp_serial_ports[co->index];

	/* is this a valid port */

	if (co->index == -1 || co->index >= NR_PORTS)
		co->index = 0;

	ret = clk_prepare(uap->clk);
	if (ret)
		return ret;

    uap->port.uartclk = clk_get_rate(uap->clk);
	/*
	 * Check whether an invalid uart number has been specified, and
	 * if so, search for the first available port that does have
	 * console support.
	 */
	if (options)
		uart_parse_options(options, &baud, &parity, &bits, &flow);

	pr_debug("zmp_serial_console_setup: baud %d\n", baud);

	return uart_set_options(&uap->port, co, baud, parity, bits, flow);
}

static struct console zmp_serial_console = {
	.name		= ZMP_SERIAL_NAME,
	.device		= uart_console_device,
	.flags		= CON_PRINTBUFFER,
	.index		= -1,
	.write		= zmp_serial_console_write,
	.setup		= zmp_serial_console_setup,
	.data = &zmp_uart_drv,
};
#endif /* CONFIG_SERIAL_ZMP_CONSOLE */


static struct uart_driver zmp_uart_drv = {
	.owner		= THIS_MODULE,
	.dev_name	= ZMP_SERIAL_NAME,
	.driver_name	= ZMP_SERIAL_NAME,
	.nr			= NR_PORTS,
	.major		= ZMP_SERIAL_MAJOR,
	.minor		= ZMP_SERIAL_MINOR,
#ifdef CONFIG_SERIAL_ZMP_CONSOLE
	.cons		= &zmp_serial_console,
#endif
};

static struct platform_driver zmp_serial_drv = {
	.probe          = zmp_serial_probe,
	.remove         = zmp_serial_remove,
	.suspend        = zmp_serial_suspend,
	.resume         = zmp_serial_resume,
	.driver         = {
		.name   = "zmp-uart",
        .of_match_table = of_match_ptr(zmp_uart_of_ids),
		.owner  = THIS_MODULE,
	},
};


/* module initialisation code */
static int __init zmp_serial_modinit(void)
{
	int ret;

	ret = uart_register_driver(&zmp_uart_drv);
	if (ret < 0) {
		printk(KERN_ERR "failed to register UART driver\n");
		return -1;
	}

	platform_driver_register(&zmp_serial_drv);

	return 0;
}

static void __exit zmp_serial_modexit(void)
{
	platform_driver_unregister(&zmp_serial_drv);

	uart_unregister_driver(&zmp_uart_drv);
}

module_init(zmp_serial_modinit);
module_exit(zmp_serial_modexit);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("zlgmcu");
MODULE_DESCRIPTION("ZMP110x Serial port driver");
