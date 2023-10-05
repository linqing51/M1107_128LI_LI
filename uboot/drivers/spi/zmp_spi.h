#ifndef _ZMP_SPI_H__
#define _ZMP_SPI_H__


enum zmpspi_cs_num {	
	ZMPSPI_ONCHIP_CS = 0,	/*on chip control cs index*/
	/*ZMPSPI_CS1,*/
	/*ZMPSPI_CS2,*/
	ZMPSPI_CS_NUM, 		
};

enum zmpspi_bus_num {
	ZMPSPI_BUS_NUM1,
	ZMPSPI_BUS_NUM2,
	ZMPSPI_MAX_BUS_NUM,
};


enum spi_xfer_dir {
	SPI_DIR_TX,
	SPI_DIR_RX,
	SPI_DIR_TXRX, 
	SPI_DIR_XFER_NUM,
};

/* ASPEN SPI controller address */
#define SPI1_CTRL_BASE_REG			0x20120000

#define HS_SPI_CLK_CFG_REG			    (CHIP_CONF_BASE_ADDR + 0x00000100)      // HIGH SPEED SPI CLK & GATE REGISTER
#define SPI1_CTRL_REG				(SPI1_CTRL_BASE_REG+ 0x0000)
#define SPI1_STA_REG		    	(SPI1_CTRL_BASE_REG+ 0x0004)
#define SPI1_DATA_COUNT_REG			(SPI1_CTRL_BASE_REG+ 0x000C)
#define SPI1_RX_BUF_REG				(SPI1_CTRL_BASE_REG+ 0x0014)
#define SPI1_ODATA_REG	    		(SPI1_CTRL_BASE_REG+ 0x0018)
#define SPI1_IDATA_REG	    		(SPI1_CTRL_BASE_REG+ 0x001C)
#define SPI1_WORK_MODE          	(SPI1_CTRL_BASE_REG+ 0x0024)



#define SPI_MASTER_INPROGRESS    (1<<9)
#define SPI_MASTER_FINISH	        (1<<8)


#define SPI_MODE0_V      			(0x0<<2)
#define SPI_MODE3_V      			(0x3<<2)
#define SPI_MASTER_V     			(0x1<<4)
#define SPI_SLAVE_V      			(0x0<<4)
#define SPI_CTRL_FORCE_CS         (1<<5)
#define SPI_CTRL_GARBAGE_MODE    	(1<<0)
#define SPI_CTRL_RX_REJECT        (1<<1)
#define	SPI_CTRL_ENABLE			(1<<6)
#define SPI_DEFAULT_DIVIDE		0x2

#define SPI1_CLOCK_EN_BIT 	(1<<5)
#define SPI2_CLOCK_EN_BIT 	(1<<6)


#define TRANS_TIMEOUT 			(10000000)
#define MAX_XFER_LEN 			(8*1024)
#define SPI_TRANS_TIMEOUT 		(5000)

#define DFT_CON 			(ZMP_SPICON_EN | ZMP_SPICON_MS)
#define DFT_DIV				(1) //5 /*127*/
#define DFT_BIT_PER_WORD 	(8)
#define FORCE_CS   			(1 << 5)
#define SPPIN_DEFAULT 		(0)

#define ZMPSPI_MAX_FREQ  	(80*1000*1000)


#define ZMPSPI_1DATAWIRE 	(0b00<<16)
#define ZMPSPI_2DATAWIRE 	(0b01<<16)
#define ZMPSPI_4DATAWIRE 	(0b10<<16)

#define ZMPSPI_XFER_MODE_DMA 	(1)
#define ZMPSPI_XFER_MODE_CPU 	(2)


#define ZMP_SPICON		(0x00)
#define ZMP_SPICON_WIRE		(0x3<<16)
#define ZMP_SPICON_CLKDIV	(0x7F<<8)
#define ZMP_SPICON_EN	(1<<6)
#define ZMP_SPICON_CS	(1<<5)
#define ZMP_SPICON_MS	(1<<4)
#define ZMP_SPICON_CPHA	(1<<3)
#define ZMP_SPICON_CPOL	(1<<2)
#define ZMP_SPICON_ARRM	(1<<1)
#define ZMP_SPICON_TGDM	(1<<0)

#define ZMP_SPISTA		(0x04)
#define ZMP_SPISTA_TIMEOUT	(1<<10)
#define ZMP_SPISTA_MPROC	(1<<9)
#define ZMP_SPISTA_TRANSF	(1<<8)
#define ZMP_SPISTA_RXOVER	(1<<7)
#define ZMP_SPISTA_RXHFULL	(1<<6)
#define ZMP_SPISTA_RXFULL	(1<<5)
#define ZMP_SPISTA_RXEMP	(1<<4)
#define ZMP_SPISTA_TXUNDER	(1<<3)
#define ZMP_SPISTA_TXHEMP	(1<<2)
#define ZMP_SPISTA_TXFULL	(1<<1)
#define ZMP_SPISTA_TXEMP	(1<<0)

#define ZMP_SPIINT		(0x08)
#define ZMP_SPIINT_TIMEOUT	(1<<10)
#define ZMP_SPIINT_MPROC	(1<<9)
#define ZMP_SPIINT_TRANSF	(1<<8)
#define ZMP_SPIINT_RXOVER	(1<<7)
#define ZMP_SPIINT_RXHFULL	(1<<6)
#define ZMP_SPIINT_RXFULL	(1<<5)
#define ZMP_SPIINT_RXEMP	(1<<4)
#define ZMP_SPIINT_TXUNDER	(1<<3)
#define ZMP_SPIINT_TXHEMP	(1<<2)
#define ZMP_SPIINT_TXFULL	(1<<1)
#define ZMP_SPIINT_TXEMP	(1<<0)

#define ZMP_SPICNT		(0x0C)

#define ZMP_SPIEXTX		(0x10)
#define ZMP_SPIEXTX_BUFEN	(1<<0)
#define ZMP_SPIEXTX_DMAEN	(1<<16)


#define ZMP_SPIEXRX		(0x14)
#define ZMP_SPIEXRX_BUFEN	(1<<0)
#define ZMP_SPIEXRX_DMAEN	(1<<16)

#define ZMP_SPIOUT		(0x18)

#define ZMP_SPIIN		(0x1C)

struct zmp_spi_slave
{
	struct spi_slave slave;
	unsigned long regs;
	unsigned long max_hz;
	unsigned long freq;
	unsigned long mode;
	unsigned long l2buf_rid;
	unsigned long l2buf_tid;

//#ifdef CONFIG_SPI_XFER_CPU
	int 			len; 	/* need transfer len */
	int			 	count;	/* have transferred len */
	unsigned char	*tx;	/* tx data buffers */
	unsigned char	*rx;	/* rx data buffers */
//#endif

};

void spi_sharepin_cfg(struct spi_slave *spi);

#endif
