/*
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

#include <linux/kernel.h>
#include <linux/sched.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/err.h>
#include <linux/clk.h>
#include <linux/slab.h>
#include <linux/export.h>

#include <asm/io.h>
#include <asm/mach/time.h>

#include <mach/map.h>

#include <linux/delay.h>

#define ZMP_TIMER1_CTRL1       	(ZMP_VA_SYSCTRL + 0xB4)
#define ZMP_TIMER1_CTRL2       	(ZMP_VA_SYSCTRL + 0xB8)
#define ZMP_TIMER2_CTRL1			(ZMP_VA_SYSCTRL + 0xBC)
#define ZMP_TIMER2_CTRL2			(ZMP_VA_SYSCTRL + 0xC0)
#define ZMP_TIMER3_CTRL1			(ZMP_VA_SYSCTRL + 0xC4)
#define ZMP_TIMER3_CTRL2			(ZMP_VA_SYSCTRL + 0xC8)
#define ZMP_TIMER4_CTRL1			(ZMP_VA_SYSCTRL + 0xCC)
#define ZMP_TIMER4_CTRL2			(ZMP_VA_SYSCTRL + 0xD0)
#define ZMP_TIMER5_CTRL1       	(ZMP_VA_SYSCTRL + 0xD4)
#define ZMP_TIMER5_CTRL2       	(ZMP_VA_SYSCTRL + 0xD8)


#define TIMER_CNT				(12000000/HZ)
#define TIMER_USEC_SHIFT		16
#define TIMER_CNT_MASK			(0x3F<<26)

#define TIMER_CLEAR_BIT			(1<<30)
#define TIMER_FEED_BIT			(1<<29)
#define TIMER_ENABLE_BIT		(1<<28)
#define TIMER_STATUS_BIT		(1<<27)
#define TIMER_READ_SEL_BIT		(1<<26)

#define MODE_AUTO_RELOAD_TIMER	0x0
#define MODE_ONE_SHOT_TIMER		0x1
#define MODE_PWM				0x2

typedef int (*timer_handler) (void *data);

struct zmptimer {
	volatile unsigned int __force *ctrl1;
	volatile unsigned int __force *ctrl2;

	int timer_bit;
	int irq;
	timer_handler handler;
	void *data;
};

static unsigned long flags;
static int timers_irq_list[7] = {0};

/* copy from plat-s3c/time.c
 *
 *  timer_mask_usec_ticks
 *
 * given a clock and divisor, make the value to pass into timer_ticks_to_usec
 * to scale the ticks into usecs
*/
static inline unsigned long
timer_mask_usec_ticks(unsigned long scaler, unsigned long pclk)
{
	unsigned long den = pclk / 1000;

	return ((1000 << TIMER_USEC_SHIFT) * scaler + (den >> 1)) / den;
}

static inline void zmp_timer_setup(struct zmptimer *ptimer)
{
	unsigned long regval;

	/* clear timeout puls, reload */
    regval = __raw_readl(ptimer->ctrl2);
    __raw_writel(regval | TIMER_CLEAR_BIT, ptimer->ctrl2);
}


int zmp_timer_stop(void *priv)
{
	struct zmptimer *ptimer = priv;

	__raw_writel(~TIMER_ENABLE_BIT, ptimer->ctrl2);
	clear_bit(ptimer->timer_bit, &flags);

	return 0;
}
EXPORT_SYMBOL(zmp_timer_stop);


/*
 * IRQ handler for the timer
 */
static irqreturn_t zmp_timer_interrupt(int irq, void *dev_id)
{
	struct zmptimer *ptimer = dev_id;

	if (__raw_readl(ptimer->ctrl2) & TIMER_STATUS_BIT) {

		ptimer->handler(ptimer->data);

		zmp_timer_setup(ptimer);
	}

	return IRQ_HANDLED;
}

void * zmp_timer_probe(int which_timer)
{
	int ret;
	int timer_bit;
	struct zmptimer *ptimer;

	ptimer = kmalloc(sizeof(struct zmptimer), GFP_KERNEL);
	if (ptimer == NULL) {
		pr_debug("%s kmalloc failed.\n", __func__);
		goto err1;
	}

	memset(ptimer, 0, sizeof(*ptimer));

	switch (which_timer) {
	case 1:
		ptimer->ctrl1 	= ZMP_TIMER1_CTRL1;
		ptimer->ctrl2 	= ZMP_TIMER1_CTRL2;
		ptimer->irq 	= timers_irq_list[0];
		break;
	case 2:
		ptimer->ctrl1 	= ZMP_TIMER2_CTRL1;
		ptimer->ctrl2 	= ZMP_TIMER2_CTRL2;
		ptimer->irq 	= timers_irq_list[1];
		break;
	case 3:
		ptimer->ctrl1 	= ZMP_TIMER3_CTRL1;
		ptimer->ctrl2 	= ZMP_TIMER3_CTRL2;
		ptimer->irq 	= timers_irq_list[2];
		break;
	case 4:
		ptimer->ctrl1 	= ZMP_TIMER4_CTRL1;
		ptimer->ctrl2 	= ZMP_TIMER4_CTRL2;
		ptimer->irq 	= timers_irq_list[3];
		break;
	case 5:
		ptimer->ctrl1 	= ZMP_TIMER5_CTRL1;
		ptimer->ctrl2 	= ZMP_TIMER5_CTRL2;
		ptimer->irq 	= timers_irq_list[4];
		break;
	default:
		pr_debug("zmp only support 5 normal timers(timer1~5).\n");

		goto err2;
		break;
	}

	timer_bit = 1<<(which_timer - 1);
	ptimer->timer_bit = timer_bit;

	/* setup irq handler for IRQ_TIMER */
	ret = request_irq(ptimer->irq, zmp_timer_interrupt, 0/*IRQF_DISABLED*/ | IRQF_TIMER | IRQF_IRQPOLL, "hwtimer", ptimer);
	if (ret) {
		pr_err("%s request irq for timer failed.\n", __func__);
		goto err2;
	}
	return ptimer;

err2:
	kfree(ptimer);
err1:
	return NULL;
}
EXPORT_SYMBOL(zmp_timer_probe);

int zmp_timer_remove(void *priv)
{
	struct zmptimer *ptimer = priv;

	zmp_timer_stop(priv);
	free_irq(ptimer->irq, ptimer);
	kfree(ptimer);

	return 0;
}
EXPORT_SYMBOL(zmp_timer_remove);

/*
 *which_timer:	1~5, total five timers.
 * */
int zmp_timer_start(timer_handler handler, void *data, void *priv, int hz)
{
	int ret = 0;
	unsigned long timecnt = (12000000/hz) - 1;
	struct zmptimer *ptimer = priv;

	if (handler == NULL) {
		pr_debug("%s handler NULL", __func__);
		ret = -EINVAL;
		goto err1;
	}

	ptimer->handler = handler;
	ptimer->data = data;

	__raw_writel(timecnt, ptimer->ctrl1);
	__raw_writel((TIMER_ENABLE_BIT | TIMER_FEED_BIT | (MODE_AUTO_RELOAD_TIMER << 24)),
		ptimer->ctrl2);

err1:
	return ret;
}
EXPORT_SYMBOL(zmp_timer_start);

/*
 * zmp_timer_set_timers_irq - set irq of timers
 * @index:		index of irq, start from 0
 * @irq:		irq-number of the  irq
 */
int zmp_timer_set_timers_irq(int index, int irq)
{
	timers_irq_list[index] = irq;
	return 0;
}
EXPORT_SYMBOL(zmp_timer_set_timers_irq);
