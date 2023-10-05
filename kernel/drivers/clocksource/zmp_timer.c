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
 */

#define pr_fmt(fmt) "zmp-timer: " fmt

#include <linux/bitops.h>
#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/irq.h>
#include <linux/clocksource.h>
#include <linux/clockchips.h>

#include <linux/clocksource.h>
#include <linux/interrupt.h>
#include <linux/irqreturn.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/of_platform.h>
#include <linux/slab.h>
#include <linux/string.h>
#include <linux/sched_clock.h>

#include <asm/irq.h>
#include <mach/map.h>


#define ZMP_TIMER1_CTRL1       	(ZMP_VA_SYSCTRL + 0xB4)
#define ZMP_TIMER1_CTRL2       	(ZMP_VA_SYSCTRL + 0xB8)
#define ZMP_TIMER2_CTRL1       	(ZMP_VA_SYSCTRL + 0xBC)
#define ZMP_TIMER2_CTRL2       	(ZMP_VA_SYSCTRL + 0xC0)
#define ZMP_TIMER3_CTRL1       	(ZMP_VA_SYSCTRL + 0xC4)
#define ZMP_TIMER3_CTRL2       	(ZMP_VA_SYSCTRL + 0xC8)
#define ZMP_TIMER4_CTRL1       	(ZMP_VA_SYSCTRL + 0xCC)
#define ZMP_TIMER4_CTRL2       	(ZMP_VA_SYSCTRL + 0xD0)
#define ZMP_TIMER5_CTRL1       	(ZMP_VA_SYSCTRL + 0xD4)
#define ZMP_TIMER5_CTRL2       	(ZMP_VA_SYSCTRL + 0xD8)

#define ZMP_TIMER6_CTRL1       	(ZMP_VA_SYSCTRL + 0x1C0)
#define ZMP_TIMER6_CTRL2       	(ZMP_VA_SYSCTRL + 0x1C4)
#define ZMP_TIMER7_CTRL1       	(ZMP_VA_SYSCTRL + 0x1C8)
#define ZMP_TIMER7_CTRL2       	(ZMP_VA_SYSCTRL + 0x1CC)

#define IRQ_TIMER           	IRQ_TIMER6
#define ZMP_CE_CTRL1       		ZMP_TIMER6_CTRL1
#define ZMP_CE_CTRL2       		ZMP_TIMER6_CTRL2
#define ZMP_CS_CTRL1       		ZMP_TIMER7_CTRL1
#define ZMP_CS_CTRL2       		ZMP_TIMER7_CTRL2

#define ZMP_CE_TIMER_INDEX		5

#define TIMER_CLK_INPUT     	(12000000)
#define TIMER_CNT           	(TIMER_CLK_INPUT/HZ)
#define TIMER_CNT_MASK			(0x3F<<26)

/* define timer control register2 bits */
#define TIMER_CLEAR_BIT         (1<<30)
#define TIMER_FEED_BIT          (1<<29)
#define TIMER_ENABLE            (1<<28)
#define TIMER_STATUS_BIT        (1<<27)
#define TIMER_READ_SEL_BIT      (1<<26)

/* define working mode */
#define MODE_AUTO_RELOAD_TIMER  (0x0<<24)
#define MODE_ONE_SHOT_TIMER     (0x1<<24)
#define MODE_PWM                (0x2<<24)   

int zmp_timer_set_timers_irq(int index, int irq);

/* use timer as clocksource device */
static cycle_t zmp_cs_timer_read(struct clocksource *cs)
{
    u32 ctrl1, ctrl2;
    unsigned long flags;

    local_irq_save(flags);

    /* select read current count mode */
	ctrl2 = __raw_readl(ZMP_CS_CTRL2);
	__raw_writel(ctrl2 | TIMER_READ_SEL_BIT, ZMP_CS_CTRL2);

    ctrl1 = __raw_readl(ZMP_CS_CTRL1);

    /* resume read mode */
    ctrl2 = __raw_readl(ZMP_CS_CTRL2);
	__raw_writel(ctrl2 & (~TIMER_READ_SEL_BIT), ZMP_CS_CTRL2);

    local_irq_restore(flags);

    return (cycle_t)~ctrl1;
}

static struct clocksource zmp_cs = {
    .name           = "zmp_cs_timer",
    .rating         = 150,
    .read           = zmp_cs_timer_read,
    .mask           = CLOCKSOURCE_MASK(32),
    .flags          = CLOCK_SOURCE_IS_CONTINUOUS,
};

/* use timer1 as clock event device */
static int zmp_ce_timer_set_mode(int mode,
    struct clock_event_device *evt)
{
	int ret = 0;

	switch (mode) {
		case 0://CLOCK_EVT_MODE_PERIODIC:
			__raw_writel((TIMER_CNT-1), ZMP_CE_CTRL1);
			__raw_writel((MODE_AUTO_RELOAD_TIMER | TIMER_ENABLE | TIMER_FEED_BIT ), ZMP_CE_CTRL2);
			break;

		case 1://CLOCK_EVT_MODE_ONESHOT:
			__raw_writel(0xffffffff, ZMP_CE_CTRL1);
			__raw_writel((MODE_ONE_SHOT_TIMER | TIMER_ENABLE | TIMER_FEED_BIT ), ZMP_CE_CTRL2);
			break;

		default:
			ret = -1;
			break;
	}

	return ret;
}

static int zmp_ce_timer_set_periodic(struct clock_event_device *evt)
{
	return zmp_ce_timer_set_mode(0, evt);
}

static int zmp_ce_timer_set_oneshot(struct clock_event_device *evt)
{
	return zmp_ce_timer_set_mode(1, evt);
}

static int zmp_ce_timer_set_next_event(unsigned long next,
    struct clock_event_device *evt)
{
    __raw_writel(next, ZMP_CE_CTRL1);
    __raw_writel((TIMER_ENABLE | MODE_ONE_SHOT_TIMER| TIMER_FEED_BIT), ZMP_CE_CTRL2);

    return 0;
}

static struct clock_event_device zmp_ced = {
    .name       = "zmp_ce_timer",
    .features   = CLOCK_EVT_FEAT_PERIODIC | CLOCK_EVT_FEAT_ONESHOT,
    .shift      = 32,
    .rating     = 150,
    .irq        = IRQ_TIMER,
    .set_next_event = zmp_ce_timer_set_next_event,
	.set_state_periodic = zmp_ce_timer_set_periodic,
	.set_state_oneshot = zmp_ce_timer_set_oneshot,
};

/* interrupt handler of timer1 */
static irqreturn_t zmp_ce_timer_interrupt(int irq, void *handle)
{
    struct clock_event_device   *dev = handle;
    u32 ctrl2;

    ctrl2 = __raw_readl(ZMP_CE_CTRL2);
    if (ctrl2 & TIMER_STATUS_BIT) {
        dev->event_handler(dev);
        __raw_writel(ctrl2 | TIMER_CLEAR_BIT, ZMP_CE_CTRL2);
        return IRQ_HANDLED;
    }      

    return IRQ_NONE;
}

static struct irqaction zmp_ce_timer_irq = {
    .name	= "zmp_ce_timer irq",
    .flags	= IRQF_TIMER | IRQF_IRQPOLL,
    .handler	= zmp_ce_timer_interrupt,
    .dev_id	= &zmp_ced,
};

/* use timer7 as sched clock */
static u64 zmp_read_sched_clock(void)
{
    u32 ctrl1, ctrl2;
    unsigned long flags;

    local_irq_save(flags);

    /* select read current count mode */
	ctrl2 = __raw_readl(ZMP_CS_CTRL2);
	__raw_writel(ctrl2 | TIMER_READ_SEL_BIT, ZMP_CS_CTRL2);

    ctrl1 = __raw_readl(ZMP_CS_CTRL1);

    /* resume read mode */
    ctrl2 = __raw_readl(ZMP_CS_CTRL2);
	__raw_writel(ctrl2 & (~TIMER_READ_SEL_BIT), ZMP_CS_CTRL2);

    local_irq_restore(flags);

    return ~ctrl1;
}

/*
 * parse_and_map_all_timers - parse timer node and map all timers
 * @node:		pointer to timer node
 *
 * @RETURN: 0-success, -1-fail
 */
static int parse_and_map_all_timers(struct device_node *node)
{
	int i;
	int irq;
	int num_irqs = 7;


	for (i = 0; i < num_irqs; i++) {
		irq = irq_of_parse_and_map(node, i);
		if (irq <= 0) {
			pr_err("%s map irq fail, irq:%d, i:%d\n", __func__, irq, i);
			return -1;
		}
		zmp_timer_set_timers_irq(i, irq);
	}

	return 0;
}

static void __init zmp_timer_init(struct device_node *node)
{
	int irq;
    pr_err("zmp_timer_init\n");

	if (parse_and_map_all_timers(node))
		panic("Can't parse&map all timers");

	irq = irq_of_parse_and_map(node, ZMP_CE_TIMER_INDEX); /* index 0 - timer1, 6 - timer7 */
	if (irq <= 0)
		panic("Can't parse IRQ");

	/* clock for timers*/

	/* enable clocksource timer */
    /* timer clocksource init */
    __raw_writel(0xffffffff, ZMP_CS_CTRL1);
    __raw_writel((TIMER_ENABLE | MODE_AUTO_RELOAD_TIMER| TIMER_FEED_BIT), 
                  ZMP_CS_CTRL2);

	/* register to clocksource framework */
	if (clocksource_register_hz(&zmp_cs, TIMER_CLK_INPUT))
		pr_err("zmp_sys_timer_init: clocksource_register failed for %s\n",
					zmp_cs.name);

    /* register to clock event framework */
    zmp_ced.cpumask    = cpumask_of(0);
    clockevents_config_and_register(&zmp_ced, TIMER_CLK_INPUT, 120000, 0xffffffff);

    if (setup_irq(irq, &zmp_ce_timer_irq))
        pr_err("zmp_sys_timer_init: irq register failed for %s\n",
                    zmp_ce_timer_irq.name);

	/* register to 64bit general sched clock framework */
    sched_clock_register(zmp_read_sched_clock, 32, TIMER_CLK_INPUT);
}

CLOCKSOURCE_OF_DECLARE(zmp_hrtimer, "zlgmcu,zmp110x-system-timer", zmp_timer_init);



