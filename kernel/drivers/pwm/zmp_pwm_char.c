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


#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/of_platform.h>
#include <linux/fs.h>
#include <linux/slab.h>
#include <linux/pwm.h>


#include <linux/clk.h>
#include <asm/io.h>
#include <mach/map.h>

/*
** all five PWMs can be used in H3D chip.
** Reference to zmp_timer.c, please.
**
*/
#define ZMP_PWM0_CTRL	(ZMP_VA_SYSCTRL+0xB4)
#define ZMP_PWM1_CTRL	(ZMP_VA_SYSCTRL+0xBC)
#define ZMP_PWM2_CTRL	(ZMP_VA_SYSCTRL+0xC4)
#define ZMP_PWM3_CTRL	(ZMP_VA_SYSCTRL+0xCC)
#define ZMP_PWM4_CTRL	(ZMP_VA_SYSCTRL+0xD4)

#define ZMP_PWM_TIMER_CTRL1 		(0x00)
#define ZMP_PWM_TIMER_CTRL2 		(0x04)

#define ZMP_PWM_HIGH_LEVEL(x) 		((x) << 16)
#define ZMP_PWM_LOW_LEVEL(x) 		(x)

#define ZMP_TIMER_TIMEOUT_CLR 		(1<<30)
#define ZMP_TIMER_FEED_TIMER 		(1<<29)
#define ZMP_PWM_TIMER_EN 			(1<<28)
#define ZMP_TIMER_TIMEOUT_STA 		(1<<27)
#define ZMP_TIMER_READ_SEL 			(1<<26)

#define ZMP_TIMER_WORK_MODE_MASK 	(0x3<<24)
#define ZMP_TIMER_WORK_MODE(x) 		((x)<<24)
#define ZMP_PWM_TIMER_PRE_DIV(x) 	((x) << 16)
#define ZMP_PWM_TIMER_PRE_DIV_MASK  ((0xff) << 16)
#define ZMP_PWM_TIMER_PRE_DIV_MAX	0xff
#define ZMP_PWM_MAX_DUTY_CYCLE		65536

#define REAL_CRYSTAL_FREQ 		(12*1000*1000)
#define PWM_MAX_FREQ 		(6*1000*1000)
#define PWM_MIN_FREQ 		(92/256)

#define ZMP_PWM_TIMER_CNT 	(5)
#define NSEC_PER_SEC	1000000000L

enum zmp_pwm_timer_mode
{
	ZMP_PT_MODE_TIMER_AUTO_LOAD = 0,
	ZMP_PT_MODE_TIMER_ONE_SHOT,
	ZMP_PT_MODE_PWM,
};


struct zmp_pwm_chip {
	struct pwm_chip chip;
	void __iomem *base;         /*pwm register base address*/
    /* PWM number. */
    int id;
	int active_cnt;
	int duty_ns;
	int period_ns;
};

static u32 pwm_freq = 0;
static u32 pwm_duty = 0;
static ssize_t pwm_info_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	return sprintf(buf, "pwm frequency: %uHz\npwm duty: %u%%\n", pwm_freq, pwm_duty*100/ZMP_PWM_MAX_DUTY_CYCLE);
}

static ssize_t pwm_status_show(struct device *dev, struct device_attribute *attr,char *buf)
{
	unsigned long regval;
    char *func, *running_status;
    struct zmp_pwm_chip *pwm;
    pwm = dev_get_drvdata(dev);

	regval = __raw_readl(pwm->base + ZMP_PWM_TIMER_CTRL2);
    if(regval & ZMP_PWM_TIMER_EN)
        running_status = "ENABLE";
    else running_status = "DISABLE";

    if(regval & ZMP_TIMER_WORK_MODE(ZMP_PT_MODE_PWM))
        func = "PWM";
    else if (regval & ZMP_TIMER_WORK_MODE(ZMP_PT_MODE_TIMER_ONE_SHOT))
        func = "ONE SHOT TIMER";
    else if (regval & ZMP_TIMER_WORK_MODE(ZMP_PT_MODE_TIMER_AUTO_LOAD))
        func = "AUTO LOAD TIMER";
    else func = "UNKNOWN";

	return sprintf(buf, "Function: %s\nRunning Status: %s\n", func, running_status);
}

static DEVICE_ATTR(freq_duty, 0400 , pwm_info_show, NULL);
static DEVICE_ATTR(running_status, 0400 , pwm_status_show, NULL);

static struct attribute *zmp_pwm_attributes[] = {
        &dev_attr_freq_duty.attr,
        &dev_attr_running_status.attr,
        NULL,
};
static const struct attribute_group zmp_pwm_attrs = {
        .attrs = zmp_pwm_attributes,
};

static inline struct zmp_pwm_chip *to_zmp_pwm_chip(struct pwm_chip *c)
{
	return container_of(c, struct zmp_pwm_chip, chip);
}

static inline int pwm_freq_is_vaild(u32 freq)
{
	return ((freq >= PWM_MIN_FREQ) && (freq <= PWM_MAX_FREQ));
}

/* To set ducy cycle */
static int zmp_pwm_set_duty_cycle(struct pwm_chip *chip, u16 high, u16 low, u32 pre_div)
{
	u32 regval;
	struct zmp_pwm_chip *pc = to_zmp_pwm_chip(chip);

	__raw_writel(high << 16 | low,pc->base + ZMP_PWM_TIMER_CTRL1);

	regval = __raw_readl(pc->base + ZMP_PWM_TIMER_CTRL2);
	regval &= ~ZMP_PWM_TIMER_PRE_DIV_MASK;
	regval |= ZMP_PWM_TIMER_PRE_DIV(pre_div);
	__raw_writel(regval,pc->base + ZMP_PWM_TIMER_CTRL2);

	return 0;
}


/* To get current ducy cycle */
int zmp_pwm_get_duty_cycle(struct pwm_chip *chip, unsigned short *high, unsigned short *low)
{
	unsigned long regval;
	struct zmp_pwm_chip *pc = to_zmp_pwm_chip(chip);

	regval = __raw_readl(pc->base + ZMP_PWM_TIMER_CTRL1);

	*high = regval >> 16;
	*low = regval & 0xFFFF;

	return 0;
}


static int zmp_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			       unsigned long long duty_ns, unsigned long long period_ns)
{
	struct zmp_pwm_chip *pc = to_zmp_pwm_chip(chip);

	__maybe_unused unsigned int regval,flag=0;
	unsigned int pre_div;
	unsigned int i = 0;
    __maybe_unused unsigned int tmp = 0;
    __maybe_unused unsigned long long hl = 0;
	int high_level= 0;
    int low_level = 0;

	regval = __raw_readl(pc->base + ZMP_PWM_TIMER_CTRL2);
	regval &= ~ZMP_TIMER_WORK_MODE_MASK;
	regval |= ZMP_TIMER_WORK_MODE(ZMP_PT_MODE_PWM);        /*set pwm/timer as PWM*/
	__raw_writel(regval,pc->base + ZMP_PWM_TIMER_CTRL2);

    /*
    ** how to calculate period_ns and duty_ns
    ** period_ns = 10^9 / pwm_freq
    ** duty_ns   = duty_cycle * period_ns
    */

    /* if pwm_freq < 100 HZ*/
    if(period_ns > 10000000){
        for (i = ZMP_PWM_TIMER_PRE_DIV_MAX+1; i>=1; i--) {
            unsigned long long tmp = REAL_CRYSTAL_FREQ/1000000 *duty_ns;
            do_div(tmp, 1000*i);
            high_level = tmp -1;
            tmp = REAL_CRYSTAL_FREQ/1000000 * period_ns;
            do_div(tmp, 1000*i);
            low_level  = tmp -2 - high_level;

            if(high_level < 0){
                high_level = 0;
            }
            if(low_level < 0){
                low_level = 0;
            }

            if(high_level > 65535)
                 high_level = 65535;
            if(low_level > 65535)
                 low_level = 65535;
            if ((high_level <= 65535) && (low_level <= 65535))
                break;
        }
    }
    else {
        for (i = 1; i<=ZMP_PWM_TIMER_PRE_DIV_MAX+1; i++) {
            unsigned long tmp = REAL_CRYSTAL_FREQ/1000000 *duty_ns;
            do_div(tmp, 1000*i);
	        high_level = tmp -1;
            tmp = REAL_CRYSTAL_FREQ/1000000 * period_ns;
            do_div(tmp, 1000*i);
            low_level  = tmp -2 - high_level;

            if(high_level < 0){
                high_level = 0;
            }
            if(low_level < 0){
                low_level = 0;
            }
            if ((high_level <= 65535) && (low_level <= 65535))
                break;
        }
    }

	pre_div = i - 1;
	pr_debug("div:%u, h:%u, l:%u, act_f:%d\n", pre_div, high_level, low_level,  (REAL_CRYSTAL_FREQ) / (pre_div + 1) / ((high_level+1)+(low_level+1)));

	return zmp_pwm_set_duty_cycle(chip, high_level, low_level, pre_div);
}

static int zmp_pwm_set_polarity(struct pwm_chip *chip,
				     struct pwm_device *pwm,
				     enum pwm_polarity polarity)
{
	/*
	 * No action needed here because pwm->polarity will be set by the core
	 * and the core will only change polarity when the PWM is not enabled.
	 * We'll handle things in set_enable().
	 */

	return 0;
}

/* To enable the pwm/timer . */
static int zmp_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 regval,regval1;
    struct zmp_pwm_chip *pc = to_zmp_pwm_chip(chip);

	regval = ZMP_TIMER_FEED_TIMER | ZMP_PWM_TIMER_EN;
    regval1 = __raw_readl(pc->base + ZMP_PWM_TIMER_CTRL2);
	regval1 |= regval;
    __raw_writel(regval1,pc->base + ZMP_PWM_TIMER_CTRL2);

	return 0;
}


static void zmp_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	u32 regval;
	struct zmp_pwm_chip *pc = to_zmp_pwm_chip(chip);

	regval = __raw_readl(pc->base + ZMP_PWM_TIMER_CTRL2);
	regval &= ~ZMP_PWM_TIMER_EN;
	__raw_writel(regval,pc->base + ZMP_PWM_TIMER_CTRL2);
}


static const struct pwm_ops zmp_pwm_ops = {
	.config = zmp_pwm_config,
	.set_polarity = zmp_pwm_set_polarity,
	.enable = zmp_pwm_enable,
	.disable = zmp_pwm_disable,
	.owner = THIS_MODULE,
};

/*PWM/GPIO sharepin setup */
static void zmp_pwm_init_setup(struct platform_device *pdev)
{
	/* config PWM sharepin as default state */
	pinctrl_pm_select_default_state(&pdev->dev);

}


/*
 *pwm init flow:
 *1.set sharepin.
 *2.disable pwm
 *3.set pwm period and duty_cycle
 *4.enable pwm
 */
static int zmp_pwm_probe(struct platform_device *pdev)
{
	struct zmp_pwm_chip *pwm;
    struct device_node *np = pdev->dev.of_node;
	int ret;
    unsigned long period_ns;
    unsigned long duty_ns;
    int pwm_enable;

	pwm = devm_kzalloc(&pdev->dev, sizeof(*pwm), GFP_KERNEL);
	if (!pwm)
		return -ENOMEM;

    pdev->id = of_alias_get_id(np, "pwm");

    pwm->id = pdev->id;

    if(pdev->id == 0)
        pwm->base = ZMP_PWM0_CTRL;
    else if(pdev->id == 1)
        pwm->base = ZMP_PWM1_CTRL;
    else if(pdev->id == 2)
        pwm->base = ZMP_PWM2_CTRL;
    else if(pdev->id == 3)
        pwm->base = ZMP_PWM3_CTRL;
    else if(pdev->id == 4)
        pwm->base = ZMP_PWM4_CTRL;
    else
        pr_err("Error,Could not read pwm id\n");


	ret = of_property_read_u32(pdev->dev.of_node, "period-ns",
				   (int *)(&(period_ns)));
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read pwm-frequency property\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "duty-ns",
				  (int *)(&(duty_ns)));
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read pwm-duty property\n");
	}

	ret = of_property_read_u32(pdev->dev.of_node, "pwm-enable",
				   &pwm_enable);
	if (ret < 0) {
		dev_warn(&pdev->dev,
			 "Could not read pwm-enable property\n");
	}

	platform_set_drvdata(pdev, pwm);

	pwm->chip.dev = &pdev->dev;
	pwm->chip.ops = &zmp_pwm_ops;          /*provide for proc*/
	pwm->chip.base = -1;
	pwm->chip.npwm = 1;
	pwm->chip.can_sleep = true;
	pwm->chip.of_xlate = of_pwm_xlate_with_flags;
	pwm->chip.of_pwm_n_cells = 3;

    /*PWM/GPIO sharepin setup */
    zmp_pwm_init_setup(pdev);

	ret = pwmchip_add(&pwm->chip);       /*add attributes of pwm into proc*/
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	/*
	*disable pwm before set pwm period and duty_cycle
	*/
	zmp_pwm_disable(&pwm->chip,NULL);

    /*init pwm */
    zmp_pwm_config(&pwm->chip,NULL,duty_ns,period_ns);
    if(pwm_enable)
        zmp_pwm_enable(&pwm->chip,NULL);
    else
        zmp_pwm_disable(&pwm->chip,NULL);

    sysfs_create_group(&pdev->dev.kobj, &zmp_pwm_attrs);

	return ret;
}

static int zmp_pwm_remove(struct platform_device *pdev)
{
	struct zmp_pwm_chip *pc = platform_get_drvdata(pdev);

    sysfs_remove_group(&pdev->dev.kobj, &zmp_pwm_attrs);

	return pwmchip_remove(&pc->chip);
}

static const struct of_device_id zmp_pwm_of_ids[] = {
    { .compatible = "zlgmcu,zmp110x-pwm0" },
    { .compatible = "zlgmcu,zmp110x-pwm1" },
    { .compatible = "zlgmcu,zmp110x-pwm2" },
    { .compatible = "zlgmcu,zmp110x-pwm3" },
    { .compatible = "zlgmcu,zmp110x-pwm4" },
	{},
};
MODULE_DEVICE_TABLE(of, zmp_pwm_of_ids);

static struct platform_driver zmp_pwm_driver = {
	.driver		= {
		.name	= "zmp-pwm",
        .of_match_table = of_match_ptr(zmp_pwm_of_ids),
		.owner	= THIS_MODULE,
	},
	.probe		= zmp_pwm_probe,
	.remove		= zmp_pwm_remove,
};

//static int __init zmp_pwm_init(void)
//{
//	return platform_driver_register(&zmp_pwm_driver);
//}
//module_init(zmp_pwm_init);
//
//static void __exit zmp_pwm_exit(void)
//{
//	platform_driver_unregister(&zmp_pwm_driver);
//}
//module_exit(zmp_pwm_exit);

module_platform_driver(zmp_pwm_driver);

MODULE_LICENSE("GPL");
MODULE_VERSION("1.0.03");
