/*
 * Driver for watchdog device controlled through GPIO-line
 *
 * Author: 2013, Alexander Shiyan <shc_work@mail.ru>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 */

#include <linux/err.h>
#include <linux/delay.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/types.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/uaccess.h>
#include <linux/miscdevice.h>
#include <linux/io.h>
#include <linux/device.h>
#include <linux/gpio.h>
#include <linux/spinlock.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>

struct hw_wdt {
	int			gpio;
	unsigned long	expires;
	struct timer_list	timer;
	bool		state;
	spinlock_t io_lock;
};

static void hw_watchdog_feed(unsigned long data)
{
	struct hw_wdt *wdt;

	if(!data)
		return;
	
	wdt = (struct hw_wdt *)data;
	wdt->state = !wdt->state;
	spin_lock(&wdt->io_lock);
	gpio_set_value_cansleep(wdt->gpio, wdt->state);
	mod_timer(&wdt->timer, jiffies + wdt->expires);
	spin_unlock(&wdt->io_lock);
}

static int hw_wdt_probe(struct platform_device *pdev)
{
	struct hw_wdt *wdt;
	enum of_gpio_flags flags;
	unsigned int hw_margin;
	int ret;

	wdt = (struct hw_wdt *)devm_kzalloc(&pdev->dev, sizeof(struct hw_wdt), GFP_KERNEL);
	if (!wdt)
		return -ENOMEM;

	wdt->gpio = of_get_gpio_flags(pdev->dev.of_node, 0, &flags);
	if (!gpio_is_valid(wdt->gpio))
		return -ENODEV;

	wdt->state = true;

	ret = devm_gpio_request_one(&pdev->dev, wdt->gpio, GPIOF_OUT_INIT_HIGH, \
				dev_name(&pdev->dev));
	if(ret)
		return ret;

	ret = of_property_read_u32(pdev->dev.of_node,
				   "hw_margin_ms", &hw_margin);
	if (ret)
		return -ENODEV;

	/* Disallow values lower than 2 and higher than 1600 ms */
	if (hw_margin < 2 || hw_margin > 1120) 
		wdt->expires = msecs_to_jiffies(1000);	/* per second by default */
	else 
		wdt->expires = msecs_to_jiffies(hw_margin);
	
	spin_lock_init(&wdt->io_lock);

	platform_set_drvdata(pdev, wdt);

	setup_timer(&wdt->timer, hw_watchdog_feed, (unsigned long)wdt);
	mod_timer(&wdt->timer, jiffies + HZ / 50);	/* reset watchdog soon */

	pr_info("zlg gpio watchdog driver registered.\n");
	return 0;

}

static int hw_wdt_remove(struct platform_device *pdev)
{
	struct hw_wdt *priv = platform_get_drvdata(pdev);
	if(!priv) 
		return -ENODEV;

	del_timer_sync(&priv->timer);
	kfree(priv);

	return 0;
}

static const struct of_device_id gpio_wdt_dt_ids[] = {
	{ .compatible = "sp706s-gpio-wdt", },
	{ .compatible = "cat82x-gpio-wdt", },
	{ .compatible = "zlg-gpio-wdt", },
	{ }
};
MODULE_DEVICE_TABLE(of, gpio_wdt_dt_ids);

static struct platform_driver platform_wdt_driver = {
	.driver	= {
		.name		= "zlg-gpio-wdt",
		.owner		= THIS_MODULE,
		.of_match_table	= gpio_wdt_dt_ids,
	},
	.probe	= hw_wdt_probe,
	.remove	= hw_wdt_remove,
};
#if 1
static int __init gpio_wdt_init(void)
{
	return platform_driver_register(&platform_wdt_driver);
}

static void __exit gpio_wdt_exit(void)
{
	return platform_driver_unregister(&platform_wdt_driver);
}

subsys_initcall(gpio_wdt_init);
module_exit(gpio_wdt_exit);
#else
module_platform_driver(platform_wdt_driver);
#endif

MODULE_AUTHOR("Shen Codebreaker <shenguiting@zlg.cn>");
MODULE_DESCRIPTION("ZLG GPIO Watchdog");
MODULE_LICENSE("GPL");
