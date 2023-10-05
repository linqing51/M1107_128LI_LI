/*
 * Copyright (c) 2015 Olliver Schinagl <oliver@schinagl.nl>
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
 *
 * This driver adds a high-resolution timer based PWM driver. Since this is a
 * bit-banged driver, accuracy will always depend on a lot of factors, such as
 * GPIO toggle speed and system load.
 */

#include <linux/mod_devicetable.h>
#include <linux/module.h>
#include <linux/gpio/consumer.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/of_device.h>
#include <linux/property.h>
#include <linux/pwm.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/device.h>
#include <linux/hrtimer.h>
#include <linux/ktime.h>
#include <linux/kernel.h>
#include <linux/platform_device.h>

#define DRV_NAME "zmp-pwm-gpio"

// zmp hardware timer
struct zmptimer;
typedef int (*timer_handler) (void *data);
int zmp_timer_start(timer_handler handler, void *data, void *priv, int hz);
int zmp_timer_stop(void *priv);
void * zmp_timer_probe(int which_timer);
int zmp_timer_remove(void *priv);

struct zmp_gpio_pwm_data {
	struct zmptimer *timer;
	struct gpio_desc *gpiod;
	bool polarity;
	bool pin_on;
	int on_time;
	int off_time;
	bool run;
};

struct zmp_gpio_pwm_chip {
	struct pwm_chip chip;
};

struct zmp_gpio_pwm_match_data {
  int timer;
};

static void zmp_gpio_pwm_off(struct zmp_gpio_pwm_data *gpio_data)
{
	gpiod_set_value_cansleep(gpio_data->gpiod, gpio_data->polarity ? 0 : 1);
}

static void zmp_gpio_pwm_on(struct zmp_gpio_pwm_data *gpio_data)
{
	gpiod_set_value_cansleep(gpio_data->gpiod, gpio_data->polarity ? 1 : 0);
}

int zmp_gpio_pwm_handler(void *arg)
{
	struct zmp_gpio_pwm_data *gpio_data = (struct zmp_gpio_pwm_data*)arg;
	zmp_timer_stop(gpio_data->timer);

	if (!gpio_data->pin_on) {
		zmp_gpio_pwm_on(gpio_data);
		gpio_data->pin_on = true;
		zmp_timer_start(zmp_gpio_pwm_handler, gpio_data,
		                gpio_data->timer, 1000000000UL / gpio_data->on_time);
	} else {
		zmp_gpio_pwm_off(gpio_data);
		gpio_data->pin_on = false;
		zmp_timer_start(zmp_gpio_pwm_handler, gpio_data,
		                gpio_data->timer, 1000000000UL / gpio_data->off_time);
	}

  return 0;
}

static int zmp_gpio_pwm_config(struct pwm_chip *chip, struct pwm_device *pwm,
			    unsigned long long duty_ns, unsigned long long period_ns)
{
	struct zmp_gpio_pwm_data *gpio_data = pwm_get_chip_data(pwm);

	gpio_data->on_time = duty_ns;
	gpio_data->off_time = period_ns - duty_ns;

	return 0;
}

static int zmp_gpio_pwm_set_polarity(struct pwm_chip *chip, struct pwm_device *pwm,
				 enum pwm_polarity polarity)
{
	struct zmp_gpio_pwm_data *gpio_data = pwm_get_chip_data(pwm);

	gpio_data->polarity = (polarity != PWM_POLARITY_NORMAL) ? true : false;

	return 0;
}

static int zmp_gpio_pwm_enable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct zmp_gpio_pwm_data *gpio_data = pwm_get_chip_data(pwm);

	if (gpio_data->run)
		return -EBUSY;

	gpio_data->run = true;

	if (gpio_data->off_time) {
		zmp_gpio_pwm_on(gpio_data);
		gpio_data->pin_on = true;
		zmp_timer_start(zmp_gpio_pwm_handler, gpio_data,
		                gpio_data->timer, 1000000000UL / gpio_data->on_time);
	} else {
		if (gpio_data->on_time)
			zmp_gpio_pwm_on(gpio_data);
		else
			zmp_gpio_pwm_off(gpio_data);
	}

	return 0;
}

static void zmp_gpio_pwm_disable(struct pwm_chip *chip, struct pwm_device *pwm)
{
	struct zmp_gpio_pwm_data *gpio_data = pwm_get_chip_data(pwm);
	zmp_timer_stop(gpio_data->timer);

	gpio_data->run = false;
	if (!gpio_data->off_time)
		zmp_gpio_pwm_off(gpio_data);
}

static const struct pwm_ops zmp_gpio_pwm_ops = {
	.config = zmp_gpio_pwm_config,
	.set_polarity = zmp_gpio_pwm_set_polarity,
	.enable = zmp_gpio_pwm_enable,
	.disable = zmp_gpio_pwm_disable,
	.owner = THIS_MODULE,
};

static int zmp_gpio_pwm_probe(struct platform_device *pdev)
{
	int ret;
	struct zmp_gpio_pwm_chip *gpio_chip;
	struct gpio_desc *gpiod;
	struct zmp_gpio_pwm_data *gpio_data;
	const struct zmp_gpio_pwm_match_data *chip_data;

	chip_data = of_device_get_match_data(&pdev->dev);
	if (!chip_data) {
		dev_err(&pdev->dev, "Error,Could not read pwm id\n");
		return -ENODEV;
	}
	dev_info(&pdev->dev, "using timer %d", chip_data->timer);

	gpio_chip = devm_kzalloc(&pdev->dev, sizeof(*gpio_chip), GFP_KERNEL);
	if (!gpio_chip)
		return -ENOMEM;

	gpio_chip->chip.dev = &pdev->dev;
	gpio_chip->chip.ops = &zmp_gpio_pwm_ops;
	gpio_chip->chip.base = -1;
	gpio_chip->chip.npwm = 1;
	gpio_chip->chip.of_xlate = of_pwm_xlate_with_flags;
	gpio_chip->chip.of_pwm_n_cells = 3;
	gpio_chip->chip.can_sleep = true;

	ret = pwmchip_add(&gpio_chip->chip);
	if (ret < 0) {
		dev_err(&pdev->dev, "failed to add PWM chip: %d\n", ret);
		return ret;
	}

	gpiod = devm_gpiod_get_optional(&pdev->dev, "pwm", GPIOD_OUT_LOW);
	if (IS_ERR(gpiod)) {
		int error;

		error = PTR_ERR(gpiod);
		if (error != -EPROBE_DEFER)
			dev_err(&pdev->dev,
				"failed to get gpio flags, error: %d\n",
				error);
		return error;
	}

	gpio_data = devm_kzalloc(&pdev->dev, sizeof(*gpio_data),
				 GFP_KERNEL);

	gpio_data->timer = zmp_timer_probe(chip_data->timer);
	gpio_data->gpiod = gpiod;
	gpio_data->pin_on = false;
	gpio_data->run = false;

	pwm_set_chip_data(&gpio_chip->chip.pwms[0], gpio_data);

	if (!gpio_data->timer) {
		dev_warn(&pdev->dev, "Can not probe hardware timer");
	}

	platform_set_drvdata(pdev, gpio_chip);

	return 0;
}

static int zmp_gpio_pwm_remove(struct platform_device *pdev)
{
	struct zmp_gpio_pwm_chip *gpio_chip;
	struct zmp_gpio_pwm_data *gpio_data;

	gpio_chip = platform_get_drvdata(pdev);
	gpio_data = pwm_get_chip_data(&gpio_chip->chip.pwms[0]);
  zmp_timer_remove(gpio_data->timer);

	return pwmchip_remove(&gpio_chip->chip);
}


static const struct zmp_gpio_pwm_match_data zmp_match_data[] = {
	{ .timer = 2 },
	{ .timer = 3 },
	{ .timer = 4 },
	{ .timer = 5 },
};

static const struct of_device_id zmp_gpio_pwm_of_match[] = {
	{ .compatible = "zlgmcu,zmp110x-pwm-gpio-1",
	  .data = &zmp_match_data[0]
	},
	{ .compatible = "zlgmcu,zmp110x-pwm-gpio-2",
	  .data = &zmp_match_data[1]
	},
	{ .compatible = "zlgmcu,zmp110x-pwm-gpio-3",
	  .data = &zmp_match_data[2]
	},
	{ .compatible = "zlgmcu,zmp110x-pwm-gpio-4",
	  .data = &zmp_match_data[3]
	},
	{/* sentinel */},
};
MODULE_DEVICE_TABLE(of, zmp_gpio_pwm_of_match);

static struct platform_driver zmp_gpio_pwm_driver = {
	.probe = zmp_gpio_pwm_probe,
	.remove = zmp_gpio_pwm_remove,
	.driver = {
		.name = DRV_NAME,
		.of_match_table = of_match_ptr(zmp_gpio_pwm_of_match),
	},
};
module_platform_driver(zmp_gpio_pwm_driver);

MODULE_AUTHOR("Olliver Schinagl <oliver@schinagl.nl>");
MODULE_AUTHOR("xinzhiting <xinzhiting@zlg.cn>");
MODULE_DESCRIPTION("GPIO bit-banged PWM driver based on ZMP110x hardware");
MODULE_LICENSE("GPL");
