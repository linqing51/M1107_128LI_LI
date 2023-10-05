#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_gpio.h>
#include <linux/platform_device.h> 
#include <linux/of_platform.h> 

struct setgpio {
	struct device *dev;
	u32  minipice_gpio;
};

static int setgpio_probe(struct platform_device *pdev)
{
	int error;
	enum of_gpio_flags flags;
	struct setgpio *set_gpio;

	set_gpio = devm_kzalloc(&pdev->dev, sizeof(struct setgpio), GFP_KERNEL); 
	if (!set_gpio)
		return -ENOMEM;

	set_gpio->dev = &pdev->dev;
	
	set_gpio->minipice_gpio = of_get_named_gpio_flags(set_gpio->dev->of_node, "reset-gpios", 0, &flags);
	
	if (!gpio_is_valid(set_gpio->minipice_gpio)) {
		dev_err(set_gpio->dev, "invalid minipcie-reset-gpios\n");
		return -EINVAL;
	}

	error = devm_gpio_request_one(&pdev->dev,
				      set_gpio->minipice_gpio,
				      GPIOF_OUT_INIT_HIGH,
				      "minipcie reset gpio");

	if (error) {
		dev_err(set_gpio->dev, "unable to request minipcie-reset-gpios\n");
		return error;
	}
	platform_set_drvdata(pdev, set_gpio);
	
	gpio_set_value(set_gpio->minipice_gpio,1);
	dev_info(set_gpio->dev, "set minipcie rst\n"); 
	
	return 0;
}

static int setgpio_remove(struct platform_device *pdev)
{
	return 0;
}



static const struct of_device_id setgpio_match[] = {
	{ .compatible = "TKM,Set-Minipcie-Gpio", },
	{}
};
MODULE_DEVICE_TABLE(of, setgpio_match); 

static struct platform_driver setgpio_driver = {
	.driver = {
		.name = "Minipcie_Set_Gpio",
		.of_match_table = of_match_ptr(setgpio_match),
	},
	.probe = setgpio_probe,
	.remove = setgpio_remove,
};

static int __init setgpio_init(void)                               
{                                                              
    return platform_driver_register(&setgpio_driver);     
}                                                              
                                                               
static void __exit setgpio_exit(void)                          
{                                                              
    platform_driver_unregister(&setgpio_driver);          
}                                                              
//module_init(setgpio_init);
device_initcall_sync(setgpio_init);
//module_platform_driver(setgpio_driver);
//module_exit(setgpio_exit); 

MODULE_AUTHOR("zlg TKM");
MODULE_DESCRIPTION("Set Some GPIO status");
MODULE_LICENSE("GPL v2");
