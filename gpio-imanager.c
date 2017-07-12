/*
 * Advantech iManager GPIO driver
 *
 * Copyright (C) 2016-2017 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/gpio.h>
#include <linux/init.h>
#include "imanager.h"
#include <linux/module.h>
#include <linux/platform_device.h>
#include "compat.h"

#define EC_GPIOF_DIR_OUT	BIT(6)
#define EC_GPIOF_DIR_IN		BIT(7)

/**
 * struct imanager_gpio_data - Defines iManager GPIO data structure
 * @imgr:	imanager_device_data structure describing the iManager
 * @chip:	abstract kernel GPIO controller
 */
struct imanager_gpio_data {
	struct imanager_device_data *imgr;
	struct gpio_chip chip;
};

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
#undef gpiochip_get_data
static inline struct imanager_gpio_data *
to_imanager_gpio_data(struct gpio_chip *chip)
{
	return container_of(chip, struct imanager_gpio_data, chip);
}
#define gpiochip_get_data(chip) to_imanager_gpio_data(chip)
#endif

static int imanager_gpio_direction_in(struct gpio_chip *chip, uint offset)
{
	struct imanager_gpio_data *data = gpiochip_get_data(chip);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_device_attribute *attr = imgr->ec.gpio.attr[offset];

	imanager_write8(imgr, EC_CMD_GPIO_DIR_WR, attr->did, EC_GPIOF_DIR_IN);

	return 0;
}

static int
imanager_gpio_direction_out(struct gpio_chip *chip, uint offset, int val)
{
	struct imanager_gpio_data *data = gpiochip_get_data(chip);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_device_attribute *attr = imgr->ec.gpio.attr[offset];

	imanager_write8(imgr, EC_CMD_GPIO_DIR_WR, attr->did, EC_GPIOF_DIR_OUT);

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0)
static int imanager_gpio_get_direction(struct gpio_chip *chip, uint offset)
{
	struct imanager_gpio_data *data = gpiochip_get_data(chip);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_device_attribute *attr = imgr->ec.gpio.attr[offset];
	int ret;

	ret = imanager_read8(imgr, EC_CMD_GPIO_DIR_RD, attr->did);
	if (ret < 0)
		return ret;

	return ret & EC_GPIOF_DIR_IN ? 1 : 0;
}
#endif

static int imanager_gpio_get(struct gpio_chip *chip, uint offset)
{
	struct imanager_gpio_data *data = gpiochip_get_data(chip);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_device_attribute *attr = imgr->ec.gpio.attr[offset];
	int ret;

	ret = imanager_read8(imgr, EC_CMD_HWP_RD, attr->did);
	if (ret < 0)
		return ret;

	return !!ret;
}

static void imanager_gpio_set(struct gpio_chip *chip, uint offset, int val)
{
	struct imanager_gpio_data *data = gpiochip_get_data(chip);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_device_attribute *attr = imgr->ec.gpio.attr[offset];

	imanager_write8(imgr, EC_CMD_HWP_WR, attr->did, val);
}

static int imanager_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_gpio_data *gpio;
	struct gpio_chip *chip;
	int ret;

	gpio = devm_kzalloc(dev, sizeof(*gpio), GFP_KERNEL);
	if (!gpio)
		return -ENOMEM;

	gpio->imgr = imgr;

	platform_set_drvdata(pdev, gpio);

	chip = &gpio->chip;

	chip->owner = THIS_MODULE;
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	chip->dev = dev;
#else
	chip->parent = dev;
#endif
	chip->label = "gpio-imanager";
	chip->base = -1;
	chip->ngpio = imgr->ec.gpio.num;
	chip->get = imanager_gpio_get;
	chip->set = imanager_gpio_set;
	chip->direction_input = imanager_gpio_direction_in;
	chip->direction_output = imanager_gpio_direction_out;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 8, 0)
	chip->get_direction = imanager_gpio_get_direction;
#endif
	if (!chip->ngpio) {
		dev_err(dev, "No GPIO pins detected\n");
		return -ENODEV;
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 5, 0)
	ret = gpiochip_add(chip);
#elif LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
	ret = gpiochip_add_data(chip, gpio);
#else
	ret = devm_gpiochip_add_data(dev, chip, gpio);
#endif
	if (ret < 0) {
		dev_err(dev, "Could not register GPIO chip\n");
		return ret;
	}

	dev_info(dev, "GPIO initialized with %d pins\n", chip->ngpio);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
static int imanager_gpio_remove(struct platform_device *pdev)
{
	struct imanager_gpio_data *data = platform_get_drvdata(pdev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 17, 0)
	int err = gpiochip_remove(&data->chip);
	if (err)
		dev_err(&pdev->dev, "Error while removing gpio device\n");
#else
	gpiochip_remove(&data->chip);
#endif
	return 0;
}
#endif

static struct platform_driver imanager_gpio_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
		.owner = THIS_MODULE,
#endif
		.name	= "imanager-gpio",
	},
	.probe	= imanager_gpio_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(4, 6, 0)
	.remove = imanager_gpio_remove,
#endif
};

module_platform_driver(imanager_gpio_driver);

MODULE_DESCRIPTION("Advantech iManager GPIO Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-gpio");
