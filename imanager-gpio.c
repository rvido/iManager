/*
 * Advantech iManager GPIO driver
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/version.h>
#include <compat.h>
#include <core.h>
#include <gpio.h>

struct imanager_gpio_data {
	struct imanager_device_data *ec;
	struct gpio_chip chip;
};

static inline struct imanager_gpio_data *
to_imanager_gpio_data(struct gpio_chip *chip)
{
	return container_of(chip, struct imanager_gpio_data, chip);
}

static int imanager_direction_in(struct gpio_chip *chip, u32 gpio_num)
{
	struct imanager_gpio_data *data = to_imanager_gpio_data(chip);
	int ret;

	mutex_lock(&data->ec->lock);

	ret = gpio_core_set_direction(gpio_num, GPIOF_DIR_IN);
	if (ret) {
		dev_err(chip->dev, "Failed to set direction to 'in' (%d)\n",
			gpio_num);
		ret = -EIO;
	}

	mutex_unlock(&data->ec->lock);

	return ret;
}

static int imanager_direction_out(struct gpio_chip *chip, u32 gpio_num, int val)
{
	struct imanager_gpio_data *data = to_imanager_gpio_data(chip);
	int ret;

	mutex_lock(&data->ec->lock);

	ret = gpio_core_set_direction(gpio_num, GPIOF_DIR_OUT);
	if (ret) {
		dev_err(chip->dev, "Failed to set direction to 'out' (%d)\n",
			gpio_num);
		ret = -EIO;
	}

	mutex_unlock(&data->ec->lock);

	return ret;
}

static int imanager_get(struct gpio_chip *chip, unsigned int gpio_num)
{
	struct imanager_gpio_data *data = to_imanager_gpio_data(chip);
	int ret;

	mutex_lock(&data->ec->lock);

	ret = gpio_core_get_state(gpio_num);
	if (ret < 0) {
		dev_err(chip->dev, "Failed to get status (%d)\n", gpio_num);
		ret = -EIO;
	}

	mutex_unlock(&data->ec->lock);

	return ret;
}

static void imanager_set(struct gpio_chip *chip, unsigned int gpio_num,
			 int val)
{
	struct imanager_gpio_data *data = to_imanager_gpio_data(chip);
	int ret;

	mutex_lock(&data->ec->lock);

	ret = gpio_core_set_state(gpio_num, val);
	if (ret < 0)
		dev_err(chip->dev, "Failed to set status (%d)\n", gpio_num);

	mutex_unlock(&data->ec->lock);
}

static int imanager_gpio_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *ec = dev_get_drvdata(dev->parent);
	struct imanager_gpio_data *data;
	struct gpio_chip *chip;
	int err;

	if (!ec) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	err = gpio_core_init();
	if (err) {
		dev_err(dev, "Failed initializing GPIO core\n");
		return -EIO;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ec = ec;

	platform_set_drvdata(pdev, data);

	chip = &data->chip;

	chip->owner	= THIS_MODULE;
	chip->dev	= dev;
	chip->label	= "imanager_gpio";

	chip->base	= -1;
	chip->ngpio	= EC_MAX_GPIO;

	chip->get	= imanager_get;
	chip->set	= imanager_set;

	chip->can_sleep	= 1;

	chip->direction_input  = imanager_direction_in;
	chip->direction_output = imanager_direction_out;

	err = gpiochip_add(chip);
	if (err < 0) {
		dev_err(dev, "Failed to register driver\n");
		gpio_core_release();
		return err;
	}

	return 0;
}

static int imanager_remove(struct platform_device *pdev)
{
	struct imanager_gpio_data *data = platform_get_drvdata(pdev);
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,17,0)
	int err;

	err = gpiochip_remove(&data->chip);
	if (err)
		dev_err(&pdev->dev, "Error while removing gpio device\n");
#else
	gpiochip_remove(&data->chip);
#endif

	gpio_core_release();

	return 0;
}

static struct platform_driver imanager_gpio_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "imanager_gpio",
	},
	.probe	= imanager_gpio_probe,
	.remove	= imanager_remove,
};

module_platform_driver(imanager_gpio_driver);

MODULE_DESCRIPTION("Advantech iManager GPIO Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager_gpio");
