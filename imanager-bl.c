/*
 * Advantech iManager Backlight driver
 * Partially derived from wm831x_bl
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
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/pwm.h>
#include <linux/version.h>
#include <compat.h>
#include <core.h>
#include <backlight.h>

#define BL_MAX_BRIGHTNESS	100

static bool polarity = PWM_POLARITY_NORMAL;
module_param(polarity, bool, 0);
MODULE_PARM_DESC(polarity, "Select backlight polarity (inverted := 1)");

static ushort unit = UNIT_1;
module_param(unit, ushort, 0);
MODULE_PARM_DESC(unit, "Select backlight control unit [0, 1] (defaults to 0)");

struct imanager_backlight_data {
	struct imanager_device_data *ec;
	struct backlight_device *bl;
};

static int get_backlight_brightness(struct backlight_device *b)
{
	struct imanager_backlight_data *data = bl_get_data(b);
	int ret;

	mutex_lock(&data->ec->lock);

	ret = bl_core_get_pwm_pulse_width(unit);
	/* Reverse percentage if polarity is set */
	if (polarity)
		ret = 100 - ret;

	mutex_unlock(&data->ec->lock);

	return ret;
}

static int set_backlight_brightness(struct backlight_device *b)
{
	struct imanager_backlight_data *data = bl_get_data(b);
	u8 brightness = (u8) b->props.brightness;
	int ret;

	if (brightness > BL_MAX_BRIGHTNESS)
		return -EINVAL;

	if (b->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (b->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (b->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	mutex_lock(&data->ec->lock);

	/* Reverse percentage if polarity is set */
	if (polarity)
		brightness = 100 - brightness;
	ret = bl_core_set_pwm_pulse_width(unit, brightness);

	mutex_unlock(&data->ec->lock);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
static struct backlight_ops imanager_bl_ops = {
#else
static const struct backlight_ops imanager_bl_ops = {
#endif
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = get_backlight_brightness,
	.update_status  = set_backlight_brightness,
};

static int imanager_backlight_init(struct device *dev,
				 struct imanager_backlight_data *data)
{
	struct backlight_device *bd;
	struct backlight_properties props;

	memset(&props, 0, sizeof(props));
#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,38)
	props.type = BACKLIGHT_PLATFORM;
#endif
	props.max_brightness = BL_MAX_BRIGHTNESS;
	bd = backlight_device_register("imanager_backlight", dev, data,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
					&imanager_bl_ops);
#else
					&imanager_bl_ops, &props);
#endif

	if (IS_ERR(bd)) {
		data->bl = NULL;
		dev_err(dev, "Unable to register backlight device\n");
		return PTR_ERR(bd);
	}

	data->bl = bd;

	bd->props.brightness = get_backlight_brightness(bd);
	bd->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(bd);

	return 0;
}

static int imanager_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *ec = dev_get_drvdata(dev->parent);
	struct imanager_backlight_data *data;
	int ret;

	if (!ec) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	ret = bl_core_init();
	if (ret) {
		dev_err(dev, "Failed to initialize backlight core\n");
		return -EIO;
	}

	ret = bl_core_set_backlight_ctrl(unit, BL_CTRL_ENABLE);
	if (ret < 0) {
		dev_err(dev, "Failed to enable backlight control (%d)\n",
				unit);
		return -EIO;
	}

	if (polarity)
		ret = bl_core_set_backlight_polarity(PWM_POLARITY_INVERSED);
	else
		ret = bl_core_set_backlight_polarity(PWM_POLARITY_NORMAL);
	if (ret < 0) {
		dev_err(dev, "Failed to set backlight polarity\n");
		return -EIO;
	}

	/* init brightness to 60% */
	bl_core_set_pwm_pulse_width(unit, 60);

	data = devm_kzalloc(dev, sizeof(struct imanager_backlight_data),
			GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ec = ec;

	ret = imanager_backlight_init(dev, data);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);

	return 0;
}

static int imanager_backlight_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_backlight_data *data = dev_get_drvdata(dev);

	backlight_device_unregister(data->bl);
	bl_core_release();

	return 0;
}

static struct platform_driver imanager_backlight_driver = {
	.driver = {
		.owner	= THIS_MODULE,
		.name	= "imanager_backlight",
	},
	.probe	= imanager_backlight_probe,
	.remove	= imanager_backlight_remove,
};

module_platform_driver(imanager_backlight_driver);

MODULE_DESCRIPTION("Advantech iManager Backlight driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager_backlight");
