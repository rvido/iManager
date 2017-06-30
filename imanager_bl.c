/*
 * Advantech iManager Backlight driver
 * Partially derived from wm831x_bl
 *
 * Copyright (C) 2016 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/backlight.h>
#include <linux/device.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include "compat.h"
#include "imanager.h"

#define BL_MAX_PWM	100

enum backlight_units { BL_UNIT_1 = 0, BL_UNIT_2 };

static bool polarity = PWM_POLARITY_NORMAL;
module_param(polarity, bool, 0444);
MODULE_PARM_DESC(polarity, "Select backlight polarity (inverted := 1)");

static ushort unit = BL_UNIT_1;
module_param(unit, ushort, 0444);
MODULE_PARM_DESC(unit, "Select backlight control unit [0, 1] (defaults to 0)");

struct imanager_backlight_data {
	struct imanager_device_data *imgr;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	struct backlight_device *bd;
#endif
};

struct brightness_level {
	uint value	: 7,	/* Brightness Value  - LSB [6..0] */
	     enable	: 1;	/* Brightness Enable - MSB [7] */
};

struct backlight_ctrl {
	uint enable	: 1,	/* Backlight Control Enable - LSB [0] */
	     pwmpol	: 1,	/* PWM Polarity		    - bit [1] */
	     blpol	: 1,	/* Backlight Polarity	    - bit [2] */
	     dnc	: 5;	/* Don't care		    - bit [7..3] */
};

static int imanager_bl_enable(struct imanager_device_data *imgr, int unit)
{
	u8 val8;
	struct brightness_level *ctrl = (struct brightness_level *)&val8;
	u8 devid = imgr->ec.bl.attr[unit]->did;
	u8 bl_unit = imgr->ec.bl.brightness[unit];
	int ret;

	ret = imanager_mem_read(imgr, EC_RAM_ACPI, bl_unit, &val8,
				sizeof(val8));
	if (ret < 0)
		return ret;

	ctrl->enable = 1;

	return imanager_mem_write(imgr, EC_RAM_ACPI, devid, &val8,
				  sizeof(val8));
}

static int
imanager_bl_set_polarity(struct imanager_device_data *imgr, uint polarity)
{
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;
	int ret;

	ret = imanager_mem_read(imgr, EC_RAM_ACPI, EC_OFFSET_BACKLIGHT_CTRL,
				&val8, sizeof(val8));
	if (ret < 0)
		return ret;

	ctrl->blpol = polarity ? 1 : 0;

	return imanager_mem_write(imgr, EC_RAM_ACPI, EC_OFFSET_BACKLIGHT_CTRL,
				  &val8, sizeof(val8));
}

static int imanager_bl_get_brightness(struct backlight_device *bd)
{
	struct imanager_backlight_data *data = bl_get_data(bd);
	u8 devid = data->imgr->ec.bl.attr[unit]->did;
	int ret;

	ret = imanager_read8(data->imgr, EC_CMD_HWP_RD, devid);
	if (ret < 0) {
		dev_warn(&bd->dev, "Failed while reading PWM\n");
		ret = 0;
	}

	return polarity ? BL_MAX_PWM - ret : ret;
}

static int imanager_bl_set_brightness(struct backlight_device *bd)
{
	struct imanager_backlight_data *data = bl_get_data(bd);
	u8 devid = data->imgr->ec.bl.attr[unit]->did;
	u8 brightness = bd->props.brightness;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	/* invert brightness if polarity is set */
	brightness = polarity ? BL_MAX_PWM - brightness : brightness;

	return imanager_write8(data->imgr, EC_CMD_HWP_WR, devid, brightness);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
static struct backlight_ops imanager_bl_ops = {
#else
static const struct backlight_ops imanager_bl_ops = {
#endif
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = imanager_bl_get_brightness,
	.update_status  = imanager_bl_set_brightness,
};

static int imanager_bl_init(struct device *dev,
			    struct imanager_backlight_data *data)
{
	struct backlight_device *bd;
	struct backlight_properties props;
	int ret;

	memset(&props, 0, sizeof(props));
#if LINUX_VERSION_CODE > KERNEL_VERSION(2, 6, 38)
	props.type = BACKLIGHT_PLATFORM;
#endif
	props.max_brightness = BL_MAX_PWM;
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 12, 0)
	bd = devm_backlight_device_register(dev, "imanager-backlight", dev,
					    data, &imanager_bl_ops, &props);
#else
	bd = backlight_device_register("imanager_backlight", dev, data,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2, 6, 34)
				       &imanager_bl_ops);
#else
				       &imanager_bl_ops, &props);
#endif
#endif

	if (IS_ERR(bd)) {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
		data->bd = NULL;
#endif
		dev_err(dev, "Unable to register backlight device\n");
		return PTR_ERR(bd);
	}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	data->bd = bd;
#endif
	bd->props.brightness = imanager_bl_get_brightness(bd);
	bd->props.max_brightness = BL_MAX_PWM;
	bd->props.power = FB_BLANK_UNBLANK;

	backlight_update_status(bd);

	ret = imanager_bl_enable(data->imgr, unit);
	if (ret < 0)
		dev_warn(dev, "Could not enable backlight control\n");

	ret = imanager_bl_set_polarity(data->imgr, polarity);
	if (ret < 0)
		dev_warn(dev, "Could not set backlight polarity\n");

	return 0;
}

static int imanager_bl_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_backlight_data *data;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->imgr = imgr;

	ret = imanager_bl_init(dev, data);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);

	return 0;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
static int imanager_bl_remove(struct platform_device *pdev)
{
	struct imanager_backlight_data *data = dev_get_drvdata(&pdev->dev);

	backlight_device_unregister(data->bd);

	return 0;
 }
#endif

static struct platform_driver imanager_backlight_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
		.owner = THIS_MODULE,
#endif
		.name	= "imanager-backlight",
	},
	.probe	= imanager_bl_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	.remove = imanager_bl_remove,
#endif
};

module_platform_driver(imanager_backlight_driver);

MODULE_DESCRIPTION("Advantech iManager Backlight driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-backlight");
