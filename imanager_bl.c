/*
 * Advantech iManager Backlight driver
 * Partially derived from wm831x_bl
 *
 * Copyright (C) 2016 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/backlight.h>
#include <linux/fb.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/pwm.h>
#include <linux/slab.h>
#include "compat.h"
#include "imanager.h"

#define BL_MAX_PWM	100

#define BL_CTRL_DISABLE	0
#define BL_CTRL_ENABLE	1

#define BL_UNIT_1	0
#define BL_UNIT_2	1

static bool polarity = PWM_POLARITY_NORMAL;
module_param(polarity, bool, 0);
MODULE_PARM_DESC(polarity, "Select backlight polarity (inverted := 1)");

static ushort unit = BL_UNIT_1;
module_param(unit, ushort, 0);
MODULE_PARM_DESC(unit, "Select backlight control unit [0, 1] (defaults to 0)");

struct imanager_backlight_data {
	struct imanager_device_data *imgr;
	struct backlight_device *bd;
};

struct brightness_level {
	unsigned value	: 7,	/* Brightness Value  - LSB [6..0] */
		 enable	: 1;	/* Brightness Enable - MSB [7] */
};

struct backlight_ctrl {
	unsigned enable	: 1,	/* Backlight Control Enable - LSB [0] */
		 pwmpol	: 1,	/* PWM Polarity		    - bit [1] */
		 blpol	: 1,	/* Backlight Polarity	    - bit [2] */
		 dnc	: 5;	/* Don't care		    - bit [7..3] */
};

static int
imanager_bd_set_crtl(struct imanager_ec_data *ec, unsigned unit, bool enable)
{
	u8 val8;
	struct brightness_level *ctrl = (struct brightness_level *)&val8;
	u8 devid = ec->idev.bl.attr[unit].did;
	u8 bl_unit = ec->idev.bl.brightness[unit];
	struct imanager_io_ops *io = &ec->io;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, bl_unit, sizeof(val8), &val8,
				sizeof(val8));
	if (ret < 0)
		return ret;

	ctrl->enable = enable ? 1 : 0;

	return imanager_write_ram(io, EC_RAM_ACPI, devid, sizeof(val8), &val8);
}

static int
imanager_bl_set_polarity(struct imanager_io_ops *io, unsigned polarity)
{
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_BLC_CTRL,
				sizeof(val8), &val8, sizeof(val8));
	if (ret < 0)
		return ret;

	ctrl->blpol = polarity ? 1 : 0;

	return imanager_write_ram(io, EC_RAM_ACPI, EC_ACPIRAM_BLC_CTRL,
				  sizeof(val8), &val8);
}

static int imanager_get_brightness(struct backlight_device *bd)
{
	struct imanager_backlight_data *data = bl_get_data(bd);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	u8 devid = imgr->ec.idev.bl.attr[unit].did;
	int pwm;

	mutex_lock(&imgr->lock);

	pwm = imanager_read8(io, EC_CMD_HWP_RD, devid);
	if (pwm < 0) {
		dev_warn(&bd->dev, "Failed while reading PWM\n");
		pwm = 0;
	}

	mutex_unlock(&imgr->lock);

	return (polarity ? BL_MAX_PWM - pwm : pwm);
}

static int imanager_set_brightness(struct backlight_device *bd)
{
	struct imanager_backlight_data *data = bl_get_data(bd);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	u8 devid = imgr->ec.idev.bl.attr[unit].did;
	u8 brightness = bd->props.brightness;
	int ret;

	if (bd->props.power != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bd->props.fb_blank != FB_BLANK_UNBLANK)
		brightness = 0;

	if (bd->props.state & BL_CORE_SUSPENDED)
		brightness = 0;

	/* if polarity is set, inverse percentage */
	brightness = polarity ? BL_MAX_PWM - brightness : brightness;

	mutex_lock(&imgr->lock);

	ret = imanager_write8(io, EC_CMD_HWP_WR, devid, brightness);
	if (ret < 0)
		dev_warn(&bd->dev, "Failed while writing PWM\n");

	mutex_unlock(&imgr->lock);

	return ret;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
static struct backlight_ops imanager_bl_ops = {
#else
static const struct backlight_ops imanager_bl_ops = {
#endif
	.options = BL_CORE_SUSPENDRESUME,
	.get_brightness = imanager_get_brightness,
	.update_status  = imanager_set_brightness,
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
	props.max_brightness = BL_MAX_PWM;
	bd = backlight_device_register("imanager_backlight", dev, data,
#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
				       &imanager_bl_ops);
#else
				       &imanager_bl_ops, &props);
#endif

	if (IS_ERR(bd)) {
		data->bd = NULL;
		pr_err("Unable to register backlight device\n");
		return PTR_ERR(bd);
	}

	data->bd = bd;

	bd->props.brightness = imanager_get_brightness(bd);
	bd->props.power = FB_BLANK_UNBLANK;
	backlight_update_status(bd);

	return 0;
}

static int imanager_backlight_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_backlight_data *data;
	u8 devid = imgr->ec.idev.bl.attr[unit].did;
	int ret;

	if (!pdev) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->imgr = imgr;

	ret = imanager_backlight_init(dev, data);
	if (ret)
		return ret;

	platform_set_drvdata(pdev, data);

	ret = imanager_bd_set_crtl(&imgr->ec, unit, BL_CTRL_ENABLE);
	if (ret < 0)
		dev_warn(dev, "Could not enable backlight control\n");

	ret = imanager_bl_set_polarity(&imgr->ec.io, polarity);
	if (ret < 0)
		dev_warn(dev, "Could not set backlight polarity\n");

	/* Set brightness to 60% */
	ret = imanager_write8(&imgr->ec.io, EC_CMD_HWP_WR, devid, 60);
	if (ret < 0)
		dev_warn(dev, "Failed while writing PWM\n");

	return 0;
}

static int imanager_backlight_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_backlight_data *data = dev_get_drvdata(dev);

	backlight_device_unregister(data->bd);

	return 0;
}

static struct platform_driver imanager_backlight_driver = {
	.driver = {
		.name	= "imanager-backlight",
	},
	.probe	= imanager_backlight_probe,
	.remove	= imanager_backlight_remove,
};

module_platform_driver(imanager_backlight_driver);

MODULE_DESCRIPTION("Advantech iManager Backlight driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-bl");
