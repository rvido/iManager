/*
 * Advantech iManager Backlight/Brightness Core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/bug.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <ec.h>
#include <backlight.h>

struct brightness_level {
	u32	value	: 7,	/* Brightness Value  - LSB [6..0] */
		enable	: 1;	/* Brightness Enable - MSB [7] */
};

struct backlight_ctrl {
	u32	enable	: 1,	/* Backlight Control Enable - LSB [0] */
		pwmpol	: 1,	/* PWM Polarity		    - bit [1] */
		blpol	: 1,	/* Backlight Polarity	    - bit [2] */
		dnc	: 5;	/* Don't care		    - bit [7..3] */
};

static const struct imanager_backlight_device *bl;

int bl_core_get_pulse_width(u32 unit)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_read_byte(EC_CMD_HWP_RD, bl->attr[unit].did);
	if (ret < 0)
		pr_err("Failed reading PWM (unit=%d)\n", unit);

	return ret;
}

int bl_core_set_pulse_width(u32 unit, u32 pwm)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	pwm = pwm > 100 ? 100 : pwm;

	ret = imanager_write_byte(EC_CMD_HWP_WR, bl->attr[unit].did, pwm);
	if (ret < 0)
		pr_err("Failed writing PWM (val=%d, unit=%d)\n", pwm, unit);

	return ret;
}

int bl_core_set_state(u32 unit, bool enable)
{
	int ret;
	u8 val8;
	struct brightness_level *pl = (struct brightness_level *)&val8;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_acpiram_read_byte(bl->brightness[unit]);
	if (ret < 0)
		return ret;
	val8 = ret;

	pl->enable = enable ? 1 : 0;

	ret = imanager_acpiram_write_byte(bl->attr[unit].did, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_set_polarity(u32 polarity)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;

	ctrl->blpol = polarity ? 1 : 0;

	ret = imanager_acpiram_write_byte(EC_ACPIRAM_BLC_CTRL, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_init(void)
{
	bl = imanager_get_backlight_device();
	if (!bl)
		return -ENODEV;

	return 0;
}

