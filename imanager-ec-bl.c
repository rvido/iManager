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
		dnc	: 5;	/* Do Not Care		    - bit [7..3] */
};

static const struct imanager_backlight_device *dev;

int bl_core_get_pwm_pulse_width(u32 unit)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_read_byte(EC_CMD_HWP_RD, dev->attr[unit].did);
	if (ret < 0)
		pr_err("Failed reading PWM value of BLC%d\n", unit);

	return ret;
}

int bl_core_set_pwm_pulse_width(u32 unit, u32 pwm)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	pwm = pwm > 100 ? 100 : pwm;

	ret = imanager_write_byte(EC_CMD_HWP_WR, dev->attr[unit].did, pwm);
	if (ret < 0)
		pr_err("Failed writing PWM value '%d' of BLC%d\n",
			pwm, unit);

	return ret;
}

int bl_core_get_pwm_frequency(u32 unit)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_read_word(EC_CMD_PWM_FREQ_RD, dev->attr[unit].did);
	if (ret < 0)
		pr_err("Failed reading PWM frequency of BLC%d\n", unit);

	return ret;
}

int bl_core_set_pwm_frequency(u32 unit, u16 freq)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	freq = freq < 10 ? 10 : freq;

	ret = imanager_write_word(EC_CMD_PWM_FREQ_WR, dev->attr[unit].did,
				  freq);
	if (ret < 0)
		pr_err("Failed writing PWM frequency (%d) of BLC%d\n",
			freq, unit);

	return ret;
}

int bl_core_set_brightness_polarity2(u32 unit, u32 polarity)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_write_byte(EC_CMD_PWM_POL_WR, dev->attr[unit].did,
				  polarity);
	if (ret < 0)
		pr_err("Failed writing PWM polarity of BLC%d\n", unit);

	return ret;
}

int bl_core_get_brightness_polarity2(u32 unit)
{
	int ret;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_read_byte(EC_CMD_PWM_POL_RD, dev->attr[unit].did);
	if (ret < 0)
		pr_err("Failed reading PWM polarity value of BLC%d\n", unit);

	return ret;
}

int bl_core_get_backlight_level(u32 unit)
{
	int ret;
	u8 val8 = 0;
	struct brightness_level *pl = (struct brightness_level *)&val8;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_acpiram_read_byte(dev->attr[unit].did);
	if (ret < 0)
		return ret;
	val8 = ret;

	return pl->value;
}

int bl_core_set_backlight_level(u32 unit, u32 level)
{
	int ret;
	u8 val8 = 0;
	struct brightness_level *pl = (struct brightness_level *)&val8;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	/* brightness range: 0..9 */
	pl->value = level > 9 ? 9 : level;
	pl->enable = 1;

	ret = imanager_acpiram_write_byte(dev->attr[unit].did, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_set_backlight_ctrl(u32 unit, bool enable)
{
	int ret;
	u8 val8;
	struct brightness_level *pl = (struct brightness_level *)&val8;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_acpiram_read_byte(dev->brightness[unit]);
	if (ret < 0)
		return ret;
	val8 = ret;

	pl->enable = enable ? 1 : 0;

	ret = imanager_acpiram_write_byte(dev->attr[unit].did, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_get_backlight_ctrl(u32 unit)
{
	int ret;
	u8 val8;
	struct brightness_level *pl = (struct brightness_level *)&val8;

	if (WARN_ON(unit >= EC_BLC_MAX_NUM))
		return -EINVAL;

	ret = imanager_acpiram_read_byte(dev->attr[unit].did);
	if (ret < 0)
		return ret;
	val8 = ret;

	return pl->enable;
}

int bl_core_set_brightness_ctrl(bool enable)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;

	ctrl->enable = enable ? 0 : 1;

	ret = imanager_acpiram_write_byte(EC_ACPIRAM_BLC_CTRL, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_get_brightness_ctrl(void)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;

	return ctrl->enable ? 0 : 1;
}

int bl_core_set_backlight_polarity(u32 polarity)
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

int bl_core_get_backlight_polarity(void)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;

	return ctrl->blpol;
}

int bl_core_set_brightness_polarity(u32 polarity)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;
	ctrl->pwmpol = polarity ? 1 : 0;

	ret = imanager_acpiram_write_byte(EC_ACPIRAM_BLC_CTRL, val8);
	if (ret)
		return ret;

	return 0;
}

int bl_core_get_brightness_polarity(void)
{
	int ret;
	u8 val8;
	struct backlight_ctrl *ctrl = (struct backlight_ctrl *)&val8;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_BLC_CTRL);
	if (ret < 0)
		return ret;
	val8 = ret;

	return ctrl->pwmpol;
}

int bl_core_init(void)
{
	dev = imanager_get_backlight_device();
	if (!dev)
		return -ENODEV;

	return 0;
}

void bl_core_release(void)
{
}

