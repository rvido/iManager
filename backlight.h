/*
 * Advantech iManager Backlight core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __BACKLIGHT_H__
#define __BACKLIGHT_H__

#include <linux/types.h>

enum backlight_control {
	BL_CTRL_DISABLE,
	BL_CTRL_ENABLE,
};

enum backlight_unit {
	UNIT_1,
	UNIT_2,
};

int bl_core_init(void);

int bl_core_set_state(u32 unit, bool enable);

int bl_core_set_polarity(u32 polarity);

int bl_core_get_pulse_width(u32 unit);
int bl_core_set_pulse_width(u32 unit, u32 pwm);

#endif
