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

enum bl_control {
	BL_CTRL_DISABLE,
	BL_CTRL_ENABLE
};

enum bl_unit {
	UNIT_1,
	UNIT_2,
};

int bl_core_init(void);
void bl_core_release(void);

int bl_core_get_backlight_ctrl(enum bl_unit unit);
int bl_core_set_backlight_ctrl(enum bl_unit unit, u32 enable);

int bl_core_get_backlight_level(enum bl_unit unit);
int bl_core_set_backlight_level(enum bl_unit unit, u32 level);

int bl_core_get_backlight_polarity(void);
int bl_core_set_backlight_polarity(u32 polarity);

int bl_core_get_brightness_ctrl(void);
int bl_core_set_brightness_ctrl(u32 enable);

int bl_core_get_brightness_polarity(void);
int bl_core_set_brightness_polarity(u32 polarity);

int bl_core_get_pwm_pulse_width(enum bl_unit unit);
int bl_core_set_pwm_pulse_width(enum bl_unit unit, u32 pwm);

int bl_core_get_pwm_frequency(enum bl_unit unit);
int bl_core_set_pwm_frequency(enum bl_unit unit, u16 freq);

int bl_core_get_brightness_polarity2(enum bl_unit unit);
int bl_core_set_brightness_polarity2(enum bl_unit unit, u32 polarity);

#endif
