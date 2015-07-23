/*
 * Advantech iManager Hardware Monitoring core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __HWMON_H__
#define __HWMON_H__

#include <linux/types.h>

#define HWM_MAX_ADC	5
#define HWM_MAX_FAN	3

/* Voltage computation (10-bit ADC, 0..3V input) */
#define SCALE_IN	2933	/* (3000mV / (2^10 - 1)) * 1000 */

/* Default Voltage Sensors */
struct hwm_voltage {
	/* if set, below values are valid */
	bool	valid;
	u32	value;
	u32	min;
	u32	max;
	u32	average;
	u32	lowest;
	u32	highest;

};

struct hwm_fan_temp_limit {
	u32	stop;
	u32	min;
	u32	max;
};

struct hwm_fan_limit {
	u32	min;
	u32	max;
};

struct hwm_fan_alert {
	u32	min;
	u32	max;
	u32	min_alarm;
	u32	max_alarm;
};

struct hwm_sensors_limit {
	struct hwm_fan_temp_limit	temp;
	struct hwm_fan_limit		pwm;
	struct hwm_fan_limit		rpm;
};

struct hwm_smartfan {
	/* if set, below values are valid */
	u32	valid;

	u32	mode,
		type,
		pwm,
		speed,
		pulse,
		alarm;
	int	temp;

	struct hwm_sensors_limit	limit;
	struct hwm_fan_alert		alert;
};

struct hwm_data {
	struct hwm_voltage		volt[HWM_MAX_ADC];
	struct hwm_smartfan		fan[HWM_MAX_FAN];
};

enum fan_unit {
	FAN_CPU,
	FAN_SYS1,
	FAN_SYS2,
};

enum fan_ctrl_type {
	CTRL_PWM,
	CTRL_RPM,
};

enum fan_mode {
	MODE_OFF,
	MODE_FULL,
	MODE_MANUAL,
	MODE_AUTO,
};

int hwm_core_init(void);
void hwm_core_release(void);

int hwm_core_check_adc(int num);
int hwm_core_check_fan(int num);

int hwm_core_get_fan_ctrl(int num, struct hwm_smartfan *fan);
int hwm_core_set_fan_ctrl(int num, int fmode, int ftype, int pwm, int pulse,
			  struct hwm_sensors_limit *limit,
			  struct hwm_fan_alert *alert);

int hwm_core_set_fan_limit_rpm(int num, int min, int max);
int hwm_core_set_fan_limit_pwm(int num, int min, int max);
int hwm_core_set_fan_limit_temp(int num, int stop, int min, int max);

int hwm_core_get_adc(int num, struct hwm_voltage *volt);

const char * hwm_core_get_adc_label(int num);
const char * hwm_core_get_fan_label(int num);
const char * hwm_core_get_fan_temp_label(int num);

#endif
