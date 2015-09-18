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
	bool valid;	/* if set, below values are valid */

	int value;
	int min;
	int max;
	int average;
	int lowest;
	int highest;

};

struct hwm_fan_temp_limit {
	int stop;
	int min;
	int max;
};

struct hwm_fan_limit {
	int min;
	int max;
};

struct hwm_fan_alert {
	int min;
	int max;
	int min_alarm;
	int max_alarm;
};

struct hwm_sensors_limit {
	struct hwm_fan_temp_limit temp;
	struct hwm_fan_limit	  pwm;
	struct hwm_fan_limit	  rpm;
};

struct hwm_smartfan {
	bool valid;	/* if set, below values are valid */

	int mode;
	int type;
	int pwm;
	int speed;
	int pulse;
	int alarm;
	int temp;

	struct hwm_sensors_limit limit;
	struct hwm_fan_alert	 alert;
};

struct hwm_data {
	struct hwm_voltage	volt[HWM_MAX_ADC];
	struct hwm_smartfan	fan[HWM_MAX_FAN];
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

int hwm_core_adc_is_available(int num);
int hwm_core_adc_get_max_count(void);
int hwm_core_adc_get_value(int num, struct hwm_voltage *volt);
const char *hwm_core_adc_get_label(int num);

int hwm_core_fan_is_available(int num);
int hwm_core_fan_get_max_count(void);
int hwm_core_fan_get_ctrl(int num, struct hwm_smartfan *fan);
int hwm_core_fan_set_ctrl(int num, int fmode, int ftype, int pwm, int pulse,
			  struct hwm_sensors_limit *limit,
			  struct hwm_fan_alert *alert);

int hwm_core_fan_set_rpm_limit(int num, int min, int max);
int hwm_core_fan_set_pwm_limit(int num, int min, int max);
int hwm_core_fan_set_temp_limit(int num, int stop, int min, int max);

const char *hwm_core_fan_get_label(int num);
const char *hwm_core_fan_get_temp_label(int num);

#endif
