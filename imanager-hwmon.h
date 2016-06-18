/*
 * Advantech iManager Hardware Monitoring driver
 *
 * Copyright (C) 2016 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __IMANAGER_HWMON_H__
#define __IMANAGER_HWMON_H__

#include <linux/types.h>

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

/*
 * iManager FAN device defs
 */
struct fan_dev_config {
	u8	did,
		hwpin,
		tachoid,
		status,
		control,
		temp_max,
		temp_min,
		temp_stop,
		pwm_max,
		pwm_min;
	u16	rpm_max;
	u16	rpm_min;
	u8	debounce;	/* debounce time, not used */
	u8	temp;		/* Current Thermal Zone Temperature */
	u16	rpm_target;	/* RPM Target Speed, not used */
} __attribute__((__packed__));

struct fan_alert_limit {
	u16	min,
		max;
} __attribute__((__packed__));

struct fan_status {
	u32	sysctl	: 1,	/* System Control flag */
		tacho	: 1,	/* FAN tacho source defined */
		pulse	: 1,	/* FAN pulse type defined */
		thermal	: 1,	/* Thermal zone init */
		i2clink	: 1,	/* I2C protocol fail flag (thermal sensor) */
		dnc	: 1,	/* don't care */
		mode	: 2;	/* FAN Control mode */
};

/*----------------------------------------------------*
 * FAN Control bit field                              *
 * enable:   0:Disabled, 1:Enabled                    *
 * type:     0:PWM,      1:RPM                        *
 * pulse:    0:Undefined 1:2 Pulses  2:4 Pulses       *
 * tacho:    1:CPU FAN,  2:SYS FAN1, 3:SYS FAN2       *
 * mode:     0:Off,      1:Full,     2:Manual, 3:Auto *
 *- 7  6 ---- 5  4 --- 3  2 ----- 1 -------- 0 -------*
 *  MODE   | TACHO  |  PULSE  |  TYPE  |    ENABLE    *
 *----------------------------------------------------*/
struct fan_ctrl {
	u32	enable	: 1,	/* SmartFAN control on/off */
		type	: 1,	/* FAN control type [0, 1] PWM/RPM */
		pulse	: 2,	/* FAN pulse [0..2] */
		tacho	: 2,	/* FAN Tacho Input [1..3] */
		mode	: 2;	/* off/full/manual/auto */
};

/* Default Voltage Sensors */
struct imanager_hwmon_adc {
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

struct imanager_hwmon_smartfan {
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

#endif
