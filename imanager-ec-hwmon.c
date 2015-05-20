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

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <linux/swab.h>
#include "ec.h"
#include "hwmon.h"

#define HWM_STATUS_UNDEFINED_ITEM	2UL
#define HWM_STATUS_UNDEFINED_DID	3UL
#define HWM_STATUS_UNDEFINED_HWPIN	4UL

struct fan_dev_settings {
	u8	did,		/* Device ID */
		hwp,		/* HW Pin number */
		tachoid,	/* Tacho ID */
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
};

struct fan_status {
	u32	sysctl	: 1,	/* System Control */
		ftacho	: 1,	/* FAN tacho source defined */
		fpulse	: 1,	/* FAN pulse type defined */
		thermal	: 1,	/* Thermal zone init */
		i2cfail	: 1,	/* I2C protocol failure (thermal sensor) */
		dnc	: 1,	/* don't care */
		mode	: 2;	/* FAN Control mode */
};

struct fan_alert_limit {
	u16	fan0_min,
		fan0_max,
		fan1_min,
		fan1_max,
		fan2_min,
		fan2_max;
};

struct fan_alert_flag {
	u32	fan0_min_alarm	: 1,
		fan0_max_alarm	: 1,
		fan1_min_alarm	: 1,
		fan1_max_alarm	: 1,
		fan2_min_alarm	: 1,
		fan2_max_alarm	: 1,
		dnc		: 2;
};

/*----------------------------------------------------*
 * FAN Control bit field                              *
 * enable:   0:Disabled, 1:Enabled                    *
 * type:     0:PWM,      1:RPM                        *
 * pulse:    0:Undefined 1:2 Pulse   2:4 Pulse        *
 * tacho:    1:CPU FAN,  2:SYS1 FAN, 3:SYS2 FAN       *
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

enum fan_dev_ctrl {
	CTRL_STATE = 3,
	OPMODE,
	IDSENSOR,
	ACTIVE,
	CTRL_MODE,
};

enum fan_limit {
	LIMIT_PWM,
	LIMIT_RPM,
	LIMIT_TEMP,
};

struct hwm_sensors {
	struct ec_info		info;
	struct adc_cfg		adc[EC_HWM_MAX_ADC];
	struct fan_cfg		fan[EC_HWM_MAX_FAN];
};

static const char * fan_temp_label[] = {
	"Temp CPU",
	"Temp SYS1",
	"Temp SYS2",
	NULL,
};

static struct hwm_sensors sensors;

static inline int hwm_get_adc_value(u8 did)
{
	return imanager_read_word(EC_CMD_HWP_RD, did);
}

static inline int hwm_get_rpm_value(u8 did)
{
	return imanager_read_word(EC_CMD_HWP_RD, did);
}

static inline int hwm_get_pwm_value(u8 did)
{
	return imanager_read_byte(EC_CMD_HWP_RD, did);
}

static inline int hwm_set_pwm_value(u8 did, u8 val)
{
	return imanager_write_byte(EC_CMD_HWP_WR, did, val);
}

static int hwm_read_fan_config(int fnum, struct fan_dev_settings *dev)
{
	int ret;
	struct ec_message msg = {
		.rlen = 0xff,
		.wlen = 0,
	};
	struct fan_dev_settings *_dev = (struct fan_dev_settings *)&msg.u.data;

	if (WARN_ON(!dev))
		return -EINVAL;

	ret = imanager_msg_read(EC_CMD_FAN_CTL_RD, fnum, &msg);
	if (ret)
		return ret;

	if (_dev->did == 0)
		return -ENODEV;

	memcpy(dev, &msg.u.data, sizeof(struct fan_dev_settings));

	return 0;
}

static int hwm_write_fan_config(int fnum, struct fan_dev_settings *dev)
{
	int ret;
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(struct fan_dev_settings),
	};

	if (WARN_ON(!dev))
		return -EINVAL;

	if (dev->did == 0)
		return -ENODEV;

	msg.data = (u8 *)dev;

	ret = imanager_msg_write(EC_CMD_FAN_CTL_WR, fnum, &msg);
	if (ret < 0)
		return ret;

	switch (ret) {
	case 0:
		break;
	case HWM_STATUS_UNDEFINED_ITEM:
	case HWM_STATUS_UNDEFINED_DID:
	case HWM_STATUS_UNDEFINED_HWPIN:
		ret = -EINVAL;
		break;
	default:
		pr_err("FAN%d CFG: Unknown error (%d)\n", fnum, ret);
		ret = -EIO;
	}

	return ret;
}

static inline void hwm_set_temp_limit(struct fan_dev_settings *dev,
				      const struct hwm_fan_temp_limit *temp)
{
	dev->temp_stop = temp->stop;
	dev->temp_min  = temp->min;
	dev->temp_max  = temp->max;
}

static inline void hwm_set_pwm_limit(struct fan_dev_settings *dev,
				     const struct hwm_fan_limit *pwm)
{
	dev->pwm_min = pwm->min;
	dev->pwm_max = pwm->max;
}

static inline void hwm_set_rpm_limit(struct fan_dev_settings *dev,
				     const struct hwm_fan_limit *rpm)
{
	dev->rpm_min = swab16(rpm->min);
	dev->rpm_max = swab16(rpm->max);
}

static inline void hwm_set_limit(struct fan_dev_settings *dev,
				 const struct hwm_sensors_limit *limit)
{
	hwm_set_temp_limit(dev, &limit->temp);
	hwm_set_pwm_limit(dev, &limit->pwm);
	hwm_set_rpm_limit(dev, &limit->rpm);
}

static int hwm_core_get_fan_alert_flag(struct fan_alert_flag *flag)
{
	int ret;
	u8 *value = (u8 *)flag;

	ret = imanager_acpiram_read_byte(EC_ACPIRAM_FAN_ALERT);
	if (ret < 0)
		return ret;

	*value = ret;

	return 0;
}

static int hwm_core_get_fan_alert_limit(enum fan_unit unit,
				        struct hwm_smartfan *fan)
{
	int ret;
	struct fan_alert_limit limit;
	struct fan_alert_flag flag;

	ret = imanager_acpiram_read_block(EC_ACPIRAM_FAN_SPEED_LIMIT,
					 (u8 *)&limit, sizeof(limit));
	if (ret < 0)
		return ret;

	ret = hwm_core_get_fan_alert_flag(&flag);
	if (ret < 0)
		return ret;

	switch (unit) {
	case FAN_CPU:
		fan->alert.min = swab16(limit.fan0_min);
		fan->alert.max = swab16(limit.fan0_max);
		fan->alert.min_alarm = flag.fan0_min_alarm;
		fan->alert.max_alarm = flag.fan0_max_alarm;
		break;
	case FAN_SYS1:
		fan->alert.min = swab16(limit.fan1_min);
		fan->alert.max = swab16(limit.fan1_max);
		fan->alert.min_alarm = flag.fan1_min_alarm;
		fan->alert.max_alarm = flag.fan1_max_alarm;
		break;
	case FAN_SYS2:
		fan->alert.min = swab16(limit.fan2_min);
		fan->alert.max = swab16(limit.fan2_max);
		fan->alert.min_alarm = flag.fan2_min_alarm;
		fan->alert.max_alarm = flag.fan2_max_alarm;
		break;
	default:
		pr_err("Unknown FAN ID %d\n", unit);
		return -EINVAL;
	}

	return 0;
}

static int hwm_core_set_fan_alert_limit(enum fan_unit unit,
					struct hwm_fan_alert *alert)
{
	int ret;
	struct fan_alert_limit limit;

	ret = imanager_acpiram_read_block(EC_ACPIRAM_FAN_SPEED_LIMIT,
					 (u8 *)&limit, sizeof(limit));
	if (ret < 0)
		return ret;

	switch (unit) {
	case FAN_CPU:
		limit.fan0_min = swab16(alert->min);
		limit.fan0_max = swab16(alert->max);
		break;
	case FAN_SYS1:
		limit.fan1_min = swab16(alert->min);
		limit.fan1_max = swab16(alert->max);
		break;
	case FAN_SYS2:
		limit.fan2_min = swab16(alert->min);
		limit.fan2_max = swab16(alert->max);
		break;
	default:
		pr_err("Unknown FAN ID %d\n", unit);
		return -EINVAL;
	}

	return imanager_acpiram_write_block(EC_ACPIRAM_FAN_SPEED_LIMIT,
					   (u8 *)&limit, sizeof(limit));
}

/* HWM CORE API */

const char * hwm_core_get_adc_label(u32 num)
{
	if (WARN_ON(num >= EC_HWM_MAX_ADC))
		return NULL;

	return sensors.adc[num].label;
}

const char * hwm_core_get_fan_label(enum fan_unit unit)
{
	if (WARN_ON(unit >= HWM_MAX_FAN))
		return NULL;

	return sensors.fan[unit].label;
}

const char * hwm_core_get_fan_temp_label(enum fan_unit unit)
{
	int ret;

	if (WARN_ON(unit >= HWM_MAX_FAN))
		return NULL;

	ret = hwm_core_check_fan(unit);

	return ret ? NULL : fan_temp_label[unit];
}

int hwm_core_check_adc(u32 num)
{
	if (WARN_ON(num >= HWM_MAX_ADC))
		return -EINVAL;

	return sensors.adc[num].did ? 0 : -ENODEV;
}

int hwm_core_get_adc(u32 num, struct hwm_voltage *volt)
{
	int val;

	if (!hwm_core_check_adc(num)) {
		val = hwm_get_adc_value(sensors.adc[num].did);
		if (val < 0)
			return val;
		volt->value = val * sensors.adc[num].scale;
		volt->valid = true;
	}
	else {
		volt->valid = false;
	}

	return 0;
}

int hwm_core_get_fan_ctrl(enum fan_unit unit, struct hwm_smartfan *fan)
{
	int ret;
	struct fan_dev_settings dev;
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&dev.control;

	if (WARN_ON((unit >= HWM_MAX_FAN) || !fan))
		return -EINVAL;

	ret = hwm_core_check_fan(unit);
	if (ret < 0)
		return ret;

	memset(fan, 0, sizeof(struct hwm_smartfan));

	ret = hwm_read_fan_config(unit, &dev);
	if (ret < 0)
		return ret;

	fan->pulse = ctrl->pulse;
	fan->type = ctrl->type;

	/*
	 * It seems that status->mode does not always report the correct
	 * FAN mode so the only way of reporting the current FAN mode is
	 * to read back ctrl->mode.
	 */
	fan->mode = ctrl->mode;

	ret = hwm_get_rpm_value(dev.tachoid);
	if (ret < 0) {
		pr_err("Failed to read FAN speed\n");
		return ret;
	}

	fan->speed = ret;

	ret = hwm_get_pwm_value(sensors.fan[unit].did);
	if (ret < 0) {
		pr_err("Failed to read FAN%d PWM\n", unit);
		return ret;
	}

	fan->pwm = ret;

	fan->alarm = ((fan->pwm > 0) && (fan->speed == 0)) ? 1 : 0;

	fan->temp		= dev.temp;
	fan->limit.temp.min	= dev.temp_min;
	fan->limit.temp.max	= dev.temp_max;
	fan->limit.temp.stop	= dev.temp_stop;
	fan->limit.pwm.min	= dev.pwm_min;
	fan->limit.pwm.max	= dev.pwm_max;
	fan->limit.rpm.min	= swab16(dev.rpm_min);
	fan->limit.rpm.max	= swab16(dev.rpm_max);

	ret = hwm_core_get_fan_alert_limit(unit, fan);
	if (ret)
		return ret;

	fan->valid = 1;

	return 0;
}

int hwm_core_set_fan_ctrl(enum fan_unit unit,
			  enum fan_mode mode,
			  enum fan_ctrl_type type,
			  u32 pwm,
			  u32 pulse,
			  struct hwm_sensors_limit *limit,
			  struct hwm_fan_alert *alert)
{
	int ret;
	struct fan_dev_settings dev;
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&dev.control;
	struct hwm_sensors_limit _limit = {{0, 0, 0}, {0, 0}, {0, 0}};

	if (WARN_ON(unit >= HWM_MAX_FAN))
		return -EINVAL;

	ret = hwm_read_fan_config(unit, &dev);
	if (ret < 0) {
		pr_err("Failed while reading FAN %s config\n",
			sensors.fan[unit].label);
		return ret;
	}

	if (!limit)
		limit = &_limit;

	switch (mode) {
	case MODE_OFF:
		ctrl->type = CTRL_PWM;
		ctrl->mode = MODE_OFF;
		break;
	case MODE_FULL:
		ctrl->type = CTRL_PWM;
		ctrl->mode = MODE_FULL;
		break;
	case MODE_MANUAL:
		ctrl->type = CTRL_PWM;
		ctrl->mode = MODE_MANUAL;
		ret = hwm_set_pwm_value(sensors.fan[unit].did, pwm);
		if (ret < 0)
			return ret;
		break;
	case MODE_AUTO:
		switch (type) {
		case CTRL_PWM:
			limit->rpm.min = 0;
			limit->rpm.max = 0;
			ctrl->type = CTRL_PWM;
			break;
		case CTRL_RPM:
			limit->pwm.min = 0;
			limit->pwm.max = 0;
			ctrl->type = CTRL_RPM;
			break;
		default:
			return -EINVAL;
		}
		ctrl->mode = MODE_AUTO;
		break;
	default:
		return -EINVAL;
	}

	hwm_set_limit(&dev, limit);

	ctrl->pulse = (pulse && (pulse < 3)) ? pulse : 0;
	ctrl->enable = 1;

	ret = hwm_write_fan_config(unit, &dev);
	if (ret < 0)
		return ret;

	if (alert)
		return hwm_core_set_fan_alert_limit(unit, alert);

	return 0;
}

int hwm_core_check_fan(enum fan_unit unit)
{
	return sensors.fan[unit].did ? 0 : -ENODEV;
}

static int hwm_core_set_fan_limit(enum fan_unit unit,
				  enum fan_limit type,
				  struct hwm_sensors_limit *limit)
{
	struct fan_dev_settings dev;
	int ret;

	if (WARN_ON(unit >= HWM_MAX_FAN))
		return -EINVAL;

	ret = hwm_read_fan_config(unit, &dev);
	if (ret < 0) {
		pr_err("Failed while reading FAN %s config\n",
			sensors.fan[unit].label);
		return ret;
	}

	switch (type) {
	case LIMIT_PWM:
		hwm_set_pwm_limit(&dev, &limit->pwm);
		break;
	case LIMIT_RPM:
		hwm_set_rpm_limit(&dev, &limit->rpm);
		break;
	case LIMIT_TEMP:
		hwm_set_temp_limit(&dev, &limit->temp);
		break;
	default:
		return -EINVAL;
	}

	return hwm_write_fan_config(unit, &dev);
}

int hwm_core_set_fan_limit_rpm(enum fan_unit unit, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.rpm = {
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(unit, LIMIT_RPM, &limit);
}

int hwm_core_set_fan_limit_pwm(enum fan_unit unit, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.pwm = {
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(unit, LIMIT_PWM, &limit);
}

int hwm_core_set_fan_limit_temp(enum fan_unit unit, int stop, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.temp = {
			.stop = stop,
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(unit, LIMIT_TEMP, &limit);
}

int hwm_core_init(void)
{
	int ret;

	memset(&sensors, 0, sizeof(sensors));

	ret = imanager_get_fw_info(&sensors.info);
	if (ret)
		return ret;

	ret = imanager_get_adc_cfg(sensors.adc);
	if (ret)
		return ret;

	ret = imanager_get_fan_cfg(sensors.fan);
	if (ret)
		return ret;

	return 0;
}

void hwm_core_release(void)
{
	memset(&sensors, 0, sizeof(sensors));
}
