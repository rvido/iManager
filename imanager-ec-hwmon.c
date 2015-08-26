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
#include <ec.h>
#include <hwmon.h>

#define HWM_STATUS_UNDEFINED_ITEM	2UL
#define HWM_STATUS_UNDEFINED_DID	3UL
#define HWM_STATUS_UNDEFINED_HWPIN	4UL

/*
 * FAN defs
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
};

struct fan_status {
	u32	sysctl	: 1,	/* System Control flag */
		tacho	: 1,	/* FAN tacho source defined */
		pulse	: 1,	/* FAN pulse type defined */
		thermal	: 1,	/* Thermal zone init */
		i2clink	: 1,	/* I2C protocol fail flag (thermal sensor) */
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
		dnc		: 2; /* don't care */
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

static const char * fan_temp_label[] = {
	"Temp CPU",
	"Temp SYS1",
	"Temp SYS2",
	NULL,
};

static const struct imanager_hwmon_device *dev;

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

static int hwm_read_fan_config(int num, struct fan_dev_config *cfg)
{
	int ret;
	struct ec_message msg = {
		.rlen = 0xff, /* use alternative message body */
		.wlen = 0,
	};
	struct fan_dev_config *_cfg = (struct fan_dev_config *)&msg.u.data;

	if (WARN_ON(!cfg))
		return -EINVAL;

	ret = imanager_msg_read(EC_CMD_FAN_CTL_RD, num, &msg);
	if (ret)
		return ret;

	if (!_cfg->did)
		return -ENODEV;

	memcpy(cfg, &msg.u.data, sizeof(struct fan_dev_config));

	return 0;
}

static int hwm_write_fan_config(int fnum, struct fan_dev_config *cfg)
{
	int ret;
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(struct fan_dev_config),
	};

	if (WARN_ON(!dev))
		return -EINVAL;

	if (!cfg->did)
		return -ENODEV;

	msg.data = (u8 *)cfg;

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

static inline void hwm_set_temp_limit(struct fan_dev_config *cfg,
				      const struct hwm_fan_temp_limit *temp)
{
	cfg->temp_stop = temp->stop;
	cfg->temp_min  = temp->min;
	cfg->temp_max  = temp->max;
}

static inline void hwm_set_pwm_limit(struct fan_dev_config *cfg,
				     const struct hwm_fan_limit *pwm)
{
	cfg->pwm_min = pwm->min;
	cfg->pwm_max = pwm->max;
}

static inline void hwm_set_rpm_limit(struct fan_dev_config *cfg,
				     const struct hwm_fan_limit *rpm)
{
	cfg->rpm_min = swab16(rpm->min);
	cfg->rpm_max = swab16(rpm->max);
}

static inline void hwm_set_limit(struct fan_dev_config *cfg,
				 const struct hwm_sensors_limit *limit)
{
	hwm_set_temp_limit(cfg, &limit->temp);
	hwm_set_pwm_limit(cfg, &limit->pwm);
	hwm_set_rpm_limit(cfg, &limit->rpm);
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

static int hwm_core_get_fan_alert_limit(int fnum,
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

	switch (fnum) {
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
		pr_err("Unknown FAN ID %d\n", fnum);
		return -EINVAL;
	}

	return 0;
}

static int hwm_core_set_fan_alert_limit(int fnum,
					struct hwm_fan_alert *alert)
{
	int ret;
	struct fan_alert_limit limit;

	ret = imanager_acpiram_read_block(EC_ACPIRAM_FAN_SPEED_LIMIT,
					 (u8 *)&limit, sizeof(limit));
	if (ret < 0)
		return ret;

	switch (fnum) {
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
		pr_err("Unknown FAN ID %d\n", fnum);
		return -EINVAL;
	}

	return imanager_acpiram_write_block(EC_ACPIRAM_FAN_SPEED_LIMIT,
					   (u8 *)&limit, sizeof(limit));
}

/* HWM CORE API */

const char * hwm_core_adc_get_label(int num)
{
	if (WARN_ON(!dev && num >= dev->adc.num))
		return NULL;

	return dev->adc.attr[num].label;
}

const char * hwm_core_fan_get_label(int num)
{
	if (WARN_ON(!dev && num >= dev->fan.num))
		return NULL;

	return dev->fan.attr[num].label;
}

const char * hwm_core_fan_get_temp_label(int num)
{
	if (WARN_ON(!dev && num >= dev->fan.num))
		return NULL;

	return fan_temp_label[num];
}

int hwm_core_adc_is_available(int num)
{
	if (WARN_ON(!dev && num >= dev->adc.num))
		return -ENODEV;

	return dev->adc.attr[num].did ? 0 : -ENODEV;
}

int hwm_core_adc_get_value(int num, struct hwm_voltage *volt)
{
	int ret;

	volt->valid = false;

	ret = hwm_core_adc_is_available(num);
	if (ret < 0)
		return ret;

	ret = hwm_get_adc_value(dev->adc.attr[num].did);
	if (ret < 0)
		return ret;

	volt->value = ret * dev->adc.attr[num].scale;
	volt->valid = true;

	return 0;
}

int hwm_core_fan_get_ctrl(int num, struct hwm_smartfan *fan)
{
	int ret;
	struct fan_dev_config cfg;
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&cfg.control;

	if (WARN_ON((num >= HWM_MAX_FAN) || !fan))
		return -EINVAL;

	fan->valid = false;

	memset(fan, 0, sizeof(struct hwm_smartfan));

	ret = hwm_read_fan_config(num, &cfg);
	if (ret < 0)
		return ret;

	fan->pulse = ctrl->pulse;
	fan->type = ctrl->type;

	/*
	 * It seems that fan->mode does not always report the correct
	 * FAN mode so the only way of reporting the current FAN mode is
	 * to read back ctrl->mode.
	 */
	fan->mode = ctrl->mode;

	ret = hwm_get_rpm_value(cfg.tachoid);
	if (ret < 0) {
		pr_err("Failed to read FAN speed\n");
		return ret;
	}

	fan->speed = ret;

	ret = hwm_get_pwm_value(dev->fan.attr[num].did);
	if (ret < 0) {
		pr_err("Failed to read FAN%d PWM\n", num);
		return ret;
	}

	fan->pwm = ret;

	fan->alarm = (fan->pwm && !fan->speed) ? 1 : 0;

	fan->limit.temp.min	= cfg.temp_min;
	fan->limit.temp.max	= cfg.temp_max;
	fan->limit.temp.stop	= cfg.temp_stop;
	fan->limit.pwm.min	= cfg.pwm_min;
	fan->limit.pwm.max	= cfg.pwm_max;
	fan->limit.rpm.min	= swab16(cfg.rpm_min);
	fan->limit.rpm.max	= swab16(cfg.rpm_max);

	ret = hwm_core_get_fan_alert_limit(num, fan);
	if (ret)
		return ret;

	fan->valid = true;

	return 0;
}

int hwm_core_fan_set_ctrl(int num, int fmode, int ftype, int pwm, int pulse,
			  struct hwm_sensors_limit *limit,
			  struct hwm_fan_alert *alert)
{
	int ret;
	struct fan_dev_config cfg;
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&cfg.control;
	struct hwm_sensors_limit _limit = { {0, 0, 0}, {0, 0}, {0, 0} };

	if (WARN_ON(num >= HWM_MAX_FAN))
		return -EINVAL;

	ret = hwm_read_fan_config(num, &cfg);
	if (ret < 0) {
		pr_err("Failed while reading FAN %s config\n",
			dev->fan.attr[num].label);
		return ret;
	}

	if (!limit)
		limit = &_limit;

	switch (fmode) {
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
		ret = hwm_set_pwm_value(dev->fan.attr[num].did, pwm);
		if (ret < 0)
			return ret;
		break;
	case MODE_AUTO:
		switch (ftype) {
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

	hwm_set_limit(&cfg, limit);

	ctrl->pulse = (pulse && (pulse < 3)) ? pulse : 0;
	ctrl->enable = 1;

	ret = hwm_write_fan_config(num, &cfg);
	if (ret < 0)
		return ret;

	if (alert)
		return hwm_core_set_fan_alert_limit(num, alert);

	return 0;
}

int hwm_core_fan_is_available(int num)
{
	return dev->fan.attr[num].did ? 0 : -ENODEV;
}

static int hwm_core_set_fan_limit(int num, int fan_limit,
				  struct hwm_sensors_limit *limit)
{
	struct fan_dev_config cfg;
	int ret;

	if (WARN_ON(num >= HWM_MAX_FAN))
		return -EINVAL;

	ret = hwm_read_fan_config(num, &cfg);
	if (ret < 0) {
		pr_err("Failed while reading FAN %s config\n",
			dev->fan.attr[num].label);
		return ret;
	}

	switch (fan_limit) {
	case LIMIT_PWM:
		hwm_set_pwm_limit(&cfg, &limit->pwm);
		break;
	case LIMIT_RPM:
		hwm_set_rpm_limit(&cfg, &limit->rpm);
		break;
	case LIMIT_TEMP:
		hwm_set_temp_limit(&cfg, &limit->temp);
		break;
	default:
		return -EINVAL;
	}

	return hwm_write_fan_config(num, &cfg);
}

int hwm_core_fan_set_rpm_limit(int num, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.rpm = {
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(num, LIMIT_RPM, &limit);
}

int hwm_core_fan_set_pwm_limit(int num, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.pwm = {
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(num, LIMIT_PWM, &limit);
}

int hwm_core_fan_set_temp_limit(int num, int stop, int min, int max)
{
	struct hwm_sensors_limit limit = {
		.temp = {
			.stop = stop,
			.min = min,
			.max = max,
		},
	};

	return hwm_core_set_fan_limit(num, LIMIT_TEMP, &limit);
}

int hwm_core_adc_get_max_count(void)
{
	return dev->adc.num;
}

int hwm_core_fan_get_max_count(void)
{
	return dev->fan.num;
}

int hwm_core_init(void)
{
	dev = imanager_get_hwmon_device();
	if (!dev)
		return -ENODEV;

	return 0;
}

