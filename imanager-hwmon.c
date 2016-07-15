/*
 * Advantech iManager Hardware Monitoring driver
 * Partially derived from nct6775
 *
 * Copyright (C) 2016 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/device.h>
#include <linux/err.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/init.h>
#include <linux/jiffies.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#define __NEED_HWMON_COMPAT__
#include "compat.h"
#include "imanager.h"

/* Voltage computation (10-bit ADC, 0..3V input) */
#define SCALE_IN			2933 /* (3000mV / (2^10 - 1)) * 1000 */

#define HWM_STATUS_UNDEFINED_ITEM	2UL
#define HWM_STATUS_UNDEFINED_DID	3UL
#define HWM_STATUS_UNDEFINED_HWPIN	4UL

/* iManager EC FW pwm[1-*]_mode values are switched */
enum imanager_pwm_mode { CTRL_PWM, CTRL_RPM };
/* EC FW defines pwm_enable mode 'full speed' besides mode 'off' */
enum imanager_pwm_enable { MODE_OFF, MODE_FULL, MODE_MANUAL, MODE_AUTO };

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

/**
 * FAN Control bit field
 * enable:   0:Disabled, 1:Enabled
 * type:     0:PWM,      1:RPM
 * pulse:    0:Undefined 1:2 Pulses  2:4 Pulses
 * tacho:    1:CPU FAN,  2:SYS FAN1, 3:SYS FAN2
 * mode:     0:Off,      1:Full,     2:Manual, 3:Auto
 */
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

struct imanager_hwmon_smartfan {
	bool valid;	/* if set, below values are valid */

	int pwm;
	int speed;
	bool speed_min_alarm;
	bool speed_max_alarm;
	struct fan_dev_config	cfg;
};

struct imanager_hwmon_data {
	struct imanager_device_data	*imgr;
	const struct attribute_group	*groups[3];
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	struct device			*hwmon_device;
#endif
	bool valid; /* if set, below values are valid */
	unsigned long			samples;
	unsigned long			last_updated;
	struct imanager_hwmon_adc	adc[EC_MAX_ADC_NUM];
	struct imanager_hwmon_smartfan	fan[EC_MAX_FAN_NUM];
};

static int imanager_hwmon_read_fan_config(struct imanager_io_ops *io, int fnum,
					  struct imanager_hwmon_smartfan *fan)
{
	struct ec_message msg = {
		.rlen = EC_FLAG_HWMON_MSG,
		.wlen = 0,
		.param = fnum,
		.data = (u8 *)&fan->cfg,
	};
	int ret;

	ret = imanager_read(io, EC_CMD_FAN_CTL_RD, &msg);
	if (ret)
		return ret;

	return fan->cfg.did ? 0 : -ENODEV;
}

static int imanager_hwmon_write_fan_config(struct imanager_io_ops *io, int fnum,
					   struct imanager_hwmon_smartfan *fan)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(fan->cfg),
		.param = fnum,
		.data = (u8 *)&fan->cfg,
	};
	int err;

	if (!fan->cfg.did)
		return -ENODEV;

	err = imanager_write(io, EC_CMD_FAN_CTL_WR, &msg);
	if (err < 0)
		return err;

	switch (err) {
	case 0:
		break;
	case HWM_STATUS_UNDEFINED_ITEM:
	case HWM_STATUS_UNDEFINED_DID:
	case HWM_STATUS_UNDEFINED_HWPIN:
		return -EFAULT;
	default:
		return -EIO;
	}

	return 0;
}

#define TEST_BIT(var, bit) ((var) & (bit))
#define FAN_ALERT_MIN_BIT(var, fnum) TEST_BIT(var, BIT(fnum << 1))
#define FAN_ALERT_MAX_BIT(var, fnum) TEST_BIT(var, BIT((fnum << 1) + 1))

static int imanager_hwmon_read_fan_alert(struct imanager_io_ops *io, int fnum,
					 struct imanager_hwmon_smartfan *fan)
{
	u8 alert_flags;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_OFFSET_FAN_ALERT,
				&alert_flags, sizeof(alert_flags));
	if (ret < 0)
		return ret;

	fan->speed_min_alarm = FAN_ALERT_MIN_BIT(alert_flags, fnum);
	fan->speed_max_alarm = FAN_ALERT_MAX_BIT(alert_flags, fnum);

	return 0;
}

static int imanager_hwmon_write_fan_alert(struct imanager_io_ops *io, int fnum,
					  struct imanager_hwmon_smartfan *fan)
{
	struct fan_alert_limit limits[EC_MAX_FAN_NUM];
	struct fan_alert_limit *limit = &limits[fnum];
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_OFFSET_FAN_ALERT_LIMIT,
				(u8 *)limits, sizeof(limits));
	if (ret < 0)
		return ret;

	limit->min = fan->cfg.rpm_min;
	limit->max = fan->cfg.rpm_max;

	return imanager_write_ram(io, EC_RAM_ACPI, EC_OFFSET_FAN_ALERT_LIMIT,
				  (u8 *)limits, sizeof(limits));
}

static int imanager_hwmon_read_adc(struct imanager_ec_data *ec, int fnum,
				   struct imanager_hwmon_adc *adc)
{
	struct ec_dev_attr *adc_attr = &ec->hwmon.adc.attr[fnum];
	int ret;

	adc->valid = false;

	if (!adc_attr->did)
		return -ENODEV;

	ret = imanager_read16(&ec->io, EC_CMD_HWP_RD, adc_attr->did);
	if (ret < 0)
		return ret;

	adc->value = ret * adc_attr->scale;
	adc->valid = true;

	return 0;
}

static int imanager_hwmon_read_fan_ctrl(struct imanager_ec_data *ec, int fnum,
					struct imanager_hwmon_smartfan *fan)
{
	struct imanager_io_ops *io = &ec->io;
	struct imanager_hwmon_device *hwmon = &ec->hwmon;
	int fan_did = hwmon->fan.attr[fnum].did;
	int ret;

	fan->valid = false;

	ret = imanager_hwmon_read_fan_config(io, fnum, fan);
	if (ret < 0)
		return ret;

	ret = imanager_read16(io, EC_CMD_HWP_RD, fan->cfg.tachoid);
	if (ret < 0)
		return ret;

	fan->speed = ret;

	ret = imanager_read8(io, EC_CMD_HWP_RD, fan_did);
	if (ret < 0)
		return ret;

	fan->pwm = ret;

	ret = imanager_hwmon_read_fan_alert(io, fnum, fan);
	if (ret)
		return ret;

	fan->valid = true;

	return 0;
}

static inline uint in_from_reg(u16 val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * SCALE_IN, 1000), 0, 65535);
}

static inline u16 in_to_reg(uint val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * 1000, SCALE_IN), 0, 65535);
}

static struct imanager_hwmon_data *
imanager_hwmon_update_device(struct device *dev)
{
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_hwmon_device *hwmon = &ec->hwmon;
	int i;

	mutex_lock(&imgr->lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		/* Measured voltages */
		for (i = 0; i < hwmon->adc.num; i++)
			imanager_hwmon_read_adc(ec, i, &data->adc[i]);

		/* Measured fan speeds */
		for (i = 0; i < hwmon->fan.num; i++)
			imanager_hwmon_read_fan_ctrl(ec, i, &data->fan[i]);

		data->last_updated = jiffies;
		data->valid = true;
	}

	mutex_unlock(&imgr->lock);

	return data;
}

static ssize_t
show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%u\n", in_from_reg(data->adc[nr].value));
}

static ssize_t
show_in_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%u\n", in_from_reg(data->adc[nr].min));
}

static ssize_t
show_in_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%u\n", in_from_reg(data->adc[nr].max));
}

static ssize_t
store_in_min(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->imgr->lock);
	data->adc[nr].min = in_to_reg(val);
	mutex_unlock(&data->imgr->lock);

	return count;
}

static ssize_t
store_in_max(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->imgr->lock);
	data->adc[nr].max = in_to_reg(val);
	mutex_unlock(&data->imgr->lock);

	return count;
}

static ssize_t
show_in_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->adc[nr];
	int val = 0;

	if (adc->valid)
		val = (adc->value < adc->min) || (adc->value > adc->max);

	return sprintf(buf, "%u\n", val);
}

static ssize_t
show_in_average(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->adc[nr];

	if (adc->average)
		adc->average =
			DIV_ROUND_CLOSEST(adc->average * data->samples +
					  adc->value, ++data->samples);
	else {
		adc->average = adc->value;
		data->samples = 1;
	}

	return sprintf(buf, "%u\n", in_from_reg(adc->average));
}

static ssize_t
show_in_lowest(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->adc[nr];

	if (!adc->lowest)
		adc->lowest = adc->highest = adc->value;
	else if (adc->value < adc->lowest)
		adc->lowest = adc->value;

	return sprintf(buf, "%u\n", in_from_reg(adc->lowest));
}

static ssize_t
show_in_highest(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->adc[nr];

	if (!adc->highest)
		adc->highest = adc->value;
	else if (adc->value > adc->highest)
		adc->highest = adc->value;

	return sprintf(buf, "%u\n", in_from_reg(adc->highest));
}

static ssize_t
store_in_reset_history(struct device *dev, struct device_attribute *attr,
		       const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_hwmon_adc *adc = &data->adc[nr];
	unsigned long reset;
	int err;

	err = kstrtoul(buf, 10, &reset);
	if (err < 0)
		return err;

	mutex_lock(&imgr->lock);

	if (reset) {
		adc->lowest = 0;
		adc->highest = 0;
	}

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->fan[nr].cfg.temp * 1000);
}

static ssize_t
show_fan_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];

	return sprintf(buf, "%u\n", fan->valid ? fan->speed : 0);
}

static inline int check_fan_alarm(const struct imanager_hwmon_smartfan *fan)
{
	int rpm_min = cpu_to_be16(fan->cfg.rpm_min);
	int rpm_max = cpu_to_be16(fan->cfg.rpm_max);

	return !fan->valid || fan->speed_min_alarm || fan->speed_max_alarm ||
	       (!fan->speed && fan->pwm) || (fan->speed < rpm_min) ||
	       (fan->speed > rpm_max);
}

static ssize_t
show_fan_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];

	return sprintf(buf, "%u\n", check_fan_alarm(fan));
}

static ssize_t
show_fan_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int rpm_min = cpu_to_be16(data->fan[nr].cfg.rpm_min);

	return sprintf(buf, "%u\n", rpm_min);
}

static ssize_t
show_fan_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int rpm_max = cpu_to_be16(data->fan[nr].cfg.rpm_max);

	return sprintf(buf, "%u\n", rpm_max);
}

static ssize_t
store_fan_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* do not apply value if not in cruise mode */
	if (ctrl->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	fan->cfg.rpm_min = cpu_to_be16(val);
	imanager_hwmon_write_fan_config(io, nr, fan);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
store_fan_max(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* do not apply value if not in cruise mode */
	if (ctrl->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	fan->cfg.rpm_max = cpu_to_be16(val);
	imanager_hwmon_write_fan_config(io, nr, fan);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	u32 val = DIV_ROUND_CLOSEST(data->fan[nr].pwm * 255, 100);

	return sprintf(buf, "%u\n", val);
}

static ssize_t
store_pwm(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	int did = ec->hwmon.fan.attr[nr].did;
	unsigned long pwm = 0;
	int err;

	err = kstrtoul(buf, 10, &pwm);
	if (err < 0)
		return err;

	pwm = DIV_ROUND_CLOSEST(pwm * 100, 255);

	mutex_lock(&imgr->lock);

	if (ctrl->mode == MODE_MANUAL) {
		ctrl->type = CTRL_PWM;
		ctrl->pulse = 0;
		ctrl->enable = 1;
		imanager_hwmon_write_fan_config(&ec->io, nr, fan);
		imanager_write8(&ec->io, EC_CMD_HWP_WR, did, pwm);
	} else if ((ctrl->mode == MODE_AUTO) && (ctrl->type == CTRL_PWM)) {
		ctrl->pulse = 0;
		ctrl->enable = 1;
		fan->cfg.rpm_min = 0;
		fan->cfg.rpm_max = 0;
		imanager_hwmon_write_fan_config(&ec->io, nr, fan);
		imanager_write8(&ec->io, EC_CMD_HWP_WR, did, pwm);
		imanager_hwmon_write_fan_alert(&ec->io, nr, fan);
	}

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_pwm_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->fan[nr].cfg.pwm_min);
}

static ssize_t
show_pwm_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->fan[nr].cfg.pwm_max);
}

static ssize_t
show_pwm_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	uint mode = ctrl->mode - 1;

	if (ctrl->mode == MODE_OFF)
		mode = 0;

	return sprintf(buf, "%u\n", mode);
}

enum pwm_enable { OFF, MANUAL, THERMAL_CRUISE };

static ssize_t
store_pwm_enable(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &imgr->ec;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	int did = ec->hwmon.fan.attr[nr].did;
	unsigned long mode = 0;
	int err;

	err = kstrtoul(buf, 10, &mode);
	if (err < 0)
		return err;

	mutex_lock(&imgr->lock);

	switch (mode) {
	case OFF:
		ctrl->mode = MODE_FULL;
		ctrl->type = CTRL_PWM;
		ctrl->enable = 1;
		imanager_hwmon_write_fan_config(&ec->io, nr, fan);
		imanager_write8(&ec->io, EC_CMD_HWP_WR, did, 100);
		break;
	case MANUAL:
		ctrl->mode = MODE_MANUAL;
		ctrl->type = CTRL_PWM;
		ctrl->enable = 1;
		imanager_hwmon_write_fan_config(&ec->io, nr, fan);
		imanager_write8(&ec->io, EC_CMD_HWP_WR, did, 0);
		break;
	case THERMAL_CRUISE:
		ctrl->mode = MODE_AUTO;
		ctrl->enable = 1;
		imanager_hwmon_write_fan_config(&ec->io, nr, fan);
		imanager_write8(&ec->io, EC_CMD_HWP_WR, did, 0);
		break;
	}

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_pwm_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	uint type = (ctrl->type == CTRL_PWM) ? 1 : 0;

	if (ctrl->mode == MODE_OFF)
		type = 0;

	return sprintf(buf, "%u\n", type);
}

static ssize_t
store_pwm_mode(struct device *dev, struct device_attribute *attr,
	       const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	if (ctrl->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	ctrl->type = val ? CTRL_RPM : CTRL_PWM;
	ctrl->enable = 1;
	imanager_hwmon_write_fan_config(&imgr->ec.io, nr, fan);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_temp_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%d\n", data->fan[nr].cfg.temp_min * 1000);
}

static ssize_t
show_temp_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->fan[nr].cfg.temp_max * 1000);
}

static ssize_t
show_temp_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct fan_dev_config *cfg = &data->fan[nr].cfg;

	return sprintf(buf, "%u\n", ((cfg->temp < cfg->temp_min) ||
		       (cfg->temp > cfg->temp_max)));
}

static ssize_t
store_temp_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* temperature in 1/10 degC */
	val = DIV_ROUND_CLOSEST(val, 1000);
	val = val > 100 ? 100 : val;

	/* do not apply value if not in cruise mode */
	if (ctrl->mode != MODE_AUTO)
		return count;

	/*
	 * The iManager provides three different temperature limit values
	 * (stop, min, and max) where stop indicates a minimum temp value
	 * (threshold) from which the FAN will turn off.  We are setting
	 * temp_stop to the same value as temp_min since it cannot be mapped
	 * to anything else.
	 */

	mutex_lock(&imgr->lock);
	fan->cfg.temp_stop = val;
	fan->cfg.temp_min = val;
	imanager_hwmon_write_fan_config(io, nr, fan);
	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
store_temp_max(struct device *dev, struct device_attribute *attr,
	       const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&fan->cfg.control;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val, 1000);
	val = val > 100 ? 100 : val;

	/* do not apply value if not in cruise mode */
	if (ctrl->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);
	fan->cfg.temp_max = val;
	imanager_hwmon_write_fan_config(io, nr, fan);
	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
store_pwm_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&imgr->lock);
	fan->cfg.pwm_min = val;
	imanager_hwmon_write_fan_config(io, nr, fan);
	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
store_pwm_max(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->fan[nr];
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&imgr->lock);
	fan->cfg.pwm_max = val;
	imanager_hwmon_write_fan_config(io, nr, fan);
	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_in_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", hwmon->adc.attr[nr].label);
}

static ssize_t
show_temp_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%s\n", hwmon->fan.temp_label[nr]);
}

static ssize_t
show_fan_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%s\n", hwmon->fan.attr[nr].label);
}

/*
 * Sysfs callback functions
 */
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "imanager_hwmon\n");
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
#endif

static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO, show_in_label, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_min, S_IWUSR | S_IRUGO, show_in_min,
			  store_in_min, 0);
static SENSOR_DEVICE_ATTR(in0_max, S_IWUSR | S_IRUGO, show_in_max,
			  store_in_max, 0);
static SENSOR_DEVICE_ATTR(in0_alarm, S_IRUGO, show_in_alarm, NULL, 0);

static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_in_label, NULL, 1);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in1_min, S_IWUSR | S_IRUGO, show_in_min,
			  store_in_min, 1);
static SENSOR_DEVICE_ATTR(in1_max, S_IWUSR | S_IRUGO, show_in_max,
			  store_in_max, 1);
static SENSOR_DEVICE_ATTR(in1_alarm, S_IRUGO, show_in_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_in_label, NULL, 2);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in2_min, S_IWUSR | S_IRUGO, show_in_min,
			  store_in_min, 2);
static SENSOR_DEVICE_ATTR(in2_max, S_IWUSR | S_IRUGO, show_in_max,
			  store_in_max, 2);
static SENSOR_DEVICE_ATTR(in2_alarm, S_IRUGO, show_in_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_temp_label, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_min, S_IRUGO | S_IWUSR, show_temp_min,
			  store_temp_min, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO | S_IWUSR, show_temp_max,
			  store_temp_max, 1);
static SENSOR_DEVICE_ATTR(temp1_alarm, S_IRUGO, show_temp_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, show_temp_label, NULL, 2);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp2_min, S_IRUGO | S_IWUSR, show_temp_min,
			  store_temp_min, 2);
static SENSOR_DEVICE_ATTR(temp2_max, S_IRUGO | S_IWUSR, show_temp_max,
			  store_temp_max, 2);
static SENSOR_DEVICE_ATTR(temp2_alarm, S_IRUGO, show_temp_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO, show_temp_label, NULL, 3);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp, NULL, 3);
static SENSOR_DEVICE_ATTR(temp3_min, S_IRUGO | S_IWUSR, show_temp_min,
			  store_temp_min, 3);
static SENSOR_DEVICE_ATTR(temp3_max, S_IRUGO | S_IWUSR, show_temp_max,
			  store_temp_max, 3);
static SENSOR_DEVICE_ATTR(temp3_alarm, S_IRUGO, show_temp_alarm, NULL, 3);

static SENSOR_DEVICE_ATTR(fan1_label, S_IRUGO, show_fan_label, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan_in, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_min, S_IWUSR | S_IRUGO, show_fan_min,
			  store_fan_min, 1);
static SENSOR_DEVICE_ATTR(fan1_max, S_IWUSR | S_IRUGO, show_fan_max,
			  store_fan_max, 1);
static SENSOR_DEVICE_ATTR(fan1_alarm, S_IRUGO, show_fan_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(fan2_label, S_IRUGO, show_fan_label, NULL, 2);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan_in, NULL, 2);
static SENSOR_DEVICE_ATTR(fan2_min, S_IWUSR | S_IRUGO, show_fan_min,
			  store_fan_min, 2);
static SENSOR_DEVICE_ATTR(fan2_max, S_IWUSR | S_IRUGO, show_fan_max,
			  store_fan_max, 2);
static SENSOR_DEVICE_ATTR(fan2_alarm, S_IRUGO, show_fan_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(fan3_label, S_IRUGO, show_fan_label, NULL, 3);
static SENSOR_DEVICE_ATTR(fan3_input, S_IRUGO, show_fan_in, NULL, 3);
static SENSOR_DEVICE_ATTR(fan3_min, S_IWUSR | S_IRUGO, show_fan_min,
			  store_fan_min, 3);
static SENSOR_DEVICE_ATTR(fan3_max, S_IWUSR | S_IRUGO, show_fan_max,
			  store_fan_max, 3);
static SENSOR_DEVICE_ATTR(fan3_alarm, S_IRUGO, show_fan_alarm, NULL, 3);

static SENSOR_DEVICE_ATTR(pwm1, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm1_min, S_IWUSR | S_IRUGO, show_pwm_min,
			  store_pwm_min, 1);
static SENSOR_DEVICE_ATTR(pwm1_max, S_IWUSR | S_IRUGO, show_pwm_max,
			  store_pwm_max, 1);
static SENSOR_DEVICE_ATTR(pwm1_enable, S_IWUSR | S_IRUGO, show_pwm_enable,
			  store_pwm_enable, 1);
static SENSOR_DEVICE_ATTR(pwm1_mode, S_IWUSR | S_IRUGO, show_pwm_mode,
			  store_pwm_mode, 1);

static SENSOR_DEVICE_ATTR(pwm2, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm2_min, S_IWUSR | S_IRUGO, show_pwm_min,
			  store_pwm_min, 2);
static SENSOR_DEVICE_ATTR(pwm2_max, S_IWUSR | S_IRUGO, show_pwm_max,
			  store_pwm_max, 2);
static SENSOR_DEVICE_ATTR(pwm2_enable, S_IWUSR | S_IRUGO, show_pwm_enable,
			  store_pwm_enable, 2);
static SENSOR_DEVICE_ATTR(pwm2_mode, S_IWUSR | S_IRUGO, show_pwm_mode,
			  store_pwm_mode, 2);

static SENSOR_DEVICE_ATTR(pwm3, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm3_min, S_IWUSR | S_IRUGO, show_pwm_min,
			  store_pwm_min, 3);
static SENSOR_DEVICE_ATTR(pwm3_max, S_IWUSR | S_IRUGO, show_pwm_max,
			  store_pwm_max, 3);
static SENSOR_DEVICE_ATTR(pwm3_enable, S_IWUSR | S_IRUGO, show_pwm_enable,
			  store_pwm_enable, 3);
static SENSOR_DEVICE_ATTR(pwm3_mode, S_IWUSR | S_IRUGO, show_pwm_mode,
			  store_pwm_mode, 3);

static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_min, S_IWUSR | S_IRUGO, show_in_min,
			  store_in_min, 4);
static SENSOR_DEVICE_ATTR(curr1_max, S_IWUSR | S_IRUGO, show_in_max,
			  store_in_max, 4);
static SENSOR_DEVICE_ATTR(curr1_alarm, S_IRUGO, show_in_alarm, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_average, S_IRUGO, show_in_average, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_lowest, S_IRUGO, show_in_lowest, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_highest, S_IRUGO, show_in_highest, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_reset_history, S_IWUSR, NULL,
			  store_in_reset_history, 4);

static SENSOR_DEVICE_ATTR(cpu0_vid, S_IRUGO, show_in, NULL, 3);

static struct attribute *imanager_in_attributes[] = {
	&sensor_dev_attr_in0_label.dev_attr.attr,
	&sensor_dev_attr_in0_input.dev_attr.attr,
	&sensor_dev_attr_in0_min.dev_attr.attr,
	&sensor_dev_attr_in0_max.dev_attr.attr,
	&sensor_dev_attr_in0_alarm.dev_attr.attr,

	&sensor_dev_attr_in1_label.dev_attr.attr,
	&sensor_dev_attr_in1_input.dev_attr.attr,
	&sensor_dev_attr_in1_min.dev_attr.attr,
	&sensor_dev_attr_in1_max.dev_attr.attr,
	&sensor_dev_attr_in1_alarm.dev_attr.attr,

	&sensor_dev_attr_in2_label.dev_attr.attr,
	&sensor_dev_attr_in2_input.dev_attr.attr,
	&sensor_dev_attr_in2_min.dev_attr.attr,
	&sensor_dev_attr_in2_max.dev_attr.attr,
	&sensor_dev_attr_in2_alarm.dev_attr.attr,

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 13, 0)
	&dev_attr_name.attr,
#endif
	NULL
};

#define to_dev(obj) container_of(obj, struct device, kobj)

static umode_t
imanager_in_is_visible(struct kobject *kobj, struct attribute *attr, int index)
{
	struct device *dev = to_dev(kobj);
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;

	if (hwmon->adc.num <= EC_MAX_ADC_NUM)
		return attr->mode;

	return 0;
}

static const struct attribute_group imanager_group_in = {
	.attrs = imanager_in_attributes,
	.is_visible = imanager_in_is_visible,
};

static struct attribute *imanager_other_attributes[] = {
	&sensor_dev_attr_curr1_input.dev_attr.attr,
	&sensor_dev_attr_curr1_min.dev_attr.attr,
	&sensor_dev_attr_curr1_max.dev_attr.attr,
	&sensor_dev_attr_curr1_alarm.dev_attr.attr,
	&sensor_dev_attr_curr1_average.dev_attr.attr,
	&sensor_dev_attr_curr1_lowest.dev_attr.attr,
	&sensor_dev_attr_curr1_highest.dev_attr.attr,
	&sensor_dev_attr_curr1_reset_history.dev_attr.attr,

	&sensor_dev_attr_cpu0_vid.dev_attr.attr,

	NULL
};

static umode_t
imanager_other_is_visible(struct kobject *kobj,
			  struct attribute *attr, int index)
{
	struct device *dev = to_dev(kobj);
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;

	/*
	 * There are either 3 or 5 VINs available
	 * vin3 is current monitoring
	 * vin4 is CPU VID
	 */
	if (hwmon->adc.num == EC_MAX_ADC_NUM)
		return attr->mode;

	return 0;
}

static const struct attribute_group imanager_group_other = {
	.attrs = imanager_other_attributes,
	.is_visible = imanager_other_is_visible,
};

static struct attribute *imanager_fan_attributes[] = {
	&sensor_dev_attr_fan1_label.dev_attr.attr,
	&sensor_dev_attr_fan1_input.dev_attr.attr,
	&sensor_dev_attr_fan1_min.dev_attr.attr,
	&sensor_dev_attr_fan1_max.dev_attr.attr,
	&sensor_dev_attr_fan1_alarm.dev_attr.attr,

	&sensor_dev_attr_fan2_label.dev_attr.attr,
	&sensor_dev_attr_fan2_input.dev_attr.attr,
	&sensor_dev_attr_fan2_min.dev_attr.attr,
	&sensor_dev_attr_fan2_max.dev_attr.attr,
	&sensor_dev_attr_fan2_alarm.dev_attr.attr,

	&sensor_dev_attr_fan3_label.dev_attr.attr,
	&sensor_dev_attr_fan3_input.dev_attr.attr,
	&sensor_dev_attr_fan3_min.dev_attr.attr,
	&sensor_dev_attr_fan3_max.dev_attr.attr,
	&sensor_dev_attr_fan3_alarm.dev_attr.attr,

	&sensor_dev_attr_temp1_label.dev_attr.attr,
	&sensor_dev_attr_temp1_input.dev_attr.attr,
	&sensor_dev_attr_temp1_min.dev_attr.attr,
	&sensor_dev_attr_temp1_max.dev_attr.attr,
	&sensor_dev_attr_temp1_alarm.dev_attr.attr,

	&sensor_dev_attr_temp2_label.dev_attr.attr,
	&sensor_dev_attr_temp2_input.dev_attr.attr,
	&sensor_dev_attr_temp2_min.dev_attr.attr,
	&sensor_dev_attr_temp2_max.dev_attr.attr,
	&sensor_dev_attr_temp2_alarm.dev_attr.attr,

	&sensor_dev_attr_temp3_label.dev_attr.attr,
	&sensor_dev_attr_temp3_input.dev_attr.attr,
	&sensor_dev_attr_temp3_min.dev_attr.attr,
	&sensor_dev_attr_temp3_max.dev_attr.attr,
	&sensor_dev_attr_temp3_alarm.dev_attr.attr,

	&sensor_dev_attr_pwm1.dev_attr.attr,
	&sensor_dev_attr_pwm1_min.dev_attr.attr,
	&sensor_dev_attr_pwm1_max.dev_attr.attr,
	&sensor_dev_attr_pwm1_enable.dev_attr.attr,
	&sensor_dev_attr_pwm1_mode.dev_attr.attr,

	&sensor_dev_attr_pwm2.dev_attr.attr,
	&sensor_dev_attr_pwm2_min.dev_attr.attr,
	&sensor_dev_attr_pwm2_max.dev_attr.attr,
	&sensor_dev_attr_pwm2_enable.dev_attr.attr,
	&sensor_dev_attr_pwm2_mode.dev_attr.attr,

	&sensor_dev_attr_pwm3.dev_attr.attr,
	&sensor_dev_attr_pwm3_min.dev_attr.attr,
	&sensor_dev_attr_pwm3_max.dev_attr.attr,
	&sensor_dev_attr_pwm3_enable.dev_attr.attr,
	&sensor_dev_attr_pwm3_mode.dev_attr.attr,

	NULL
};

static umode_t
imanager_fan_is_visible(struct kobject *kobj, struct attribute *attr, int index)
{
	struct device *dev = to_dev(kobj);
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.hwmon;

	if ((index >= 0) && (index <= 14)) { /* fan */
		if (!hwmon->fan.attr[index / 5].did)
			return 0;
	} else if ((index >= 15) && (index <= 29)) { /* temp */
		if (!hwmon->fan.attr[(index - 15) / 5].did)
			return 0;
	} else if ((index >= 30) && (index <= 34)) { /* pwm */
		if (!hwmon->fan.attr[(index - 30) / 5].did)
			return 0;
	}

	return attr->mode;
}

static const struct attribute_group imanager_group_fan = {
	.attrs = imanager_fan_attributes,
	.is_visible = imanager_fan_is_visible,
};

static int imanager_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_hwmon_device *hwmon = &ec->hwmon;
	struct imanager_hwmon_data *data;
	struct device *hwmon_dev;
	int i, num_attr_groups = 0;
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	int err;
#endif

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->imgr = imgr;
	platform_set_drvdata(pdev, data);

	/* read fan control settings */
	for (i = 0; i < hwmon->fan.num; i++)
		imanager_hwmon_read_fan_ctrl(ec, i, &data->fan[i]);

	data->groups[num_attr_groups++] = &imanager_group_in;

	if (hwmon->adc.num > 3)
		data->groups[num_attr_groups++] = &imanager_group_other;

	if (hwmon->fan.num)
		data->groups[num_attr_groups++] = &imanager_group_fan;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	err = sysfs_create_groups(&dev->kobj, data->groups);
	if (err < 0)
		return err;

	hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(hwmon_dev)) {
		sysfs_remove_groups(&dev->kobj, data->groups);
		return PTR_ERR(hwmon_dev);
	}
	data->hwmon_device = hwmon_dev;
#else
	hwmon_dev = devm_hwmon_device_register_with_groups(dev,
							   "imanager_hwmon",
							   data, data->groups);
#endif

	return PTR_ERR_OR_ZERO(hwmon_dev);
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static int imanager_hwmon_remove(struct platform_device *pdev)
{
	struct imanager_hwmon_data *data = dev_get_drvdata(&pdev->dev);

	hwmon_device_unregister(data->hwmon_device);
	sysfs_remove_groups(&pdev->dev.kobj, data->groups);

	return 0;
}
#endif

static struct platform_driver imanager_hwmon_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
		.owner = THIS_MODULE,
#endif
		.name  = "imanager-hwmon",
	},
	.probe	= imanager_hwmon_probe,
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	.remove	= imanager_hwmon_remove,
#endif
};

module_platform_driver(imanager_hwmon_driver);

MODULE_DESCRIPTION("Advantech iManager HWmon Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-hwmon");
