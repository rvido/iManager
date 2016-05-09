/*
 * Advantech iManager Hardware Monitoring driver
 * Derived from nct6775 driver
 *
 * Copyright (C) 2016 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

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
#include <linux/slab.h>
#include "imanager.h"
#include "imanager-hwmon.h"
#define __NEED_HWMON_COMPAT__
#include "compat.h"

/* Voltage computation (10-bit ADC, 0..3V input) */
#define SCALE_IN	2933 /* (3000mV / (2^10 - 1)) * 1000 */

#define HWM_STATUS_UNDEFINED_ITEM	2UL
#define HWM_STATUS_UNDEFINED_DID	3UL
#define HWM_STATUS_UNDEFINED_HWPIN	4UL

struct imanager_hwmon_data {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	struct device *hwmon_device;
#endif
	struct imanager_device_data *imgr;
	bool valid;	/* if set, below values are valid */
	struct imanager_hwmon_devdata hwmon_dev;
	unsigned long samples;
	unsigned long last_updated;
	const struct attribute_group *groups[3];
};

static int imanager_hwmon_read_fan_config(struct imanager_io_ops *io, int fnum,
					  struct fan_dev_config *fdev)
{
	struct ec_message msg = {
		.rlen = EC_MSG_FLAG_HWMON,
		.wlen = 0,
		.param = fnum,
		.data = NULL,
	};
	struct fan_dev_config *cfg = (struct fan_dev_config *)&msg.u.data;
	int ret;

	ret = imanager_read(io, EC_CMD_FAN_CTL_RD, &msg);
	if (ret)
		return ret;

	if (!cfg->did)
		return -ENODEV;

	memcpy(fdev, &msg.u.data, sizeof(*fdev));

	return 0;
}

static int
imanager_hwmon_write_fan_config(struct imanager_io_ops *io, int fnum,
				struct fan_dev_config *fdev)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(*fdev),
		.param = fnum,
		.data = (u8 *)fdev,
	};
	int ret;

	if (!fdev->did)
		return -ENODEV;

	ret = imanager_write(io, EC_CMD_FAN_CTL_WR, &msg);
	if (ret < 0)
		return ret;

	switch (ret) {
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

#define CHECK_BIT(var, pos) (var & BIT(pos))
#define FAN_ALERT_MIN_BIT(var, fnum) CHECK_BIT(var, (fnum << 1))
#define FAN_ALERT_MAX_BIT(var, fnum) CHECK_BIT(var, (fnum << 1) + 1)

static int imanager_hwmon_read_fan_alert(struct imanager_ec_data *ec, int fnum,
					 struct imanager_hwmon_smartfan *fan)
{
	struct imanager_io_ops *io = &ec->io;
	struct fan_alert_limit limits[HWM_MAX_FAN];
	struct fan_alert_limit *limit = &limits[fnum];
	u8 alert_flags;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FAN_SPEED_LIMIT,
				(u8 *)limits, sizeof(limits));
	if (ret < 0)
		return ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FAN_ALERT,
				&alert_flags, sizeof(alert_flags));
	if (ret < 0)
		return ret;

	fan->alert.min = swab16(limit->min);
	fan->alert.max = swab16(limit->max);
	fan->alert.min_alarm = FAN_ALERT_MIN_BIT(alert_flags, fnum);
	fan->alert.max_alarm = FAN_ALERT_MAX_BIT(alert_flags, fnum);

	return 0;
}

static int imanager_hwmon_write_fan_alert(struct imanager_io_ops *io, int fnum,
					  struct hwm_fan_alert *alert)
{
	struct fan_alert_limit limits[HWM_MAX_FAN];
	struct fan_alert_limit *limit = &limits[fnum];
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FAN_SPEED_LIMIT,
				(u8 *)limits, sizeof(limits));
	if (ret < 0)
		return ret;

	limit->min = swab16(alert->min);
	limit->max = swab16(alert->max);

	return imanager_write_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FAN_SPEED_LIMIT,
				  (u8 *)limits, sizeof(limits));
}

static int imanager_hwmon_read_adc(struct imanager_ec_data *ec, int fnum,
				   struct imanager_hwmon_adc *adc)
{
	struct imanager_hwmon_device *hwmon = &ec->idev.hwmon;
	struct imanager_io_ops *io = &ec->io;
	struct ec_dev_attr *adc_attr = &hwmon->adc.attr[fnum];
	int ret;

	adc->valid = false;

	if (!adc_attr->did)
		return -ENODEV;

	ret = imanager_read16(io, EC_CMD_HWP_RD, adc_attr->did);
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
	struct imanager_hwmon_device *hwmon = &ec->idev.hwmon;
	struct fan_dev_config cfg = {
		.control = 0,
	};
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&cfg.control;
	int fan_did = hwmon->fan.attr[fnum].did;
	int ret;

	ret = imanager_hwmon_read_fan_config(io, fnum, &cfg);
	if (ret < 0)
		return ret;

	fan->pulse = ctrl->pulse;
	fan->type = ctrl->type;

	fan->temp = cfg.temp; /* Read thermal zone temp */

	/*
	 * It seems that fan->mode does not always report the correct
	 * FAN mode so the only way of reporting the current FAN mode
	 * is to read ctrl->mode.
	 */
	fan->mode = ctrl->mode;

	ret = imanager_read16(io, EC_CMD_HWP_RD, cfg.tachoid);
	if (ret < 0)
		return ret;

	fan->speed = ret;

	ret = imanager_read8(io, EC_CMD_HWP_RD, fan_did);
	if (ret < 0)
		return ret;

	fan->pwm = ret;

	fan->alarm = (fan->pwm && !fan->speed) ? 1 : 0;

	fan->limit.temp.min	= cfg.temp_min;
	fan->limit.temp.max	= cfg.temp_max;
	fan->limit.temp.stop	= cfg.temp_stop;
	fan->limit.pwm.min	= cfg.pwm_min;
	fan->limit.pwm.max	= cfg.pwm_max;
	fan->limit.rpm.min	= swab16(cfg.rpm_min);
	fan->limit.rpm.max	= swab16(cfg.rpm_max);

	ret = imanager_hwmon_read_fan_alert(ec, fnum, fan);
	if (ret)
		return ret;

	fan->valid = true;

	return 0;
}

static int
imanager_hwmon_write_fan_ctrl(struct imanager_ec_data *ec, int fnum, int fmode,
			      int ftype, int pwm, int pulse,
			      struct hwm_sensors_limit *limit,
			      struct hwm_fan_alert *alert)
{
	struct imanager_io_ops *io = &ec->io;
	struct imanager_hwmon_device *hwmon = &ec->idev.hwmon;
	struct fan_dev_config cfg;
	struct fan_ctrl *ctrl = (struct fan_ctrl *)&cfg.control;
	struct hwm_sensors_limit _limit = { {0, 0, 0}, {0, 0}, {0, 0} };
	int fan_did = hwmon->fan.attr[fnum].did;
	int ret;

	ret = imanager_hwmon_read_fan_config(io, fnum, &cfg);
	if (ret < 0)
		return ret;

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
		ret = imanager_write8(io, EC_CMD_HWP_WR, fan_did, pwm);
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

	cfg.rpm_min	= swab16(limit->rpm.min);
	cfg.rpm_max	= swab16(limit->rpm.max);
	cfg.pwm_min	= limit->pwm.min;
	cfg.pwm_max	= limit->pwm.max;
	cfg.temp_min	= limit->temp.min;
	cfg.temp_max	= limit->temp.max;
	cfg.temp_stop	= limit->temp.stop;

	ctrl->pulse = (pulse && (pulse < 3)) ? pulse : 0;
	ctrl->enable = 1;

	ret = imanager_hwmon_write_fan_config(io, fnum, &cfg);
	if (ret < 0)
		return ret;

	if (alert)
		return imanager_hwmon_write_fan_alert(io, fnum, alert);

	return 0;
}

static inline unsigned in_from_reg(u16 val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * SCALE_IN, 1000), 0, 65535);
}

static inline u16 in_to_reg(unsigned val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * 1000, SCALE_IN), 0, 65535);
}

static struct imanager_hwmon_data *
imanager_hwmon_update_device(struct device *dev)
{
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &data->imgr->ec;
	struct imanager_hwmon_device *hwmon = &ec->idev.hwmon;
	int i;

	mutex_lock(&imgr->lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		/* Measured voltages */
		for (i = 0; i < hwmon->adc.num; i++)
			imanager_hwmon_read_adc(ec, i, &data->hwmon_dev.adc[i]);

		/* Measured fan speeds */
		for (i = 0; i < hwmon->fan.num; i++)
			imanager_hwmon_read_fan_ctrl(ec, i, &data->hwmon_dev.fan[i]);

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
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

	return sprintf(buf, "%u\n", in_from_reg(adc->value));
}

static ssize_t
show_in_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

	return sprintf(buf, "%u\n", in_from_reg(adc->min));
}

static ssize_t
show_in_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

	return sprintf(buf, "%u\n", in_from_reg(adc->max));
}

static ssize_t
store_in_min(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->imgr->lock);

	adc->min = in_to_reg(val);

	mutex_unlock(&data->imgr->lock);

	return count;
}

static ssize_t
store_in_max(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];
	unsigned long val;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->imgr->lock);

	adc->max = in_to_reg(val);

	mutex_unlock(&data->imgr->lock);

	return count;
}

static ssize_t
show_in_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index;
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];
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
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

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
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

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
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];

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
	struct imanager_hwmon_adc *adc = &data->hwmon_dev.adc[nr];
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
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];

	return sprintf(buf, "%u\n", fan->temp * 1000);
}

static ssize_t
show_fan_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];

	return sprintf(buf, "%u\n", fan->valid ? fan->speed : 0);
}

static inline int is_alarm(const struct imanager_hwmon_smartfan *fan)
{
	/*
	 * Do not set ALARM flag if FAN is in speed cruise mode (3)
	 * as this mode automatically turns on the FAN
	 * Set ALARM flag when pwm is set but speed is 0 as this
	 * could be a defective FAN or no FAN is present
	 */
	return (!fan->valid ||
		((fan->mode == MODE_AUTO) && fan->alarm) ||
		(fan->speed > fan->limit.rpm.max));
}

static ssize_t
show_fan_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];

	return sprintf(buf, "%u\n", fan->valid ? is_alarm(fan) : 0);
}

static ssize_t
show_fan_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_fan_limit *rpm = &data->hwmon_dev.fan[nr].limit.rpm;

	return sprintf(buf, "%u\n", rpm->min);
}

static ssize_t
show_fan_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_fan_limit *rpm = &data->hwmon_dev.fan[nr].limit.rpm;

	return sprintf(buf, "%u\n", rpm->max);
}

static ssize_t
store_fan_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_io_ops *io = &imgr->ec.io;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	struct fan_dev_config cfg;
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.rpm_min = swab16(val);
	imanager_hwmon_write_fan_config(io, nr, &cfg);
/* ToDo: store new settings rather than reading it back from EC */
	imanager_hwmon_read_fan_ctrl(ec, nr, fan); /* update */

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
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	struct fan_dev_config cfg;
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.rpm_max = swab16(val);
	imanager_hwmon_write_fan_config(io, nr, &cfg);
	imanager_hwmon_read_fan_ctrl(&imgr->ec, nr, fan);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	u32 val = DIV_ROUND_CLOSEST(data->hwmon_dev.fan[nr].pwm * 255, 100);

	return sprintf(buf, "%u\n", val);
}

static ssize_t
store_pwm(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&imgr->lock);

	switch (fan->mode) {
	case MODE_MANUAL:
		imanager_hwmon_write_fan_ctrl(&imgr->ec, nr, MODE_MANUAL,
					      CTRL_PWM, val, 0, NULL, NULL);
		break;
	case MODE_AUTO:
		if (fan->type == CTRL_RPM)
			break;
		imanager_hwmon_write_fan_ctrl(&imgr->ec, nr, MODE_AUTO,
					      CTRL_PWM, val, 0, &fan->limit,
					      &fan->alert);
		break;
	}

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_pwm_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->hwmon_dev.fan[nr].limit.pwm.min);
}

static ssize_t
show_pwm_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%u\n", data->hwmon_dev.fan[nr].limit.pwm.max);
}

static ssize_t
show_pwm_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	unsigned mode = fan->mode - 1;

	if (fan->mode == MODE_OFF)
		mode = 0;

	return sprintf(buf, "%u\n", mode);
}

static ssize_t
store_pwm_enable(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_ec_data *ec = &imgr->ec;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	unsigned long mode = 0;
	int err;

	err = kstrtoul(buf, 10, &mode);
	if (err < 0)
		return err;

	mutex_lock(&imgr->lock);

	switch (mode) {
	case 0:
		imanager_hwmon_write_fan_ctrl(ec, nr, MODE_FULL, CTRL_PWM, 100,
					      fan->pulse, NULL, NULL);
		break;
	case 1:
		imanager_hwmon_write_fan_ctrl(ec, nr, MODE_MANUAL, CTRL_PWM, 0,
					      fan->pulse, NULL, NULL);
		break;
	case 2:
		imanager_hwmon_write_fan_ctrl(ec, nr, MODE_AUTO, fan->type, 0,
					      fan->pulse, &fan->limit, NULL);
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
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	unsigned type = (fan->type == CTRL_PWM) ? 1 : 0;

	if (fan->mode == MODE_OFF)
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
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	unsigned long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	if (fan->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	imanager_hwmon_write_fan_ctrl(&imgr->ec, nr, fan->mode,
				      val ? CTRL_RPM : CTRL_PWM, fan->pwm,
				      fan->pulse, &fan->limit, &fan->alert);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_temp_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int val = data->hwmon_dev.fan[nr].limit.temp.min;

	return sprintf(buf, "%d\n", val * 1000);
}

static ssize_t
show_temp_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int val = data->hwmon_dev.fan[nr].limit.temp.max;

	return sprintf(buf, "%u\n", val * 1000);
}

static ssize_t
show_temp_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	struct hwm_fan_temp_limit *temp = &fan->limit.temp;

	return sprintf(buf, "%u\n", (fan->temp && (fan->temp >= temp->max)));
}

static ssize_t
store_temp_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	struct fan_dev_config cfg;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	/* temperatur in 1/10 degC */
	val = DIV_ROUND_CLOSEST(val, 1000);
	val = val > 100 ? 100 : val;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return count;

	/* The EC imanager provides three different temperature limit values
	 * (stop, min, and max) where stop indicates a minimum temp value
	 * (threshold) from which the FAN will turn off.  We are setting
	 * temp_stop to the same value as temp_min.
	 */

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.temp_stop = val;
	cfg.temp_min = val;
	imanager_hwmon_write_fan_config(io, nr, &cfg);
	imanager_hwmon_read_fan_ctrl(&imgr->ec, nr, fan);

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
	struct imanager_hwmon_smartfan *fan = &data->hwmon_dev.fan[nr];
	struct fan_dev_config cfg;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val, 1000);
	val = val > 100 ? 100 : val;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return count;

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.temp_max = val;
	imanager_hwmon_write_fan_config(io, nr, &cfg);
	imanager_hwmon_read_fan_ctrl(&imgr->ec, nr, fan);

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
	struct fan_dev_config cfg;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.pwm_min = val;
	imanager_hwmon_write_fan_config(io, nr, &cfg);

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
	struct fan_dev_config cfg;
	long val = 0;
	int err;

	err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&imgr->lock);

	imanager_hwmon_read_fan_config(io, nr, &cfg);
	cfg.pwm_max = val;
	imanager_hwmon_write_fan_config(io, nr, &cfg);

	mutex_unlock(&imgr->lock);

	return count;
}

static ssize_t
show_in_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;
	int nr = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", hwmon->adc.attr[nr].label);
}

static ssize_t
show_temp_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%s\n", hwmon->fan.temp_label[nr]);
}

static ssize_t
show_fan_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;
	int nr = to_sensor_dev_attr(attr)->index - 1;

	return sprintf(buf, "%s\n", hwmon->fan.attr[nr].label);
}

/*
 * Sysfs callback functions
 */

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

	NULL
};

#define to_dev(obj) container_of(obj, struct device, kobj)

static umode_t
imanager_in_is_visible(struct kobject *kobj, struct attribute *attr, int index)
{
	struct device *dev = to_dev(kobj);
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;

	if (hwmon->adc.num <= HWM_MAX_ADC)
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
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;

	/*
	 * There are either 3 or 5 VINs available
	 * vin3 is current monitoring
	 * vin4 is CPU VID
	 */
	if (hwmon->adc.num == HWM_MAX_ADC)
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
	struct imanager_hwmon_device *hwmon = &data->imgr->ec.idev.hwmon;

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

/*
 * Module stuff
 */
static int imanager_hwmon_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_hwmon_device *hwmon = &ec->idev.hwmon;
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

	for (i = 0; i < hwmon->fan.num; i++) {
		/* set fan to automatic speed control */
		imanager_hwmon_write_fan_ctrl(ec, i, MODE_AUTO, CTRL_RPM, 0, 0,
					      NULL, NULL);
		/* update internal fan control settings */
		imanager_hwmon_read_fan_ctrl(ec, i, &data->hwmon_dev.fan[i]);
	}

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
		.name  = "imanager_hwmon",
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
MODULE_ALIAS("platform:imanager_hwmon");
