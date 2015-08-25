/*
 * Advantech iManager Hardware Monitoring driver
 * Derived from nct6775 driver
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/jiffies.h>
#include <linux/platform_device.h>
#include <linux/hwmon.h>
#include <linux/hwmon-sysfs.h>
#include <linux/hwmon-vid.h>
#include <linux/err.h>
#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/version.h>
#include <core.h>
#include <hwmon.h>
#define __NEED_HWMON_COMPAT__
#include <compat.h>

struct imanager_hwmon_data {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	struct device *hwmon_dev;
#endif
	struct imanager_device_data *ec;
	bool valid;	/* if set, below values are valid */
	struct hwm_data hwm;
	int adc_num;
	int fan_num;
	unsigned long samples;
	unsigned long last_updated;
	const struct attribute_group *groups[3];
};

static inline u32 in_from_reg(u16 val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * SCALE_IN, 1000), 0, 65535);
}

static inline u16 in_to_reg(u32 val)
{
	return clamp_val(DIV_ROUND_CLOSEST(val * 1000, SCALE_IN), 0, 65535);
}

static struct imanager_hwmon_data *
imanager_hwmon_update_device(struct device *dev)
{
	struct imanager_hwmon_data *data = dev_get_drvdata(dev);
	int i;

	mutex_lock(&data->ec->lock);

	if (time_after(jiffies, data->last_updated + HZ + HZ / 2)
	    || !data->valid) {
		/* Measured voltages */
		for (i = 0; i < data->adc_num; i++)
			hwm_core_adc_get_value(i, &data->hwm.volt[i]);

		/* Measured fan speeds */
		for (i = 0; i < data->fan_num; i++)
			hwm_core_fan_get_ctrl(i, &data->hwm.fan[i]);

		data->last_updated = jiffies;
		data->valid = true;
	}

	mutex_unlock(&data->ec->lock);

	return data;
}

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
static ssize_t
show_name(struct device *dev, struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "imanager_hwmon\n");
}
static DEVICE_ATTR(name, S_IRUGO, show_name, NULL);
#endif

static ssize_t
show_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	if (!adc)
		return -EINVAL;

	return sprintf(buf, "%u\n", in_from_reg(adc->value));
}

static ssize_t
show_in_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	if (!adc)
		return -EINVAL;

	return sprintf(buf, "%u\n", in_from_reg(adc->min));
}

static ssize_t
show_in_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	if (!adc)
		return -EINVAL;

	return sprintf(buf, "%u\n", in_from_reg(adc->max));
}

static ssize_t
store_in_min(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	unsigned long val;
	int err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->ec->lock);

	adc->min = in_to_reg(val);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
store_in_max(struct device *dev, struct device_attribute *attr,
	     const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	unsigned long val;
	int err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->ec->lock);

	adc->max = in_to_reg(val);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_in_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	int val = 0;

	if (adc->valid)
		val = (adc->value < adc->min) || (adc->value > adc->max);

	return sprintf(buf, "%u\n", val);
}

static ssize_t
show_in_average(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];

	if (adc->average)
		adc->average = DIV_ROUND_CLOSEST(adc->average * data->samples
				+ adc->value, ++data->samples);
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
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];

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
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];

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
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_voltage *adc = &data->hwm.volt[index];
	unsigned long val;
	int err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	mutex_lock(&data->ec->lock);

	if (val == 1) {
		adc->lowest = 0;
		adc->highest = 0;
	}
	else {
		count = -EINVAL;
	}

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_temp(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];

	return sprintf(buf, "%u\n", fan->temp * 1000);
}

static ssize_t
show_fan_in(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];

	return sprintf(buf, "%u\n", fan->valid ? fan->speed : 0);
}

static inline int is_alarm(const struct hwm_smartfan *fan)
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
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];

	return sprintf(buf, "%u\n", fan->valid ? is_alarm(fan) : 0);
}

static ssize_t
show_fan_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_fan_limit *rpm = &data->hwm.fan[index - 1].limit.rpm;

	return sprintf(buf, "%u\n", rpm->min);
}

static ssize_t
show_fan_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_fan_limit *rpm = &data->hwm.fan[index - 1].limit.rpm;

	return sprintf(buf, "%u\n", rpm->max);
}

static ssize_t
store_fan_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];
	struct hwm_fan_limit *rpm = &fan->limit.rpm;
	unsigned long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_rpm_limit(index - 1, val, rpm->max);
	hwm_core_fan_get_ctrl(index - 1, fan); /* update */

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
store_fan_max(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];
	struct hwm_fan_limit *rpm = &fan->limit.rpm;
	unsigned long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_rpm_limit(index - 1, rpm->min, val);
	hwm_core_fan_get_ctrl(index - 1, fan);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_pwm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	u32 val = DIV_ROUND_CLOSEST(data->hwm.fan[index - 1].pwm * 255, 100);

	return sprintf(buf, "%u\n", val);
}

static ssize_t
store_pwm(struct device *dev, struct device_attribute *attr,
	  const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];
	unsigned long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	mutex_lock(&data->ec->lock);

	switch (fan->mode) {
	case MODE_MANUAL:
		hwm_core_fan_set_ctrl(index - 1, MODE_MANUAL, CTRL_PWM,
				      val, 0, NULL, NULL);
		break;
	case MODE_AUTO:
		if (fan->type == CTRL_RPM)
			break;
		hwm_core_fan_set_ctrl(index - 1, MODE_AUTO, CTRL_PWM,
				      val, 0, &fan->limit, &fan->alert);
		break;
	}

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_pwm_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%u\n", data->hwm.fan[index - 1].limit.pwm.min);
}

static ssize_t
show_pwm_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%u\n", data->hwm.fan[index - 1].limit.pwm.max);
}

static ssize_t
show_pwm_enable(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];

	if (fan->mode == MODE_OFF)
		return -EINVAL;

	return sprintf(buf, "%u\n", fan->mode - 1);
}

static ssize_t
store_pwm_enable(struct device *dev, struct device_attribute *attr,
		 const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];
	unsigned long mode = 0;
	int err = kstrtoul(buf, 10, &mode);
	if (err)
		return err;

	if (mode > MODE_AUTO)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	switch (mode) {
	case 0:
		if (mode != 0)
			hwm_core_fan_set_ctrl(nr, MODE_FULL, CTRL_PWM, 100,
					      fan->pulse, NULL, NULL);
		break;
	case 1:
		if (mode != 1)
			hwm_core_fan_set_ctrl(nr, MODE_MANUAL, CTRL_PWM, 0,
					      fan->pulse, NULL, NULL);
		break;
	case 2:
		if (mode != 2)
			hwm_core_fan_set_ctrl(nr, MODE_AUTO, fan->type, 0,
					      fan->pulse, &fan->limit, NULL);
		break;
	}

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_pwm_mode(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_smartfan *fan = &data->hwm.fan[index - 1];

	if (fan->mode == MODE_OFF)
		return -EINVAL;

	return sprintf(buf, "%u\n", fan->type == CTRL_PWM ? 1 : 0);
}

static ssize_t
store_pwm_mode(struct device *dev, struct device_attribute *attr,
	       const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];
	unsigned long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err)
		return err;

	if (fan->mode != MODE_AUTO)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_ctrl(nr, fan->mode, val ? CTRL_RPM : CTRL_PWM,
			      fan->pwm, fan->pulse, &fan->limit, &fan->alert);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_temp_min(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int val = data->hwm.fan[nr].limit.temp.min;

	return sprintf(buf, "%d\n", val * 1000);
}

static ssize_t
show_temp_max(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	int val = data->hwm.fan[nr].limit.temp.max;

	return sprintf(buf, "%u\n", val * 1000);
}

static ssize_t
show_temp_alarm(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];
	struct hwm_fan_temp_limit *temp = &fan->limit.temp;

	return sprintf(buf, "%u\n", (fan->temp && (fan->temp >= temp->max)));
}

static ssize_t
store_temp_min(struct device *dev, struct device_attribute *attr,
		const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];
	struct hwm_fan_temp_limit *temp = &fan->limit.temp;
	long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val, 1000);

	if (val > 100)
		return -EINVAL;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return -EINVAL;

	/* The EC imanager provides three different temperature limit values
	 * (stop, min, and max) where stop indicates a minimum temp value
	 * (threshold) from which the FAN will turn off.  We are setting
	 * temp_stop to the same value as temp_min.
	 */

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_temp_limit(nr, val, val, temp->max);
	hwm_core_fan_get_ctrl(nr, fan);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
store_temp_max(struct device *dev, struct device_attribute *attr,
	       const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int nr = to_sensor_dev_attr(attr)->index - 1;
	struct hwm_smartfan *fan = &data->hwm.fan[nr];
	struct hwm_fan_temp_limit *temp = &fan->limit.temp;
	long val = 0;
	int err = kstrtoul(buf, 10, &val);
	if (err < 0)
		return err;

	val = DIV_ROUND_CLOSEST(val, 1000);

	if (val > 100)
		return -EINVAL;

	/* do not apply value if not in 'fan cruise mode' */
	if (fan->mode != MODE_AUTO)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_temp_limit(nr, temp->stop, temp->min, val);
	hwm_core_fan_get_ctrl(nr, fan);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
store_pwm_min(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_fan_limit *pwm = &data->hwm.fan[index - 1].limit.pwm;
	long val = 0;
	int ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	if (val > 100)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_pwm_limit(index - 1, val, pwm->max);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
store_pwm_max(struct device *dev, struct device_attribute *attr,
	      const char *buf, size_t count)
{
	struct imanager_hwmon_data *data = imanager_hwmon_update_device(dev);
	int index = to_sensor_dev_attr(attr)->index;
	struct hwm_fan_limit *pwm = &data->hwm.fan[index - 1].limit.pwm;
	long val = 0;
	int ret = kstrtoul(buf, 10, &val);
	if (ret)
		return -EINVAL;

	val = DIV_ROUND_CLOSEST(val * 100, 255);

	if (val > 100)
		return -EINVAL;

	mutex_lock(&data->ec->lock);

	hwm_core_fan_set_pwm_limit(index - 1, pwm->min, val);

	mutex_unlock(&data->ec->lock);

	return count;
}

static ssize_t
show_in_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", hwm_core_adc_get_label(index));
}

static ssize_t
show_temp_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", hwm_core_fan_get_temp_label(index - 1));
}

static ssize_t
show_fan_label(struct device *dev, struct device_attribute *attr, char *buf)
{
	int index = to_sensor_dev_attr(attr)->index;

	return sprintf(buf, "%s\n", hwm_core_fan_get_label(index - 1));
}

/*
 * Sysfs callback functions
 */

static SENSOR_DEVICE_ATTR(in0_label, S_IRUGO, show_in_label, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_input, S_IRUGO, show_in, NULL, 0);
static SENSOR_DEVICE_ATTR(in0_min, S_IWUSR | S_IRUGO, show_in_min, store_in_min, 0);
static SENSOR_DEVICE_ATTR(in0_max, S_IWUSR | S_IRUGO, show_in_max, store_in_max, 0);
static SENSOR_DEVICE_ATTR(in0_alarm, S_IRUGO, show_in_alarm, NULL, 0);

static SENSOR_DEVICE_ATTR(in1_label, S_IRUGO, show_in_label, NULL, 1);
static SENSOR_DEVICE_ATTR(in1_input, S_IRUGO, show_in, NULL, 1);
static SENSOR_DEVICE_ATTR(in1_min, S_IWUSR | S_IRUGO, show_in_min, store_in_min, 1);
static SENSOR_DEVICE_ATTR(in1_max, S_IWUSR | S_IRUGO, show_in_max, store_in_max, 1);
static SENSOR_DEVICE_ATTR(in1_alarm, S_IRUGO, show_in_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(in2_label, S_IRUGO, show_in_label, NULL, 2);
static SENSOR_DEVICE_ATTR(in2_input, S_IRUGO, show_in, NULL, 2);
static SENSOR_DEVICE_ATTR(in2_min, S_IWUSR | S_IRUGO, show_in_min, store_in_min, 2);
static SENSOR_DEVICE_ATTR(in2_max, S_IWUSR | S_IRUGO, show_in_max, store_in_max, 2);
static SENSOR_DEVICE_ATTR(in2_alarm, S_IRUGO, show_in_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(temp1_label, S_IRUGO, show_temp_label, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_input, S_IRUGO, show_temp, NULL, 1);
static SENSOR_DEVICE_ATTR(temp1_min, S_IRUGO | S_IWUSR, show_temp_min, store_temp_min, 1);
static SENSOR_DEVICE_ATTR(temp1_max, S_IRUGO | S_IWUSR, show_temp_max, store_temp_max, 1);
static SENSOR_DEVICE_ATTR(temp1_alarm, S_IRUGO, show_temp_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(temp2_label, S_IRUGO, show_temp_label, NULL, 2);
static SENSOR_DEVICE_ATTR(temp2_input, S_IRUGO, show_temp, NULL, 2);
static SENSOR_DEVICE_ATTR(temp2_min, S_IRUGO | S_IWUSR, show_temp_min, store_temp_min, 2);
static SENSOR_DEVICE_ATTR(temp2_max, S_IRUGO | S_IWUSR, show_temp_max, store_temp_max, 2);
static SENSOR_DEVICE_ATTR(temp2_alarm, S_IRUGO, show_temp_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(temp3_label, S_IRUGO, show_temp_label, NULL, 3);
static SENSOR_DEVICE_ATTR(temp3_input, S_IRUGO, show_temp, NULL, 3);
static SENSOR_DEVICE_ATTR(temp3_min, S_IRUGO | S_IWUSR, show_temp_min, store_temp_min, 3);
static SENSOR_DEVICE_ATTR(temp3_max, S_IRUGO | S_IWUSR, show_temp_max, store_temp_max, 3);
static SENSOR_DEVICE_ATTR(temp3_alarm, S_IRUGO, show_temp_alarm, NULL, 3);

static SENSOR_DEVICE_ATTR(fan1_label, S_IRUGO, show_fan_label, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_input, S_IRUGO, show_fan_in, NULL, 1);
static SENSOR_DEVICE_ATTR(fan1_min, S_IWUSR | S_IRUGO, show_fan_min, store_fan_min, 1);
static SENSOR_DEVICE_ATTR(fan1_max, S_IWUSR | S_IRUGO, show_fan_max, store_fan_max, 1);
static SENSOR_DEVICE_ATTR(fan1_alarm, S_IRUGO, show_fan_alarm, NULL, 1);

static SENSOR_DEVICE_ATTR(fan2_label, S_IRUGO, show_fan_label, NULL, 2);
static SENSOR_DEVICE_ATTR(fan2_input, S_IRUGO, show_fan_in, NULL, 2);
static SENSOR_DEVICE_ATTR(fan2_min, S_IWUSR | S_IRUGO, show_fan_min, store_fan_min, 2);
static SENSOR_DEVICE_ATTR(fan2_max, S_IWUSR | S_IRUGO, show_fan_max, store_fan_max, 2);
static SENSOR_DEVICE_ATTR(fan2_alarm, S_IRUGO, show_fan_alarm, NULL, 2);

static SENSOR_DEVICE_ATTR(fan3_label, S_IRUGO, show_fan_label, NULL, 3);
static SENSOR_DEVICE_ATTR(fan3_input, S_IRUGO, show_fan_in, NULL, 3);
static SENSOR_DEVICE_ATTR(fan3_min, S_IWUSR | S_IRUGO, show_fan_min, store_fan_min, 3);
static SENSOR_DEVICE_ATTR(fan3_max, S_IWUSR | S_IRUGO, show_fan_max, store_fan_max, 3);
static SENSOR_DEVICE_ATTR(fan3_alarm, S_IRUGO, show_fan_alarm, NULL, 3);

static SENSOR_DEVICE_ATTR(pwm1, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 1);
static SENSOR_DEVICE_ATTR(pwm1_min, S_IWUSR | S_IRUGO, show_pwm_min, store_pwm_min, 1);
static SENSOR_DEVICE_ATTR(pwm1_max, S_IWUSR | S_IRUGO, show_pwm_max, store_pwm_max, 1);
static SENSOR_DEVICE_ATTR(pwm1_enable, S_IWUSR | S_IRUGO, show_pwm_enable, store_pwm_enable, 1);
static SENSOR_DEVICE_ATTR(pwm1_mode, S_IWUSR | S_IRUGO, show_pwm_mode, store_pwm_mode, 1);

static SENSOR_DEVICE_ATTR(pwm2, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 2);
static SENSOR_DEVICE_ATTR(pwm2_min, S_IWUSR | S_IRUGO, show_pwm_min, store_pwm_min, 2);
static SENSOR_DEVICE_ATTR(pwm2_max, S_IWUSR | S_IRUGO, show_pwm_max, store_pwm_max, 2);
static SENSOR_DEVICE_ATTR(pwm2_enable, S_IWUSR | S_IRUGO, show_pwm_enable, store_pwm_enable, 2);
static SENSOR_DEVICE_ATTR(pwm2_mode, S_IWUSR | S_IRUGO, show_pwm_mode, store_pwm_mode, 2);

static SENSOR_DEVICE_ATTR(pwm3, S_IWUSR | S_IRUGO, show_pwm, store_pwm, 3);
static SENSOR_DEVICE_ATTR(pwm3_min, S_IWUSR | S_IRUGO, show_pwm_min, store_pwm_min, 3);
static SENSOR_DEVICE_ATTR(pwm3_max, S_IWUSR | S_IRUGO, show_pwm_max, store_pwm_max, 3);
static SENSOR_DEVICE_ATTR(pwm3_enable, S_IWUSR | S_IRUGO, show_pwm_enable, store_pwm_enable, 3);
static SENSOR_DEVICE_ATTR(pwm3_mode, S_IWUSR | S_IRUGO, show_pwm_mode, store_pwm_mode, 3);

static SENSOR_DEVICE_ATTR(curr1_input, S_IRUGO, show_in, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_min, S_IWUSR | S_IRUGO, show_in_min, store_in_min, 4);
static SENSOR_DEVICE_ATTR(curr1_max, S_IWUSR | S_IRUGO, show_in_max, store_in_max, 4);
static SENSOR_DEVICE_ATTR(curr1_alarm, S_IRUGO, show_in_alarm, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_average, S_IRUGO, show_in_average, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_lowest, S_IRUGO, show_in_lowest, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_highest, S_IRUGO, show_in_highest, NULL, 4);
static SENSOR_DEVICE_ATTR(curr1_reset_history, S_IWUSR, NULL, store_in_reset_history, 4);

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

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	&dev_attr_name.attr,
#endif
	NULL
};

#ifdef __RHEL6__
static mode_t
#else
static umode_t
#endif
imanager_in_is_visible(struct kobject *kobj, struct attribute *attr, int index)
{
	int err;

	if ((index >= 0) && (index <= 14)) { /* vin */
		err = hwm_core_adc_is_available(index / 5);
		if (err < 0)
			return 0;
	}

	return attr->mode;
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

#ifdef __RHEL6__
static mode_t
#else
static umode_t
#endif
imanager_other_is_visible(struct kobject *kobj,
			  struct attribute *attr, int index)
{
	int ret = hwm_core_adc_get_max_count();

	/*
	 * There are either 3 or 5 VINs
	 * vin3 is current monitoring
	 * vin4 is CPU VID
	 */
	if (ret < 5)
		return 0;

	return attr->mode;
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

#ifdef __RHEL6__
static mode_t
#else
static umode_t
#endif
imanager_fan_is_visible(struct kobject *kobj, struct attribute *attr, int index)
{
	int err;

	if ((index >= 0) && (index <= 14)) { /* fan */
		err = hwm_core_fan_is_available(index / 5);
		if (err < 0)
			return 0;
	} else if ((index >= 15) && (index <= 29)) { /* temp */
		err = hwm_core_fan_is_available((index - 15) / 5);
		if (err < 0)
			return 0;
	} else if ((index >= 30) && (index <= 34)) { /* pwm */
		err = hwm_core_fan_is_available((index - 30) / 5);
		if (err < 0)
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
	struct imanager_device_data *ec = dev_get_drvdata(dev->parent);
	struct imanager_hwmon_data *data;
	struct device *hwmon_dev;
	int err, i, num_attr_groups = 0;

	if (!ec) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	err = hwm_core_init();
	if (err) {
		dev_err(dev, "Hwmon core init failed\n");
		return -EIO;
	}

	data = devm_kzalloc(dev, sizeof(struct imanager_hwmon_data),
			    GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->ec = ec;
	platform_set_drvdata(pdev, data);

	data->adc_num = hwm_core_adc_get_max_count();
	data->fan_num = hwm_core_fan_get_max_count();

	for (i = 0; i < data->fan_num; i++) {
		/* set active fan to automatic speed control */
		hwm_core_fan_set_ctrl(i, MODE_AUTO, CTRL_RPM, 0, 0,
				      NULL, NULL);
		hwm_core_fan_get_ctrl(i, &data->hwm.fan[i]);
	}

	data->groups[num_attr_groups++] = &imanager_group_in;

	if (data->adc_num > 3)
		data->groups[num_attr_groups++] = &imanager_group_other;

	if (data->fan_num)
		data->groups[num_attr_groups++] = &imanager_group_fan;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,13,0)
	err = sysfs_create_groups(&dev->kobj, data->groups);
	if (err < 0)
		return err;

	hwmon_dev = hwmon_device_register(dev);
	if (IS_ERR(hwmon_dev)) {
		sysfs_remove_groups(&dev->kobj, data->groups);
		return PTR_ERR(data->hwmon_dev);
	}
	data->hwmon_dev = hwmon_dev;
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

	hwmon_device_unregister(data->hwmon_dev);
	sysfs_remove_groups(&pdev->dev.kobj, data->groups);

	return 0;
}
#endif

static struct platform_driver imanager_hwmon_driver = {
	.driver = {
		.owner = THIS_MODULE,
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
