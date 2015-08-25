/*
 * Advantech iManager MFD core driver
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

#include <linux/module.h>
#include <linux/init.h>
#include <linux/platform_device.h>
#include <linux/ioport.h>
#include <linux/io.h>
#include <linux/version.h>
#include <linux/mfd/core.h>
#include <compat.h>
#include <core.h>
#include <ec.h>

static unsigned short force_id;
module_param(force_id, ushort, 0);
MODULE_PARM_DESC(force_id, "Override detected device ID");

static struct platform_device *pdev;

enum imanager_cells {
	IMANAGER_BL,
	IMANAGER_GPIO,
	IMANAGER_HWMON,
	IMANAGER_I2C,
	IMANAGER_WDT,
};

enum imanager_project_state {
	RELEASE,
	ES,
	CUSTOM,
	NA,
};

static const char * const chip_names[] = {
	"it8516",
	"it8518",
	"it8528",
	NULL
};

static const char * const project_state[] = {
	"Release",
	"Engineering Sample",
	"Custom",
	"Unspecified",
	NULL
};

static struct resource imanager_ioresource = {
	.start  = IT8516_DAT_PORT,
	.end    = IT8518_DAT_PORT,
	.flags  = IORESOURCE_IO,
};

/*
 * Devices which are part of the iManager and are available via firmware.
 */
static struct mfd_cell imanager_devs[] = {
	[IMANAGER_BL] = {
		.name = "imanager_backlight",
	},
	[IMANAGER_GPIO] = {
		.name = "imanager_gpio",
	},
	[IMANAGER_HWMON] = {
		.name = "imanager_hwmon",
	},
	[IMANAGER_I2C] = {
		.name = "imanager_i2c",
	},
	[IMANAGER_WDT] = {
		.name = "imanager_wdt",
	},
};

const char *project_type_to_str(int type)
{
	int state;

	switch (type) {
	case 'V':
		state = RELEASE;
		break;
	case 'X':
		state = ES;
		break;
	case 'A' ... 'U':
		state = CUSTOM;
		break;
	default:
		state = NA;
		break;
	}

	return project_state[state];
}

static ssize_t imanager_name_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);
	const struct ec_info *info = &pdata->dev->info;

	return scnprintf(buf, PAGE_SIZE, "%s\n", info->pcb_name);
}

static ssize_t imanager_kversion_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);
	const struct ec_info *info = &pdata->dev->info;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 info->version.kernel_major,
			 info->version.kernel_minor);
}

static ssize_t imanager_fwversion_show(struct device *dev,
				       struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);
	const struct ec_info *info = &pdata->dev->info;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 info->version.firmware_major,
			 info->version.firmware_minor);
}

static ssize_t imanager_type_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);
	const struct ec_info *info = &pdata->dev->info;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			project_type_to_str(info->version.type));
}

static ssize_t imanager_chip_name_show(struct device *dev,
			struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", pdata->chip_name);
}

static DEVICE_ATTR(imanager_name, S_IRUGO, imanager_name_show, NULL);
static DEVICE_ATTR(imanager_kversion, S_IRUGO, imanager_kversion_show, NULL);
static DEVICE_ATTR(imanager_fwversion, S_IRUGO, imanager_fwversion_show, NULL);
static DEVICE_ATTR(imanager_type, S_IRUGO, imanager_type_show, NULL);
static DEVICE_ATTR(imanager_chip_name, S_IRUGO, imanager_chip_name_show, NULL);

static struct attribute *imanager_core_attributes[] = {
	&dev_attr_imanager_name.attr,
	&dev_attr_imanager_kversion.attr,
	&dev_attr_imanager_fwversion.attr,
	&dev_attr_imanager_type.attr,
	&dev_attr_imanager_chip_name.attr,
	NULL
};

static const struct attribute_group imanager_core_attr_group = {
	.attrs = imanager_core_attributes,
};

static int imanager_platform_create(void)
{
	struct device *dev;
	struct imanager_platform_data platdata;
	int err;

	pdev = platform_device_alloc("imanager-core", -1);
	if (!pdev)
		return -ENOMEM;

	dev = &pdev->dev;

	err = platform_device_add_data(pdev, &platdata, sizeof(platdata));
	if (err)
		goto exit_device_put;

	err = platform_device_add_resources(pdev, &imanager_ioresource, 1);
	if (err)
		goto exit_device_put;

	err = platform_device_add(pdev);
	if (err)
		goto exit_device_put;

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
	err = mfd_add_devices(dev, pdev->id, imanager_devs,
		ARRAY_SIZE(imanager_devs), NULL, -1);
#else
	err = mfd_add_devices(dev, pdev->id, imanager_devs,
		ARRAY_SIZE(imanager_devs), NULL, -1, NULL);
#endif
	if (err)
		goto exit_device_unregister;

	return 0;

exit_device_unregister:
	platform_device_unregister(pdev);
exit_device_put:
	platform_device_put(pdev);

	return err;
}

static int imanager_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_platform_data *pdata = dev_get_platdata(dev);
	struct imanager_device_data *imanager;
	int ret;

	if (!pdev)
		return -EINVAL;

	imanager = devm_kzalloc(dev, sizeof(*imanager), GFP_KERNEL);
	if (!imanager)
		return -ENOMEM;

	imanager->dev = dev;
	mutex_init(&imanager->lock);

	platform_set_drvdata(pdev, imanager);

	pdata->dev = imanager_get_ec_device();
	pdata->chip_name = chip_names[pdata->dev->id];

	dev_info(dev, "Found Advantech iManager %s - %s %d.%d/%d.%d (%s)\n",
		 pdata->chip_name,
		 pdata->dev->info.pcb_name,
		 pdata->dev->info.version.kernel_major,
		 pdata->dev->info.version.kernel_minor,
		 pdata->dev->info.version.firmware_major,
		 pdata->dev->info.version.firmware_minor,
		 project_type_to_str(pdata->dev->info.version.type));

	dev_info(dev, "Available iManager Devices: %s, %s, %s, %s, %s, %s\n",
		imanager_get_gpio_device()->num ? "GPIO" : "No GPIO",
		imanager_get_hwmon_device()->adc.num ? "ADC" : "No ADC",
		imanager_get_hwmon_device()->fan.num ? "FAN" : "No FAN",
		imanager_get_i2c_device()->num ? "I2C" : "No I2C",
		imanager_get_backlight_device()->num ? "BL" : "No BL",
		imanager_get_watchdog_device()->num ? "WD" : "No WD");

	ret = sysfs_create_group(&dev->kobj, &imanager_core_attr_group);
	if (ret)
		return ret;

	return 0;
}

static int imanager_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;

	sysfs_remove_group(&dev->kobj, &imanager_core_attr_group);

	mfd_remove_devices(dev);

	return 0;
}

static struct platform_driver imanager_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imanager-core",
	},
	.probe	= imanager_probe,
	.remove	= imanager_remove,
};

static int __init imanager_init(void)
{
	int ret;

	ret = imanager_ec_probe(EC_BASE_ADDR);
	if (ret < 0)
		return ret;

	ret = imanager_platform_create();
	if (ret)
		return ret;

	ret = platform_driver_register(&imanager_driver);
	if (ret)
		return ret;

	return 0;
}

static void __exit imanager_exit(void)
{
	if (pdev)
		platform_device_unregister(pdev);

	platform_driver_unregister(&imanager_driver);
}

module_init(imanager_init);
module_exit(imanager_exit);

MODULE_DESCRIPTION("Advantech iManager Core Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-core");
