/*
 * Advantech iManager MFD core (EC IT8518/28)
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
#include "compat.h"
#include "core.h"
#include "ec.h"

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

static const char * const imanager_names[] = {
	"it8516",
	"it8518",
	"it8528",
};

static struct resource imanager_ioresource = {
	.start  = IT8516_DAT_PORT,
	.end    = IT8518_DAT_PORT,
	.flags  = IORESOURCE_IO,
};

/*
 * Devices that are part of the iManager and are available via firmware.
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

const char *imanager_project_code_str(const struct ec_version *version)
{
	const char *str;

	if (WARN_ON(!version))
		return NULL;

	switch (version->type) {
	case 'V':
		str = "release";
		break;
	case 'X':
		str = "engineering sample";
		break;
	case 'A' ... 'U':
		str = "custom";
		break;
	default:
		str = "unspecified";
		break;
	}

	return str;
}

static ssize_t imanager_name_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", pdata->info.pcb_name);
}

static ssize_t imanager_kversion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 pdata->info.version.kernel_major,
			 pdata->info.version.kernel_minor);
}

static ssize_t imanager_fwversion_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 pdata->info.version.firmware_major,
			 pdata->info.version.firmware_minor);
}

static ssize_t imanager_type_show(struct device *dev,
		struct device_attribute *attr, char *buf)
{
	struct imanager_platform_data *pdata = dev_get_platdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 imanager_project_code_str(&pdata->info.version));
}

static DEVICE_ATTR(imanager_name, S_IRUGO, imanager_name_show, NULL);
static DEVICE_ATTR(imanager_kversion, S_IRUGO, imanager_kversion_show, NULL);
static DEVICE_ATTR(imanager_fwversion, S_IRUGO, imanager_fwversion_show, NULL);
static DEVICE_ATTR(imanager_type, S_IRUGO, imanager_type_show, NULL);

static struct attribute *imanager_core_attributes[] = {
	&dev_attr_imanager_name.attr,
	&dev_attr_imanager_kversion.attr,
	&dev_attr_imanager_fwversion.attr,
	&dev_attr_imanager_type.attr,
	NULL
};

static const struct attribute_group imanager_core_attr_group = {
	.attrs = imanager_core_attributes,
};

static int imanager_core_init(struct imanager_device_data *ec)
{
	int chipid;

	chipid = imanager_get_chipid();

	switch (chipid) {
	case SIO_DEVID_IT8516:
		pr_err("IT8516 not supported.\n");
		return -ENODEV;
	case SIO_DEVID_IT8518:
		ec->type = IT8518;
		break;
	case SIO_DEVID_IT8528:
		ec->type = IT8528;
		break;
	default:
		return -ENODEV;
	}

	return 0;
}

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
	struct imanager_device_data *ec;
	struct ec_info *info = &pdata->info;
	int ret;

	if (!pdev)
		return -EINVAL;

	ec = devm_kzalloc(dev, sizeof(*ec), GFP_KERNEL);
	if (!ec)
		return -ENOMEM;

	ret = imanager_core_init(ec);
	if (ret)
		return ret;

	ec->dev = dev;
	ec->name = imanager_names[ec->type];

	mutex_init(&ec->lock);

	platform_set_drvdata(pdev, ec);

	imanager_get_fw_info(info);

	dev_info(dev, "Found Advantech iManager %s - %s %d.%d/%d.%d (%s)\n",
		 ec->name, info->pcb_name,
		 info->version.kernel_major, info->version.kernel_minor,
		 info->version.firmware_major, info->version.firmware_minor,
		 imanager_project_code_str(&info->version));

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

	ret = imanager_ec_probe(SIO_BASE_ADDR);
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
