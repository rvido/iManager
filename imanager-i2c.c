/*
 * Advantech iManager I2C bus
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
#include <linux/slab.h>
#include <linux/platform_device.h>
#include <linux/i2c.h>
#include <linux/version.h>
#include <linux/mfd/imanager/compat.h>
#include <linux/mfd/imanager/core.h>
#include <linux/mfd/imanager/i2c.h>

static uint bus_frequency = 100;
module_param(bus_frequency, uint, 0);
MODULE_PARM_DESC(bus_frequency,
	"I2C bus frequency [50, 100, 400]kHz (defaults to 100kHz)");

struct imanager_i2c_data {
	struct imanager_device_data *ec;
	struct i2c_adapter adapter;
};

static int imanager_smb_access(struct i2c_adapter *adap, u16 addr,
	unsigned short flags, char read_write, u8 command,
	int size, union i2c_smbus_data *smb_data)
{
	struct imanager_i2c_data *data = i2c_get_adapdata(adap);
	struct device *dev = data->adapter.dev.parent;
	int ret = 0;
	int val = 0;

	if (!data)
		return -ENODEV;

	addr <<= 1;

	mutex_lock(&data->ec->lock);

	switch (size) {
	case I2C_SMBUS_QUICK:
		ret = i2c_core_write_quick(addr);

		dev_dbg(dev, "I2C_SMBUS_QUICK: addr=0x%02X, rw='%s', ret=%d\n",
			addr, read_write?"read":"write", ret);
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) /* NOT tested */
			val = i2c_core_write_byte(addr, command);
		else
			val = i2c_core_read_byte(addr);

		if (val < 0)
			ret = val;
		else
			smb_data->byte = val;

		dev_dbg(dev, "I2C_SMBUS_BYTE: addr=0x%02X, rw='%s', "
			"cmd=0x%02X, val=0x%02X, ret=%d\n",
			addr, read_write?"read":"write", command, val, ret);
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_WRITE)
			val = i2c_core_write_byte_data(addr, command,
				smb_data->byte);
		else
			val = i2c_core_read_byte_data(addr, command);

		if (val < 0)
			ret = val;
		else
			smb_data->byte = val;

		dev_dbg(dev, "I2C_SMBUS_BYTE_DATA: addr=0x%02X, rw='%s', "
			"cmd=0x%02X, val=0x%02X, ret=%d\n",
			addr, read_write?"read":"write", command, val, ret);
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE)
			val = i2c_core_write_word_data(addr, command,
				smb_data->word);
		else
			val = i2c_core_read_word_data(addr, command);

		if (val < 0)
			ret = val;
		else
			smb_data->word = val;

		dev_dbg(dev, "I2C_SMBUS_WORD_DATA: addr=0x%02X, rw='%s', "
			"cmd=0x%02X, val=0x%02X, ret=%d\n",
			addr, read_write?"read":"write", command, val, ret);
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE)
			ret = i2c_core_write_block_data(addr, command,
				smb_data->block);
		else
			ret = i2c_core_read_block_data(addr, command,
				smb_data->block);

		dev_dbg(dev, "I2C_SMBUS_BLOCK_DATA: addr=0x%02X, rw='%s', "
			"cmd=0x%02X, val=0x%02X, ret=%d\n",
			addr, read_write?"read":"write", command, val, ret);
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE)
			ret = i2c_core_write_i2c_block_data(addr, command,
				smb_data->block);
		else
			ret = i2c_core_read_i2c_block_data(addr, command,
				smb_data->block);

		dev_dbg(dev, "I2C_SMBUS_I2C_BLOCK_DATA: addr=0x%02X, rw='%s', "
			"cmd=0x%02X, val=0x%02X, ret=%d\n",
			addr, read_write?"read":"write", command, val, ret);
		break;
	default:
		dev_err(dev, "Unsupported SMB transaction %d\n", size);
		ret = -EOPNOTSUPP;
	}

	mutex_unlock(&data->ec->lock);

	return ret;
}

static int imanager_i2c_access(struct i2c_adapter *adap, struct i2c_msg *msg,
			int num)
{
	struct imanager_i2c_data *data = i2c_get_adapdata(adap);
	struct device *dev = data->adapter.dev.parent;

	/*
	 * To be implemented
	 */

	dev_info(dev, "msg=%p, num=%d\n", msg, num);

	return 0;
}

static u32 imanager_smb_i2c_func(struct i2c_adapter *adapter)
{
	return	I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA |
		I2C_FUNC_SMBUS_I2C_BLOCK | I2C_FUNC_I2C;
}

static const struct i2c_algorithm imanager_algorithm = {
	.smbus_xfer     = imanager_smb_access,
	.master_xfer    = imanager_i2c_access,
	.functionality  = imanager_smb_i2c_func,
};

static int imanager_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *ec = dev_get_drvdata(dev->parent);
	struct imanager_i2c_data *i2c;
	int ret;

	if (!ec) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	ret = i2c_core_init();
	if (ret) {
		dev_err(dev, "Failed to initialize I2C core\n");
		return -EIO;
	}

	if (bus_frequency > 100)
		bus_frequency = 400;
	else if (bus_frequency < 50)
		bus_frequency = 50;
	else
		bus_frequency = 100;

	ret = i2c_core_smb_set_freq(I2C_OEM_1, bus_frequency);
	if (ret < 0) {
		dev_err(dev, "Failed to set I2C bus frequency to %d kHz\n",
				bus_frequency);
		return ret;
	}

	ret = i2c_core_smb_get_freq(I2C_OEM_1);
	if (ret < 0) {
		dev_err(dev, "Failed to get I2C bus frequency\n");
		return ret;
	}
	bus_frequency = ret;
	dev_info(dev, "Bus frequency: %d kHz\n", bus_frequency);

	i2c = devm_kzalloc(dev, sizeof(struct imanager_i2c_data), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	platform_set_drvdata(pdev, i2c);
	i2c_set_adapdata(&i2c->adapter, i2c);

	i2c->ec = ec;

	i2c->adapter.owner	= THIS_MODULE;
	i2c->adapter.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD;
	i2c->adapter.algo	= &imanager_algorithm;

	/* set up the sysfs linkage to our parent device */
	i2c->adapter.dev.parent = dev;

	/* Retry up to 3 times on lost arbitration */
	i2c->adapter.retries = 3;

	snprintf(i2c->adapter.name, sizeof(i2c->adapter.name),
		"IT8518/28 I2C driver");

	ret = i2c_add_adapter(&i2c->adapter);
	if (ret) {
		dev_err(dev, "Failed to add SMBus adapter\n");
		return ret;
	}

	return 0;
}

static int imanager_i2c_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_i2c_data *i2c = dev_get_drvdata(dev);

	i2c_del_adapter(&i2c->adapter);
	i2c_set_adapdata(&i2c->adapter, NULL);

	i2c_core_release();

	return 0;
}

static struct platform_driver imanager_i2c_driver = {
	.driver = {
		.owner = THIS_MODULE,
		.name  = "imanager_i2c",
	},
	.probe	= imanager_i2c_probe,
	.remove	= imanager_i2c_remove,
};

module_platform_driver(imanager_i2c_driver);

MODULE_DESCRIPTION("Advantech iManager I2C Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager_i2c");
