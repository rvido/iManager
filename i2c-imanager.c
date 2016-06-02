/*
 * Advantech iManager SMBus bus driver
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

#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/slab.h>
#include "imanager.h"
#include "compat.h"

#define I2C_SMBUS_BLOCK_SIZE	32UL
#define I2C_MAX_READ_SIZE	I2C_SMBUS_BLOCK_SIZE
#define I2C_MAX_WRITE_SIZE	(I2C_SMBUS_BLOCK_SIZE - 1)
#define CHECK_RD_SIZE(x)	(x ? x : I2C_MAX_READ_SIZE)

#define EC_HWRAM_OFFSET_STATUS	0UL

#define I2C_ERR_PROTO		0x19UL
#define I2C_ERR_TIMEOUT		0x18UL
#define I2C_ERR_ACCESS		0x17UL
#define I2C_ERR_UNKNOWN		0x13UL
#define I2C_ERR_ADDR_NACK	0x10UL

#define EC_I2C_XFER_COMPLETE	0UL

static uint bus_frequency = 100;
module_param(bus_frequency, uint, 0);
MODULE_PARM_DESC(bus_frequency,
	"SMBus bus frequency [50, 100, 400]kHz (defaults to 100kHz)");

/* Used for setting SMBus frequency */
enum smb_bus_id {
	SMB_OEM_0 = 0x28,
	SMB_OEM_1,
	SMB_OEM_2,
	SMB_EEPROM,
	SMB_TH_0,
	SMB_TH_1,
	SMB_SECURITY_EEPROM,
	I2C_OEM_1,
};

struct ec_i2c_status {
	u32 error	: 7;
	u32 complete	: 1;
};

struct imanager_i2c_data {
	struct device			*dev;
	struct imanager_device_data	*imgr;
	struct i2c_adapter		adapter;
};

static int imanager_i2c_eval_status(u8 status)
{
	struct ec_i2c_status *_status = (struct ec_i2c_status *)&status;
	int ret = 0;

	switch (_status->error) {
	case 0:
		break;
	case I2C_ERR_ADDR_NACK:
		ret = -ENODEV;
		break;
	case I2C_ERR_ACCESS:
	case I2C_ERR_UNKNOWN:
		ret = -EAGAIN;
		break;
	case I2C_ERR_TIMEOUT:
		ret = -ETIME;
		break;
	case I2C_ERR_PROTO:
		ret = -EPROTO;
		break;
	default:
		ret = -EIO;
		break;
	}

	return ret;
}

static int imanager_wait_proc_complete(struct imanager_io_ops *io)
{
	int ret, i;
	u8 val;

	for (i = 0; i < EC_MAX_RETRY; i++) {
		ret = imanager_read_ram(io, EC_RAM_HW, EC_HWRAM_OFFSET_STATUS,
					&val, sizeof(val));
		if (ret < 0)
			return ret;

		if (val == EC_I2C_XFER_COMPLETE)
			return 0;

		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	}

	return -EIO;
}

static int
imanager_i2c_block_wr_rw_combined(struct imanager_io_ops *io, unsigned proto,
				  struct ec_message *msg)
{
	int ret;

	ret = imanager_wait_proc_complete(io);
	if (ret)
		return ret;

	ret = imanager_write(io, proto, msg);
	if (ret)
		return imanager_i2c_eval_status(ret);

	if (msg->rlen) {
		if (msg->rlen == 1)
			return msg->u.data[0];
		else if (msg->rlen == 2)
			return (msg->u.data[1] << 8) | msg->u.data[0];
		else
			return msg->rlen;
	}

	return 0;
}

#define imanager_i2c_wr_combined(io, message) \
	imanager_i2c_block_wr_rw_combined(io, EC_CMD_I2C_WR, message)

#define imanager_i2c_rw_combined(io, message) \
	imanager_i2c_block_wr_rw_combined(io, EC_CMD_I2C_RW, message)

static int imanager_i2c_read_freq(struct imanager_io_ops *io, unsigned bus_id)
{
	int ret = 0, f;
	int freq_id, freq;

	ret = imanager_read16(io, EC_CMD_SMB_FREQ_RD, bus_id);
	if (ret < 0)
		return ret;

	freq_id = HIBYTE16(ret);
	f = LOBYTE16(ret);
	switch (freq_id) {
	case 0:
		freq = f;
		break;
	case 1:
		freq = 50;
		break;
	case 2:
		freq = 100;
		break;
	case 3:
		freq = 400;
		break;
	default:
		return -EINVAL;
	}

	return freq;
}

static int imanager_i2c_write_freq(struct imanager_io_ops *io, unsigned bus_id,
				   unsigned freq)
{
	unsigned val;

	switch (freq) {
	case 50:
		val = 0x0100;
		break;
	case 100:
		val = 0x0200;
		break;
	case 400:
		val = 0x0300;
		break;
	default:
		if (freq < 50)
			val = freq;
		else
			return -EINVAL;
	}

	return imanager_write16(io, EC_CMD_SMB_FREQ_WR, bus_id, val);
}

static int imanager_i2c_read_block(struct imanager_io_ops *io,
				   struct ec_message *msg, u8 *buf)
{
	int ret;

	if (!buf[0] || (buf[0] > I2C_MAX_READ_SIZE))
		return -EINVAL;

	ret = imanager_i2c_wr_combined(io, msg);
	if (ret < 0)
		return ret;

	buf[0] = ret;
	memcpy(&buf[1], msg->u.data, ret);

	return 0;
}

static int imanager_i2c_write_block(struct imanager_io_ops *io,
				    struct ec_message *msg, u8 *buf)
{
	if (!buf[0] || (buf[0] > I2C_MAX_WRITE_SIZE))
		return -EINVAL;

	memcpy(&msg->u.data[EC_MSG_HDR_SIZE], &buf[1], buf[0]);

	return imanager_i2c_wr_combined(io, msg);
}

static s32 imanager_i2c_xfer(struct i2c_adapter *adap, u16 addr, ushort flags,
			     char read_write, u8 command, int size,
			     union i2c_smbus_data *smb_data)
{
	struct imanager_i2c_data *data = i2c_get_adapdata(adap);
	struct imanager_device_data *imgr = data->imgr;
	struct imanager_io_ops *io = &imgr->ec.io;
	struct device *dev = data->dev;
	int val, len, ret = 0;
	u8 *buf = smb_data->block;
	struct ec_message msg = {
		.param = imgr->ec.idev.i2c.i2coem->did,
		.rlen = 0,
		.wlen = EC_MSG_HDR_SIZE,
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr <<= 1),
				.addr_high = HIADDR16(addr),
				.rlen = 0,
				.wlen = 0,
			},
		},
	};

	mutex_lock(&imgr->lock);

	switch (size) {
	case I2C_SMBUS_QUICK:
		msg.rlen = 0;
		msg.u.smb.hdr.rlen = 0;
		msg.u.smb.hdr.wlen = 1;
		ret = imanager_i2c_wr_combined(io, &msg);
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.u.smb.hdr.rlen = 1;
			msg.u.smb.hdr.wlen = 1;
			msg.u.smb.hdr.cmd = command;
			val = imanager_i2c_wr_combined(io, &msg);
			if (val < 0)
				ret = val;
		} else {
			msg.rlen = 1;
			msg.u.smb.hdr.rlen = 1;
			msg.u.smb.hdr.wlen = 0;
			val = imanager_i2c_rw_combined(io, &msg);
			if (val < 0)
				ret = val;
			else
				smb_data->byte = val;
		}
		break;
	case I2C_SMBUS_BYTE_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += 1;
			msg.u.smb.hdr.rlen = 0;
			msg.u.smb.hdr.wlen = 2;
			msg.u.smb.hdr.cmd = command;
			msg.u.smb.data[0] = smb_data->byte;
			val = imanager_i2c_wr_combined(io, &msg);
		} else {
			msg.rlen = 1;
			msg.u.smb.hdr.rlen = 1;
			msg.u.smb.hdr.wlen = 1;
			msg.u.smb.hdr.cmd = command;
			val = imanager_i2c_wr_combined(io, &msg);
		}
		if (val < 0)
			ret = val;
		else
			smb_data->byte = val;
		break;
	case I2C_SMBUS_WORD_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += 2;
			msg.u.smb.hdr.rlen = 0;
			msg.u.smb.hdr.wlen = 3;
			msg.u.smb.hdr.cmd = command;
			msg.u.smb.data[0] = LOBYTE16(smb_data->word);
			msg.u.smb.data[1] = HIBYTE16(smb_data->word);
			val = imanager_i2c_wr_combined(io, &msg);
		} else {
			msg.rlen = 2;
			msg.u.smb.hdr.rlen = 2;
			msg.u.smb.hdr.wlen = 1;
			msg.u.smb.hdr.cmd = command;
			val = imanager_i2c_wr_combined(io, &msg);
		}
		if (val < 0)
			ret = val;
		else
			smb_data->word = val;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += buf[0];
			msg.u.smb.hdr.rlen = 0;
			msg.u.smb.hdr.wlen = 1 + buf[0];
			msg.u.smb.hdr.cmd = command;
			ret = imanager_i2c_write_block(io, &msg, buf);
		} else {
			len = CHECK_RD_SIZE(buf[0]);
			msg.rlen = len;
			msg.u.smb.hdr.rlen = len;
			msg.u.smb.hdr.wlen = 1;
			msg.u.smb.hdr.cmd = command;
			/* If buf[0] == 0 EC will read I2C_MAX_READ_SIZE */
			ret = imanager_i2c_wr_combined(io, &msg);
			if (ret >= 0) {
				buf[0] = ret;
				memcpy(&buf[1], msg.u.data, ret);
				ret = 0;
			}
		}
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += buf[0];
			msg.u.smb.hdr.rlen = 0;
			msg.u.smb.hdr.wlen = 1 + buf[0];
			msg.u.smb.hdr.cmd = command;
			ret = imanager_i2c_write_block(io, &msg, buf);
		} else {
			len = CHECK_RD_SIZE(buf[0]);
			msg.rlen += len;
			msg.u.smb.hdr.rlen = len;
			msg.u.smb.hdr.wlen = 1;
			msg.u.smb.hdr.cmd = command;
			ret = imanager_i2c_read_block(io, &msg, buf);
		}
		break;
	default:
		dev_err(dev, "Unsupported transaction %d\n", size);
		ret = -EOPNOTSUPP;
	}

	mutex_unlock(&imgr->lock);

	return ret;
}

static u32 imanager_i2c_func(struct i2c_adapter *adapter)
{
	return	I2C_FUNC_SMBUS_QUICK | I2C_FUNC_SMBUS_BYTE |
		I2C_FUNC_SMBUS_BYTE_DATA | I2C_FUNC_SMBUS_WORD_DATA |
		I2C_FUNC_SMBUS_BLOCK_DATA | I2C_FUNC_SMBUS_I2C_BLOCK;
}

static const struct i2c_algorithm imanager_i2c_algorithm = {
	.smbus_xfer     = imanager_i2c_xfer,
	.functionality  = imanager_i2c_func,
};

static const struct i2c_adapter imanager_i2c_adapter = {
	.owner		= THIS_MODULE,
	.name		= "SMBus iManager adapter",
	.class		= I2C_CLASS_HWMON | I2C_CLASS_SPD,
	.algo		= &imanager_i2c_algorithm,
	.retries	= 3,
};

static int imanager_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_i2c_data *i2c;
	int ret;

	i2c = devm_kzalloc(dev, sizeof(*i2c), GFP_KERNEL);
	if (!i2c)
		return -ENOMEM;

	i2c->imgr = imgr;
	i2c->dev = dev;
	i2c->adapter = imanager_i2c_adapter;
	i2c->adapter.dev.parent = dev;

	i2c_set_adapdata(&i2c->adapter, i2c);
	platform_set_drvdata(pdev, i2c);

	ret = i2c_add_adapter(&i2c->adapter);
	if (ret) {
		dev_err(dev, "Failed to add SMBus adapter\n");
		return ret;
	}

	if (bus_frequency > 100)
		bus_frequency = 400;
	else if (bus_frequency < 50)
		bus_frequency = 50;
	else
		bus_frequency = 100;

	ret = imanager_i2c_write_freq(&imgr->ec.io, I2C_OEM_1, bus_frequency);
	if (ret < 0)
		dev_warn(dev, "Could not set SMBus frequency\n");

	ret = imanager_i2c_read_freq(&imgr->ec.io, I2C_OEM_1);
	if (ret < 0)
		dev_warn(dev, "Could not read SMBus frequency (%d)\n", ret);
	else
		dev_info(dev, "SMBus frequency: %d kHz\n", bus_frequency = ret);

	return 0;
}

static int imanager_i2c_remove(struct platform_device *pdev)
{
	struct imanager_i2c_data *data = dev_get_drvdata(&pdev->dev);

	i2c_del_adapter(&data->adapter);
	i2c_set_adapdata(&data->adapter, NULL);

	return 0;
}

static struct platform_driver imanager_i2c_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
		.owner = THIS_MODULE,
#endif
		.name  = "imanager-smbus",
	},
	.probe	= imanager_i2c_probe,
	.remove	= imanager_i2c_remove,
};

module_platform_driver(imanager_i2c_driver);

MODULE_DESCRIPTION("Advantech iManager SMBus Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-smbus");
