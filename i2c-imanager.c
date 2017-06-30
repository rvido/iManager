/*
 * Advantech iManager SMBus bus driver
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

#include <linux/device.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include "compat.h"
#include "imanager.h"

#define I2C_SMBUS_BLOCK_SIZE	32UL
#define I2C_MAX_READ_SIZE	I2C_SMBUS_BLOCK_SIZE
#define I2C_MAX_WRITE_SIZE	(I2C_SMBUS_BLOCK_SIZE - 1)

#define EC_HWRAM_OFFSET_STATUS	0UL

#define I2C_ERR_PROTO		0x19UL
#define I2C_ERR_TIMEOUT		0x18UL
#define I2C_ERR_ACCESS		0x17UL
#define I2C_ERR_UNKNOWN		0x13UL
#define I2C_ERR_ADDR_NACK	0x10UL

#define SMBUS_FREQ_50KHZ	0x0100
#define SMBUS_FREQ_100KHZ	0x0200
#define SMBUS_FREQ_400KHZ	0x0300

struct ec_i2c_status {
	u32 error	: 7;
	u32 complete	: 1;
};

struct adapter_info {
	struct i2c_adapter adapter;
	int smb_devid;
};

struct imanager_i2c_data {
	struct device *dev;
	struct imanager_device_data *imgr;
	struct adapter_info adap_info[EC_MAX_SMB_NUM];
	int nadap;
};

static int imanager_i2c_wait_proc_complete(struct imanager_device_data *imgr)
{
	int ret, retries = EC_MAX_RETRIES;
	u8 val;

	do {
		ret = imanager_mem_read(imgr, EC_RAM_HW, EC_HWRAM_OFFSET_STATUS,
					&val, sizeof(val));
		if (ret < 0)
			return ret;

		if (!val)
			return 0;

		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (retries--);

	return -ETIME;
}

static inline int imanager_i2c_eval_error(int error)
{
	switch (error) {
	case I2C_ERR_ADDR_NACK:
		return -ENXIO;
	case I2C_ERR_ACCESS:
		return -EACCES;
	case I2C_ERR_UNKNOWN:
		return -EAGAIN;
	case I2C_ERR_TIMEOUT:
		return -ETIME;
	case I2C_ERR_PROTO:
		return -EPROTO;
	}

	return -EIO;
}

static int imanager_i2c_block_wr_rw_combined(struct imanager_device_data *imgr,
					     struct imanager_ec_message *msg)
{
	int ret;
	struct ec_i2c_status *status = (struct ec_i2c_status *) &ret;

	ret = imanager_i2c_wait_proc_complete(imgr);
	if (ret)
		return ret;

	ret = imanager_write(imgr, msg);
	if (status->error)
		return imanager_i2c_eval_error(status->error);

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

static inline int
imanager_i2c_wr_combined(struct imanager_device_data *imgr,
			 struct imanager_ec_message *msg)
{
	msg->cmd = EC_CMD_I2C_WR;
	return imanager_i2c_block_wr_rw_combined(imgr, msg);
}

static inline int
imanager_i2c_rw_combined(struct imanager_device_data *imgr,
			 struct imanager_ec_message *msg)
{
	msg->cmd = EC_CMD_I2C_RW;
	return imanager_i2c_block_wr_rw_combined(imgr, msg);
}

static inline int
imanager_i2c_write_freq(struct imanager_device_data *imgr, int did, int freq)
{
	return imanager_write16(imgr, EC_CMD_SMB_FREQ_WR, did, freq);
}

static inline int eval_read_len(int len)
{
	return len && len <= I2C_MAX_READ_SIZE ? len : I2C_MAX_READ_SIZE;
}

static inline int
imanager_i2c_read_block(struct imanager_device_data *imgr,
			struct imanager_ec_message *msg, u8 *buf)
{
	int ret;

	ret = imanager_i2c_wr_combined(imgr, msg);
	if (ret < 0)
		return ret;

	buf[0] = ret;
	memcpy(&buf[1], msg->u.data, ret);

	return 0;
}

static inline int
imanager_i2c_write_block(struct imanager_device_data *imgr,
			 struct imanager_ec_message *msg, u8 *buf)
{
	if (!buf[0] || (buf[0] > I2C_MAX_WRITE_SIZE))
		return -EINVAL;

	memcpy(&msg->u.data[EC_MSG_HDR_SIZE], &buf[1], buf[0]);

	return imanager_i2c_wr_combined(imgr, msg);
}

static s32 imanager_i2c_xfer(struct i2c_adapter *adap, u16 addr, ushort flags,
			     char read_write, u8 command, int size,
			     union i2c_smbus_data *smb_data)
{
	struct imanager_i2c_data *data = i2c_get_adapdata(adap);
	struct imanager_device_data *imgr = data->imgr;
	struct device *dev = data->dev;
	int smb_devid = *(int *)adap->algo_data;
	int val, ret = 0;
	u16 addr16 = addr << 1;
	u8 *buf = smb_data->block;
	struct imanager_ec_message msg = {
		.rlen = 0,
		.wlen = EC_MSG_HDR_SIZE,
		.param = smb_devid,
		.u = {
			.smb.hdr = {
				.addr_low  = addr16 & 0x00ff,
				.addr_high = addr16 >> 8,
				.rlen = 0,
				.wlen = 0,
			},
		},
	};
	struct imanager_ec_smb_message *smb = &msg.u.smb;

	switch (size) {
	case I2C_SMBUS_QUICK:
		msg.rlen = 0;
		smb->hdr.rlen = 0;
		smb->hdr.wlen = 1;
		ret = imanager_i2c_wr_combined(imgr, &msg);
		break;
	case I2C_SMBUS_BYTE:
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			smb->hdr.rlen = 1;
			smb->hdr.wlen = 1;
			smb->hdr.cmd = command;
			val = imanager_i2c_wr_combined(imgr, &msg);
			if (val < 0)
				ret = val;
		} else {
			if (!smb_data) {
				ret = -EINVAL;
				break;
			}
			msg.rlen = 1;
			smb->hdr.rlen = 1;
			smb->hdr.wlen = 0;
			val = imanager_i2c_rw_combined(imgr, &msg);
			if (val < 0)
				ret = val;
			else
				smb_data->byte = val;
			break;
		}
	case I2C_SMBUS_BYTE_DATA:
		if (!smb_data) {
			ret = -EINVAL;
			break;
		}
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += 1;
			smb->hdr.rlen = 0;
			smb->hdr.wlen = 2;
			smb->hdr.cmd = command;
			smb->data[0] = smb_data->byte;
			val = imanager_i2c_wr_combined(imgr, &msg);
		} else {
			msg.rlen = 1;
			smb->hdr.rlen = 1;
			smb->hdr.wlen = 1;
			smb->hdr.cmd = command;
			val = imanager_i2c_wr_combined(imgr, &msg);
		}
		if (val < 0)
			ret = val;
		else
			smb_data->byte = val;
		break;
	case I2C_SMBUS_WORD_DATA:
		if (!smb_data) {
			ret = -EINVAL;
			break;
		}
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += 2;
			smb->hdr.rlen = 0;
			smb->hdr.wlen = 3;
			smb->hdr.cmd = command;
			smb->data[0] = smb_data->word & 0x00ff;
			smb->data[1] = smb_data->word >> 8;
			val = imanager_i2c_wr_combined(imgr, &msg);
		} else {
			msg.rlen = 2;
			smb->hdr.rlen = 2;
			smb->hdr.wlen = 1;
			smb->hdr.cmd = command;
			val = imanager_i2c_wr_combined(imgr, &msg);
		}
		if (val < 0)
			ret = val;
		else
			smb_data->word = val;
		break;
	case I2C_SMBUS_BLOCK_DATA:
		if (!smb_data) {
			ret = -EINVAL;
			break;
		}
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += buf[0];
			smb->hdr.rlen = 0;
			smb->hdr.wlen = 1 + buf[0];
			smb->hdr.cmd = command;
			ret = imanager_i2c_write_block(imgr, &msg, buf);
		} else {
			msg.rlen = eval_read_len(buf[0]);
			smb->hdr.rlen = msg.rlen;
			smb->hdr.wlen = 1;
			smb->hdr.cmd = command;
			ret = imanager_i2c_read_block(imgr, &msg, buf);
		}
		break;
	case I2C_SMBUS_I2C_BLOCK_DATA:
		if (!smb_data) {
			ret = -EINVAL;
			break;
		}
		if (read_write == I2C_SMBUS_WRITE) {
			msg.rlen = 1;
			msg.wlen += buf[0];
			smb->hdr.rlen = 0;
			smb->hdr.wlen = 1 + buf[0];
			smb->hdr.cmd = command;
			ret = imanager_i2c_write_block(imgr, &msg, buf);
		} else {
			msg.rlen = eval_read_len(buf[0]);
			smb->hdr.rlen = msg.rlen;
			smb->hdr.wlen = 1;
			smb->hdr.cmd = command;
			ret = imanager_i2c_read_block(imgr, &msg, buf);
		}
		break;
	default:
		dev_err(dev, "Unsupported transaction %d\n", size);
		ret = -EOPNOTSUPP;
	}

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

static const struct i2c_adapter imanager_i2c_adapters[] = {
	[SMB_EEP] = {
		.owner	= THIS_MODULE,
		.name	= "iManager SMB EEP adapter",
		.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD,
		.algo	= &imanager_i2c_algorithm,
	},
	[I2C_OEM] = {
		.owner	= THIS_MODULE,
		.name	= "iManager I2C OEM adapter",
		.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD,
		.algo	= &imanager_i2c_algorithm,
	},
	[SMB_1] = {
		.owner	= THIS_MODULE,
		.name	= "iManager SMB 1 adapter",
		.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD,
		.algo	= &imanager_i2c_algorithm,
	},
	[SMB_PECI] = {
		.owner	= THIS_MODULE,
		.name	= "iManager SMB PECI adapter",
		.class	= I2C_CLASS_HWMON | I2C_CLASS_SPD,
		.algo	= &imanager_i2c_algorithm,
	},
};

static int
imanager_i2c_add_bus(struct imanager_i2c_data *i2c, struct adapter_info *info,
		     const struct i2c_adapter *adap, int did, int freq)
{
	int ret;

	info->adapter = *adap;
	info->adapter.dev.parent = i2c->dev;
	info->smb_devid = did;
	info->adapter.algo_data = &info->smb_devid;
	i2c_set_adapdata(&info->adapter, i2c);

	ret = i2c_add_adapter(&info->adapter);
	if (ret) {
		dev_warn(i2c->dev, "Failed to add %s\n", info->adapter.name);
		return ret;
	}

	ret = imanager_i2c_write_freq(i2c->imgr, did, freq);
	if (ret < 0)
		dev_warn(i2c->dev, "Failed to set bus frequency of %s\n",
			 info->adapter.name);

	return 0;
}

static int imanager_i2c_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_device_attribute **attr = imgr->ec.i2c.attr;
	struct imanager_i2c_data *data;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->imgr = imgr;
	data->dev = dev;

	if (attr[SMB_EEP])
		imanager_i2c_add_bus(data, &data->adap_info[data->nadap++],
				     &imanager_i2c_adapters[SMB_EEP],
				     attr[SMB_EEP]->did, SMBUS_FREQ_100KHZ);

	if (attr[I2C_OEM])
		imanager_i2c_add_bus(data, &data->adap_info[data->nadap++],
				     &imanager_i2c_adapters[I2C_OEM],
				     attr[I2C_OEM]->did, SMBUS_FREQ_400KHZ);

	if (attr[SMB_1])
		imanager_i2c_add_bus(data, &data->adap_info[data->nadap++],
				     &imanager_i2c_adapters[SMB_1],
				     attr[SMB_1]->did, SMBUS_FREQ_100KHZ);

	if (attr[SMB_PECI])
		imanager_i2c_add_bus(data, &data->adap_info[data->nadap++],
				     &imanager_i2c_adapters[SMB_PECI],
				     attr[SMB_PECI]->did, SMBUS_FREQ_100KHZ);

	platform_set_drvdata(pdev, data);

	return 0;
}

static int imanager_i2c_remove(struct platform_device *pdev)
{
	struct imanager_i2c_data *i2c = dev_get_drvdata(&pdev->dev);
	int i;

	for (i = 0; i < i2c->nadap; i++) {
		i2c_del_adapter(&i2c->adap_info[i].adapter);
		i2c_set_adapdata(&i2c->adap_info[i].adapter, NULL);
	}

	return 0;
}

static struct platform_driver imanager_i2c_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
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
