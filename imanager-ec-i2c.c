/*
 * Advantech iManager I2C bus core
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
#include <linux/mfd/imanager/ec.h>
#include <linux/mfd/imanager/i2c.h>

#define I2C_SMBUS_BLOCK_SIZE	32

#define EVAL_WR_SIZE(x)	\
		(x < I2C_SMBUS_BLOCK_SIZE ? x : I2C_SMBUS_BLOCK_SIZE - 1)
#define EVAL_RD_SIZE(x) \
		(x && (x <= I2C_SMBUS_BLOCK_SIZE) ? x : I2C_SMBUS_BLOCK_SIZE)

#define EC_HWRAM_OFFSET_STATUS	0UL

#define I2C_ERR_PROTO		0x19UL
#define I2C_ERR_TIMEOUT		0x18UL
#define I2C_ERR_ACCESS		0x17UL
#define I2C_ERR_UNKNOWN		0x13UL
#define I2C_ERR_ADDR_NACK	0x10UL

#define EC_SMB_DID(N)		(0x28 + N)

struct i2c_data {
	struct ec_info info;
	struct i2c_cfg cfg;
};

struct ec_i2c_status {
	u32 error	: 7;
	u32 complete	: 1;
};

static struct i2c_data i2c;

static int i2c_core_eval_status(u8 _status)
{
	struct ec_i2c_status *status = (struct ec_i2c_status *)&_status;
	int err = 0;

	switch (status->error) {
	case 0:
		break;
	case I2C_ERR_ADDR_NACK:
		err = -ENODEV;
		break;
	case I2C_ERR_ACCESS:
	case I2C_ERR_UNKNOWN:
		err = -EAGAIN;
		break;
	case I2C_ERR_TIMEOUT:
		err = -ETIME;
		break;
	case I2C_ERR_PROTO:
		err = -EPROTO;
		break;
	default:
		pr_err("Undefined status code 0x%02X\n", status->error);
		err = -EIO;
		break;
	}

	return err;
}

static inline int i2c_send_message(u8 cmd, u8 param, struct ec_message *msg)
{
	int err;

	err = imanager_msg_write(cmd, param, msg);
	if (err)
		return i2c_core_eval_status(err);

	return 0;
}

static int i2c_core_blk_wr_rw_combined(u8 proto, struct ec_message *msg)
{
	int err;

	if (WARN_ON(!msg))
		return -EINVAL;

	err = imanager_wait_proc_complete(EC_HWRAM_OFFSET_STATUS, 0);
	if (err)
		return err;

	err = i2c_send_message(proto, i2c.cfg.i2coem, msg);
	if (err)
		return err;

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

static inline int i2c_core_wr_combined(struct ec_message *msg)
{
	return i2c_core_blk_wr_rw_combined(EC_CMD_I2C_WR, msg);
}

static inline int i2c_core_rw_combined(struct ec_message *msg)
{
	return i2c_core_blk_wr_rw_combined(EC_CMD_I2C_RW, msg);
}

/*
 * iManager I2C core API
 */
int i2c_core_write_quick(u16 addr)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 0,
				.wlen = 1,
			},
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_read_byte(u16 addr)
{
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 1,
				.wlen = 0,
			},
		},
	};

	return i2c_core_rw_combined(&msg);
}

int i2c_core_write_byte(u16 addr, u8 cmd)
{
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 1,
				.wlen = 1,
				.cmd  = cmd,
			},
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_read_byte_data(u16 addr, u8 cmd)
{
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 1,
				.wlen = 1,
				.cmd  = cmd,
			},
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_write_byte_data(u16 addr, u8 cmd, u8 value)
{
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header) + 1,
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 0,
				.wlen = 2,
				.cmd  = cmd,
			},
			.smb.data[0] = value,
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_read_word_data(u16 addr, u8 cmd)
{
	struct ec_message msg = {
		.rlen = 2,
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 2,
				.wlen = 1,
				.cmd  = cmd,
			},
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_write_word_data(u16 addr, u8 cmd, u16 value)
{
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header) + 2,
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 0,
				.wlen = 3,
				.cmd  = cmd,
			},
			.smb.data[0] = LOBYTE16(value),
			.smb.data[1] = HIBYTE16(value),
		},
	};

	return i2c_core_wr_combined(&msg);
}

int i2c_core_write_block_data(u16 addr, u8 cmd, u8 *buf)
{
	int i;
	struct ec_message msg = {
		.rlen = 1,
		.wlen = sizeof(struct ec_message_header) +
			EVAL_WR_SIZE(buf[0]),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = 0,
				.wlen = 1 + EVAL_WR_SIZE(buf[0]),
				.cmd  = cmd,
			},
		},
	};

	if ((buf[0] == 0) || (buf[0] >= I2C_MAX_WRITE_BYTES)) {
		pr_err("Invalid write length %d\n", buf[0]);
		return -EINVAL;
	}

	for (i = 0; i < EVAL_WR_SIZE(buf[0]); i++)
		msg.u.data[i + sizeof(struct ec_message_header)] = buf[i + 1];

	return i2c_core_wr_combined(&msg);
}

int i2c_core_read_block_data(u16 addr, u8 cmd, u8 *buf)
{
	int i;
	int ret;
	struct ec_message msg = {
		.rlen = EVAL_RD_SIZE(buf[0]),
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = EVAL_RD_SIZE(buf[0]),
				.wlen = 1,
				.cmd  = cmd,
			},
		},
	};

	/*
	 * If buf[0] == 0 EC will read I2C_MAX_READ_BYTES
	 */
	ret = i2c_core_wr_combined(&msg);
	if (ret < 0) {
		pr_err("I2C transaction failed\n");
		return ret;
	}

	buf[0] = ret;
	for (i = 0; i < ret; i++)
		buf[i + 1] = msg.u.data[i];

	return 0;
}

int i2c_core_read_i2c_block_data(u16 addr, u8 cmd, u8 *buf)
{
	int i;
	int ret;
	struct ec_message msg = {
		.rlen = EVAL_RD_SIZE(buf[0]),
		.wlen = sizeof(struct ec_message_header),
		.u = {
			.smb.hdr = {
				.addr_low  = LOADDR16(addr),
				.addr_high = HIADDR16(addr),
				.rlen = EVAL_RD_SIZE(buf[0]),
				.wlen = 1,
				.cmd  = cmd,
			},
		},
	};

	if ((buf[0] == 0) || (buf[0] > I2C_MAX_READ_BYTES)) {
		pr_err("Invalid read length\n");
		return -EINVAL;
	}

	ret = i2c_core_wr_combined(&msg);
	if (ret < 0) {
		pr_err("I2C transaction failed\n");
		return ret;
	}

	buf[0] = ret;
	for (i = 0; i < ret; i++)
		buf[i + 1] = msg.u.data[i];

	return 0;
}

int i2c_core_write_i2c_block_data(u16 addr, u8 cmd, u8 *buf)
{
	if (WARN_ON(!buf))
		return -EINVAL;

	return i2c_core_write_block_data(addr, cmd, buf);
}

int i2c_core_smb_get_freq(u32 bus_id)
{
	int val = 0, f;
	int freq_id, freq;
	int chipid = imanager_get_chipid();

	if (WARN_ON(bus_id > I2C_OEM_1))
		return -EINVAL;

	switch (chipid) {
	case SIO_DEVID_IT8518:
	case SIO_DEVID_IT8528:
		val = imanager_read_word(EC_CMD_SMB_FREQ_RD,
					 EC_SMB_DID(bus_id));
		if (val < 0) {
			pr_err("Failed to get bus frequency\n");
			return -EIO;
		}

		freq_id = HIBYTE16(val);
		f = LOBYTE16(val);
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
		break;
	default:
		pr_err("EC version not supported!\n");
		return -ENODEV;
	}

	return freq;
}

int i2c_core_smb_set_freq(u32 bus_id, u32 freq)
{
	int err;
	u16 val;
	int chipid = imanager_get_chipid();

	if (WARN_ON(bus_id > I2C_OEM_1))
		return -EINVAL;

	switch (chipid) {
	case SIO_DEVID_IT8518:
	case SIO_DEVID_IT8528:
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
			else {
				pr_err("Out-of-range frequency (%d)\n", freq);
				return -EINVAL;
			}
		}

		err = imanager_write_word(EC_CMD_SMB_FREQ_WR,
					  EC_SMB_DID(bus_id), val);
		if (err) {
			pr_err("Failed to set bus frequency\n");
			return err;
		}
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

int i2c_core_init(void)
{
	int ret;

	memset(&i2c, 0, sizeof(i2c));

	ret = imanager_get_fw_info(&i2c.info);
	if (ret)
		return ret;

	return imanager_get_i2c_cfg(&i2c.cfg);
}

void i2c_core_release(void)
{
	memset(&i2c, 0, sizeof(i2c));
}

