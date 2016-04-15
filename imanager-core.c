/*
 * Advantech iManager MFD core driver
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

#if defined(__RHEL6__)
#include <asm/byteorder.h>
#endif
#include <linux/bug.h>
#include <linux/byteorder/generic.h>
#include <linux/delay.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/swab.h>
#include <linux/types.h>
#include "compat.h"
#include "imanager.h"
#include "imanager-ec.h"

static struct platform_device *pdev;

static const char * const chip_names[] = {
	"it8516",
	"it8518",
	"it8528",
	NULL
};

static const char * const fan_temp_label[] = {
	"Temp CPU",
	"Temp SYS1",
	"Temp SYS2",
	NULL,
};

enum imanager_cells {
	IMANAGER_BACKLIGHT,
	IMANAGER_GPIO,
	IMANAGER_HWMON,
	IMANAGER_I2C,
	IMANAGER_WDT,
};

/**
 * EC I/O
 */

static int wait_ibf_cleared(void)
{
	int i = 0;

	do {
		if (!(inb(IT8516_CMD_PORT) & EC_FLAG_INBUF))
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (i++ < EC_MAX_RETRY);

	return -ETIME;
}

static int wait_obf_set(void)
{
	int i = 0;

	do {
		if (inb(IT8516_CMD_PORT) & EC_FLAG_OUTBUF)
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (i++ < EC_MAX_RETRY);

	return -ETIME;
}

static inline int ec_inb(int addr, int reg)
{
	outb(reg, addr);
	return inb(addr + 1);
}

static inline void ec_outb(int addr, int reg, int val)
{
	outb(reg, addr);
	outb(val, addr + 1);
}

static inline int ec_io28_inb(int addr, int reg)
{
	int ret;

	ret = wait_ibf_cleared();
	if (ret)
		return ret;

	/* dummy read to prevent lock */
	inb(addr - 1);

	outb(reg, addr);

	ret = wait_obf_set();
	if (ret)
		return ret;

	return inb(addr - 1);
}

static inline int ec_io28_outb(int addr, int reg, int val)
{
	int ret;

	ret = wait_ibf_cleared();
	if (ret)
		return ret;

	outb(reg, addr);

	ret = wait_ibf_cleared();
	if (ret)
		return ret;

	outb(val, addr - 1);

	return 0;
}

static inline int ec_io18_read(int cmd)
{
	return ec_inb(IT8518_CMD_PORT, cmd);
}

static inline int ec_io18_write(int cmd, int value)
{
	ec_outb(IT8518_CMD_PORT, cmd, value);

	return 0;
}

static inline int ec_io28_read(int cmd)
{
	return ec_io28_inb(IT8516_CMD_PORT, cmd + EC_CMD_OFFSET_READ);
}

static inline int ec_io28_write(int cmd, int value)
{
	return ec_io28_outb(IT8516_CMD_PORT, cmd + EC_CMD_OFFSET_WRITE, value);
}

static int ec_wait_cmd_clear(struct imanager_io_ops *io)
{
	int i = 0;

	do {
		if (!io->read(0))
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (i++ < EC_MAX_RETRY);

	return -ETIME;
}

int imanager_read_ram(struct imanager_io_ops *io, u8 bank, u8 offset, u8 len,
		      u8 *buf, u8 bufsz)
{
	int i;
	int ret;

	if (WARN_ON(!buf))
		return -EINVAL;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, bank);
	io->write(EC_MSG_OFFSET_DATA(0), offset);
	io->write(EC_MSG_OFFSET_DATA(0x2C), len);
	io->write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_RD);

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	ret = io->read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	for (i = 0; (i < len) && (len < EC_MSG_SIZE) && (len <= bufsz); i++)
		buf[i] = io->read(EC_MSG_OFFSET_DATA(i + 1));

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_read_ram);

int imanager_write_ram(struct imanager_io_ops *io, u8 bank, u8 offset, u8 len,
		       u8 *buf)
{
	int ret, i;

	if (WARN_ON(!buf))
		return -EINVAL;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, bank);
	io->write(EC_MSG_OFFSET_DATA(0), offset);
	io->write(EC_MSG_OFFSET_DATA(0x2C), len);

	for (i = 0; (i < len) && (len < EC_MSG_SIZE); i++)
		io->write(EC_MSG_OFFSET_DATA(i + 1), buf[i]);

	io->write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_WR);

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	ret = io->read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_write_ram);

static int imanager_read_device_config(struct imanager_ec_data *ec)
{
	struct imanager_io_ops *io = &ec->io;
	struct ec_message devcfg[] = {
		{	/* iManager Device ID */
			.rlen = EC_MAX_DID,
			.wlen = 0,
			.param = EC_DT_DID,
			.data = NULL,
		}, {	/* iManager Hardware Pin */
			.rlen = EC_MAX_DID,
			.wlen = 0,
			.param = EC_DT_HWP,
			.data = NULL,
		}, {	/* iManager Device Polarity */
			.rlen = EC_MAX_DID,
			.wlen = 0,
			.param = EC_DT_POL,
			.data = NULL,
		},
	};
	struct ec_dyn_devtbl *cfg;
	unsigned i, j;
	int ret;

	for (i = 0; i < ARRAY_SIZE(devcfg); i++) {
		ret = imanager_read(io, EC_CMD_DEV_TBL_RD, &devcfg[i]);
		if (ret)
			return ret;
	}

	for (i = 0; (i < EC_MAX_DID) && devcfg[0].u.data[i]; i++) {
		cfg = &ec->dyn[i];
		for (j = 0; j < ARRAY_SIZE(devtbl); j++) {
			if (devtbl[j].id == devcfg[0].u.data[i]) {
				cfg->did = devcfg[0].u.data[i];
				cfg->hwp = devcfg[1].u.data[i];
				cfg->pol = devcfg[2].u.data[i];
				cfg->devtbl = &devtbl[j];
				break;
			}
		}
	}

	return 0;
}

static int imanager_read_buffer(struct imanager_io_ops *io, u8 *data, int rlen)
{
	int ret, i, j;
	int pages = rlen % EC_I2C_BLOCK_SIZE;
	int rem = rlen / EC_I2C_BLOCK_SIZE;

	/* pre-condition: rlen <= 256 */

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	for (i = 0; i < pages; i++) {
		io->write(EC_MSG_OFFSET_PARAM, i);
		io->write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear(io);
		if (ret)
			return ret;

		ret = io->read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < EC_I2C_BLOCK_SIZE; j++)
			data[i * EC_I2C_BLOCK_SIZE + j] =
				io->read(EC_MSG_OFFSET_DATA(j));
	}

	if (rem) {
		io->write(EC_MSG_OFFSET_PARAM, pages);
		io->write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear(io);
		if (ret)
			return ret;

		ret = io->read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < rem; j++)
			data[pages * EC_I2C_BLOCK_SIZE + j] =
				io->read(EC_MSG_OFFSET_DATA(j));
	}

	return 0;
}

static int imanager_msg_trans(struct imanager_io_ops *io, u8 cmd,
			      struct ec_message *msg, bool payload)
{
	int ret, i, len;
	unsigned offset;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, msg->param);

	if (msg && msg->wlen) {
		if (!msg->data) {
			for (i = 0; i < msg->wlen; i++)
				io->write(EC_MSG_OFFSET_DATA(i),
					msg->u.data[i]);
		} else {
			for (i = 0; i < msg->wlen; i++)
				io->write(EC_MSG_OFFSET_DATA(i), msg->data[i]);
			io->write(EC_MSG_OFFSET_DATA(0x2c), msg->wlen);
		}
	}

	io->write(EC_MSG_OFFSET_CMD, cmd);
	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	/* GPIO and I2C have different success return values */
	ret = io->read(EC_MSG_OFFSET_STATUS);
	if ((ret != EC_STATUS_SUCCESS) && !(ret & EC_STATUS_CMD_COMPLETE))
		return -EIO;
	/*
	 * EC I2C may return an error code which we need to hand-off
	 * to the caller
	 */
	else if (ret & 0x007e)
		return ret;

	if (msg && msg->data) {
		ret = imanager_read_buffer(io, msg->data, msg->rlen);
		if (ret < 0)
			return ret;
	} else if (msg && msg->rlen) {
		if (msg->rlen == 0xff)
			/* Use alternate message body for hwmon */
			len = io->read(EC_MSG_OFFSET_DATA(0x2C));
		else
			len = (msg->rlen > EC_MSG_SIZE ? EC_MSG_SIZE :
			       msg->rlen);
		offset = payload ? EC_MSG_OFFSET_PAYLOAD(0) :
				   EC_MSG_OFFSET_DATA(0);
		for (i = 0; i < len; i++)
			msg->u.data[i] = io->read(offset + i);
	}

	return 0;
}

int imanager_read(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg)
{
	return imanager_msg_trans(io, cmd, msg, false);
}
EXPORT_SYMBOL_GPL(imanager_read);

int imanager_write(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg)
{
	return imanager_msg_trans(io, cmd, msg, true);
}
EXPORT_SYMBOL_GPL(imanager_write);

int imanager_read8(struct imanager_io_ops *io, u8 cmd, u8 param)
{
	int ret;
	struct ec_message msg = {
		.rlen = 1,
		.wlen = 0,
		.param = param,
		.data = NULL,
	};

	ret = imanager_read(io, cmd, &msg);
	if (ret)
		return ret;

	return msg.u.data[0];
}
EXPORT_SYMBOL_GPL(imanager_read8);

int imanager_read16(struct imanager_io_ops *io, u8 cmd, u8 param)
{
	int ret;
	struct ec_message msg = {
		.rlen = 2,
		.wlen = 0,
		.param = param,
		.data = NULL,
	};

	ret = imanager_read(io, cmd, &msg);
	if (ret)
		return ret;

	return (msg.u.data[0] << 8 | msg.u.data[1]);
}
EXPORT_SYMBOL_GPL(imanager_read16);

int imanager_write8(struct imanager_io_ops *io, u8 cmd, u8 param, u8 byte)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = 1,
		.param = param,
		.u = {
			.data = { byte, 0 },
		},
	};

	return imanager_write(io, cmd, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write8);

int imanager_write16(struct imanager_io_ops *io, u8 cmd, u8 param, u16 word)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = 2,
		.param = param,
		.u = {
			.data = { HIBYTE16(word), LOBYTE16(word), 0 },
		},
	};

	return imanager_write(io, cmd, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write16);

static inline void
ec_get_dev_attr(struct ec_dev_attr *attr, const struct ec_dyn_devtbl *tbl)
{
	attr->did = tbl->did;
	attr->hwp = tbl->hwp;
	attr->pol = tbl->pol;
	attr->scale = tbl->devtbl->scale;
	attr->label = tbl->devtbl->label;
}

static void imanager_get_gpio(struct imanager_ec_data *data)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_gpio_device *gpio = &data->idev.gpio;

	for (i = 0; i < ARRAY_SIZE(data->dyn) && data->dyn[i].did; i++) {
		dyn = &data->dyn[i];
		if (dyn->devtbl->type == GPIO) {
			switch (dyn->did) {
			case ALTGPIO0:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO1:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO2:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO3:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO4:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO5:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO6:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case ALTGPIO7:
				ec_get_dev_attr(&gpio->attr[gpio->num++], dyn);
				break;
			case BUTTON0:
			case BUTTON1:
			case BUTTON2:
			case BUTTON3:
			case BUTTON4:
			case BUTTON5:
			case BUTTON6:
			case BUTTON7:
			case POWERLED:
			case BATLEDG:
			case OEMLED0:
			case OEMLED1:
			case OEMLED2:
			case BATLEDR:
			case WDNMI:
			case BACKLIGHT1:
			case BACKLIGHT2:
				break;
			}
		}
	}
}

static void imanager_get_hwmon_adc(struct imanager_ec_data *data)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct ec_dev_adc *adc = &data->idev.hwmon.adc;

	for (i = 0; i < ARRAY_SIZE(data->dyn) && data->dyn[i].did; i++) {
		dyn = &data->dyn[i];
		if (dyn->devtbl->type == ADC) {
			switch (dyn->did) {
			case ADC12VS0:
			case ADC12VS0_2:
			case ADC12VS0_10:
				ec_get_dev_attr(&adc->attr[0], dyn);
				adc->num++;
				break;
			case ADC5VS5:
			case ADC5VS5_2:
			case ADC5VS5_10:
				ec_get_dev_attr(&adc->attr[1], dyn);
				adc->num++;
				break;
			case CMOSBAT:
			case CMOSBAT_2:
			case CMOSBAT_10:
				ec_get_dev_attr(&adc->attr[2], dyn);
				adc->num++;
				break;
			case VCOREA:
			case ADC5VS0:
			case ADC5VS0_2:
			case ADC5VS0_10:
				ec_get_dev_attr(&adc->attr[3], dyn);
				adc->num++;
				break;
			case CURRENT:
			case ADC33VS0:
			case ADC33VS0_2:
			case ADC33VS0_10:
				ec_get_dev_attr(&adc->attr[4], dyn);
				adc->num++;
				break;
			}
		}
	}
}

static void imanager_get_hwmon_fan(struct imanager_ec_data *data)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct ec_dev_fan *fan = &data->idev.hwmon.fan;

	for (i = 0; i < ARRAY_SIZE(data->dyn) && data->dyn[i].did; i++) {
		dyn = &data->dyn[i];
		if ((dyn->devtbl->type == TACH) ||
		    (dyn->devtbl->type == PWM)) {
			switch (dyn->did) {
			case CPUFAN_2P:
			case CPUFAN_4P:
				fan->temp_label[fan->num] = fan_temp_label[0];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			case SYSFAN1_2P:
			case SYSFAN1_4P:
				fan->temp_label[fan->num] = fan_temp_label[1];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			case SYSFAN2_2P:
			case SYSFAN2_4P:
				fan->temp_label[fan->num] = fan_temp_label[2];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			case TACHO0:
			case TACHO1:
			case TACHO2:
			case BRIGHTNESS:
			case BRIGHTNESS2:
			case PCBEEP:
				break;
			}
		}
	}
}

static void imanager_get_i2c(struct imanager_ec_data *data)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_i2c_device *i2c = &data->idev.i2c;

	for (i = 0; i < ARRAY_SIZE(data->dyn) && data->dyn[i].did; i++) {
		dyn = &data->dyn[i];
		if (dyn->devtbl->type == SMB) {
			switch (dyn->did) {
			case SMBEEPROM:
				ec_get_dev_attr(&i2c->attr[0], dyn);
				i2c->eeprom = &i2c->attr[0];
				i2c->num++;
				break;
			case I2COEM:
				ec_get_dev_attr(&i2c->attr[1], dyn);
				i2c->i2coem = &i2c->attr[1];
				i2c->num++;
				break;
			case SMBOEM0:
			case SMBOEM1:
			case SMBOEM2:
			case SMBTHERMAL0:
			case SMBTHERMAL1:
			case SMBSECEEP:
			case SMBEEP2K:
			case OEMEEP:
			case OEMEEP2K:
			case PECI:
			case SMBOEM3:
			case SMLINK:
			case SMBSLV:
			case SMARTBAT1:
			case SMARTBAT2:
				break;
			}
		}
	}
}

static void imanager_get_backlight(struct imanager_ec_data *data)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_backlight_device *bl = &data->idev.bl;

	for (i = 0; i < ARRAY_SIZE(data->dyn) && data->dyn[i].did; i++) {
		dyn = &data->dyn[i];
		if (dyn->devtbl->type == PWM) {
			switch (dyn->did) {
			case BRIGHTNESS:
				ec_get_dev_attr(&bl->attr[0], dyn);
				bl->brightness[0] = EC_ACPIRAM_BRIGHTNESS1;
				bl->num++;
				break;
			case BRIGHTNESS2:
				ec_get_dev_attr(&bl->attr[1], dyn);
				bl->brightness[1] = EC_ACPIRAM_BRIGHTNESS2;
				bl->num++;
				break;
			case CPUFAN_2P:
			case CPUFAN_4P:
			case SYSFAN1_2P:
			case SYSFAN1_4P:
			case SYSFAN2_2P:
			case SYSFAN2_4P:
			case PCBEEP:
			case TACHO0:
			case TACHO1:
			case TACHO2:
				break;
			}
		}
	}
}

static void imanager_get_wdt(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_watchdog_device *wdt = &ec->idev.wdt;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
		if (dyn->devtbl->type == IRQ) {
			switch (dyn->did) {
			case WDIRQ:
				ec_get_dev_attr(&wdt->attr[0], dyn);
				wdt->irq = &wdt->attr[0];
				wdt->num++;
				break;
			case WDNMI:
				ec_get_dev_attr(&wdt->attr[1], dyn);
				wdt->nmi = &wdt->attr[1];
				wdt->num++;
				break;
			}
		}
	}
}

static int imanager_read_firmware_version(struct imanager_ec_data *ec)
{
	struct ec_info *info = &ec->idev.info;
	struct ec_version *version = &info->version;
	struct imanager_io_ops *io = &ec->io;
	struct ec_message msg = {
		.rlen = ARRAY_SIZE(info->pcb_name),
		.wlen = 0,
		.param = 0,
		.data = NULL,
	};
	struct ec_version_raw ver;
	unsigned raw, pos = PCB_NAME_MAX_SIZE;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FW_RELEASE_RD,
				sizeof(ver), (u8 *)&ver, sizeof(ver));
	if (ret < 0)
		return ret;

	raw = swab16(ver.kernel);
	version->kernel_major = EC_KERNEL_MAJOR(raw);
	version->kernel_minor = EC_KERNEL_MINOR(raw);

	raw = swab16(ver.firmware);
	version->firmware_major = EC_FIRMWARE_MAJOR(raw);
	version->firmware_minor = EC_FIRMWARE_MINOR(raw);

	raw = swab16(ver.project_code);
	version->type = EC_PROJECT_CODE(raw);

	ret = imanager_read(io, EC_CMD_FW_INFO_RD, &msg);
	if (ret)
		return ret;

	/*
	 * Sadly, the string is not Null-terminated so we will need to read a
	 * fixed amount of chars. There is, apparently, no exact definition
	 * of board name (SOM6867 vs. SOM-6867).
	 */
	memcpy(info->pcb_name, msg.u.data, PCB_NAME_MAX_SIZE);
	pos = strchr(info->pcb_name, '-') ? pos : pos - 1;
	info->pcb_name[pos] = '\0';

	return 0;
}

static inline void ec_clear_ports(void)
{
	/* Prevent firmware lock */
	inb(IT8516_DAT_PORT);
	inb(IT8518_DAT_PORT);
}

static int imanager_ec_init(struct imanager_ec_data *ec)
{
	int ret;

	ec_clear_ports();

	ret = imanager_read_firmware_version(ec);
	if (ret)
		return ret;

	ret = imanager_read_device_config(ec);
	if (ret)
		return ret;

	imanager_get_backlight(ec);
	imanager_get_gpio(ec);
	imanager_get_hwmon_adc(ec);
	imanager_get_hwmon_fan(ec);
	imanager_get_i2c(ec);
	imanager_get_wdt(ec);

	return 0;
}

static inline unsigned ec_read_chipid(u16 addr)
{
	return (ec_inb(addr, DEVID_REG_MSB) << 8 |
		ec_inb(addr, DEVID_REG_LSB));
}

static int imanager_check_system(struct imanager_ec_data *ec)
{
	int chipid = ec_read_chipid(EC_BASE_ADDR);

	if (chipid == EC_DEVID_IT8518) {
		ec->io.read = ec_io18_read;
		ec->io.write = ec_io18_write;
		ec->idev.info.chipid = chipid;
		ec->idev.info.kind = IT8518;
	} else if (chipid == EC_DEVID_IT8528) {
		ec->io.read = ec_io28_read;
		ec->io.write = ec_io28_write;
		ec->idev.info.chipid = chipid;
		ec->idev.info.kind = IT8528;
	} else {
		return -ENODEV;
	}

	imanager_ec_init(ec);

	return 0;
}

/*
 * Devices which are part of the iManager and are available via firmware.
 */
static struct mfd_cell imanager_devs[] = {
	[IMANAGER_BACKLIGHT] = {
		.name = "imanager-backlight",
	},
	[IMANAGER_GPIO] = {
		.name = "imanager-gpio",
	},
	[IMANAGER_HWMON] = {
		.name = "imanager_hwmon",
	},
	[IMANAGER_I2C] = {
		.name = "imanager-i2c",
	},
	[IMANAGER_WDT] = {
		.name = "imanager-wdt",
	},
};

static struct resource imanager_ioresource = {
	.start  = IT8516_DAT_PORT,
	.end    = IT8518_DAT_PORT,
	.flags  = IORESOURCE_IO,
};

const char *project_type_to_str(int type)
{
	const char *version_type;

	switch (type) {
	case 'V':
		version_type = "Release";
		break;
	case 'X':
		version_type = "Engineering Sample";
		break;
	case 'A' ... 'U':
		version_type = "Custom";
		break;
	default:
		version_type = "Unknown";
		break;
	}

	return version_type;
}

static ssize_t
imanager_name_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%s\n", info->pcb_name);
}

static ssize_t
imanager_kversion_show(struct device *dev, struct device_attribute *attr,
		       char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 info->version.kernel_major,
			 info->version.kernel_minor);
}

static ssize_t
imanager_fwversion_show(struct device *dev, struct device_attribute *attr,
		        char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 info->version.firmware_major,
			 info->version.firmware_minor);
}

static ssize_t
imanager_type_show(struct device *dev,
		   struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			project_type_to_str(info->version.type));
}

static ssize_t
imanager_chip_name_show(struct device *dev,
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
	struct imanager_platform_data platdata;
	int err;

	pdev = platform_device_alloc("imanager-core", -1);
	if (!pdev)
		return -ENOMEM;

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
	err = mfd_add_devices(&pdev->dev, pdev->id, imanager_devs,
		ARRAY_SIZE(imanager_devs), NULL, -1);
#else
	err = mfd_add_devices(&pdev->dev, pdev->id, imanager_devs,
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
	struct imanager_device_data *data;
	struct ec_info *info;
	int ret;

	if (!pdev) {
		dev_err(dev, "Invalid platform data\n");
		return -EINVAL;
	}

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->dev = dev;
	mutex_init(&data->lock);

	ret = imanager_check_system(&data->ec);
	if (ret < 0)
		return ret;

	platform_set_drvdata(pdev, data);

	info = &data->ec.idev.info;
	pdata->chip_name = chip_names[info->kind];

	dev_info(dev, "Found Advantech iManager %s - %s %d.%d/%d.%d (%s)\n",
		 pdata->chip_name,
		 info->pcb_name,
		 info->version.kernel_major,
		 info->version.kernel_minor,
		 info->version.firmware_major,
		 info->version.firmware_minor,
		 project_type_to_str(info->version.type));

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
		.name  = "imanager-core",
	},
	.probe	= imanager_probe,
	.remove	= imanager_remove,
};

static int __init imanager_init(void)
{
	int ret;

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
