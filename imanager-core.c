/*
 * Advantech iManager MFD driver
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
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include <linux/swab.h>
#include <linux/types.h>
#include "imanager.h"
#include "imanager-ec.h"
#include "compat.h"

static struct platform_device *imanager_pdev;

static const char * const chip_names[] = {
	"it8518",
	"it8528",
	NULL
};

static const char * const fan_temp_labels[] = {
	"Temp CPU",
	"Temp SYS1",
	"Temp SYS2",
	NULL,
};

/**
 * EC I/O
 */

static inline int wait_inbuf_cleared(void)
{
	int i = 0;

	do {
		if (!(inb(IT8528_CMD_PORT) & EC_FLAG_INBUF))
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (i++ < EC_MAX_RETRY);

	return -ETIME;
}

static inline int wait_outbuf_set(void)
{
	int i = 0;

	do {
		if (inb(IT8528_CMD_PORT) & EC_FLAG_OUTBUF)
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

	ret = wait_inbuf_cleared();
	if (ret)
		return ret;

	/* dummy read to prevent lock */
	inb(addr - 1);

	outb(reg, addr);

	ret = wait_outbuf_set();
	if (ret)
		return ret;

	return inb(addr - 1);
}

static inline int ec_io28_outb(int addr, int reg, int val)
{
	int ret;

	ret = wait_inbuf_cleared();
	if (ret)
		return ret;

	outb(reg, addr);

	ret = wait_inbuf_cleared();
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
	return ec_io28_inb(IT8528_CMD_PORT, cmd + EC_CMD_OFFSET_READ);
}

static inline int ec_io28_write(int cmd, int value)
{
	return ec_io28_outb(IT8528_CMD_PORT, cmd + EC_CMD_OFFSET_WRITE, value);
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

int imanager_read_ram(struct imanager_io_ops *io, int ram_type, int offset,
		      u8 *data, u8 size)
{
	int i, j, ret;

	if (WARN_ON(!data))
		return -EINVAL;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, ram_type);
	io->write(EC_MSG_OFFSET_DATA(0), offset);
	io->write(EC_MSG_OFFSET_DATA(0x2C), size);
	io->write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_RD);

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	ret = io->read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	for (i = 0, j = EC_MSG_OFFSET_DATA(1); i < size; i++, j++)
		data[i] = io->read(j);

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_read_ram);

int imanager_write_ram(struct imanager_io_ops *io, int ram_type, int offset,
		       u8 *data, u8 size)
{
	int i, j, ret;

	if (WARN_ON(!data))
		return -EINVAL;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, ram_type);
	io->write(EC_MSG_OFFSET_DATA(0), offset);
	io->write(EC_MSG_OFFSET_DATA(0x2C), size);

	for (i = 0, j = EC_MSG_OFFSET_DATA(1); i < size; i++, j++)
		io->write(j, data[i]);

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

static inline void data_to_ec(struct imanager_io_ops *io, u8* data, u8 size)
{
	int i, j;

	for (i = 0, j = EC_MSG_OFFSET_DATA(0); i < size; i++, j++)
		io->write(j, data[i]);
}

static inline void
data_from_ec(struct imanager_io_ops *io, u8* data, u8 size, int offset)
{
	int i;

	for (i = 0; i < size; i++, offset++)
		data[i] = io->read(offset);
}

static int imanager_msg_xfer(struct imanager_io_ops *io, u8 cmd,
			     struct ec_message *msg, bool payload)
{
	int ret;
	int offset = EC_MSG_OFFSET_DATA(0);

	if (WARN_ON(!msg))
		return -EINVAL;

	ret = ec_wait_cmd_clear(io);
	if (ret)
		return ret;

	io->write(EC_MSG_OFFSET_PARAM, msg->param);

	if (msg->wlen) {
		if (msg->data) {
			data_to_ec(io, msg->data, msg->wlen);
			io->write(EC_MSG_OFFSET_DATA(0x2c), msg->wlen);
		} else {
			data_to_ec(io, msg->u.data, msg->wlen);
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
	 * EC I2C may return an error code which we need to handoff
	 * to the caller
	 */
	else if (ret & 0x007e)
		return ret;

	if (msg->rlen) {
		if (msg->rlen == EC_MSG_FLAG_HWMON)
			msg->rlen = io->read(EC_MSG_OFFSET_DATA(0x2C));
		if (payload) /* i2c, hwmon, wdt */
			offset = EC_MSG_OFFSET_PAYLOAD(0);
		data_from_ec(io, msg->u.data, msg->rlen, offset);
	}

	return 0;
}

int imanager_read(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg)
{
	return imanager_msg_xfer(io, cmd, msg, false);
}
EXPORT_SYMBOL_GPL(imanager_read);

int imanager_write(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg)
{
	return imanager_msg_xfer(io, cmd, msg, true);
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

static void imanager_get_gpio(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_gpio_device *gpio = &ec->idev.gpio;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
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
			default:
				break;
			}
		}
	}
	if (gpio->num)
		ec->idev.features |= IMANAGER_FEATURE_GPIO;
}

static void imanager_get_hwmon_adc(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct ec_dev_adc *adc = &ec->idev.hwmon.adc;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
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
			default:
				break;
			}
		}
	}
	if (adc->num)
		ec->idev.features |= IMANAGER_FEATURE_HWMON_ADC;
}

static void imanager_get_hwmon_fan(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct ec_dev_fan *fan = &ec->idev.hwmon.fan;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
		if ((dyn->devtbl->type == TACH) ||
		    (dyn->devtbl->type == PWM)) {
			switch (dyn->did) {
			case CPUFAN_2P:
			case CPUFAN_4P:
				fan->temp_label[fan->num] = fan_temp_labels[0];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			case SYSFAN1_2P:
			case SYSFAN1_4P:
				fan->temp_label[fan->num] = fan_temp_labels[1];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			case SYSFAN2_2P:
			case SYSFAN2_4P:
				fan->temp_label[fan->num] = fan_temp_labels[2];
				ec_get_dev_attr(&fan->attr[fan->num++], dyn);
				break;
			default:
				break;
			}
		}
	}
	if (fan->num)
		ec->idev.features |= IMANAGER_FEATURE_HWMON_FAN;
}

static void imanager_get_i2c(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_i2c_device *i2c = &ec->idev.i2c;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
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
			default:
				break;
			}
		}
	}
	if (i2c->num)
		ec->idev.features |= IMANAGER_FEATURE_SMBUS;
}

static void imanager_get_backlight(struct imanager_ec_data *ec)
{
	size_t i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_backlight_device *bl = &ec->idev.bl;

	for (i = 0; i < ARRAY_SIZE(ec->dyn) && ec->dyn[i].did; i++) {
		dyn = &ec->dyn[i];
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
			default:
				break;
			}
		}
	}
	if (bl->num)
		ec->idev.features |= IMANAGER_FEATURE_BACKLIGHT;
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
			default:
				break;
			}
		}
	}
	if (wdt->num)
		ec->idev.features |= IMANAGER_FEATURE_WDT;
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
	unsigned val, pos = PCB_NAME_MAX_SIZE;
	int ret;

	ret = imanager_read_ram(io, EC_RAM_ACPI, EC_ACPIRAM_FW_RELEASE_RD,
				(u8 *)&ver, sizeof(ver));
	if (ret < 0)
		return ret;

	val = swab16(ver.kernel);
	version->kernel_major = EC_KERNEL_MAJOR(val);
	version->kernel_minor = EC_KERNEL_MINOR(val);

	val = swab16(ver.firmware);
	version->firmware_major = EC_FIRMWARE_MAJOR(val);
	version->firmware_minor = EC_FIRMWARE_MINOR(val);

	val = swab16(ver.project_code);
	version->type = EC_PROJECT_CODE(val);

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

static int imanager_ec_init(struct imanager_ec_data *ec)
{
	int ret;

	/* Clear ports to prevent firmware lock */
	inb(IT8528_DAT_PORT);
	inb(IT8518_DAT_PORT);

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

enum imanager_cells {
	IMANAGER_BACKLIGHT = 0,
	IMANAGER_GPIO,
	IMANAGER_HWMON,
	IMANAGER_SMB,
	IMANAGER_WDT,
};

/*
 * Devices which are part of the iManager and are available via firmware.
 */
static const struct mfd_cell imanager_devs[] = {
	[IMANAGER_BACKLIGHT] = {
		.name = "imanager_bl",
	},
	[IMANAGER_GPIO] = {
		.name = "imanager-gpio",
	},
	[IMANAGER_HWMON] = {
		.name = "imanager_hwmon",
	},
	[IMANAGER_SMB] = {
		.name = "imanager-smbus",
	},
	[IMANAGER_WDT] = {
		.name = "imanager_wdt",
	},
};

static int imanager_register_cells(struct imanager_device_data *imgr)
{
	struct imanager_device *idev = &imgr->ec.idev;
	struct mfd_cell devs[ARRAY_SIZE(imanager_devs)];
	int i = 0;

	if (idev->features & IMANAGER_FEATURE_BACKLIGHT)
		devs[i++] = imanager_devs[IMANAGER_BACKLIGHT];

	if (idev->features & IMANAGER_FEATURE_GPIO)
		devs[i++] = imanager_devs[IMANAGER_GPIO];

	if (idev->features & IMANAGER_FEATURE_HWMON_ADC)
		devs[i++] = imanager_devs[IMANAGER_HWMON];

	if (idev->features & IMANAGER_FEATURE_SMBUS)
		devs[i++] = imanager_devs[IMANAGER_SMB];

	if (idev->features & IMANAGER_FEATURE_WDT)
		devs[i++] = imanager_devs[IMANAGER_WDT];

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,6,0)
	return mfd_add_devices(imgr->dev, -1, devs, i, NULL, 0);
#else
	return mfd_add_devices(imgr->dev, -1, devs, i, NULL, 0, NULL);
#endif
}

static struct resource imanager_ioresource = {
	.start  = IT8528_DAT_PORT,
	.end    = IT8518_DAT_PORT,
	.flags  = IORESOURCE_IO,
};

const char *project_type_to_str(int type)
{
	const char *version_type;

	switch (type) {
	case 'V':
		version_type = "release";
		break;
	case 'X':
		version_type = "debug";
		break;
	case 'A' ... 'U':
		version_type = "custom";
		break;
	default:
		version_type = "unknown";
		break;
	}

	return version_type;
}

static ssize_t
imanager_board_show(struct device *dev, struct device_attribute *attr,
		    char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%s\n", info->pcb_name);
}

static ssize_t imanager_kernel_show(struct device *dev,
				    struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_version *version = &data->ec.idev.info.version;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n", version->kernel_major,
			 version->kernel_minor);
}

static ssize_t imanager_firmware_show(struct device *dev,
				      struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_version *version = &data->ec.idev.info.version;

	return scnprintf(buf, PAGE_SIZE, "%d.%d\n",
			 version->firmware_major,
			 version->firmware_minor);
}

static ssize_t
imanager_type_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_version *version = &data->ec.idev.info.version;

	return scnprintf(buf, PAGE_SIZE, "%s\n",
			 project_type_to_str(version->type));
}

static ssize_t
imanager_chip_show(struct device *dev, struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	const struct ec_info *info = &data->ec.idev.info;

	return scnprintf(buf, PAGE_SIZE, "%s\n", chip_names[info->kind]);
}

static DEVICE_ATTR(imanager_board, S_IRUGO, imanager_board_show, NULL);
static DEVICE_ATTR(imanager_kernel, S_IRUGO, imanager_kernel_show, NULL);
static DEVICE_ATTR(imanager_firmware, S_IRUGO, imanager_firmware_show, NULL);
static DEVICE_ATTR(imanager_type, S_IRUGO, imanager_type_show, NULL);
static DEVICE_ATTR(imanager_chip, S_IRUGO, imanager_chip_show, NULL);

static struct attribute *imanager_attributes[] = {
	&dev_attr_imanager_board.attr,
	&dev_attr_imanager_kernel.attr,
	&dev_attr_imanager_firmware.attr,
	&dev_attr_imanager_type.attr,
	&dev_attr_imanager_chip.attr,
	NULL
};

static const struct attribute_group imanager_attr_group = {
	.attrs = imanager_attributes,
};

static int imanager_platform_create(void)
{
	int ret;

	imanager_pdev = platform_device_alloc("imanager", -1);
	if (!imanager_pdev)
		return -ENOMEM;

	/* No platform device data required */

	ret = platform_device_add_resources(imanager_pdev,
					    &imanager_ioresource, 1);
	if (ret)
		goto err;

	ret = platform_device_add(imanager_pdev);
	if (ret)
		goto err;

	return 0;
err:
	platform_device_put(imanager_pdev);
	return ret;
}

static inline int ec_read_chipid(u16 addr)
{
	return (ec_inb(addr, EC_DID_REG_MSB) << 8 |
		ec_inb(addr, EC_DID_REG_LSB));
}

static int imanager_detect_device(struct imanager_device_data *imgr)
{
	struct imanager_ec_data *ec = &imgr->ec;
	struct device *dev = imgr->dev;
	struct ec_info *info = &imgr->ec.idev.info;
	int chipid = ec_read_chipid(EC_BASE_ADDR);
	int ret;

	if (chipid == CHIP_ID_IT8518) {
		ec->io.read  = ec_io18_read;
		ec->io.write = ec_io18_write;
		info->chipid = chipid;
		info->kind   = IT8518;
	} else if (chipid == CHIP_ID_IT8528) {
		ec->io.read  = ec_io28_read;
		ec->io.write = ec_io28_write;
		info->chipid = chipid;
		info->kind   = IT8528;
	}

	ret = imanager_ec_init(ec);
	if (ret) {
		dev_err(dev, "iManager firmware communication error\n");
		return ret;
	}

	dev_info(dev, "Found Advantech iManager %s - %s %d.%d/%d.%d (%s)\n",
		 chip_names[info->kind], info->pcb_name,
		 info->version.kernel_major, info->version.kernel_minor,
		 info->version.firmware_major, info->version.firmware_minor,
		 project_type_to_str(info->version.type));

	ret = sysfs_create_group(&dev->kobj, &imanager_attr_group);
	if (ret)
		return ret;

	ret = imanager_register_cells(imgr);
	if (ret)
		sysfs_remove_group(&dev->kobj, &imanager_attr_group);

	return ret;
}

static int imanager_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr;

	imgr = devm_kzalloc(dev, sizeof(*imgr), GFP_KERNEL);
	if (!imgr)
		return -ENOMEM;

	imgr->dev = dev;
	mutex_init(&imgr->lock);

	platform_set_drvdata(pdev, imgr);

	return imanager_detect_device(imgr);
}

static int imanager_remove(struct platform_device *pdev)
{
	sysfs_remove_group(&pdev->dev.kobj, &imanager_attr_group);
	mfd_remove_devices(&pdev->dev);

	return 0;
}

static struct platform_driver imanager_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,19,0)
		.owner = THIS_MODULE,
#endif
		.name  = "imanager",
	},
	.probe	= imanager_probe,
	.remove	= imanager_remove,
};

static int __init imanager_init(void)
{
	int chipid = ec_read_chipid(EC_BASE_ADDR);
	int ret;

	/* Check for the presence of the EC chip */
	if ((chipid != CHIP_ID_IT8518) && (chipid != CHIP_ID_IT8528))
		return -ENODEV;

	ret = imanager_platform_create();
	if (ret)
		return ret;

	return platform_driver_register(&imanager_driver);
}

static void __exit imanager_exit(void)
{
	if (imanager_pdev)
		platform_device_unregister(imanager_pdev);

	platform_driver_unregister(&imanager_driver);
}

module_init(imanager_init);
module_exit(imanager_exit);

MODULE_DESCRIPTION("Advantech iManager Core Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-core");
