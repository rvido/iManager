/*
 * Advantech iManager MFD driver
 * Partially derived from kempld-core
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

#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/mfd/core.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/platform_device.h>
#include <linux/string.h>
#include "compat.h"
#include "imanager.h"
#include "imanager-ec.h"

/* iManager flags */
#define IMANAGER_FEATURE_BACKLIGHT	BIT(0)
#define IMANAGER_FEATURE_GPIO		BIT(1)
#define IMANAGER_FEATURE_HWMON_ADC	BIT(2)
#define IMANAGER_FEATURE_HWMON_FAN	BIT(3)
#define IMANAGER_FEATURE_SMBUS		BIT(4)
#define IMANAGER_FEATURE_WDT		BIT(5)

enum kinds { IT8518, IT8528 };

static struct platform_device *imanager_pdev;

static const char * const chip_names[] = {
	"it8518",
	"it8528",
	NULL
};

static const struct imanager_ec_device ecdev_table[] = {
	/* GPIO */
	{ IMANAGER_EC_DEVICE(GPIO0, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO1, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO2, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO3, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO4, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO5, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO6, GPIO, -1) },
	{ IMANAGER_EC_DEVICE(GPIO7, GPIO, -1) },
	/* FAN */
	{ IMANAGER_EC_DEVICE(CPUFAN_2P,  PWM, 2) },
	{ IMANAGER_EC_DEVICE(CPUFAN_4P,  PWM, 4) },
	{ IMANAGER_EC_DEVICE(SYSFAN1_2P, PWM, 2) },
	{ IMANAGER_EC_DEVICE(SYSFAN1_4P, PWM, 4) },
	{ IMANAGER_EC_DEVICE(SYSFAN2_2P, PWM, 2) },
	{ IMANAGER_EC_DEVICE(SYSFAN2_4P, PWM, 4) },
	/* ADC */
	{ IMANAGER_EC_DEVICE(ADC12VS0,    ADC, 1) },
	{ IMANAGER_EC_DEVICE(ADC12VS0_2,  ADC, 2) },
	{ IMANAGER_EC_DEVICE(ADC12VS0_10, ADC, 10) },
	{ IMANAGER_EC_DEVICE(ADC5VS0,     ADC, 1) },
	{ IMANAGER_EC_DEVICE(ADC5VS0_2,   ADC, 2) },
	{ IMANAGER_EC_DEVICE(ADC5VS0_10,  ADC, 10) },
	{ IMANAGER_EC_DEVICE(ADC5VS5,     ADC, 1) },
	{ IMANAGER_EC_DEVICE(ADC5VS5_2,   ADC, 2) },
	{ IMANAGER_EC_DEVICE(ADC5VS5_10,  ADC, 10) },
	{ IMANAGER_EC_DEVICE(ADC33VS0,    ADC, 1) },
	{ IMANAGER_EC_DEVICE(ADC33VS0_2,  ADC, 2) },
	{ IMANAGER_EC_DEVICE(ADC33VS0_10, ADC, 10) },
	{ IMANAGER_EC_DEVICE(CMOSBAT,     ADC, 1) },
	{ IMANAGER_EC_DEVICE(CMOSBAT_2,   ADC, 2) },
	{ IMANAGER_EC_DEVICE(CMOSBAT_10,  ADC, 10) },
	{ IMANAGER_EC_DEVICE(VCOREA,      ADC, 1) },
	{ IMANAGER_EC_DEVICE(CURRENT,     ADC, 1) },
	/* I2C/SMBus */
	{ IMANAGER_EC_DEVICE(SMBEEPROM,   SMB, -1) },
	{ IMANAGER_EC_DEVICE(I2COEM,      SMB, -1) },
	{ IMANAGER_EC_DEVICE(SMBOEM0,     SMB, -1) },
	{ IMANAGER_EC_DEVICE(SMBPECI,     SMB, -1) },
	/* Backlight/Brightness */
	{ IMANAGER_EC_DEVICE(BRIGHTNESS,  PWM, -1) },
	{ IMANAGER_EC_DEVICE(BRIGHTNESS2, PWM, -1) },
	/* Watchdog */
	{ IMANAGER_EC_DEVICE(WDIRQ, IRQ,  -1) },
	{ IMANAGER_EC_DEVICE(WDNMI, GPIO, -1) },
	{ }
};

/**
 * iManager I/O
 */

enum imanager_io_buffer_status { IS_CLEARED = 0, IS_SET };

#define CHECK_BIT(reg, bit) ((reg) & (bit))

static inline int check_io28_ready(uint bit, uint state)
{
	int ret, retries = EC_MAX_RETRIES;

	do {
		ret = inb(IT8528_CMD_PORT);
		if (CHECK_BIT(ret, bit) == state)
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (retries--);

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

	ret = check_io28_ready(EC_IO28_INBUF, IS_CLEARED);
	if (ret)
		return ret;

	/* prevent firmware lock */
	inb(addr - 1);

	outb(reg, addr);

	ret = check_io28_ready(EC_IO28_OUTBUF, IS_SET);
	if (ret)
		return ret;

	return inb(addr - 1);
}

static inline int ec_io28_outb(int addr, int reg, int val)
{
	int ret;

	ret = check_io28_ready(EC_IO28_INBUF, IS_CLEARED);
	if (ret)
		return ret;

	outb(reg, addr);

	ret = check_io28_ready(EC_IO28_INBUF, IS_CLEARED);
	if (ret)
		return ret;

	outb(val, addr - 1);

	return 0;
}

static int ec_io18_read(int cmd)
{
	return ec_inb(IT8518_CMD_PORT, cmd);
}

static int ec_io18_write(int cmd, int value)
{
	ec_outb(IT8518_CMD_PORT, cmd, value);

	return 0;
}

static int ec_io28_read(int cmd)
{
	return ec_io28_inb(IT8528_CMD_PORT, cmd + EC_CMD_OFFSET_READ);
}

static int ec_io28_write(int cmd, int value)
{
	return ec_io28_outb(IT8528_CMD_PORT, cmd + EC_CMD_OFFSET_WRITE, value);
}

static int imanager_check_ec_ready(struct imanager_io_ops *io)
{
	int retries = EC_MAX_RETRIES;

	do {
		if (!io->read(EC_CMD_CHK_RDY))
			return 0;
		usleep_range(EC_DELAY_MIN, EC_DELAY_MAX);
	} while (retries--);

	return -ETIME;
}

/**
 * iManager Device Configuration
 */

static void imanager_add_attribute(struct imanager_ec_data *ec,
				   struct imanager_device_attribute *attr)
{
	struct imanager_gpio_device *gpio = &ec->gpio;
	struct imanager_adc_device *adc = &ec->hwmon.adc;
	struct imanager_fan_device *fan = &ec->hwmon.fan;
	struct imanager_i2c_device *i2c = &ec->i2c;
	struct imanager_backlight_device *bl = &ec->bl;
	struct imanager_watchdog_device *wdt = &ec->wdt;

	switch (attr->ecdev->type) {
	case GPIO:
		switch (attr->did) {
		case GPIO0:
		case GPIO1:
		case GPIO2:
		case GPIO3:
		case GPIO4:
		case GPIO5:
		case GPIO6:
		case GPIO7:
			gpio->attr[gpio->num++] = attr;
			break;
		case WDNMI:
			wdt->attr[1] = attr;
			wdt->num++;
			break;
		}
	case ADC:
		switch (attr->did) {
		case ADC12VS0:
		case ADC12VS0_2:
		case ADC12VS0_10:
			adc->attr[0] = attr;
			adc->label[0] = "+12VS0";
			adc->num++;
			break;
		case ADC5VS5:
		case ADC5VS5_2:
		case ADC5VS5_10:
			adc->attr[1] = attr;
			adc->label[1] = "+5VS0";
			adc->num++;
			break;
		case CMOSBAT:
		case CMOSBAT_2:
		case CMOSBAT_10:
			adc->attr[2] = attr;
			adc->label[2] = "+3.3VS0";
			adc->num++;
			break;
		case VCOREA:
		case ADC5VS0:
		case ADC5VS0_2:
		case ADC5VS0_10:
			adc->attr[3] = attr;
			adc->num++;
			break;
		case CURRENT:
		case ADC33VS0:
		case ADC33VS0_2:
		case ADC33VS0_10:
			adc->attr[4] = attr;
			adc->num++;
			break;
		}
	case PWM:
		switch (attr->did) {
		case CPUFAN_2P:
		case CPUFAN_4P:
			fan->attr[0] = attr;
			fan->label[0] = "FAN CPU";
			fan->temp_label[0] = "Temp CPU";
			fan->num++;
			break;
		case SYSFAN1_2P:
		case SYSFAN1_4P:
			fan->attr[1] = attr;
			fan->label[1] = "FAN SYS1";
			fan->temp_label[1] = "Temp SYS1";
			fan->num++;
			break;
		case SYSFAN2_2P:
		case SYSFAN2_4P:
			fan->attr[2] = attr;
			fan->label[2] = "FAN SYS2";
			fan->temp_label[2] = "Temp SYS2";
			fan->num++;
			break;
		case BRIGHTNESS:
			bl->attr[0] = attr;
			bl->brightness[0] = EC_OFFSET_BRIGHTNESS1;
			bl->num++;
			break;
		case BRIGHTNESS2:
			bl->attr[1] = attr;
			bl->brightness[1] = EC_OFFSET_BRIGHTNESS2;
			bl->num++;
			break;
		}
	case SMB:
		switch (attr->did) {
		case SMBEEPROM:
			i2c->attr[SMB_EEP] = attr;
			i2c->num++;
			break;
		case I2COEM:
			i2c->attr[I2C_OEM] = attr;
			i2c->num++;
			break;
		case SMBOEM0:
			i2c->attr[SMB_1] = attr;
			i2c->num++;
			break;
		case SMBPECI:
			i2c->attr[SMB_PECI] = attr;
			i2c->num++;
			break;
		}
	case IRQ:
		if (attr->did == WDIRQ) {
			wdt->attr[0] = attr;
			wdt->num++;
			break;
		}
	}
}

enum imanager_device_table_type { DEVID = 0, HWPIN, POLARITY };

static int imanager_read_device_config(struct imanager_device_data *imgr)
{
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_ec_message msgs[] = {
		{ IMANAGER_MSG(EC_MAX_DID, 0, DEVID, EC_CMD_DEVTBL_RD) },
		{ IMANAGER_MSG(EC_MAX_DID, 0, HWPIN, EC_CMD_DEVTBL_RD) },
		{ IMANAGER_MSG(EC_MAX_DID, 0, POLARITY, EC_CMD_DEVTBL_RD) },
	};
	struct imanager_device_attribute *attr;
	int i, j, ret;

	/* Read iManager device configurations */
	for (i = 0; i < ARRAY_SIZE(msgs); i++) {
		ret = imanager_read(imgr, &msgs[i]);
		if (ret)
			return ret;
	}

	/* Generate iManager device atributes */
	for (i = 0; i < EC_MAX_DID && msgs[DEVID].u.data[i]; i++) {
		attr = &ec->attr[i];
		for (j = 0; j < ARRAY_SIZE(ecdev_table); j++) {
			if (ecdev_table[j].did == msgs[DEVID].u.data[i]) {
				attr->did = msgs[DEVID].u.data[i];
				attr->hwp = msgs[HWPIN].u.data[i];
				attr->pol = msgs[POLARITY].u.data[i];
				attr->ecdev = &ecdev_table[j];
				imanager_add_attribute(ec, attr);
				break;
			}
		}
	}

	if (ec->gpio.num)
		ec->features |= IMANAGER_FEATURE_GPIO;
	if (ec->hwmon.adc.num)
		ec->features |= IMANAGER_FEATURE_HWMON_ADC;
	if (ec->hwmon.fan.num)
		ec->features |= IMANAGER_FEATURE_HWMON_FAN;
	if (ec->i2c.num)
		ec->features |= IMANAGER_FEATURE_SMBUS;
	if (ec->bl.num)
		ec->features |= IMANAGER_FEATURE_BACKLIGHT;
	if (ec->wdt.num)
		ec->features |= IMANAGER_FEATURE_WDT;

	return 0;
}

static const char *project_code_to_str(unsigned int code)
{
	switch ((char)code) {
	case 'V':
		return "release";
	case 'X':
		return "debug";
	case 'A' ... 'U':
	case 'Y':
	case 'Z':
		return "custom";
	}

	return "unspecified";
}

static int imanager_read_firmware_version(struct imanager_device_data *imgr)
{
	char pcb_name[IMANAGER_PCB_NAME_LEN] = { 0 };
	struct imanager_info *info = &imgr->ec.info;
	struct imanager_ec_message msg = {
		.rlen = ARRAY_SIZE(pcb_name) - 1,
		.wlen = 0,
		.param = 0,
		.cmd = EC_CMD_FW_INFO_RD,
		.data = pcb_name,
	};
	struct imanager_ec_version ver;
	unsigned int val;
	int ret;

	ret = imanager_mem_read(imgr, EC_RAM_ACPI, EC_OFFSET_FW_RELEASE,
				(u8 *)&ver, sizeof(ver));
	if (ret < 0)
		return ret;

	val = cpu_to_be16(ver.kernel);
	info->kernel_major = EC_KERNEL_MAJOR(val);
	info->kernel_minor = EC_KERNEL_MINOR(val);

	val = cpu_to_be16(ver.firmware);
	info->firmware_major = EC_FIRMWARE_MAJOR(val);
	info->firmware_minor = EC_FIRMWARE_MINOR(val);

	val = cpu_to_be16(ver.project_code);
	info->type = project_code_to_str(EC_PROJECT_CODE(val));

	/*
	 * The PCB name string, in some FW releases, is not Null-terminated,
	 * so we need to read a fixed amount of chars. Also, the name length
	 * may vary by one char (SOM6867 vs. SOM-6867).
	 */
	ret = imanager_read(imgr, &msg);
	if (ret)
		return ret;

	if (!strchr(pcb_name, '-'))
		pcb_name[IMANAGER_PCB_NAME_LEN - 2] = '\0';

	return scnprintf(info->version, sizeof(info->version),
			 "%s_k%d.%d_f%d.%d_%s", pcb_name, info->kernel_major,
			 info->kernel_minor, info->firmware_major,
			 info->firmware_minor, info->type);
}

static int imanager_ec_init(struct imanager_device_data *imgr)
{
	int ret;

	/* Prevent firmware lock */
	inb(IT8528_DAT_PORT);
	inb(IT8518_DAT_PORT);

	ret = imanager_read_firmware_version(imgr);
	if (ret < 0)
		return ret;

	return imanager_read_device_config(imgr);
}

static inline void
data_to_ec(struct imanager_device_data *imgr, u8 *data, u8 len, int offset)
{
	int i = 0;

	while (i < len)
		imgr->ec.iop.write(offset++, data[i++]);
}

static inline void
data_from_ec(struct imanager_device_data *imgr, u8 *data, u8 len, int offset)
{
	int i = 0;

	while (i < len)
		data[i++] = imgr->ec.iop.read(offset++);
}

static inline int set_offset(bool payload)
{
	return payload ? EC_MSG_OFFSET_PAYLOAD : EC_MSG_OFFSET_DATA;
}

static int imanager_msg_xfer(struct imanager_device_data *imgr,
			     struct imanager_ec_message *msg, bool payload)
{
	int ret;

	ret = imanager_check_ec_ready(&imgr->ec.iop);
	if (ret)
		return ret;

	imgr->ec.iop.write(EC_MSG_OFFSET_PARAM, msg->param);

	if (msg->wlen) {
		if (msg->data) {
			data_to_ec(imgr, msg->data, msg->wlen,
				   EC_MSG_OFFSET_DATA);
			imgr->ec.iop.write(EC_MSG_OFFSET_LEN, msg->wlen);
		} else {
			data_to_ec(imgr, msg->u.data, msg->wlen,
				   EC_MSG_OFFSET_DATA);
		}
	}

	/* Execute command */
	imgr->ec.iop.write(EC_MSG_OFFSET_CMD, msg->cmd);
	ret = imanager_check_ec_ready(&imgr->ec.iop);
	if (ret)
		return ret;

	/* GPIO and I2C have different success return values */
	ret = imgr->ec.iop.read(EC_MSG_OFFSET_STATUS);
	if ((ret != EC_F_SUCCESS) && !(ret & EC_F_CMD_COMPLETE))
		return -EFAULT;
	/*
	 * EC I2C may return an error code which we need to handoff
	 * to the caller
	 */
	else if (ret & EC_I2C_STATUS_MASK)
		return ret;

	if (msg->rlen) {
		if (msg->rlen == EC_F_HWMON_MSG)
			msg->rlen = imgr->ec.iop.read(EC_MSG_OFFSET_LEN);
		if (msg->data)
			data_from_ec(imgr, msg->data, msg->rlen,
				     set_offset(payload));
		else
			data_from_ec(imgr, msg->u.data, msg->rlen,
				     set_offset(payload));
	}

	return 0;
}

static int imanager_mem_rw(struct imanager_device_data *imgr,
			   struct imanager_ec_message *msg)
{
	int ret;

	if (!msg->data || !(msg->rlen != msg->wlen))
		return -EINVAL;

	ret = imanager_check_ec_ready(&imgr->ec.iop);
	if (ret)
		return ret;

	if (msg->rlen) {
		imgr->ec.iop.write(EC_MSG_OFFSET_LEN, msg->rlen);
	} else if (msg->wlen) {
		data_to_ec(imgr, msg->data, msg->wlen, EC_MSG_OFFSET_RAM_DATA);
		imgr->ec.iop.write(EC_MSG_OFFSET_LEN, msg->wlen);
	}

	imgr->ec.iop.write(EC_MSG_OFFSET_PARAM, msg->param);
	imgr->ec.iop.write(EC_MSG_OFFSET_DATA, msg->u.data[0]);
	imgr->ec.iop.write(EC_MSG_OFFSET_CMD, msg->cmd);
	ret = imanager_check_ec_ready(&imgr->ec.iop);
	if (ret)
		return ret;

	ret = imgr->ec.iop.read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_F_SUCCESS)
		return -EFAULT;

	if (msg->rlen)
		data_from_ec(imgr, msg->data, msg->rlen,
			     EC_MSG_OFFSET_RAM_DATA);

	return 0;
}

/**
 * imanager_mem_read - read 'len' amount of data @ 'offset' of 'ram_type'
 * @imgr:	imanager_device_data structure describing the iManager
 * @ram_type:	RAM type such as ACPI, HW, or EXternal
 * @offset:	offset within the RAM segment
 * @data:	data pointer
 * @len:	data length
 */
int imanager_mem_read(struct imanager_device_data *imgr, int ram_type,
		      u8 offset, u8 *data, u8 len)
{
	int ret;
	struct imanager_ec_message msg = {
		.rlen = len,
		.wlen = 0,
		.param = ram_type,
		.cmd = EC_CMD_RAM_RD,
		.u.data[0] = offset,
		.data = data,
	};

	mutex_lock(&imgr->lock);
	ret = imanager_mem_rw(imgr, &msg);
	mutex_unlock(&imgr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(imanager_mem_read);

/**
 * imanager_mem_write - write 'len' amount of data @ 'offset' of 'ram_type'
 * @imgr:	imanager_device_data structure describing the iManager
 * @mem_type:	Type of memory such as ACPI, HW, or EXternal
 * @offset:	offset within the RAM segment
 * @data:	data pointer
 * @len:	data length
 */
int imanager_mem_write(struct imanager_device_data *imgr, int mem_type,
		       u8 offset, u8 *data, u8 len)
{
	int ret;
	struct imanager_ec_message msg = {
		.rlen = 0,
		.wlen = len,
		.param = mem_type,
		.cmd = EC_CMD_RAM_WR,
		.u.data[0] = offset,
		.data = data,
	};

	mutex_lock(&imgr->lock);
	ret = imanager_mem_rw(imgr, &msg);
	mutex_unlock(&imgr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(imanager_mem_write);

/**
 * imanager_read - read data through request/response messaging
 * @imgr:	imanager_device_data structure describing the iManager
 * @msg:	imanager_ec_message structure holding the message
 */
int imanager_read(struct imanager_device_data *imgr,
		  struct imanager_ec_message *msg)
{
	int ret;

	mutex_lock(&imgr->lock);
	ret = imanager_msg_xfer(imgr, msg, false);
	mutex_unlock(&imgr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(imanager_read);

/**
 * imanager_write - write data through request/response messaging
 * @imgr:	imanager_device_data structure describing the iManager
 * @msg:	imanager_ec_message structure holding the message
 */
int imanager_write(struct imanager_device_data *imgr,
		   struct imanager_ec_message *msg)
{
	int ret;

	mutex_lock(&imgr->lock);
	ret = imanager_msg_xfer(imgr, msg, true);
	mutex_unlock(&imgr->lock);

	return ret;
}
EXPORT_SYMBOL_GPL(imanager_write);

/**
 * imanager_read8 - read 8-bit data
 * @imgr:	imanager_device_data structure describing the iManager
 * @cmd:	imanager EC firmware command
 * @param:	parameter depening on cmd - device ID, offset or unit number
 */
int imanager_read8(struct imanager_device_data *imgr, u8 cmd, u8 param)
{
	int ret;
	struct imanager_ec_message msg = {
		.rlen = 1,
		.wlen = 0,
		.param = param,
		.cmd = cmd,
		.data = NULL,
	};

	ret = imanager_read(imgr, &msg);
	if (ret)
		return ret;

	return msg.u.data[0];
}
EXPORT_SYMBOL_GPL(imanager_read8);

/**
 * imanager_read16 - read 16-bit data
 * @imgr:	imanager_device_data structure describing the iManager
 * @cmd:	imanager EC firmware command
 * @param:	parameter depening on cmd - device ID, offset or unit number
 */
int imanager_read16(struct imanager_device_data *imgr, u8 cmd, u8 param)
{
	int ret;
	struct imanager_ec_message msg = {
		.rlen = 2,
		.wlen = 0,
		.param = param,
		.cmd = cmd,
		.data = NULL,
	};

	ret = imanager_read(imgr, &msg);
	if (ret)
		return ret;

	return (msg.u.data[0] << 8 | msg.u.data[1]);
}
EXPORT_SYMBOL_GPL(imanager_read16);

/**
 * imanager_write8 - write 8-bit data
 * @imgr:	imanager_device_data structure describing the iManager
 * @cmd:	imanager EC firmware command
 * @param:	parameter depening on cmd - device ID, offset or unit number
 * @byte:	8-bit data
 */
int imanager_write8(struct imanager_device_data *imgr, u8 cmd, u8 param,
		    u8 byte)
{
	struct imanager_ec_message msg = {
		.rlen = 0,
		.wlen = 1,
		.param = param,
		.cmd = cmd,
		.u = {
			.data = { byte, 0 },
		},
	};

	return imanager_write(imgr, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write8);

/**
 * imanager_write16 - write 16-bit data
 * @imgr:	imanager_device_data structure describing the iManager
 * @cmd:	imanager EC firmware command
 * @param:	parameter depening on cmd - device ID, offset or unit number
 * @word:	16-bit data
 */
int imanager_write16(struct imanager_device_data *imgr, u8 cmd, u8 param,
		     u16 word)
{
	struct imanager_ec_message msg = {
		.rlen = 0,
		.wlen = 2,
		.param = param,
		.cmd = cmd,
		.u = {
			.data = { (word >> 8), (word & 0xff), 0 },
		},
	};

	return imanager_write(imgr, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write16);

enum imanager_cells {
	IMANAGER_BACKLIGHT = 0,
	IMANAGER_GPIO,
	IMANAGER_HWMON,
	IMANAGER_SMB,
	IMANAGER_WDT,
};

/**
 * iManager devices which are available via firmware.
 */

static const struct mfd_cell imanager_devs[] = {
	[IMANAGER_BACKLIGHT] = {
		.name = "imanager-backlight",
	},
	[IMANAGER_GPIO] = {
		.name = "imanager-gpio",
	},
	[IMANAGER_HWMON] = {
		.name = "imanager-hwmon",
	},
	[IMANAGER_SMB] = {
		.name = "imanager-smbus",
	},
	[IMANAGER_WDT] = {
		.name = "imanager-wdt",
	},
};

static int imanager_register_cells(struct imanager_device_data *imgr)
{
	struct imanager_ec_data *ec = &imgr->ec;
	struct mfd_cell devs[ARRAY_SIZE(imanager_devs)];
	int i = 0;

	if (ec->features & IMANAGER_FEATURE_BACKLIGHT)
		devs[i++] = imanager_devs[IMANAGER_BACKLIGHT];

	if (ec->features & IMANAGER_FEATURE_GPIO)
		devs[i++] = imanager_devs[IMANAGER_GPIO];

	if (ec->features & IMANAGER_FEATURE_HWMON_ADC)
		devs[i++] = imanager_devs[IMANAGER_HWMON];

	if (ec->features & IMANAGER_FEATURE_SMBUS)
		devs[i++] = imanager_devs[IMANAGER_SMB];

	if (ec->features & IMANAGER_FEATURE_WDT)
		devs[i++] = imanager_devs[IMANAGER_WDT];

#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 6, 0)
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

static ssize_t imanager_version_show(struct device *dev,
				     struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);
	struct imanager_info *info = &data->ec.info;

	return scnprintf(buf, PAGE_SIZE, "%s\n", info->version);
}

static ssize_t imanager_chip_show(struct device *dev,
				  struct device_attribute *attr, char *buf)
{
	struct imanager_device_data *data = dev_get_drvdata(dev);

	return scnprintf(buf, PAGE_SIZE, "%s\n", data->ec.chip_name);
}

static DEVICE_ATTR(imanager_version, 0444, imanager_version_show, NULL);
static DEVICE_ATTR(imanager_chip, 0444, imanager_chip_show, NULL);

static struct attribute *imanager_attributes[] = {
	&dev_attr_imanager_version.attr,
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
	return (ec_inb(addr, CHIP_DEVID_MSB) << 8 |
		ec_inb(addr, CHIP_DEVID_LSB));
}

static int imanager_detect_device(struct imanager_device_data *imgr)
{
	struct imanager_ec_data *ec = &imgr->ec;
	struct imanager_info *info = &imgr->ec.info;
	int chipid = ec_read_chipid(EC_BASE_ADDR);
	int ret;

	if (chipid == CHIP_ID_IT8518) {
		ec->iop.read	= ec_io18_read;
		ec->iop.write	= ec_io18_write;
		ec->chip_name	= chip_names[IT8518];
	} else if (chipid == CHIP_ID_IT8528) {
		ec->iop.read	= ec_io28_read;
		ec->iop.write	= ec_io28_write;
		ec->chip_name	= chip_names[IT8528];
	}

	ret = imanager_ec_init(imgr);
	if (ret) {
		dev_err(imgr->dev, "iManager firmware communication error\n");
		return ret;
	}

	dev_info(imgr->dev, "Found Advantech iManager %s: %s (%s)\n",
		 ec->chip_name, info->version, info->type);

	ret = sysfs_create_group(&imgr->dev->kobj, &imanager_attr_group);
	if (ret)
		return ret;

	ret = imanager_register_cells(imgr);
	if (ret)
		sysfs_remove_group(&imgr->dev->kobj, &imanager_attr_group);

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
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
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
