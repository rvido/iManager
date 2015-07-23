/*
 * Advantech iManager Core - Firmware Interface
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
#include <linux/module.h>
#include <linux/swab.h>

#include <ec.h>

/**
 * This is the delay time between two EC transactions.
 * Values lower than 200us are not encouraged and may
 * cause I/O errors
 */
#define EC_MICRO_DELAY			200
#define EC_MAX_RETRY			1000

#define EC_MSG_OFFSET_CMD		0UL
#define EC_MSG_OFFSET_STATUS		1UL
#define EC_MSG_OFFSET_PARAM		2UL
#define EC_MSG_OFFSET_DATA(N)		(3UL + N)
#define EC_MSG_OFFSET_PAYLOAD(N)	(7UL + N)

/* The Device ID registers - 16 bit */
#define DEVID_REG_MSB			0x20
#define DEVID_REG_LSB			0x21

/*
 * IT8528 based firmware require a read/write command offset.
 */
#define EC_CMD_OFFSET_READ		0xA0UL
#define EC_CMD_OFFSET_WRITE		0x50UL

#define EC_FLAG_IO_18			BIT(0)
#define EC_FLAG_IO_28			BIT(1)

#define EC_STATUS_SUCCESS		BIT(0)
#define EC_STATUS_CMD_COMPLETE		BIT(7)

#define PCB_NAME_MAX_SIZE		8UL
#define EC_I2C_BLOCK_SIZE		32
#define EC_MAX_DID			32UL

#define DID_LABEL_SIZE			24UL
#define DID_DESC_SIZE			32UL

#define EC_KERNEL_MINOR(x)		(LOBYTE16(x))
#define EC_KERNEL_MAJOR(x)	__extension__ ({	\
		typeof(x) __x = (HIBYTE16(x));		\
		((__x >> 4) * 10 + (__x & 0x000f));	\
	})
#define EC_FIRMWARE_MINOR(x)		(LOBYTE16(x))
#define EC_FIRMWARE_MAJOR(x)		EC_KERNEL_MAJOR(x)
#define EC_PROJECT_CODE(x)		((char)(LOBYTE16(x)))

enum ec_device_type {
	ADC = 1,
	DAC,
	GPIO,
	IRQ,
	PWM,
	SMB,
	TACH
};

enum ec_device_id {
	/* GPIO */
	ALTGPIO0	= 0x10,
	ALTGPIO1,
	ALTGPIO2,
	ALTGPIO3,
	ALTGPIO4,
	ALTGPIO5,
	ALTGPIO6,
	ALTGPIO7,
	/* Button (GPIO) */
	BUTTON0,
	BUTTON1,
	BUTTON2,
	BUTTON3,
	BUTTON4,
	BUTTON5,
	BUTTON6,
	BUTTON7,
	/* FAN */
	CPUFAN_2P,
	CPUFAN_4P,
	SYSFAN1_2P,
	SYSFAN1_4P,
	SYSFAN2_2P,
	SYSFAN2_4P,
	/* Brightness Control */
	BRIGHTNESS,
	/* System Speaker */
	PCBEEP,
	/* SMBus */
	SMBOEM0,
	SMBOEM1,
	SMBOEM2,
	SMBEEPROM,
	SMBTHERMAL0,
	SMBTHERMAL1,
	SMBSECEEP,
	I2COEM,
	/* Speaker */
	SPEAKER		= 0x30,
	/* SMBus */
	SMBEEP2K	= 0x38,
	OEMEEP,
	OEMEEP2K,
	PECI,
	SMBOEM3,
	SMLINK,
	SMBSLV,
	/* LED */
	POWERLED	= 0x40,
	BATLEDG,
	OEMLED0,
	OEMLED1,
	OEMLED2,
	BATLEDR,
	/* Smart Battery */
	SMARTBAT1	= 0x48,
	SMARTBAT2,
	/* ADC */
	CMOSBAT		= 0x50,
	CMOSBATx2,
	CMOSBATx10,
	LIBAT,
	LIBATx2,
	LIBATx10,
	ADC5VS0,
	ADC5VS0x2,
	ADC5VS0x10,
	ADC5VS5,
	ADC5VS5x2,
	ADC5VS5x10,
	ADC33VS0,
	ADC33VS0x2,
	ADC33VS0x10,
	ADC33VS5,
	ADC33VS5x2,	/* 0x60 */
	ADC33VS5x10,
	ADC12VS0,
	ADC12VS0x2,
	ADC12VS0x10,
	VCOREA,
	VCOREAx2,
	VCOREAx10,
	VCOREB,
	VCOREBx2,
	VCOREBx10,
	ADCDC,
	ADCDCx2,
	ADCDCx10,
	VSTBY,
	VSTBYx2,
	VSTBYx10,	/* 0x70 */
	VAUX,
	VAUXx2,
	VAUXx10,
	CURRENT,
	/* Watchdog */
	WDIRQ		= 0x78,
	WDNMI,
	/* FAN Tacho */
	TACHO0		= 0x80,
	TACHO1,
	TACHO2,
	/* Brightness/Backlight Control */
	BRIGHTNESS2	= 0x88,
	BACKLIGHT1,
	BACKLIGHT2
};

enum ec_dynamic_table_type {
	EC_DYN_DID,
	EC_DYN_HWP,
	EC_DYN_POL
};

struct ec_devtbl {
	int id;
	int type;
	int scale;
	const char label[DID_LABEL_SIZE];
	const char description[DID_DESC_SIZE];
};

struct ec_dyn_devtbl {
	int did;	/* Device ID */
	int hwp;	/* Hardware Pin */
	int pol;	/* Polarity */
	const struct ec_devtbl *devtbl; /* Device table Entry */
};

struct ec_data {
	int	ioflag;
	int	chipid;
	u16	addr;

	struct ec_dyn_devtbl dyn[EC_MAX_DID];

	struct ec_info	info;

	struct imanager_hwmon_device		sensors;
	struct imanager_gpio_device		gpio;
	struct imanager_smbus_device		smb;
	struct imanager_watchdog_device		wdt;
	struct imanager_backlight_device	blc;
};

struct ec_version_raw {
	u16	kernel,
		chipid,
		project_code,
		firmware;
};

enum ec_ram_type {
	EC_RAM_ACPI = 1,
	EC_RAM_HW,
	EC_RAM_EXT
};

static struct ec_data ec;

static const struct ec_devtbl devtbl[] = {
	{ ALTGPIO0,	GPIO,	-1,	"gpio0" },
	{ ALTGPIO1,	GPIO,	-1,	"gpio1" },
	{ ALTGPIO2,	GPIO,	-1,	"gpio2" },
	{ ALTGPIO3,	GPIO,	-1,	"gpio3" },
	{ ALTGPIO4,	GPIO,	-1,	"gpio4" },
	{ ALTGPIO5,	GPIO,	-1,	"gpio5" },
	{ ALTGPIO6,	GPIO,	-1,	"gpio6" },
	{ ALTGPIO7,	GPIO,	-1,	"gpio7" },
	{ BUTTON0,	GPIO,	-1,	"button0" },
	{ BUTTON1,	GPIO,	-1,	"button1" },
	{ BUTTON2,	GPIO,	-1,	"button2" },
	{ BUTTON3,	GPIO,	-1,	"button3" },
	{ BUTTON4,	GPIO,	-1,	"button4" },
	{ BUTTON5,	GPIO,	-1,	"button4" },
	{ BUTTON6,	GPIO,	-1,	"button4" },
	{ BUTTON7,	GPIO,	-1,	"button4" },
	{ CPUFAN_2P,	PWM,	2,	"FAN CPU" },
	{ CPUFAN_4P,	PWM,	4,	"FAN CPU" },
	{ SYSFAN1_2P,	PWM,	2,	"FAN SYS1" },
	{ SYSFAN1_4P,	PWM,	4,	"FAN SYS1" },
	{ SYSFAN2_2P,	PWM,	2,	"FAN SYS2" },
	{ SYSFAN2_4P,	PWM,	4,	"FAN SYS2" },
	{ BRIGHTNESS,	PWM,	-1,	"Brightness1" },
	{ PCBEEP,	PWM,	-1,	"Beep" },
	{ SMBOEM0,	SMB,	-1,	"SMB1" },
	{ SMBOEM1,	SMB,	-1,	"SMB2" },
	{ SMBOEM2,	SMB,	-1,	"SMB3" },
	{ SMBEEPROM,	SMB,	-1,	"SMBEEP" },
	{ SMBTHERMAL0,	SMB,	-1,	"SMBTHERM0" },
	{ SMBTHERMAL1,	SMB,	-1,	"SMBTHERM1" },
	{ SMBSECEEP,	SMB,	-1,	"SMBSECEEP" },
	{ I2COEM,	SMB,	-1,	"I2COEM" },
	{ SPEAKER,	DAC,	-1,	"Speaker" },
	{ SMBEEP2K,	SMB,	-1,	"SMBEEP2K" },
	{ OEMEEP,	SMB,	-1,	"OEMEEP" },
	{ OEMEEP2K,	SMB,	-1,	"OEMEEP2K" },
	{ PECI,		SMB,	-1,	"SMB_PECI" },
	{ SMBOEM3,	SMB,	-1,	"SMBOEM3" },
	{ SMLINK,	SMB,	-1,	"SMLINK" },
	{ SMBSLV,	SMB,	-1,	"SMBSLV" },
	{ POWERLED,	GPIO,	-1,	"Power LED" },
	{ BATLEDG,	GPIO,	-1,	"BATLEDG" },
	{ OEMLED0,	GPIO,	-1,	"OEMLED0" },
	{ OEMLED1,	GPIO,	-1,	"OEMLED1" },
	{ OEMLED2,	GPIO,	-1,	"OEMLED2" },
	{ BATLEDR,	GPIO,	-1,	"OEMLEDR" },
	{ SMARTBAT1,	SMB,	-1,	"SmartBat1" },
	{ SMARTBAT2,	SMB,	-1,	"SmartBat2" },
	{ CMOSBAT,	ADC,	1,	"VBat" },
	{ CMOSBATx2,	ADC,	2,	"VBat" },
	{ CMOSBATx10,	ADC,	10,	"VBat" },
	{ LIBAT,	ADC,	1,	"VBat2" },
	{ LIBATx2,	ADC,	2,	"VBat2" },
	{ LIBATx10,	ADC,	10,	"VBat2" },
	{ ADC5VS0,	ADC,	1,	"+5V" },
	{ ADC5VS0x2,	ADC,	2,	"+5V" },
	{ ADC5VS0x10,	ADC,	10,	"+5V" },
	{ ADC5VS5,	ADC,	1,	"+5V" },
	{ ADC5VS5x2,	ADC,	2,	"+5V" },
	{ ADC5VS5x10,	ADC,	10,	"+5V" },
	{ ADC33VS0,	ADC,	1,	"+3.3V" },
	{ ADC33VS0x2,	ADC,	2,	"+3.3V" },
	{ ADC33VS0x10,	ADC,	10,	"+3.3V" },
	{ ADC33VS5,	ADC,	1,	"+3.3V" },
	{ ADC33VS5x2,	ADC,	2,	"+3.3V" },
	{ ADC33VS5x10,	ADC,	10,	"+3.3V" },
	{ ADC12VS0,	ADC,	1,	"+12V" },
	{ ADC12VS0x2,	ADC,	2,	"+12V" },
	{ ADC12VS0x10,	ADC,	10,	"+12V" },
	{ VCOREA,	ADC,	1,	"VCore" },
	{ VCOREAx2,	ADC,	2,	"VCore" },
	{ VCOREAx10,	ADC,	10,	"VCore" },
	{ VCOREB,	ADC,	1,	"VCore2" },
	{ VCOREBx2,	ADC,	2,	"VCore2" },
	{ VCOREBx10,	ADC,	10,	"VCore2" },
	{ ADCDC,	ADC,	1,	"ADCDC" },
	{ ADCDCx2,	ADC,	2,	"ADCDCx2" },
	{ ADCDCx10,	ADC,	10,	"ADCDCx10" },
	{ VSTBY,	ADC,	1,	"Vsb" },
	{ VSTBYx2,	ADC,	2,	"Vsb" },
	{ VSTBYx10,	ADC,	10,	"Vsb" },
	{ VAUX,		ADC,	1,	"VAUX" },
	{ VAUXx2,	ADC,	2,	"VAUX" },
	{ VAUXx10,	ADC,	10,	"VAUX" },
	{ CURRENT,	ADC,	1,	"Imon" },
	{ WDIRQ,	IRQ,	-1,	"WDIRQ" },
	{ WDNMI,	GPIO,	-1,	"WDNMI" },
	{ TACHO0,	TACH,	-1,	"Tacho1" },
	{ TACHO1,	TACH,	-1,	"Tacho2" },
	{ TACHO2,	TACH,	-1,	"Tacho3" },
	{ BRIGHTNESS2,	PWM,	-1,	"Brightness2" },
	{ BACKLIGHT1,	GPIO,	-1,	"Backlight1" },
	{ BACKLIGHT2,	GPIO,	-1,	"Backlight2" },
	{ 0, 0, 0, "" }
};

/**
 * EC I/O
 */

static inline void imanager_delay(void)
{
	udelay(EC_MICRO_DELAY);
}

static int wait_ibf_cleared(void)
{
	int i = 0;

	do {
		if (!(inb(IT8516_CMD_PORT) & BIT(1)))
			return 0;
		imanager_delay();
	} while (i++ < EC_MAX_RETRY);

	return -ETIME;
}

static int wait_obf_set(void)
{
	int i = 0;

	do {
		if (inb(IT8516_CMD_PORT) & BIT(0))
			return 0;
		imanager_delay();
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

static inline int _ec_inb(int addr, int reg)
{
	int ret;

	ret = wait_ibf_cleared();
	if (ret)
		return ret;

	/* clear data to prevent lock */
	inb(addr - 1);

	outb(reg, addr);

	ret = wait_obf_set();
	if (ret)
		return ret;

	return inb(addr - 1);
}

static inline int _ec_outb(int addr, int reg, int val)
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

static int ec_read(u8 cmd)
{
	int ret;

	if (ec.ioflag == EC_FLAG_IO_28)
		ret = _ec_inb(IT8516_CMD_PORT, cmd + EC_CMD_OFFSET_READ);
	else if (ec.ioflag == EC_FLAG_IO_18)
		ret = ec_inb(IT8518_CMD_PORT, cmd);
	else
		ret = -EINVAL;

	return ret;
}

static int ec_write(u8 cmd, u8 value)
{
	int ret = 0;

	if (ec.ioflag == EC_FLAG_IO_28)
		ret = _ec_outb(IT8516_CMD_PORT, cmd + EC_CMD_OFFSET_WRITE,
			       value);
	else if (ec.ioflag == EC_FLAG_IO_18)
		ec_outb(IT8518_CMD_PORT, cmd, value);
	else
		ret = -EINVAL;

	return ret;
}

/* Prevent FW lock */
static void ec_clear_ports(void)
{
	inb(IT8516_DAT_PORT);
	inb(IT8518_DAT_PORT);
}

static inline u16 ec_read_chipid(u16 addr)
{
	return ((ec_inb(addr, DEVID_REG_MSB) << 8) | \
		 ec_inb(addr, DEVID_REG_LSB));
}

static int ec_wait_cmd_clear(void)
{
	int i = 0;

	do {
		if (!ec_read(0))
			return 0;
		imanager_delay();
	} while (i++ < EC_MAX_RETRY);

	pr_err("No respons from EC (timeout)\n");

	return -ETIME;
}

static int ec_read_ram(u8 bank, u8 offset, u8 len, u8 *buf, u8 bufsz)
{
	int i;
	int ret;

	if (WARN_ON(!buf))
		return -EINVAL;

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ec_write(EC_MSG_OFFSET_PARAM, bank);
	ec_write(EC_MSG_OFFSET_DATA(0), offset);
	ec_write(EC_MSG_OFFSET_DATA(0x2C), len);
	ec_write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_RD);

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ret = ec_read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	for (i = 0; (i < len) && (len < EC_MSG_SIZE) && (len <= bufsz); i++)
		buf[i] = ec_read(EC_MSG_OFFSET_DATA(i + 1));

	return 0;
}

static int ec_write_ram(u8 bank, u8 offset, u8 len, u8 *buf)
{
	int i;
	int ret;

	if (WARN_ON(!buf))
		return -EINVAL;

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ec_write(EC_MSG_OFFSET_PARAM, bank);
	ec_write(EC_MSG_OFFSET_DATA(0), offset);
	ec_write(EC_MSG_OFFSET_DATA(0x2C), len);

	for (i = 0; (i < len) && (len < EC_MSG_SIZE); i++)
		ec_write(EC_MSG_OFFSET_DATA(i + 1), buf[i]);

	ec_write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_WR);

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ret = ec_read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	return 0;
}

static int ec_read_dynamic_devtbl(struct ec_data *ec)
{
	u32 i, j;
	int ret;
	struct ec_message did = {
		.rlen = EC_MAX_DID,
		.wlen = 0,
	};
	struct ec_message hwp = {
		.rlen = EC_MAX_DID,
		.wlen = 0,
	};
	struct ec_message pol = {
		.rlen = EC_MAX_DID,
		.wlen = 0,
	};
	struct ec_dyn_devtbl *dyn;

	memset(ec->dyn, 0, sizeof(ec->dyn));

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, EC_DYN_DID, &did);
	if (ret)
		return -EIO;

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, EC_DYN_HWP, &hwp);
	if (ret)
		return -EIO;

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, EC_DYN_POL, &pol);
	if (ret)
		return -EIO;

	for (i = 0; (i < EC_MAX_DID) && did.u.data[i]; i++) {
		dyn = &ec->dyn[i];
		for (j = 0; j < ARRAY_SIZE(devtbl); j++) {
			if (devtbl[j].id == did.u.data[i]) {
				dyn->did = did.u.data[i];
				dyn->hwp = hwp.u.data[i];
				dyn->pol = pol.u.data[i];
				dyn->devtbl = &devtbl[j];
				break;
			}
		}
	}

	return 0;
}

static int ec_read_buffer(u8 *data, int rlen)
{
	int ret, i, j;
	int pages = rlen % EC_I2C_BLOCK_SIZE;
	int remainder = rlen / EC_I2C_BLOCK_SIZE;

	/* pre-condition: rlen <= 256 */

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	for (i = 0; i < pages; i++) {
		ec_write(EC_MSG_OFFSET_PARAM, i);
		ec_write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear();
		if (ret)
			return ret;

		ret = ec_read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < EC_I2C_BLOCK_SIZE; j++)
			data[i * EC_I2C_BLOCK_SIZE + j] =
				ec_read(EC_MSG_OFFSET_DATA(j));
	}

	if (remainder) {
		ec_write(EC_MSG_OFFSET_PARAM, pages);
		ec_write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear();
		if (ret)
			return ret;

		ret = ec_read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < remainder; j++)
			data[pages * EC_I2C_BLOCK_SIZE + j] =
				ec_read(EC_MSG_OFFSET_DATA(j));
	}

	return 0;
}

static int imanager_msg_trans(u8 cmd, u8 param, struct ec_message *msg, bool payload)
{
	int ret, i, len;
	u32 offset;

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ec_write(EC_MSG_OFFSET_PARAM, param);

	if (msg && msg->wlen) {
		if (!msg->data) {
			for (i = 0; i < msg->wlen; i++)
				ec_write(EC_MSG_OFFSET_DATA(i),
					msg->u.data[i]);
		} else {
			for (i = 0; i < msg->wlen; i++)
				ec_write(EC_MSG_OFFSET_DATA(i), msg->data[i]);
			ec_write(EC_MSG_OFFSET_DATA(0x2c), msg->wlen);
		}
	}

	ec_write(EC_MSG_OFFSET_CMD, cmd);
	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	/* GPIO and I2C have different success return values */
	ret = ec_read(EC_MSG_OFFSET_STATUS);
	if ((ret != EC_STATUS_SUCCESS) && !(ret & EC_STATUS_CMD_COMPLETE))
		return -EIO;
	/*
	 * EC I2C may return an error code which we need to hand-off
	 * to the caller
	 */
	else if (ret & 0x07e)
		return ret;

	if (msg && msg->data) {
		ret = ec_read_buffer(msg->data, msg->rlen);
		if (ret < 0)
			return ret;
	} else if (msg && msg->rlen) {
		if (msg->rlen == 0xff)
			/* Use alternate message body for hwmon */
			len = ec_read(EC_MSG_OFFSET_DATA(0x2C));
		else
			len = (msg->rlen > EC_MSG_SIZE ? EC_MSG_SIZE :
				msg->rlen);
		offset = payload ? EC_MSG_OFFSET_PAYLOAD(0) :
				EC_MSG_OFFSET_DATA(0);
		for (i = 0; i < len; i++)
			msg->u.data[i] = ec_read(offset + i);
	}

	return 0;
}

const struct ec_info *imanager_get_fw_info(void)
{
	return &ec.info;
}
EXPORT_SYMBOL_GPL(imanager_get_fw_info);

int imanager_msg_read(u8 cmd, u8 param, struct ec_message *msg)
{
	return imanager_msg_trans(cmd, param, msg, false);
}
EXPORT_SYMBOL_GPL(imanager_msg_read);

int imanager_msg_write(u8 cmd, u8 param, struct ec_message *msg)
{
	return imanager_msg_trans(cmd, param, msg, true);
}
EXPORT_SYMBOL_GPL(imanager_msg_write);

int imanager_read_byte(u8 cmd, u8 param)
{
	int ret;
	struct ec_message msg = {
		.rlen = 1,
		.wlen = 0,
	};

	ret = imanager_msg_read(cmd, param, &msg);
	if (ret)
		return ret;

	return msg.u.data[0];
}
EXPORT_SYMBOL_GPL(imanager_read_byte);

int imanager_read_word(u8 cmd, u8 param)
{
	int ret;
	struct ec_message msg = {
		.rlen = 2,
		.wlen = 0,
	};

	ret = imanager_msg_read(cmd, param, &msg);
	if (ret)
		return ret;

	return (msg.u.data[0] << 8 | msg.u.data[1]);
}
EXPORT_SYMBOL_GPL(imanager_read_word);

int imanager_write_byte(u8 cmd, u8 param, u8 byte)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = 1,
		.u = {
			.data = { byte, 0 },
		},
	};

	return imanager_msg_write(cmd, param, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write_byte);

int imanager_write_word(u8 cmd, u8 param, u16 word)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = 2,
		.u = {
			.data = { HIBYTE16(word), LOBYTE16(word), 0 },
		},
	};

	return imanager_msg_write(cmd, param, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write_word);

static int ec_hwram_read_byte(u8 offset)
{
	int ret;
	u8 val;

	ret = ec_read_ram(EC_RAM_HW, offset, sizeof(val), &val, sizeof(val));
	if (ret < 0) {
		pr_err("Failed to read from HWRAM @ 0x%02X\n", offset);
		return ret;
	}

	return val;
}

int imanager_acpiram_read_byte(u8 offset)
{
	int ret;
	u8 value;

	ret = ec_read_ram(EC_RAM_ACPI, offset, sizeof(value), (u8 *)&value,
			  sizeof(value));
	if (ret < 0) {
		pr_err("Failed to read from ACPI RAM @ 0x%02X\n", offset);
		return ret;
	}

	return value;
}
EXPORT_SYMBOL_GPL(imanager_acpiram_read_byte);

int imanager_acpiram_write_byte(u8 offset, u8 value)
{
	int ret;

	ret = ec_write_ram(EC_RAM_ACPI, offset, sizeof(value), &value);
	if (ret) {
		pr_err("Failed to write to ACPI RAM @ 0x%02X\n", offset);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_acpiram_write_byte);

int imanager_acpiram_read_block(u8 offset, u8 *buf, u8 len)
{
	int ret;

	ret = ec_read_ram(EC_RAM_ACPI, offset, len, buf, len);
	if (ret < 0) {
		pr_err("Failed to read from ACPI RAM @ 0x%02X\n", offset);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_acpiram_read_block);

int imanager_acpiram_write_block(u8 offset, u8 *buf, u8 len)
{
	int ret;

	ret = ec_write_ram(EC_RAM_ACPI, offset, len, buf);
	if (ret) {
		pr_err("Failed to write to ACPI RAM @ 0x%02X\n", offset);
		return ret;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_acpiram_write_block);

int imanager_wait_proc_complete(u8 offset, int cond)
{
	int ret, i;

	for (i = 0; i < EC_MAX_RETRY; i++) {
		ret = ec_hwram_read_byte(offset);
		if (ret < 0)
			return ret;

		if (ret == cond)
			return 0;

		imanager_delay();
	}

	return -EIO;
}
EXPORT_SYMBOL_GPL(imanager_wait_proc_complete);

static inline void ec_get_dev_attr(struct ec_dev_attr *attr,
				   const struct ec_dyn_devtbl *tbl)
{
	attr->did = tbl->did;
	attr->hwp = tbl->hwp;
	attr->pol = tbl->pol;
	attr->scale = tbl->devtbl->scale;
	attr->label = tbl->devtbl->label;
}

static int ec_get_dev_adc(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct dev_adc *adc = &ec->sensors.adc;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && (dyn->devtbl->type == ADC)) {
			switch (dyn->did) {
			case ADC12VS0:
			case ADC12VS0x2:
			case ADC12VS0x10:
				ec_get_dev_attr(&adc->attr[0], dyn);
				adc->num++;
				break;
			case ADC5VS5:
			case ADC5VS5x2:
			case ADC5VS5x10:
				ec_get_dev_attr(&adc->attr[1], dyn);
				adc->num++;
				break;
			case CMOSBAT:
			case CMOSBATx2:
			case CMOSBATx10:
				ec_get_dev_attr(&adc->attr[2], dyn);
				adc->num++;
				break;
			case VCOREA:
			case ADC5VS0:
			case ADC5VS0x2:
			case ADC5VS0x10:
				ec_get_dev_attr(&adc->attr[3], dyn);
				adc->num++;
				break;
			case CURRENT:
			case ADC33VS0:
			case ADC33VS0x2:
			case ADC33VS0x10:
				ec_get_dev_attr(&adc->attr[4], dyn);
				adc->num++;
				break;
			default:
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int ec_get_dev_fan(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct dev_fan *fan = &ec->sensors.fan;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && ((dyn->devtbl->type == TACH) || \
				 (dyn->devtbl->type == PWM))) {
			switch (dyn->did) {
			case CPUFAN_2P:
			case CPUFAN_4P:
				ec_get_dev_attr(&fan->attr[0], dyn);
				fan->num++;
				break;
			case SYSFAN1_2P:
			case SYSFAN1_4P:
				ec_get_dev_attr(&fan->attr[1], dyn);
				fan->num++;
				break;
			case SYSFAN2_2P:
			case SYSFAN2_4P:
				ec_get_dev_attr(&fan->attr[2], dyn);
				fan->num++;
				break;
			case TACHO0:
			case TACHO1:
			case TACHO2:
			case BRIGHTNESS:
			case BRIGHTNESS2:
			case PCBEEP:
				break;
			default:
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	return 0;
}

static int ec_get_dev_hwmon(struct ec_data *ec)
{
	int ret;

	ret = ec_get_dev_adc(ec);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_fan(ec);
	if (ret < 0)
		return ret;

	ec->sensors.info = &ec->info;

	return 0;
}

static int ec_get_dev_blc(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_backlight_device *blc = &ec->blc;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && (dyn->devtbl->type == PWM)) {
			switch (dyn->did) {
			case BRIGHTNESS:
				ec_get_dev_attr(&blc->attr[0], dyn);
				blc->brightness[0] = EC_ACPIRAM_BRIGHTNESS1;
				blc->num++;
				break;
			case BRIGHTNESS2:
				ec_get_dev_attr(&blc->attr[1], dyn);
				blc->brightness[1] = EC_ACPIRAM_BRIGHTNESS2;
				blc->num++;
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
			default:
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	blc->info = &ec->info;

	return 0;
}

static int ec_get_dev_gpio(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_gpio_device *gpio = &ec->gpio;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && (dyn->devtbl->type == GPIO)) {
			switch (dyn->did) {
			case ALTGPIO0:
				ec_get_dev_attr(&gpio->attr[0], dyn);
				gpio->num++;
				break;
			case ALTGPIO1:
				ec_get_dev_attr(&gpio->attr[1], dyn);
				gpio->num++;
				break;
			case ALTGPIO2:
				ec_get_dev_attr(&gpio->attr[2], dyn);
				gpio->num++;
				break;
			case ALTGPIO3:
				ec_get_dev_attr(&gpio->attr[3], dyn);
				gpio->num++;
				break;
			case ALTGPIO4:
				ec_get_dev_attr(&gpio->attr[4], dyn);
				gpio->num++;
				break;
			case ALTGPIO5:
				ec_get_dev_attr(&gpio->attr[5], dyn);
				gpio->num++;
				break;
			case ALTGPIO6:
				ec_get_dev_attr(&gpio->attr[6], dyn);
				gpio->num++;
				break;
			case ALTGPIO7:
				ec_get_dev_attr(&gpio->attr[7], dyn);
				gpio->num++;
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
			default:
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	gpio->info = &ec->info;

	return 0;
}

static int ec_get_dev_smb(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_smbus_device *smb = &ec->smb;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && (dyn->devtbl->type == SMB)) {
			switch (dyn->did) {
			case SMBEEPROM:
				ec_get_dev_attr(&smb->attr[0], dyn);
				smb->smbeeprom = &smb->attr[0];
				smb->num++;
				break;
			case I2COEM:
				ec_get_dev_attr(&smb->attr[1], dyn);
				smb->i2coem = &smb->attr[1];
				smb->num++;
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
			default:
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	smb->info = &ec->info;

	return 0;
}

static int ec_get_dev_wdt(struct ec_data *ec)
{
	int i;
	struct ec_dyn_devtbl *dyn;
	struct imanager_watchdog_device *wdt = &ec->wdt;

	for (i = 0; i < ARRAY_SIZE(ec->dyn); i++) {
		dyn = &ec->dyn[i];
		if (dyn->did && (dyn->devtbl->type == IRQ)) {
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
				pr_err("DID 0x%02X not handled\n", dyn->did);
				return -EINVAL;
			}
		}
	}

	wdt->info = &ec->info;

	return 0;
}

const struct imanager_hwmon_device *imanager_get_hwmon_device(void)
{
	return &ec.sensors;
}
EXPORT_SYMBOL_GPL(imanager_get_hwmon_device);

const struct imanager_gpio_device *imanager_get_gpio_device(void)
{
	return &ec.gpio;
}
EXPORT_SYMBOL_GPL(imanager_get_gpio_device);

const struct imanager_smbus_device *imanager_get_smb_device(void)
{
	return &ec.smb;
}
EXPORT_SYMBOL_GPL(imanager_get_smb_device);

const struct imanager_backlight_device *imanager_get_backlight_device(void)
{
	return &ec.blc;
}
EXPORT_SYMBOL_GPL(imanager_get_backlight_device);

const struct imanager_watchdog_device *imanager_get_watchdog_device(void)
{
	return &ec.wdt;
}
EXPORT_SYMBOL_GPL(imanager_get_watchdog_device);

int imanager_get_chipid(void)
{
	if (!ec.chipid)
		return -EINVAL;

	return ec.chipid;
}
EXPORT_SYMBOL_GPL(imanager_get_chipid);

static int ec_get_version(struct ec_version *version)
{
	int ret;
	u16 raw;
	struct ec_version_raw ver;

	if (WARN_ON(!version))
		return -EINVAL;

	ret = ec_read_ram(EC_RAM_ACPI, EC_ACPIRAM_FW_RELEASE_RD, sizeof(ver),
			 (u8 *)&ver, sizeof(ver));
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

	return 0;
}

static int ec_get_pcb_name(struct ec_info *info)
{
	int ret;
	struct ec_message msg = {
		.rlen = ARRAY_SIZE(info->pcb_name),
		.wlen = 0,
	};

	if (WARN_ON(!info))
		return -EINVAL;

	ret = imanager_msg_read(EC_CMD_FW_INFO_RD, 0, &msg);
	if (ret)
		return ret;

	/*
	 * Sadly, the string is not Null-terminated so we will need to read a
	 * fixed amount of chars. There is, apparently, no exact definition
	 * of board name (SOM6867 vs. MIO-5271).
	 */
	memset(info->pcb_name, 0, ARRAY_SIZE(info->pcb_name));
	strncpy(info->pcb_name, (const char *)msg.u.data, PCB_NAME_MAX_SIZE);

	if (strchr(info->pcb_name, '-') == NULL)
		info->pcb_name[PCB_NAME_MAX_SIZE - 1] = '\0';

	return 0;
}

static int ec_get_fw_info(struct ec_info *info)
{
	int ret;

	if (WARN_ON(!info))
		return -EINVAL;

	ret = ec_get_version(&info->version);
	if (ret)
		return ret;

	return ec_get_pcb_name(info);
}

static int ec_init(void)
{
	int ret;

	ec_clear_ports();

	ret = ec_read_dynamic_devtbl(&ec);
	if (ret)
		return ret;

	ret = ec_get_fw_info(&ec.info);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_hwmon(&ec);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_gpio(&ec);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_smb(&ec);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_blc(&ec);
	if (ret < 0)
		return ret;

	ret = ec_get_dev_wdt(&ec);
	if (ret < 0)
		return ret;

	return 0;
}

int imanager_ec_probe(u16 addr)
{
	int chipid;
	unsigned int ioflag;

	memset((void *)&ec, 0, sizeof(ec));

	chipid = ec_read_chipid(addr);

	switch (chipid) {
	case SIO_DEVID_IT8516:
		pr_err("EC IT8516 not supported\n");
		return -ENODEV;
	case SIO_DEVID_IT8518:
		ioflag = EC_FLAG_IO_18;
		break;
	case SIO_DEVID_IT8528:
		ioflag = EC_FLAG_IO_28;
		break;
	default:
		return -ENODEV;
	}

	ec.chipid = chipid;
	ec.addr   = addr;
	ec.ioflag = ioflag;

	return ec_init();
}

