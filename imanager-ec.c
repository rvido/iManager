/*
 * Advantech iManager Core (EC IT8518/28) - Firmware Interface
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
#include <linux/mfd/imanager/ec.h>

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

/*
 * IT8528 based firmware require a read/write command offset.
 */
#define EC_CMD_OFFSET_READ		0xA0
#define EC_CMD_OFFSET_WRITE		0x50

#define EC_FLAG_IO_18			BIT(0)
#define EC_FLAG_IO_28			BIT(1)

#define EC_STATUS_SUCCESS		BIT(0)
#define EC_STATUS_CMD_COMPLETE		BIT(7)

#define PCB_NAME_READ_SIZE		8
#define EC_I2C_BLOCK_SIZE		32
#define EC_MAX_DID			32

#define EC_KERNEL_MINOR(x)		(LOBYTE16(x))
#define EC_KERNEL_MAJOR(x)	__extension__ ({	\
		typeof(x) __x = (HIBYTE16(x));		\
		((__x >> 4) * 10 + (__x & 0x000f));	\
	})
#define EC_FIRMWARE_MINOR(x)		(LOBYTE16(x))
#define EC_FIRMWARE_MAJOR(x)		EC_KERNEL_MAJOR(x)
#define EC_PROJECT_CODE(x)		((char)(LOBYTE16(x)))

#define DID_MAX_LABEL_SIZE		24

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
	AltGPIO0	= 0x10,
	AltGPIO1,
	AltGPIO2,
	AltGPIO3,
	AltGPIO4,
	AltGPIO5,
	AltGPIO6,
	AltGPIO7,
	/* Button (GPIO) */
	Btn0,
	Btn1,
	Btn2,
	Btn3,
	Btn4,
	Btn5,
	Btn6,
	Btn7,
	/* FAN */
	CPUFAN_2P,
	CPUFAN_4P,
	SYSFAN1_2P,
	SYSFAN1_4P,
	SYSFAN2_2P,
	SYSFAN2_4P,
	/* Brightness Control */
	PWMBRIGHTNESS,
	/* System Speaker */
	PWMBEEP,
	/* SMBus */
	SMBOEM0,
	SMBOEM1,
	SMBOEM2,
	SMBEEPROM,
	SMBTHERMAL0,
	SMBTHERMAL1,
	SMBSecurityEEP,
	I2COEM,
	/* Speaker */
	DACSPEAKER	= 0x30,
	/* SMBus */
	SMBEEP2K	= 0x38,
	OEMEEP,
	OEMEEP2K,
	PECI,
	SMBOEM3,
	SMLINK,
	SMBSLV,
	/* LED */
	PowerLED	= 0x40,
	BatLEDG,
	OEMLED0,
	OEMLED1,
	OEMLED2,
	BatLEDR,
	/* Smart Battery */
	SmartBat1	= 0x48,
	SmartBat2,
	/* ADC */
	ADCCMOSBAT	= 0x50,
	ADCCMOSBATx2,
	ADCCMOSBATx10,
	ADCBAT,
	ADCBATx2,
	ADCBATx10,
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
	ADCVCOREA,
	ADCVCOREAx2,
	ADCVCOREAx10,
	ADCVCOREB,
	ADCVCOREBx2,
	ADCVCOREBx10,
	ADCDC,
	ADCDCx2,
	ADCDCx10,
	ADCDCSTBY,
	ADCDCSTBYx2,
	ADCDCSTBYx10,	/* 0x70 */
	ADCDCOther,
	ADCDCOtherx2,
	ADCDCOtherx10,
	ADCCurrent,
	/* Watchdog */
	WDIRQ		= 0x78,
	WDNMI,
	/* FAN Tacho */
	TACHO0		= 0x80,
	TACHO1,
	TACHO2,
	/* Brightness/Backlight Control */
	PWMBRIGHTNESS2	= 0x88,
	BACKLIGHT1,
	BACKLIGHT2
};

struct ec_device_table {
	enum ec_device_id	id;
	enum ec_device_type	type;
	char			label[DID_MAX_LABEL_SIZE];
};

struct ec_device_attr {
	u8 did;			/* Device ID */
	u8 hwp;			/* Hardware Pin */
	u8 pol;			/* Polarity */
};

struct ec_dev_cfg {
	struct ec_device_attr	attr;
	enum ec_device_type	type;
	const char		*label;
	const char		*desc;
};

struct ec_data {
	u32			ioflag;
	u32			chipid;
	u16			addr;
	struct ec_dev_cfg	cfg[EC_MAX_DID];
};

struct ec_version_raw {
	u16	kernel,
		chipid,
		project_code,
		firmware;
};

static struct ec_data ec;

static const struct ec_device_table devtbl[] = {
	{
		.id = AltGPIO0,
		.type = GPIO,
		.label = "gpio0",
	}, {
		.id = AltGPIO1,
		.type = GPIO,
		.label = "gpio1",
	}, {
		.id = AltGPIO2,
		.type = GPIO,
		.label = "gpio2",
	}, {
		.id = AltGPIO3,
		.type = GPIO,
		.label = "gpio3",
	}, {
		.id = AltGPIO4,
		.type = GPIO,
		.label = "gpio4",
	}, {
		.id = AltGPIO5,
		.type = GPIO,
		.label = "gpio5",
	}, {
		.id = AltGPIO6,
		.type = GPIO,
		.label = "gpio6",
	}, {
		.id = AltGPIO7,
		.type = GPIO,
		.label = "gpio7",
	}, {
		.id = Btn0,
		.type = GPIO,
		.label = "button0",
	}, {
		.id = Btn1,
		.type = GPIO,
		.label = "button1",
	}, {
		.id = Btn2,
		.type = GPIO,
		.label = "button2",
	}, {
		.id = Btn3,
		.type = GPIO,
		.label = "button3",
	}, {
		.id = Btn4,
		.type = GPIO,
		.label = "button4",
	}, {
		.id = Btn5,
		.type = GPIO,
		.label = "button5",
	}, {
		.id = Btn6,
		.type = GPIO,
		.label = "button6",
	}, {
		.id = Btn7,
		.type = GPIO,
		.label = "button7",
	}, {
		.id = CPUFAN_2P,
		.type = PWM,
		.label = "FAN CPU",
	}, {
		.id = CPUFAN_4P,
		.type = PWM,
		.label = "FAN CPU",
	}, {
		.id = SYSFAN1_2P,
		.type = PWM,
		.label = "FAN SYS1",
	}, {
		.id = SYSFAN1_4P,
		.type = PWM,
		.label = "FAN SYS1",
	}, {
		.id = SYSFAN2_2P,
		.type = PWM,
		.label = "FAN SYS2",
	}, {
		.id = SYSFAN2_4P,
		.type = PWM,
		.label = "FAN SYS2",
	}, {
		.id = PWMBRIGHTNESS,
		.type = PWM,
		.label = "Brightness1",
	}, {
		.id = PWMBEEP,
		.type = PWM,
		.label = "Beep",
	}, {
		.id = SMBOEM0,
		.type = SMB,
		.label = "smb1",
	}, {
		.id = SMBOEM1,
		.type = SMB,
		.label = "smb2",
	}, {
		.id = SMBOEM2,
		.type = SMB,
		.label = "smb3",
	}, {
		.id = SMBEEPROM,
		.type = SMB,
		.label = "SMBEEP",
	}, {
		.id = SMBTHERMAL0,
		.type = SMB,
		.label = "SMBThermal0",
	}, {
		.id = SMBTHERMAL1,
		.type = SMB,
		.label = "SMBThermal1",
	}, {
		.id = SMBSecurityEEP,
		.type = SMB,
		.label = "SMBSecurityEEP",
	}, {
		.id = I2COEM,
		.type = SMB,
		.label = "i2c",
	}, {
		.id = DACSPEAKER,
		.type = DAC,
		.label = "Speaker",
	}, {
		.id = SMBEEP2K,
		.type = SMB,
		.label = "SMBEEP2K",
	}, {
		.id = OEMEEP,
		.label = "OEMEEP",
		.type = SMB,
	}, {
		.id = OEMEEP2K,
		.type = SMB,
		.label = "OEMEEP2K",
	}, {
		.id = PECI,
		.type = SMB,
		.label = "SMB_PECI",
	}, {
		.id = SMBOEM3,
		.type = SMB,
		.label = "SMBOEM3",
	}, {
		.id = SMLINK,
		.type = SMB,
		.label = "SMLINK",
	}, {
		.id = SMBSLV,
		.type = SMB,
		.label = "SMBSLV",
	}, {
		.id = PowerLED,
		.type = GPIO,
		.label = "Power LED",
	}, {
		.id = BatLEDG,
		.type = GPIO,
		.label = "Bat_LED_Green",
	}, {
		.id = OEMLED0,
		.type = GPIO,
		.label = "LED1",
	}, {
		.id = OEMLED1,
		.type = GPIO,
		.label = "LED2",
	}, {
		.id = OEMLED2,
		.type = GPIO,
		.label = "LED3",
	}, {
		.id = BatLEDR,
		.type = GPIO,
		.label = "Bat_LED_Red",
	}, {
		.id = SmartBat1,
		.type = SMB,
		.label = "SmartBat1",
	}, {
		.id = SmartBat2,
		.type = SMB,
		.label = "SmartBat2",
	}, {
		.id = ADCCMOSBAT,
		.type = ADC,
		.label = "VBat",
	}, {
		.id = ADCCMOSBATx2,
		.type = ADC,
		.label = "VBat",
	}, {
		.id = ADCCMOSBATx10,
		.type = ADC,
		.label = "VBat",
	}, {
		.id = ADCBAT,
		.type = ADC,
		.label = "VBat2",
	}, {
		.id = ADCBATx2,
		.type = ADC,
		.label = "VBat2",
	}, {
		.id = ADCBATx10,
		.type = ADC,
		.label = "VBat2",
	}, {
		.id = ADC5VS0,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC5VS0x2,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC5VS0x10,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC5VS5,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC5VS5x2,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC5VS5x10,
		.type = ADC,
		.label = "+5V",
	}, {
		.id = ADC33VS0,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC33VS0x2,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC33VS0x10,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC33VS5,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC33VS5x2,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC33VS5x10,
		.type = ADC,
		.label = "+3.3V",
	}, {
		.id = ADC12VS0,
		.type = ADC,
		.label = "+12V",
	}, {
		.id = ADC12VS0x2,
		.type = ADC,
		.label = "+12V",
	}, {
		.id = ADC12VS0x10,
		.type = ADC,
		.label = "+12V",
	}, {
		.id = ADCVCOREA,
		.type = ADC,
		.label = "VCore",
	}, {
		.id = ADCVCOREAx2,
		.type = ADC,
		.label = "VCore",
	}, {
		.id = ADCVCOREAx10,
		.type = ADC,
		.label = "VCore",
	}, {
		.id = ADCVCOREB,
		.type = ADC,
		.label = "VCore2",
	}, {
		.id = ADCVCOREBx2,
		.type = ADC,
		.label = "VCore2",
	}, {
		.id = ADCVCOREBx10,
		.type = ADC,
		.label = "VCore2",
	}, {
		.id = ADCDC,
		.type = ADC,
		.label = "ADCDC",
	}, {
		.id = ADCDCx2,
		.type = ADC,
		.label = "ADCDCx2",
	}, {
		.id = ADCDCx10,
		.type = ADC,
		.label = "ADCDCx10",
	}, {
		.id = ADCDCSTBY,
		.type = ADC,
		.label = "Vsb",
	}, {
		.id = ADCDCSTBYx2,
		.type = ADC,
		.label = "Vsb",
	}, {
		.id = ADCDCSTBYx10,
		.type = ADC,
		.label = "Vsb",
	}, {
		.id = ADCDCOther,
		.type = ADC,
		.label = "ADCDCOther",
	}, {
		.id = ADCDCOtherx2,
		.type = ADC,
		.label = "ADCDCOtherx2",
	}, {
		.id = ADCDCOtherx10,
		.type = ADC,
		.label = "ADCDCOtherx10",
	}, {
		.id = ADCCurrent,
		.type = ADC,
		.label = "VImon",
	}, {
		.id = WDIRQ,
		.type = IRQ,
		.label = "WDIRQ",
	}, {
		.id = WDNMI,
		.type = GPIO,
		.label = "WDNMI",
	}, {
		.id = TACHO0,
		.type = TACH,
		.label = "Tacho1",
	}, {
		.id = TACHO1,
		.type = TACH,
		.label = "Tacho2",
	}, {
		.id = TACHO2,
		.type = TACH,
		.label = "Tacho3",
	}, {
		.id = PWMBRIGHTNESS2,
		.type = PWM,
		.label = "Brightness2",
	}, {
		.id = BACKLIGHT1,
		.type = GPIO,
		.label = "Backlight1",
	}, {
		.id = BACKLIGHT2,
		.type = GPIO,
		.label = "Backlight2",
	},
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

static int _ec_read(u8 cmd)
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

static int _ec_write(u8 cmd, u8 value)
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
	return ((ec_inb(addr, 0x20) << 8) | ec_inb(addr, 0x21));
}

static int ec_wait_cmd_clear(void)
{
	int i = 0;

	do {
		if (!_ec_read(0))
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

	_ec_write(EC_MSG_OFFSET_PARAM, bank);
	_ec_write(EC_MSG_OFFSET_DATA(0), offset);
	_ec_write(EC_MSG_OFFSET_DATA(0x2C), len);
	_ec_write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_RD);

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ret = _ec_read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	for (i = 0; (i < len) && (len < EC_MSG_SIZE) && (len <= bufsz); i++)
		buf[i] = _ec_read(EC_MSG_OFFSET_DATA(i + 1));

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

	_ec_write(EC_MSG_OFFSET_PARAM, bank);
	_ec_write(EC_MSG_OFFSET_DATA(0), offset);
	_ec_write(EC_MSG_OFFSET_DATA(0x2C), len);

	for (i = 0; (i < len) && (len < EC_MSG_SIZE); i++)
		_ec_write(EC_MSG_OFFSET_DATA(i + 1), buf[i]);

	_ec_write(EC_MSG_OFFSET_CMD, EC_CMD_RAM_WR);

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	ret = _ec_read(EC_MSG_OFFSET_STATUS);
	if (ret != EC_STATUS_SUCCESS)
		return -EIO;

	return 0;
}

static int ec_read_dynamic_devtbl(void)
{
	size_t i, j;
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

	memset(ec.cfg, 0, sizeof(ec.cfg));

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, 0x00, &did);
	if (ret)
		return -EIO;

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, 0x01, &hwp);
	if (ret)
		return -EIO;

	ret = imanager_msg_read(EC_CMD_DYN_TBL_RD, 0x02, &pol);
	if (ret)
		return -EIO;

	for (i = 0; i < EC_MAX_DID; i++) {
		if (!did.u.data[i])
			break;
		for (j = 0; j < ARRAY_SIZE(devtbl); j++) {
			if (devtbl[j].id == did.u.data[i]) {
				ec.cfg[i].type = devtbl[j].type;
				ec.cfg[i].label = devtbl[j].label;
				ec.cfg[i].attr.did = did.u.data[i];
				ec.cfg[i].attr.hwp = hwp.u.data[i];
				ec.cfg[i].attr.pol = pol.u.data[i];
				break;
			}
		}
	}

	return 0;
}

static int ec_init(void)
{
	int ret;

	ec_clear_ports();

	ret = ec_read_dynamic_devtbl();
	if (ret)
		return ret;

	return 0;
}

static int ec_get_version(struct ec_version *version)
{
	int ret;
	u16 raw;
	struct ec_version_raw ver;

	if (WARN_ON(!version))
		return -EINVAL;

	ret = ec_read_ram(ACPI, EC_ACPIRAM_FW_RELEASE_RD, sizeof(ver),
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
	strncpy(info->pcb_name, (const char *)msg.u.data, PCB_NAME_READ_SIZE);

	if (strchr(info->pcb_name, '-') == NULL)
		info->pcb_name[PCB_NAME_READ_SIZE - 1] = '\0';

	return 0;
}

static int ec_read_buffer(u8 *data, int rlen)
{
	int i, j;
	int ret;
	int pages = rlen % EC_I2C_BLOCK_SIZE;
	int reminder = rlen / EC_I2C_BLOCK_SIZE;

	/* pre-condition: rlen <= 256 */

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	for (i = 0; i < pages; i++) {
		_ec_write(EC_MSG_OFFSET_PARAM, i);
		_ec_write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear();
		if (ret)
			return ret;

		ret = _ec_read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < EC_I2C_BLOCK_SIZE; j++)
			data[i * EC_I2C_BLOCK_SIZE + j] =
				_ec_read(EC_MSG_OFFSET_DATA(j));
	}

	if (reminder) {
		_ec_write(EC_MSG_OFFSET_PARAM, pages);
		_ec_write(EC_MSG_OFFSET_CMD, EC_CMD_BUF_RD);

		ret = ec_wait_cmd_clear();
		if (ret)
			return ret;

		ret = _ec_read(EC_MSG_OFFSET_STATUS);
		if (ret != EC_STATUS_SUCCESS)
			return -EIO;

		for (j = 0; j < reminder; j++)
			data[pages * EC_I2C_BLOCK_SIZE + j] =
				_ec_read(EC_MSG_OFFSET_DATA(j));
	}

	return 0;
}

int imanager_get_fw_info(struct ec_info *info)
{
	int ret;

	if (WARN_ON(!info))
		return -EINVAL;

	ret = ec_get_version(&info->version);
	if (ret)
		return ret;

	return ec_get_pcb_name(info);
}
EXPORT_SYMBOL_GPL(imanager_get_fw_info);

static int imanager_msg_trans(u8 cmd, u8 param, struct ec_message *msg, bool payload)
{
	int ret, i, len;
	u32 offset;

	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	_ec_write(EC_MSG_OFFSET_PARAM, param);

	if (msg && msg->wlen) {
		if (!msg->data) {
			for (i = 0; i < msg->wlen; i++)
				_ec_write(EC_MSG_OFFSET_DATA(i),
					msg->u.data[i]);
		} else {
			for (i = 0; i < msg->wlen; i++)
				_ec_write(EC_MSG_OFFSET_DATA(i), msg->data[i]);
			_ec_write(EC_MSG_OFFSET_DATA(0x2c), msg->wlen);
		}
	}

	_ec_write(EC_MSG_OFFSET_CMD, cmd);
	ret = ec_wait_cmd_clear();
	if (ret)
		return ret;

	/* GPIO and I2C have different success return values */
	ret = _ec_read(EC_MSG_OFFSET_STATUS);
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
			len = _ec_read(EC_MSG_OFFSET_DATA(0x2C));
		else
			len = (msg->rlen > EC_MSG_SIZE ? EC_MSG_SIZE :
				msg->rlen);
		offset = payload ? EC_MSG_OFFSET_PAYLOAD(0) : EC_MSG_OFFSET_DATA(0);
		for (i = 0; i < len; i++)
			msg->u.data[i] = _ec_read(offset + i);
	}

	return 0;
}

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
			.data = {byte, 0},
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
			.data = {HIBYTE16(word), LOBYTE16(word), 0},
		},
	};

	return imanager_msg_write(cmd, param, &msg);
}
EXPORT_SYMBOL_GPL(imanager_write_word);

static int ec_hwram_read_byte(u8 offset)
{
	int ret;
	u8 val;

	ret = ec_read_ram(HW, offset, sizeof(val), &val, sizeof(val));
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

	ret = ec_read_ram(ACPI, offset, sizeof(value), (u8 *)&value,
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

	ret = ec_write_ram(ACPI, offset, sizeof(value), &value);
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

	ret = ec_read_ram(ACPI, offset, len, buf, len);
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

	ret = ec_write_ram(ACPI, offset, len, buf);
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

int imanager_get_adc_cfg(struct adc_cfg *adc)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!adc))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == ADC) {
			switch (cfg->attr.did) {
			case ADC12VS0:
				adc[0].did = cfg->attr.did;
				adc[0].scale = 1;
				adc[0].label = cfg->label;
				break;
			case ADC12VS0x2:
				adc[0].did = cfg->attr.did;
				adc[0].scale = 2;
				adc[0].label = cfg->label;
				break;
			case ADC12VS0x10:
				adc[0].did = cfg->attr.did;
				adc[0].scale = 10;
				adc[0].label = cfg->label;
				break;
			case ADC5VS5:
				adc[1].did = cfg->attr.did;
				adc[1].scale = 1;
				adc[1].label = cfg->label;
				break;
			case ADC5VS5x2:
				adc[1].did = cfg->attr.did;
				adc[1].scale = 2;
				adc[1].label = cfg->label;
				break;
			case ADC5VS5x10:
				adc[1].did = cfg->attr.did;
				adc[1].scale = 10;
				adc[1].label = cfg->label;
				break;
			case ADCCMOSBAT:
				adc[2].did = cfg->attr.did;
				adc[2].scale = 1;
				adc[2].label = cfg->label;
				break;
			case ADCCMOSBATx2:
				adc[2].did = cfg->attr.did;
				adc[2].scale = 2;
				adc[2].label = cfg->label;
				break;
			case ADCCMOSBATx10:
				adc[2].did = cfg->attr.did;
				adc[2].scale = 10;
				adc[2].label = cfg->label;
				break;
			case ADCVCOREA:
				adc[3].did = cfg->attr.did;
				adc[3].scale = 1;
				adc[3].label = cfg->label;
				break;
			case ADC5VS0:
				adc[3].did = cfg->attr.did;
				adc[3].scale = 1;
				adc[3].label = cfg->label;
				break;
			case ADC5VS0x2:
				adc[3].did = cfg->attr.did;
				adc[3].scale = 2;
				adc[3].label = cfg->label;
				break;
			case ADC5VS0x10:
				adc[3].did = cfg->attr.did;
				adc[3].scale = 10;
				adc[3].label = cfg->label;
				break;
			case ADCCurrent:
				adc[4].did = cfg->attr.did;
				adc[4].scale = 1;
				adc[4].label = cfg->label;
				break;
			case ADC33VS0:
				adc[4].did = cfg->attr.did;
				adc[4].scale = 1;
				adc[4].label = cfg->label;
				break;
			case ADC33VS0x2:
				adc[4].did = cfg->attr.did;
				adc[4].scale = 2;
				adc[4].label = cfg->label;
				break;
			case ADC33VS0x10:
				adc[4].did = cfg->attr.did;
				adc[4].scale = 10;
				adc[4].label = cfg->label;
				break;
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_adc_cfg);

int imanager_get_fan_cfg(struct fan_cfg *fan)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!fan))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == TACH || cfg->type == PWM) {
			switch (cfg->attr.did) {
			case CPUFAN_2P:
				fan[0].did = cfg->attr.did;
				fan[0].hwp = cfg->attr.hwp;
				fan[0].pulse = 1;
				fan[0].label = cfg->label;
				break;
			case CPUFAN_4P:
				fan[0].did = cfg->attr.did;
				fan[0].hwp = cfg->attr.hwp;
				fan[0].pulse = 2;
				fan[0].label = cfg->label;
				break;
			case SYSFAN1_2P:
				fan[1].did = cfg->attr.did;
				fan[1].hwp = cfg->attr.hwp;
				fan[1].pulse = 1;
				fan[1].label = cfg->label;
				break;
			case SYSFAN1_4P:
				fan[1].did = cfg->attr.did;
				fan[1].hwp = cfg->attr.hwp;
				fan[1].pulse = 2;
				fan[1].label = cfg->label;
				break;
			case SYSFAN2_2P:
				fan[2].did = cfg->attr.did;
				fan[2].hwp = cfg->attr.hwp;
				fan[2].pulse = 1;
				fan[2].label = cfg->label;
				break;
			case SYSFAN2_4P:
				fan[2].did = cfg->attr.did;
				fan[2].hwp = cfg->attr.hwp;
				fan[2].pulse = 2;
				fan[2].label = cfg->label;
				break;
			case TACHO0:
			case TACHO1:
			case TACHO2:
			case PWMBRIGHTNESS:
			case PWMBRIGHTNESS2:
			case PWMBEEP:
				break;
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_fan_cfg);

int imanager_get_backlight_cfg(struct backlight_cfg *blc)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!blc))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == PWM) {
			switch (cfg->attr.did) {
			case PWMBRIGHTNESS:
				blc[0].did = cfg->attr.did;
				blc[0].label = cfg->label;
				break;
			case PWMBRIGHTNESS2:
				blc[1].did = cfg->attr.did;
				blc[1].label = cfg->label;
				break;
			case CPUFAN_2P:
			case CPUFAN_4P:
			case SYSFAN1_2P:
			case SYSFAN1_4P:
			case SYSFAN2_2P:
			case SYSFAN2_4P:
			case PWMBEEP:
			case TACHO0:
			case TACHO1:
			case TACHO2:
				break;
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_backlight_cfg);

int imanager_get_gpio_cfg(struct gpio_cfg *gpio)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!gpio))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == GPIO) {
			switch (cfg->attr.did) {
			case AltGPIO0:
				gpio[0].did = cfg->attr.did;
				gpio[0].label = cfg->label;
				break;
			case AltGPIO1:
				gpio[1].did = cfg->attr.did;
				gpio[1].label = cfg->label;
				break;
			case AltGPIO2:
				gpio[2].did = cfg->attr.did;
				gpio[2].label = cfg->label;
				break;
			case AltGPIO3:
				gpio[3].did = cfg->attr.did;
				gpio[3].label = cfg->label;
				break;
			case AltGPIO4:
				gpio[4].did = cfg->attr.did;
				gpio[4].label = cfg->label;
				break;
			case AltGPIO5:
				gpio[5].did = cfg->attr.did;
				gpio[5].label = cfg->label;
				break;
			case AltGPIO6:
				gpio[6].did = cfg->attr.did;
				gpio[6].label = cfg->label;
				break;
			case AltGPIO7:
				gpio[7].did = cfg->attr.did;
				gpio[7].label = cfg->label;
				break;
			case Btn0:
			case Btn1:
			case Btn2:
			case Btn3:
			case Btn4:
			case Btn5:
			case Btn6:
			case Btn7:
			case PowerLED:
			case BatLEDG:
			case OEMLED0:
			case OEMLED1:
			case OEMLED2:
			case BatLEDR:
			case WDNMI:
			case BACKLIGHT1:
			case BACKLIGHT2:
				break;
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_gpio_cfg);

int imanager_get_i2c_cfg(struct i2c_cfg *i2c)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!i2c))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == SMB) {
			switch (cfg->attr.did) {
			case SMBEEPROM:
				i2c->smbeeprom = cfg->attr.did;
				break;
			case I2COEM:
				i2c->i2coem = cfg->attr.did;
				break;
			case SMBOEM0:
			case SMBOEM1:
			case SMBOEM2:
			case SMBTHERMAL0:
			case SMBTHERMAL1:
			case SMBSecurityEEP:
			case SMBEEP2K:
			case OEMEEP:
			case OEMEEP2K:
			case PECI:
			case SMBOEM3:
			case SMLINK:
			case SMBSLV:
			case SmartBat1:
			case SmartBat2:
				break;
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_i2c_cfg);

int imanager_get_wdt_cfg(struct wdt_cfg *wdt)
{
	size_t i;
	struct ec_dev_cfg *cfg;

	if (WARN_ON(!wdt))
		return -EINVAL;

	for (i = 0; i < ARRAY_SIZE(ec.cfg); i++) {
		cfg = &ec.cfg[i];
		if (cfg->type == IRQ) {
			switch (cfg->attr.did) {
			case WDIRQ:
				wdt->irq = cfg->attr.did;
				break;
			/* WDNMI is not handled since it is of type 'GPIO' */
			default:
				pr_err("DID 0x%02X not handled\n",
					cfg->attr.did);
				return -EINVAL;
			}
		}
	}

	return 0;
}
EXPORT_SYMBOL_GPL(imanager_get_wdt_cfg);

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

int imanager_get_chipid(void)
{
	if (!ec.chipid)
		return -EINVAL;

	return ec.chipid;
}
EXPORT_SYMBOL_GPL(imanager_get_chipid);
