/*
 * Advantech iManager core - firmware interface
 *
 * Copyright (C) 2016 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __IMANAGER_EC_H__
#define __IMANAGER_EC_H__

#include <linux/types.h>

/**
 * This is the delay time between two EC transactions.
 * Values lower than 200us are not encouraged and may
 * cause I/O errors
 */
#define EC_DELAY_MIN			200 /* micro seconds */
#define EC_DELAY_MAX			210
#define EC_MAX_RETRY			500

#define EC_DEVID_IT8516			0x8516
#define EC_DEVID_IT8518			0x8518
#define EC_DEVID_IT8528			0x8528

#define EC_BASE_ADDR			0x029C

#define IT8516_CMD_PORT			0x029A /* it8528 */
#define IT8516_DAT_PORT			0x0299

#define IT8518_CMD_PORT			0x029E /* it8518 */
#define IT8518_DAT_PORT			0x029F

#define EC_GPIO_MAX_NUM			8
#define EC_HWM_MAX_ADC			5
#define EC_HWM_MAX_FAN			3
#define EC_BLC_MAX_NUM			2
#define EC_SMB_MAX_NUM			4
#define EC_WDT_MAX_NUM			2

#define PCB_NAME_SIZE			32
#define EC_PAYLOAD_SIZE			40
#define EC_MSG_SIZE			sizeof(struct ec_smb_message)
#define EC_MSG_HDR_SIZE			sizeof(struct ec_smb_message_header)

#define EC_MAX_DID			32UL
#define DID_LABEL_SIZE			24UL

#define LOBYTE16(x)			(x & 0x00FF)
#define HIBYTE16(x)			(LOBYTE16(x >> 8))
#define LOADDR16(x)			LOBYTE16(x)
#define HIADDR16(x)			(x >= 0xF000 ? LOBYTE16(x >> 8) : 0)

/*
 * iManager commands
 */
#define EC_CMD_HWP_RD			0x11UL
#define EC_CMD_HWP_WR			0x12UL
#define EC_CMD_GPIO_DIR_RD		0x30UL
#define EC_CMD_GPIO_DIR_WR		0x31UL
#define EC_CMD_PWM_FREQ_RD		0x36UL
#define EC_CMD_PWM_FREQ_WR		0x32UL
#define EC_CMD_PWM_POL_RD		0x37UL
#define EC_CMD_PWM_POL_WR		0x33UL
#define EC_CMD_SMB_FREQ_RD		0x34UL
#define EC_CMD_SMB_FREQ_WR		0x35UL
#define EC_CMD_FAN_CTL_RD		0x40UL
#define EC_CMD_FAN_CTL_WR		0x41UL
#define EC_CMD_THZ_RD			0x42UL
#define EC_CMD_DEV_TBL_RD		0x20UL
#define EC_CMD_FW_INFO_RD		0xF0UL
#define EC_CMD_BUF_CLR			0xC0UL
#define EC_CMD_BUF_RD			0xC1UL
#define EC_CMD_BUF_WR			0xC2UL
#define EC_CMD_RAM_RD			0x1EUL
#define EC_CMD_RAM_WR			0x1FUL
#define EC_CMD_I2C_RW			0x0EUL
#define EC_CMD_I2C_WR			0x0FUL
#define EC_CMD_WDT_CTRL			0x28UL

/*
 * ACPI and HW RAM offsets
 */
#define EC_ACPIRAM_FAN_ALERT		0x6FUL
#define EC_ACPIRAM_FAN_SPEED_LIMIT	0x76UL
#define EC_ACPIRAM_BRIGHTNESS1		0x50UL
#define EC_ACPIRAM_BRIGHTNESS2		0x52UL
#define EC_ACPIRAM_BLC_CTRL		0x99UL
#define EC_ACPIRAM_FW_RELEASE_RD	0xF8UL

/*
 * iManager feature flags
 */
#define IMANAGER_FEATURE_BACKLIGHT	BIT(0)
#define IMANAGER_FEATURE_GPIO		BIT(1)
#define IMANAGER_FEATURE_HWMON		BIT(2)
#define IMANAGER_FEATURE_HWMON_FAN1	BIT(3)
#define IMANAGER_FEATURE_HWMON_FAN2	BIT(4)
#define IMANAGER_FEATURE_HWMON_FAN3	BIT(5)
#define IMANAGER_FEATURE_I2C		BIT(6)
#define IMANAGER_FEATURE_WDT		BIT(7)

#define EC_FLAG_OUTBUF			BIT(0)
#define EC_FLAG_INBUF			BIT(1)

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

#define EC_STATUS_SUCCESS		BIT(0)
#define EC_STATUS_CMD_COMPLETE		BIT(7)

#define PCB_NAME_MAX_SIZE		8UL
#define EC_I2C_BLOCK_SIZE		32UL

#define EC_KERNEL_MINOR(x)		(LOBYTE16(x))
#define EC_KERNEL_MAJOR(x) ({		\
	typeof(x) __x = (HIBYTE16(x));	\
	((__x >> 4) * 10 + (__x & 0x000f)); })
#define EC_FIRMWARE_MINOR(x)		(LOBYTE16(x))
#define EC_FIRMWARE_MAJOR(x)		EC_KERNEL_MAJOR(x)
#define EC_PROJECT_CODE(x)		((char)(LOBYTE16(x)))

enum kinds { IT8516, IT8518, IT8528 };

enum ec_ram_type {
	EC_RAM_ACPI = 1,
	EC_RAM_HW,
	EC_RAM_EXT
};

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
	CMOSBAT_2,
	CMOSBAT_10,
	LIBAT,
	LIBAT_2,
	LIBAT_10,
	ADC5VS0,
	ADC5VS0_2,
	ADC5VS0_10,
	ADC5VS5,
	ADC5VS5_2,
	ADC5VS5_10,
	ADC33VS0,
	ADC33VS0_2,
	ADC33VS0_10,
	ADC33VS5,
	ADC33VS5_2,	/* 0x60 */
	ADC33VS5_10,
	ADC12VS0,
	ADC12VS0_2,
	ADC12VS0_10,
	VCOREA,
	VCOREA_2,
	VCOREA_10,
	VCOREB,
	VCOREB_2,
	VCOREB_10,
	ADCDC,
	ADCDC_2,
	ADCDC_10,
	VSTBY,
	VSTBY_2,
	VSTBY_10,	/* 0x70 */
	VAUX,
	VAUX_2,
	VAUX_10,
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

enum ec_device_table_type {
	EC_DT_DID,
	EC_DT_HWP,
	EC_DT_POL
};

struct ec_devtbl {
	int id;
	int type;
	int scale;
	char label[DID_LABEL_SIZE];
};

struct ec_dyn_devtbl {
	int did;	/* Device ID */
	int hwp;	/* Hardware Pin */
	int pol;	/* Polarity */
	const struct ec_devtbl *devtbl; /* Device table Entry */
};

static const struct ec_devtbl devtbl[] = {
	/* ID		Type	Scale	Label */
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
	{ BUTTON5,	GPIO,	-1,	"button5" },
	{ BUTTON6,	GPIO,	-1,	"button6" },
	{ BUTTON7,	GPIO,	-1,	"button7" },
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
	{ CMOSBAT_2,	ADC,	2,	"VBat" },
	{ CMOSBAT_10,	ADC,	10,	"VBat" },
	{ LIBAT,	ADC,	1,	"VBat2" },
	{ LIBAT_2,	ADC,	2,	"VBat2" },
	{ LIBAT_10,	ADC,	10,	"VBat2" },
	{ ADC5VS0,	ADC,	1,	"+5V" },
	{ ADC5VS0_2,	ADC,	2,	"+5V" },
	{ ADC5VS0_10,	ADC,	10,	"+5V" },
	{ ADC5VS5,	ADC,	1,	"+5V" },
	{ ADC5VS5_2,	ADC,	2,	"+5V" },
	{ ADC5VS5_10,	ADC,	10,	"+5V" },
	{ ADC33VS0,	ADC,	1,	"+3.3V" },
	{ ADC33VS0_2,	ADC,	2,	"+3.3V" },
	{ ADC33VS0_10,	ADC,	10,	"+3.3V" },
	{ ADC33VS5,	ADC,	1,	"+3.3V" },
	{ ADC33VS5_2,	ADC,	2,	"+3.3V" },
	{ ADC33VS5_10,	ADC,	10,	"+3.3V" },
	{ ADC12VS0,	ADC,	1,	"+12V" },
	{ ADC12VS0_2,	ADC,	2,	"+12V" },
	{ ADC12VS0_10,	ADC,	10,	"+12V" },
	{ VCOREA,	ADC,	1,	"VCore" },
	{ VCOREA_2,	ADC,	2,	"VCore" },
	{ VCOREA_10,	ADC,	10,	"VCore" },
	{ VCOREB,	ADC,	1,	"VCore2" },
	{ VCOREB_2,	ADC,	2,	"VCore2" },
	{ VCOREB_10,	ADC,	10,	"VCore2" },
	{ ADCDC,	ADC,	1,	"ADCDC" },
	{ ADCDC_2,	ADC,	2,	"ADCDCx2" },
	{ ADCDC_10,	ADC,	10,	"ADCDCx10" },
	{ VSTBY,	ADC,	1,	"Vsb" },
	{ VSTBY_2,	ADC,	2,	"Vsb" },
	{ VSTBY_10,	ADC,	10,	"Vsb" },
	{ VAUX,		ADC,	1,	"VAUX" },
	{ VAUX_2,	ADC,	2,	"VAUX" },
	{ VAUX_10,	ADC,	10,	"VAUX" },
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

struct ec_smb_message_header {
	u8 addr_low;	/* SMB low-byte address or data low-byte */
			/* of byte-/word-transaction */
	u8 addr_high;	/* SMB high-byte address or data high-byte */
			/* of word-transaction */
	u8 rlen;	/* SMB read length */
	u8 wlen;	/* SMB write length */
	u8 cmd;		/* SMB command */
} __attribute__((__packed__));

struct ec_smb_message {
	struct ec_smb_message_header hdr;
	u8 data[EC_PAYLOAD_SIZE];
} __attribute__((__packed__));

struct ec_version_raw {
	u16	kernel,
		chipid,
		project_code,
		firmware;
} __attribute__((__packed__));

struct ec_message {
	unsigned rlen;	/* EC message read length */
	unsigned wlen;	/* EC message write length */
	unsigned param;	/* Message parameter (offset, id, or unit) */
	union {
		struct ec_smb_message smb;
		u8 data[EC_MSG_SIZE];
	} u;

	u8 *data;
};

struct ec_version {
	unsigned kernel_major;
	unsigned kernel_minor;
	unsigned firmware_major;
	unsigned firmware_minor;
	unsigned type;
};

struct ec_info {
	unsigned chipid;
	enum kinds kind;
	struct ec_version version;
	char pcb_name[PCB_NAME_SIZE];
};

struct ec_dev_attr {
	int did;	/* Device ID */
	int hwp;	/* Hardware Pin number */
	int pol;	/* Polarity */
	int scale;	/* Scaling factor */
	const char *label;
};

struct imanager_gpio_device {
	unsigned		num;
	struct ec_dev_attr	attr[EC_GPIO_MAX_NUM];
};

struct ec_dev_adc {
	unsigned		num;
	struct ec_dev_attr	attr[EC_HWM_MAX_ADC];
};

struct ec_dev_fan {
	unsigned		num;
	struct ec_dev_attr	attr[EC_HWM_MAX_FAN];
	const char		*temp_label[EC_HWM_MAX_FAN];
};

struct imanager_hwmon_device {
	struct ec_dev_adc	adc;
	struct ec_dev_fan	fan;
};

struct imanager_i2c_device {
	unsigned		num;
	struct ec_dev_attr	attr[EC_SMB_MAX_NUM];
	struct ec_dev_attr	*eeprom;
	struct ec_dev_attr	*i2coem;
};

struct imanager_backlight_device {
	unsigned		num;
	struct ec_dev_attr	attr[EC_BLC_MAX_NUM];
	u8			brightness[EC_BLC_MAX_NUM];
};

struct imanager_watchdog_device {
	unsigned		num;
	struct ec_dev_attr	attr[EC_WDT_MAX_NUM];
	struct ec_dev_attr	*irq;
	struct ec_dev_attr	*nmi;
};

struct imanager_device {
	struct ec_info	info;

	unsigned features;

	struct imanager_gpio_device		gpio;
	struct imanager_hwmon_device		hwmon;
	struct imanager_i2c_device		i2c;
	struct imanager_backlight_device	bl;
	struct imanager_watchdog_device		wdt;
};

struct imanager_io_ops {
	int (*read)(int cmd);
	int (*write)(int cmd, int value);
};

struct imanager_ec_data {
	struct imanager_io_ops io;
	struct ec_dyn_devtbl dyn[EC_MAX_DID];
	struct imanager_device idev;
};

#endif
