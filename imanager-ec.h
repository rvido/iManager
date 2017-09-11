/*
 * Advantech iManager - firmware interface
 *
 * Copyright (C) 2016-2017 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _LINUX_MFD_IMANAGER_EC_H_
#define _LINUX_MFD_IMANAGER_EC_H_

#include <linux/types.h>

/* Delay time for port polling in micro seconds */
#define EC_DELAY_MIN			200UL
#define EC_DELAY_MAX			250UL

#define EC_MAX_RETRIES			400UL

#define EC_REG_BASE			0x029C
#define EC_REG_DEVID			0x20	/* Device ID (2 bytes) */
#define EC_IT8518_ID			0x8518
#define EC_IT8528_ID			0x8528

#define IT8528_CMD_PORT			0x029A
#define IT8528_DAT_PORT			0x0299
#define IT8518_CMD_PORT			0x029E
#define IT8518_DAT_PORT			0x029F

#define EC_MAX_GPIO_NUM			8UL
#define EC_MAX_ADC_NUM			5UL
#define EC_MAX_FAN_NUM			3UL
#define EC_MAX_BLC_NUM			2UL
#define EC_MAX_SMB_NUM			4UL
#define EC_MAX_WDT_NUM			2UL
#define EC_MAX_DID			32UL

#define EC_PAYLOAD_SIZE			40UL
#define EC_MSG_SIZE			sizeof(struct imanager_ec_smb_message)
#define EC_MSG_HDR_SIZE			sizeof(struct imanager_ec_smb_msg_hdr)

/*
 * iManager commands
 */
#define EC_CMD_CHK_RDY			0UL
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
#define EC_CMD_DEVTBL_RD		0x20UL
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
 * ACPI/HW RAM offsets
 */
#define EC_OFFSET_FAN_ALERT		0x6FUL
#define EC_OFFSET_FAN_ALERT_LIMIT	0x76UL
#define EC_OFFSET_BRIGHTNESS1		0x50UL
#define EC_OFFSET_BRIGHTNESS2		0x52UL
#define EC_OFFSET_BACKLIGHT_CTRL	0x99UL
#define EC_OFFSET_FW_RELEASE		0xF8UL
#define EC_OFFSET_I2C_STATUS		0UL

/* iManager EC flags */
#define EC_IO28_OUTBUF			BIT(0)
#define EC_IO28_INBUF			BIT(1)

#define EC_F_SUCCESS			BIT(0)
#define EC_F_CMD_COMPLETE		BIT(7)
#define EC_F_HWMON_MSG			BIT(9)

#define EC_I2C_STATUS_MASK		0x7E

/* iManager message offsets */
#define EC_MSG_OFFSET(N)		(0UL + (N))
#define EC_MSG_OFFSET_CMD		EC_MSG_OFFSET(0)
#define EC_MSG_OFFSET_STATUS		EC_MSG_OFFSET(1)
#define EC_MSG_OFFSET_PARAM		EC_MSG_OFFSET(2)
#define EC_MSG_OFFSET_DATA		EC_MSG_OFFSET(3)
#define EC_MSG_OFFSET_MEM_DATA		EC_MSG_OFFSET(4)
#define EC_MSG_OFFSET_PAYLOAD		EC_MSG_OFFSET(7)
#define EC_MSG_OFFSET_LEN		EC_MSG_OFFSET(0x2F)

/* IT8528 based firmware require a read/write command offset. */
#define EC_CMD_OFFSET_READ		0xA0UL
#define EC_CMD_OFFSET_WRITE		0x50UL

#define EC_KERNEL_MINOR(x)		((x) & 0xff)
#define EC_KERNEL_MAJOR(x)		({ typeof(x) __x = (x >> 8); \
					((__x >> 4) * 10 + (__x & 0x0f)); })
#define EC_FIRMWARE_MINOR(x)		EC_KERNEL_MINOR(x)
#define EC_FIRMWARE_MAJOR(x)		EC_KERNEL_MAJOR(x)

enum imanager_smb_cells { SMB_EEP = 0, I2C_OEM, SMB_1, SMB_PECI };

enum imanager_device_type { ADC = 1, DAC, GPIO, IRQ, PWM, SMB };

enum imanager_device_id {
	/* GPIO */
	GPIO0 = 0x10, GPIO1, GPIO2, GPIO3, GPIO4, GPIO5, GPIO6, GPIO7,
	/* FAN */
	CPUFAN_2P = 0x20, CPUFAN_4P, SYSFAN1_2P, SYSFAN1_4P, SYSFAN2_2P,
	SYSFAN2_4P,
	/* Brightness Control */
	BRIGHTNESS = 0x26, BRIGHTNESS2 = 0x88,
	/* SMBus */
	SMBOEM0	 = 0x28, SMBOEM1, SMBOEM2, SMBEEPROM,
	SMBTHM0  = 0x2C, SMBTHM1, SMBSECEEP, I2COEM,
	SMBEEP2K = 0x38, OEMEEP, OEMEEP2K, SMBPECI,
	/* ADC */
	CMOSBAT  = 0x50, CMOSBAT_2, CMOSBAT_10,
	ADC5VS0  = 0x56, ADC5VS0_2, ADC5VS0_10,
	ADC5VS5  = 0x59, ADC5VS5_2, ADC5VS5_10,
	ADC33VS0 = 0x5C, ADC33VS0_2, ADC33VS0_10,
	ADC33VS5 = 0x5F, ADC33VS5_2, ADC33VS5_10,
	ADC12VS0 = 0x62, ADC12VS0_2, ADC12VS0_10,
	VCOREA   = 0x65, VCOREA_2, VCOREA_10,
	CURRENT  = 0x74,
	/* Watchdog */
	WDIRQ    = 0x78, WDNMI,
};

/**
 * struct imanager_ec_device - Describes iManager EC Device
 * @did:	iManager Device ID
 * @type:	iManager Device Type
 * @scale:	Scaling factor
 */
struct imanager_ec_device {
	unsigned int did;
	unsigned int type;
	unsigned int scale;
};

/**
 * IMANAGER_EC_DEVICE - macro used to describe a specific iManager device
 * @device_id:		the 8 bit iManager device ID
 * @device_type:	the iManager device type
 * @scaling_factor:	the iManager sensor device scaling factor
 *
 * This macro is used to create a struct imanager_ec_device that matches a
 * specific iManager device
 */
#define IMANAGER_EC_DEVICE(device_id, device_type, scaling_factor) \
	.did = (device_id), .type = (device_type), .scale = (scaling_factor)

/**
 * struct imanager_io_ops - iManager I/O operation structure
 * @read:	iManager read call-back
 * @write:	iManager write call-back
 */
struct imanager_io_ops {
	int (*read)(int cmd);
	int (*write)(int cmd, int value);
};

/**
 * struct imanager_ec_smb_msg_hdr - Defines iManager EC SMBus message header
 * @addr_low:	low-byte of word address (or data)
 * @addr_high:	high-byte of word address (or data)
 * @rlen:	SMB read length
 * @wlen:	SMB write length
 * @cmd:	SMB command
 */
struct imanager_ec_smb_msg_hdr {
	unsigned char addr_low;
	unsigned char addr_high;
	unsigned char rlen;
	unsigned char wlen;
	unsigned char cmd;
} __attribute__((__packed__));

/**
 * struct imanager_ec_smb_message - Defines iManager SMBus message
 * @hdr:	iManager SMBus message header
 * @data:	iManager SMBus message data field (payload)
 */
struct imanager_ec_smb_message {
	struct imanager_ec_smb_msg_hdr hdr;
	unsigned char data[EC_PAYLOAD_SIZE];
} __attribute__((__packed__));

/**
 * struct imanager_ec_version - Defines iManager EC firmware version structure
 * @kernel:		iManager EC FW kernel release
 * @chipid:		iManager EC chip ID
 * @project_code:	iManager EC FW status
 * @firmware:		iManager EC FW release
 */
struct imanager_ec_version {
	unsigned short kernel;
	unsigned short chipid;
	unsigned short project_code;
	unsigned short firmware;
} __attribute__((__packed__));

#endif
