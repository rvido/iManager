/*
 * Advantech iManager Core (EC IT8518/28) - firmware interface
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __EC_H__
#define __EC_H__

#include <linux/types.h>

#define SIO_DEVID_IT8516		0x8516
#define SIO_DEVID_IT8518		0x8518
#define SIO_DEVID_IT8528		0x8528

#define SIO_BASE_ADDR			0x029C

#define IT8516_CMD_PORT			0x029A /* it8528 */
#define IT8516_DAT_PORT			0x0299

#define IT8518_CMD_PORT			0x029E /* it8518 */
#define IT8518_DAT_PORT			0x029F

#define EC_GPIO_MAX_NUM			8
#define EC_HWM_MAX_ADC			5
#define EC_HWM_MAX_FAN			3
#define EC_BLC_MAX_NUM			2

#define PCB_NAME_SIZE			32
#define EC_PAYLOAD_SIZE			40
#define EC_MSG_SIZE			sizeof(struct ec_smb_message)

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
#define EC_CMD_DYN_TBL_RD		0x20UL
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
#define EC_ACPIRAM_FAN_ALERT		0x6F
#define EC_ACPIRAM_FAN_SPEED_LIMIT	0x76
#define EC_ACPIRAM_BLC_LEVEL1		0x50
#define EC_ACPIRAM_BLC_LEVEL2		0x52
#define EC_ACPIRAM_BLC_CTRL		0x99
#define EC_ACPIRAM_FW_RELEASE_RD	0xF8
#define EC_ACPIRAM_THERMAL_LOCAL1	0x60
#define EC_ACPIRAM_THERMAL_LOCAL2	0x63
#define EC_ACPIRAM_THERMAL_REMOTE1	0x61
#define EC_ACPIRAM_THERMAL_REMOTE2	0x64

struct ec_message_header {
	u8 addr_low;	/* SMB low-byte address or data byte
			   (low-byte) of byte-/word-transaction */
	u8 addr_high;	/* SMB high-byte address or
			   data high-byte of word-transaction */
	u8 rlen;	/* SMB read length */
	u8 wlen;	/* SMB write length */
	u8 cmd;		/* SMB command */
};

struct ec_smb_message {
	struct ec_message_header hdr;
	u8 data[EC_PAYLOAD_SIZE];
};

struct ec_message {
	u8 rlen;	/* EC message read length */
	u8 wlen;	/* EC message write length */
	union {
		struct ec_smb_message smb;
		u8 data[EC_MSG_SIZE];
	} u;

	u8 *data;
};

struct ec_version {
	u32 kernel_major;
	u32 kernel_minor;
	u32 firmware_major;
	u32 firmware_minor;
	u32 type;
};

struct ec_info {
	struct ec_version version;
	char pcb_name[PCB_NAME_SIZE];
};

enum ec_ram_type {
	ACPI = 1,
	HW,
	EXT
};

struct adc_cfg {
	u32		did;		/* Device ID */
	u32		scale;		/* Scaling factor */
	const char	*label;
};

struct fan_cfg {
	u32		did;
	u32		hwp;		/* Hardware Pin number */
	u32		pulse;		/* FAN pulse type */
	const char	*label;
	bool		has_thermal_sens;
};

struct gpio_cfg {
	u32		did;
	const char	*label;
};

struct backlight_cfg {
	u32		did;
	u32		blid;		/* Brightness Level ID */
	const char	*label;
};

struct i2c_cfg {
	u32		smbeeprom;
	u32		i2coem;
	const char	*label;
};

struct wdt_cfg {
	u32		irq;
	const char	*label;
};

/* imanager_ec_probe() MUST be called first to ensure proper communication */
int imanager_ec_probe(u16 addr);
int imanager_get_chipid(void);

int imanager_get_fw_info(struct ec_info *info);

int imanager_msg_write(u8 cmd, u8 param, struct ec_message *msg);
int imanager_msg_read(u8 cmd, u8 param, struct ec_message *msg);

int imanager_read_byte(u8 cmd, u8 param);
int imanager_read_word(u8 cmd, u8 param);

int imanager_write_byte(u8 cmd, u8 param, u8 b);
int imanager_write_word(u8 cmd, u8 param, u16 w);

int imanager_acpiram_read_byte(u8 offset);
int imanager_acpiram_write_byte(u8 offset, u8 value);
int imanager_acpiram_read_block(u8 offset, u8 *buf, u8 len);
int imanager_acpiram_write_block(u8 offset, u8 *buf, u8 len);

int imanager_wait_proc_complete(u8 offset, int cond);

int imanager_get_adc_cfg(struct adc_cfg *adc);
int imanager_get_fan_cfg(struct fan_cfg *fan);
int imanager_get_gpio_cfg(struct gpio_cfg *gpio);
int imanager_get_i2c_cfg(struct i2c_cfg *i2c);
int imanager_get_wdt_cfg(struct wdt_cfg *wdt);
int imanager_get_backlight_cfg(struct backlight_cfg *blc);

#endif
