/*
 * Advantech iManager MFD
 *
 * Copyright (C) 2016 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef _LINUX_MFD_IMANAGER_H_
#define _LINUX_MFD_IMANAGER_H_

#include <linux/mutex.h>
#include "imanager-ec.h"

#define IMANAGER_MSG_SIMPLE(read_len, write_len, parameter, _data) \
	.rlen = (read_len), .wlen = (write_len), \
	.param = (parameter), .data = (_data)

/**
 * struct imanager_ec_message - Describes iManager EC message
 * @rlen:	iManager message read length
 * @wlen:	iManager message write length
 * @param:	iManager message parameter (offset, id, or unit number)
 * @u:		union holding struct imanager_ec_smb_message and data field
 * @data:	pointer to data field - source or target
 */
struct imanager_ec_message {
	unsigned int rlen;
	unsigned int wlen;
	unsigned int param;
	union {
		struct imanager_ec_smb_message smb;
		unsigned char data[EC_MSG_SIZE];
	} u;

	unsigned char *data;
};

/**
 * struct imanager_device_attribute - Describes iManager Device attribute
 * @did:	iManager Device ID
 * @hwp:	iManager Hardware Pin number
 * @pol:	iManager Device Polarity
 * @devtbl:	pointer to iManager device table entry
 */
struct imanager_device_attribute {
	unsigned int did;
	unsigned int hwp;
	unsigned int pol;
	const struct imanager_device_table_entry *devtbl;
};

/**
 * struct imanager_gpio_device - Describes iManager GPIO device
 * @num:	available GPIO pins
 * @attr:	pointer to array of iManager GPIO device attribute
 */
struct imanager_gpio_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_GPIO_NUM];
};

/**
 * struct imanager_adc_device - Describes iManager ADC device
 * @num:	available ADC devices
 * @attr:	pointer to array of iManager ADC device attribute
 * @label	pointer to ADC label
 */
struct imanager_adc_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_ADC_NUM];
	const char *label[EC_MAX_ADC_NUM];
};

/**
 * struct imanager_fan_device - Describes iManager FAN device
 * @num:	available FAN devices
 * @attr:	pointer to array of iManager FAN device attribute
 * @label	pointer to FAN label
 * @temp_label	pointer to FAN temperature label
 */
struct imanager_fan_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_FAN_NUM];
	const char *label[EC_MAX_FAN_NUM];
	const char *temp_label[EC_MAX_FAN_NUM];
};

/**
 * struct imanager_hwmon_device - Describes iManager hwmon device
 * @adc:	iManager ADC device
 * @fan:	iManager FAN device
 */
struct imanager_hwmon_device {
	struct imanager_adc_device adc;
	struct imanager_fan_device fan;
};

/**
 * struct imanager_i2c_device - Describes iManager I2C device
 * @num:	available I2C devices
 * @attr:	pointer to array of iManager GPIO device attribute
 */
struct imanager_i2c_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_SMB_NUM];
};

/**
 * struct imanager_backlight_device - Describes iManager backlight device
 * @num:	available backlight devices
 * @attr:	pointer to array of iManager backlight device attribute
 * @brightnes:	array of brightness devices
 */
struct imanager_backlight_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_BLC_NUM];
	unsigned char brightness[EC_MAX_BLC_NUM];
};

/**
 * struct imanager_watchdog_device - Describes iManager watchdog device
 * @num:	available WD devices
 * @attr:	pointer to array of iManager watchdog device attribute
 */
struct imanager_watchdog_device {
	unsigned int num;
	struct imanager_device_attribute *attr[EC_MAX_BLC_NUM];
};

/**
 * struct imanager_info - iManager device information structure
 * @kernel_major:	iManager EC kernel major revision
 * @kernel_minor:	iManager EC kernel minor revision
 * @firmware_major:	iManager EC firmware major revision
 * @firmware_minor:	iManager EC firmware minor revision
 * @type:		iManager type - release/debug/custom
 * @pcb_name:		PC board name
 */
struct imanager_info {
	unsigned int kernel_major;
	unsigned int kernel_minor;
	unsigned int firmware_major;
	unsigned int firmware_minor;
	const char *type;
	char pcb_name[EC_MAX_LABEL_SIZE];
};

/**
 * struct imanager_ec_data - iManager EC data structure
 * @features:	iManager feature mask
 * @attr:	array of iManager device attribute structure
 * @io:		imanager_io_ops structure providing I/O operations
 * @gpio:	iManager GPIO device structure
 * @hwmon:	iManager Hardware monitor device structure
 * @i2c:	iManager I2C/SMBus device structure
 * @bl:		iManager Backlight/Brightness device structure
 * @wdt:	iManager Watchdog device structure
 */
struct imanager_ec_data {
	unsigned int features;
	const char *chip_name;
	struct imanager_device_attribute	attr[EC_MAX_DID];
	struct imanager_io_ops			io;
	struct imanager_gpio_device		gpio;
	struct imanager_hwmon_device		hwmon;
	struct imanager_i2c_device		i2c;
	struct imanager_backlight_device	bl;
	struct imanager_watchdog_device		wdt;
	struct imanager_info			info;
};

/**
 * struct imanager_device_data - Internal representation of the iManager device
 * @ec:		iManager data structure describing the EC
 * @dev:	Pointer to kernel device structure
 * @lock:	iManager mutex
 */
struct imanager_device_data {
	struct imanager_ec_data	ec;
	struct device		*dev;
	struct mutex		lock;
};

enum ec_ram_type { EC_RAM_ACPI = 1, EC_RAM_HW, EC_RAM_EXT };

int imanager_read(struct imanager_ec_data *ec, u8 cmd,
		  struct imanager_ec_message *msg);
int imanager_write(struct imanager_ec_data *ec, u8 cmd,
		   struct imanager_ec_message *msg);

int imanager_read8(struct imanager_ec_data *ec, u8 cmd, u8 param);
int imanager_write8(struct imanager_ec_data *ec, u8 cmd, u8 param, u8 byte);

int imanager_read16(struct imanager_ec_data *ec, u8 cmd, u8 param);
int imanager_write16(struct imanager_ec_data *ec, u8 cmd, u8 param, u16 word);

int imanager_read_ram(struct imanager_ec_data *ec, int ram_type, u8 offset,
		      u8 *buf, u8 len);
int imanager_write_ram(struct imanager_ec_data *ec, int ram_type, u8 offset,
		       u8 *data, u8 size);

#endif
