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
 * struct imanager_io_ops - iManager I/O operation structure
 * @read:	iManager read call-back
 * @write:	iManager write call-back
 */
struct imanager_io_ops {
	int (*read)(int cmd);
	int (*write)(int cmd, int value);
};

/**
 * struct imanager_ec_data - iManager EC data structure
 * @features:	iManager feature mask
 * @cfg:	iManager EC device configuration structure
 * @io:		imanager_io_ops structure providing I/O operations
 * @gpio:	iManager GPIO device structure
 * @hwmon:	iManager Hardware monitor device structure
 * @i2c:	iManager I2C/SMBus device structure
 * @bl:		iManager Backlight/Brightness device structure
 * @wdt:	iManager Watchdog device structure
 */
struct imanager_ec_data {
	unsigned int features;
	struct imanager_device_config		cfg[EC_MAX_DID];
	struct imanager_io_ops			io;
	struct imanager_gpio_device		gpio;
	struct imanager_hwmon_device		hwmon;
	struct imanager_i2c_device		i2c;
	struct imanager_backlight_device	bl;
	struct imanager_watchdog_device		wdt;
	struct imanager_info			info;
	const char *chip_name;
};

/**
 * struct imanager_device_data - Internal representation of the iManager device
 * @ec:		iManager embedded controller device data
 * @dev:	Pointer to kernel device structure
 * @lock:	iManager mutex
 */
struct imanager_device_data {
	struct imanager_ec_data	ec;
	struct device		*dev;
	struct mutex		lock;
};

enum ec_ram_type { EC_RAM_ACPI = 1, EC_RAM_HW, EC_RAM_EXT };

int imanager_read(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg);
int imanager_write(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg);

int imanager_read8(struct imanager_io_ops *io, u8 cmd, u8 param);
int imanager_write8(struct imanager_io_ops *io, u8 cmd, u8 param, u8 byte);

int imanager_read16(struct imanager_io_ops *io, u8 cmd, u8 param);
int imanager_write16(struct imanager_io_ops *io, u8 cmd, u8 param, u16 word);

int imanager_read_ram(struct imanager_io_ops *io, int ram_type, u8 offset,
		      u8 *buf, u8 len);
int imanager_write_ram(struct imanager_io_ops *io, int ram_type, u8 offset,
		       u8 *data, u8 size);

#endif
