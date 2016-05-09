/*
 * Advantech iManager MFD
 *
 * Copyright (C) 2016 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __IMANAGER_CORE_H__
#define __IMANAGER_CORE_H__

#include <linux/mutex.h>
#include <linux/types.h>
#include "imanager-ec.h"

struct imanager_device_data {
	struct imanager_ec_data ec;
	struct device *dev;
	struct mutex lock;
};

int imanager_read(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg);
int imanager_write(struct imanager_io_ops *io, u8 cmd, struct ec_message *msg);

int imanager_read8(struct imanager_io_ops *io, u8 cmd, u8 param);
int imanager_write8(struct imanager_io_ops *io, u8 cmd, u8 param, u8 byte);

int imanager_read16(struct imanager_io_ops *io, u8 cmd, u8 param);
int imanager_write16(struct imanager_io_ops *io, u8 cmd, u8 param, u16 word);

int imanager_read_ram(struct imanager_io_ops *io, int ram_type, int offset,
		      u8 *buf, u8 len);
int imanager_write_ram(struct imanager_io_ops *io, int ram_type, int offset,
		       u8 *data, u8 size);

#endif
