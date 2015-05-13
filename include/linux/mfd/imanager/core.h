/*
 * Advantech iManager MFD core (EC IT8518/28)
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __CORE_H__
#define __CORE_H__

#include <linux/mutex.h>
#include <linux/io.h>
#include <linux/types.h>
#include <linux/mfd/imanager/ec.h>

enum chips { IT8516, IT8518, IT8528 };

struct imanager_platform_data {
	struct ec_info	info;
};

struct imanager_device_data {
	enum chips	type;
	u16		addr;
	const char	*name;
	struct device	*dev;
	struct mutex	lock;
};

#endif
