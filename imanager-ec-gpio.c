/*
 * Advantech iManager GPIO core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#include <linux/types.h>
#include <linux/errno.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <ec.h>
#include <gpio.h>

#define EC_GPIOF_DIR_OUT	(1 << 6)
#define EC_GPIOF_DIR_IN		(1 << 7)
#define EC_GPIOF_LOW		(0 << 0)
#define EC_GPIOF_HIGH		(1 << 0)

/*
 * Power-on default:
 * GPIO[7..4] := Input
 * GPIO[3..0] := Output
 */

static const struct imanager_gpio_device *gpio;

int gpio_core_get_state(u32 num)
{
	int ret;

	if (WARN_ON(num >= gpio->num))
		return -EINVAL;

	ret = imanager_read_byte(EC_CMD_HWP_RD, gpio->attr[num].did);
	if (ret < 0)
		pr_err("Failed to get GPIO pin state (%x)\n", num);

	return ret;
}

int gpio_core_set_state(u32 num, bool state)
{
	int ret;

	if (WARN_ON(num >= gpio->num))
		return -EINVAL;

	ret = imanager_write_byte(EC_CMD_HWP_WR, gpio->attr[num].did,
			    state ? EC_GPIOF_HIGH : EC_GPIOF_LOW);
	if (ret) {
		pr_err("Failed to set GPIO pin state (%x)\n", num);
		return ret;
	}

	return 0;
}

int gpio_core_set_direction(u32 num, int dir)
{
	int ret;

	if (WARN_ON(num >= gpio->num))
		return -EINVAL;

	ret = imanager_write_byte(EC_CMD_GPIO_DIR_WR, gpio->attr[num].did,
			    dir ? EC_GPIOF_DIR_IN : EC_GPIOF_DIR_OUT);
	if (ret) {
		pr_err("Failed to set GPIO direction (%x, '%s')\n", num,
			dir == GPIOF_DIR_OUT ? "OUT" : "IN");
		return ret;
	}

	return 0;
}

int gpio_core_get_max_count(void)
{
	return gpio->num;
}

int gpio_core_init(void)
{
	gpio = imanager_get_gpio_device();
	if (!gpio)
		return -ENODEV;

	return 0;
}

