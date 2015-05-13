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

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <linux/mfd/imanager/compat.h>
#include <linux/mfd/imanager/ec.h>
#include <linux/mfd/imanager/gpio.h>

#define EC_GPIOF_DIR_OUT	(1 << 6)
#define EC_GPIOF_DIR_IN		(1 << 7)
#define EC_GPIOF_LOW		(0 << 0)
#define EC_GPIOF_HIGH		(1 << 0)

/*
 * All GPIO Device IDs are retrieved from the EC firmware
 *
 * Note: After system power-on, BIOS pre-sets GPIO pins to:
 * GPIO[7..4] := Input
 * GPIO[3..0] := Output
 *
 */

struct gpio_data {
	struct ec_info info;
	struct gpio_cfg cfg[EC_GPIO_MAX_NUM];
};

static struct gpio_data gpio;

int gpio_core_get_state(u32 num)
{
	int ret;

	if (WARN_ON(num >= EC_GPIO_MAX_NUM))
		return -EINVAL;

	ret = imanager_read_byte(EC_CMD_HWP_RD, gpio.cfg[num].did);
	if (ret < 0)
		pr_err("Failed to get GPIO pin state (%x)\n", num);

	return ret;
}

int gpio_core_set_state(u32 num, u32 state)
{
	int ret;

	if (WARN_ON(num >= EC_GPIO_MAX_NUM))
		return -EINVAL;

	ret = imanager_write_byte(EC_CMD_HWP_WR, gpio.cfg[num].did,
			    state ? EC_GPIOF_HIGH : EC_GPIOF_LOW);
	if (ret) {
		pr_err("Failed to set GPIO pin state (%x)\n", num);
		return ret;
	}

	return 0;
}

int gpio_core_set_direction(u32 num, u32 dir)
{
	int ret;

	if (WARN_ON(num >= EC_GPIO_MAX_NUM))
		return -EINVAL;

	ret = imanager_write_byte(EC_CMD_GPIO_DIR_WR, gpio.cfg[num].did,
			    dir ? EC_GPIOF_DIR_IN : EC_GPIOF_DIR_OUT);
	if (ret) {
		pr_err("Failed to set GPIO direction (%x, '%s')\n", num,
			dir == GPIOF_DIR_OUT ? "OUT" : "IN");
		return ret;
	}

	return 0;
}

int gpio_core_init(void)
{
	int ret;

	memset(&gpio, 0, sizeof(gpio));

	ret = imanager_get_fw_info(&gpio.info);
	if (ret)
		return ret;

	return imanager_get_gpio_cfg(gpio.cfg);
}

void gpio_core_release(void)
{
	memset(&gpio, 0, sizeof(gpio));
}

