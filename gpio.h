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

#ifndef __GPIO_H__
#define __GPIO_H__

#include <linux/gpio.h>
#include <linux/types.h>

/*
 * EC FW supports 8-bit GPIO.
 */
#define EC_MAX_GPIO	8

int gpio_core_init(void);
void gpio_core_release(void);

int gpio_core_get_state(u32 gpio);
int gpio_core_set_state(u32 gpio, u32 state);
int gpio_core_set_direction(u32 gpio, u32 dir);

#endif
