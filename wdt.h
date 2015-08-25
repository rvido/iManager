/*
 * Advantech iManager Watchdog core
 *
 * Copyright (C) 2015 Advantech Co., Ltd., Irvine, CA, USA
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __WDT_H__
#define __WDT_H__

#include <linux/types.h>

enum wdt_event {
	DELAY,
	PWRBTN,
	NMI,
	RESET,
	WDPIN,
	SCI,
};

int wdt_core_init(void);

int wdt_core_set_timeout(enum wdt_event type, u32 timeout);

int wdt_core_disable_all(void);
int wdt_core_start_timer(void);
int wdt_core_stop_timer(void);
int wdt_core_reset_timer(void);
int wdt_core_stop_boot_timer(void);

#endif
