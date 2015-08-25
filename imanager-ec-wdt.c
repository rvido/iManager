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
 *
 * Note: Current release only implements RESET of the EC WDT.
 *       In other words, no PWR button, NMI, SCI, IRQ, or WDPin support yet!
 */

#include <linux/io.h>
#include <linux/delay.h>
#include <linux/string.h>
#include <linux/byteorder/generic.h>
#include <linux/swab.h>
#include <ec.h>
#include <wdt.h>

/* Timer resolution */
#define WDT_FREQ	10 /* Hz */

enum wdt_ctrl {
	START = 1,
	STOP,
	RST,
	GET_TIMEOUT,
	SET_TIMEOUT,
	STOPBOOT = 8,
};

struct wdt_event_delay {
	u16	delay,
		pwrbtn,
		nmi,
		reset,
		wdpin,
		sci,
		dummy;
};

static const struct imanager_watchdog_device *dev;

static inline int set_timer(enum wdt_ctrl ctrl)
{
	if (WARN_ON(ctrl == SET_TIMEOUT))
		return -EINVAL;

	return imanager_msg_write(EC_CMD_WDT_CTRL, ctrl, NULL);
}

static int
wdt_ctrl(enum wdt_ctrl ctrl, enum wdt_event type, unsigned int timeout)
{
	u16 val;
	int ret;
	struct ec_message msg = {
		.rlen = 0,
		.wlen = 0,
	};
	u8 *fevent = &msg.u.data[0];
	struct wdt_event_delay *event =
				(struct wdt_event_delay *)&msg.u.data[1];

	switch (ctrl) {
	case SET_TIMEOUT:
		memset(event, 0xff, sizeof(*event));
		msg.wlen = sizeof(*event);
		*fevent = 0;
		val = (!timeout) ? 0xffff : swab16(timeout * WDT_FREQ);

		switch (type) {
		case DELAY:
			event->delay = val;
			break;
		case PWRBTN:
			event->pwrbtn = val;
			break;
		case NMI:
			event->nmi = val;
			break;
		case RESET:
			event->reset = val;
			break;
		case WDPIN:
			event->wdpin = val;
			break;
		case SCI:
			event->sci = val;
			break;
		default:
			return -EINVAL;
		}

		ret = imanager_msg_write(EC_CMD_WDT_CTRL, SET_TIMEOUT, &msg);
		if (ret < 0) {
			pr_err("Failed to set timeout\n");
			return ret;
		}
		break;
	case START:
	case STOP:
	case RST:
	case STOPBOOT:
		/* simple command, no data */
		return imanager_msg_write(EC_CMD_WDT_CTRL, ctrl, NULL);
	default:
		return -EINVAL;
	}

	return timeout;
}

int wdt_core_start_timer(void)
{
	return set_timer(START);
}

int wdt_core_stop_timer(void)
{
	return set_timer(STOP);
}

int wdt_core_reset_timer(void)
{
	return set_timer(RST);
}

int wdt_core_stop_boot_timer(void)
{
	return set_timer(STOPBOOT);
}

inline int wdt_core_set_timeout(enum wdt_event type, u32 timeout)
{
	return wdt_ctrl(SET_TIMEOUT, type, timeout);
}

int wdt_core_disable_all(void)
{
	struct ec_message msg = {
		.rlen = 0,
		.wlen = sizeof(struct wdt_event_delay),
	};
	struct wdt_event_delay *event =
				(struct wdt_event_delay *)&msg.u.data[1];

	memset(event, 0xff, sizeof(*event));

	return  (wdt_core_stop_timer() ||
		 wdt_core_stop_boot_timer() ||
		 imanager_msg_write(EC_CMD_WDT_CTRL, SET_TIMEOUT, &msg));
}

int wdt_core_init(void)
{
	dev = imanager_get_watchdog_device();
	if (!dev)
		return -ENODEV;

	return 0;
}

