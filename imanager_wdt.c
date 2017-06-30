/*
 * Advantech iManager Watchdog driver
 *
 * Copyright (C) 2016 Advantech Co., Ltd.
 * Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt

#include <linux/bitops.h>
#include <linux/byteorder/generic.h>
#include <linux/device.h>
#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/platform_device.h>
#include <linux/watchdog.h>
#include "compat.h"
#include "imanager.h"

#define WDT_DEFAULT_TIMEOUT	30 /* seconds */
#define WDT_FREQ		10 /* Hz */

struct imanager_wdt_data {
	struct imanager_device_data *imgr;
	struct watchdog_device wdt;
	ulong last_updated;
	uint timeout;
};

static uint timeout = WDT_DEFAULT_TIMEOUT;
module_param(timeout, uint, 0444);
MODULE_PARM_DESC(timeout,
		 "Watchdog timeout in seconds. 1 <= timeout <= 65534, default="
		 __MODULE_STRING(WDT_DEFAULT_TIMEOUT) ".");

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
static bool nowayout = WATCHDOG_NOWAYOUT;
module_param(nowayout, bool, 0444);
MODULE_PARM_DESC(nowayout,
		 "Watchdog cannot be stopped once started (default="
		 __MODULE_STRING(WATCHDOG_NOWAYOUT) ")");
#endif

enum wdt_ctrl {
	START = 1, STOP, RESET, GET_TIMEOUT, SET_TIMEOUT, STOPBOOT = 8
};

enum imanager_wdt_event {
	WDT_EVT_NONE,
	WDT_EVT_DELAY,
	WDT_EVT_PWRBTN,
	WDT_EVT_NMI,
	WDT_EVT_RESET,
	WDT_EVT_WDPIN,
	WDT_EVT_SCI,
};

struct event_delay {
	u16	delay,
		pwrbtn,
		nmi,
		reset,
		wdpin,
		sci,
		dummy;
} __attribute__((__packed__));

static int imanager_wdt_ctrl(struct imanager_device_data *imgr, int ctrl,
			     int event_type, uint timeout)
{
	struct imanager_ec_message msg = {
		IMANAGER_MSG(0, 0, ctrl, EC_CMD_WDT_CTRL),
	};
	u8 *fevent = &msg.u.data[0];
	struct event_delay *event = (struct event_delay *)&msg.u.data[1];
	int val;

	if (ctrl == SET_TIMEOUT) {
		memset(event, 0xff, sizeof(*event));
		msg.wlen = sizeof(*event);
		*fevent = 0;
		val = (!timeout) ? 0xffff : cpu_to_be16(timeout * WDT_FREQ);

		switch (event_type) {
		case WDT_EVT_DELAY:
			event->delay = val;
			break;
		case WDT_EVT_PWRBTN:
			event->pwrbtn = val;
			break;
		case WDT_EVT_NMI:
			event->nmi = val;
			break;
		case WDT_EVT_RESET:
			event->reset = val;
			break;
		case WDT_EVT_WDPIN:
			event->wdpin = val;
			break;
		case WDT_EVT_SCI:
			event->sci = val;
			break;
		default:
			return -EINVAL;
		}
	}

	return imanager_write(imgr, &msg);
}

static inline int imanager_wdt_disable_all(struct imanager_wdt_data *data)
{
	return (imanager_wdt_ctrl(data->imgr, STOP, WDT_EVT_NONE, 0) ||
		imanager_wdt_ctrl(data->imgr, STOPBOOT, WDT_EVT_NONE, 0));
}

static int imanager_wdt_set_timeout(struct watchdog_device *wdt, uint timeout)
{
	struct imanager_wdt_data *data = watchdog_get_drvdata(wdt);
	struct imanager_device_data *imgr = data->imgr;
	int ret;

	if (time_before(jiffies, data->last_updated + HZ + HZ / 2))
		return 0;

	if (data->timeout == timeout)
		return 0;

	ret = imanager_wdt_ctrl(imgr, SET_TIMEOUT, WDT_EVT_PWRBTN, timeout);
	if (ret < 0)
		return ret;

	data->timeout = timeout;
	data->last_updated = jiffies;

	return 0;
}

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
static uint imanager_wdt_get_timeleft(struct watchdog_device *wdt)
{
	struct imanager_wdt_data *data = watchdog_get_drvdata(wdt);
	uint timeleft = 0;
	ulong time_diff = ((jiffies - data->last_updated) / HZ);

	if (data->last_updated && (data->timeout > time_diff))
		timeleft = data->timeout - time_diff;

	return timeleft;
}
#endif

static int imanager_wdt_start(struct watchdog_device *wdt)
{
	struct imanager_wdt_data *data = watchdog_get_drvdata(wdt);
	int ret;

	ret = imanager_wdt_ctrl(data->imgr, START, WDT_EVT_NONE, 0);
	if (ret < 0)
		return ret;

	data->last_updated = jiffies;

	return 0;
}

static int imanager_wdt_stop(struct watchdog_device *wdt)
{
	struct imanager_wdt_data *data = watchdog_get_drvdata(wdt);
	int ret;

	ret = imanager_wdt_ctrl(data->imgr, STOP, WDT_EVT_NONE, 0);
	if (ret < 0)
		return ret;

	data->last_updated = 0;

	return 0;
}

static int imanager_wdt_ping(struct watchdog_device *wdt)
{
	struct imanager_wdt_data *data = watchdog_get_drvdata(wdt);
	int ret;

	ret = imanager_wdt_ctrl(data->imgr, RESET, WDT_EVT_NONE, 0);
	if (ret < 0)
		return ret;

	data->last_updated = jiffies;

	return 0;
}

static const struct watchdog_info imanager_wdt_info = {
	.options		= WDIOF_SETTIMEOUT |
				  WDIOF_KEEPALIVEPING |
				  WDIOF_MAGICCLOSE,
	.firmware_version	= 0,
	.identity		= "imanager-wdt",
};

static const struct watchdog_ops imanager_wdt_ops = {
	.owner		= THIS_MODULE,
	.start		= imanager_wdt_start,
	.stop		= imanager_wdt_stop,
	.ping		= imanager_wdt_ping,
	.set_timeout	= imanager_wdt_set_timeout,
#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
	.get_timeleft	= imanager_wdt_get_timeleft,
#endif
};

static int imanager_wdt_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct imanager_device_data *imgr = dev_get_drvdata(dev->parent);
	struct imanager_wdt_data *data;
	struct watchdog_device *wdt_dev;
	int ret;

	data = devm_kzalloc(dev, sizeof(*data), GFP_KERNEL);
	if (!data)
		return -ENOMEM;

	data->imgr = imgr;

	wdt_dev = &data->wdt;
	wdt_dev->info		= &imanager_wdt_info;
	wdt_dev->ops		= &imanager_wdt_ops;
	wdt_dev->timeout	= WDT_DEFAULT_TIMEOUT;
	wdt_dev->min_timeout	= 1;
	wdt_dev->max_timeout	= 0xfffe;

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
	watchdog_set_nowayout(wdt_dev, nowayout);
#endif
	watchdog_set_drvdata(wdt_dev, data);

	ret = watchdog_register_device(wdt_dev);
	if (ret) {
		dev_err(dev, "Could not register watchdog device\n");
		return ret;
	}

	platform_set_drvdata(pdev, data);

	imanager_wdt_disable_all(data);
	imanager_wdt_set_timeout(wdt_dev, timeout);

	dev_info(dev, "Driver loaded (timeout=%d seconds)\n", timeout);

	return 0;
}

static int imanager_wdt_remove(struct platform_device *pdev)
{
	struct imanager_wdt_data *data = platform_get_drvdata(pdev);

#if LINUX_VERSION_CODE > KERNEL_VERSION(3, 3, 0)
	if (!nowayout)
		imanager_wdt_disable_all(data);
#endif

	watchdog_unregister_device(&data->wdt);
	platform_set_drvdata(pdev, NULL);

	return 0;
}

static void imanager_wdt_shutdown(struct platform_device *pdev)
{
	struct imanager_device_data *imgr = dev_get_drvdata(pdev->dev.parent);

	imanager_wdt_ctrl(imgr, STOP, WDT_EVT_NONE, 0);
}

static struct platform_driver imanager_wdt_driver = {
	.driver = {
#if LINUX_VERSION_CODE < KERNEL_VERSION(3, 19, 0)
		.owner = THIS_MODULE,
#endif
		.name	= "imanager-wdt",
	},
	.probe		= imanager_wdt_probe,
	.remove		= imanager_wdt_remove,
	.shutdown	= imanager_wdt_shutdown,
};

module_platform_driver(imanager_wdt_driver);

MODULE_DESCRIPTION("Advantech iManager Watchdog Driver");
MODULE_AUTHOR("Richard Vidal-Dorsch <richard.dorsch at advantech.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:imanager-wdt");
