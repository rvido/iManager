/*
 * Platform device compatibility
 *
 * This program is free software; you can redistribute  it and/or modify it
 * under  the terms of  the GNU General  Public License as published by the
 * Free Software Foundation;  either version 2 of the  License, or (at your
 * option) any later version.
 */

#ifndef __COMPAT_H__
#define __COMPAT_H__

#include <linux/io.h>
#include <linux/ioport.h>
#include <linux/version.h>

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,32)
#error This driver is for kernel versions 2.6.32 and later
#endif

/* module_platform_driver() - Helper macro for drivers that don't do
 * anything special in module init/exit.  This eliminates a lot of
 * boilerplate.  Each module may only use this macro once, and
 * calling it replaces module_init() and module_exit()
 */
#ifndef module_platform_driver
#define module_platform_driver(__platform_driver) \
static int __init __platform_driver##_init(void) \
{ \
        return platform_driver_register(&(__platform_driver)); \
} \
module_init(__platform_driver##_init); \
static void __exit __platform_driver##_exit(void) \
{ \
        platform_driver_unregister(&(__platform_driver)); \
} \
module_exit(__platform_driver##_exit);
#endif /* module_platform_driver */

#ifdef __NEED_HWMON_COMPAT__
#if LINUX_VERSION_CODE < KERNEL_VERSION(3,12,0) && ! defined(__RHEL7__)
#if ! defined(__RHEL6__)
static int sysfs_create_groups(struct kobject *kobj,
			       const struct attribute_group **groups)
{
	int error = 0;
	int i;

	if (!groups)
		return 0;

	for (i = 0; groups[i]; i++) {
		error = sysfs_create_group(kobj, groups[i]);
		if (error) {
			while (--i >= 0)
				sysfs_remove_group(kobj, groups[i]);
			break;
		}
	}
	return error;
}
#endif

static void sysfs_remove_groups(struct kobject *kobj,
				const struct attribute_group **groups)
{
	int i;

	if (!groups)
		return;
	for (i = 0; groups[i]; i++)
		sysfs_remove_group(kobj, groups[i]);
}

#if ! defined(__RHEL6__)
static inline int __must_check PTR_ERR_OR_ZERO(__force const void *ptr)
{
	if (IS_ERR(ptr))
		return PTR_ERR(ptr);
	else
		return 0;
}
#endif
#endif
#endif /* __NEED_HWMON_COMPAT__ */

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,7,0) || defined(__RHEL6__)
enum pwm_polarity {
	PWM_POLARITY_NORMAL,
	PWM_POLARITY_INVERSED,
};
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(2,6,34)
#define GPIOF_DIR_OUT   0UL
#define GPIOF_DIR_IN    1UL
#endif

#if LINUX_VERSION_CODE < KERNEL_VERSION(3,3,0) || defined(__RHEL6__)
#undef umode_t
#define umode_t mode_t
#endif

#endif /* __COMPAT_H__ */
