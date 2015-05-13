################################################################################
# Makefile - Build Linux kernel modules with Makefile.kbuild receipe
#            for building out-of-tree kernel modules
#
# Copyright (C) 2015 Advantech Corp., Irvine, CA, USA
# Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
################################################################################

all: clean modules

EXTRA_CFLAGS += -Wall -O2 -g
# Uncomment below for Red Hat/CentOS/Scientific Linux <= 7.1
# EXTRA_CFLAGS += -D__RHEL7__

NAME		:= imanager

DEPMOD		:= $(shell which depmod)
STRIP		:= $(shell which strip)
DRVPATH		:= /lib/modules/$(shell uname -r)/extra/$(NAME)

# MFD driver is always anabled.
CONFIG_MFD		:= m
CONFIG_GPIO		:= m
CONFIG_I2C		:= m
CONFIG_SENSORS		:= m
CONFIG_BACKLIGHT	:= m
CONFIG_WDT		:= m

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE), )

	ccflags-y += $(EXTRA_CFLAGS) -I$(src)/include
	include $(src)/Makefile.kbuild
else

# Otherwise we were called directly from the command
# line; invoke the kernel build system.
KDIR   := /lib/modules/$(shell uname -r)/build
PWD    := $(shell pwd)

endif

modules:
	$(MAKE) -C $(KDIR) M=$(PWD) modules

clean:
	$(MAKE) -C $(KDIR) M=$(PWD) clean
	$(RM) -rf *.bak include/linux/mfd/imanager/*.bak

install: modules
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_DIR=extra/$(NAME) modules_install
	$(DEPMOD) -a

uninstall:
	$(RM) -r $(DRVPATH)
	$(DEPMOD) -a

