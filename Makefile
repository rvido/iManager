################################################################################
# Makefile - Build Linux kernel modules with Makefile.kbuild receipe
#            for building out-of-tree kernel modules
#
# Copyright (C) 2015 Advantech Corp., Irvine, CA, USA
# Author: Richard Vidal-Dorsch <richard.dorsch@advantech.com>
################################################################################

all: clean modules

ccflags-y += -Wall -O2 -I$(src)

# OS specific switches
# EXTRA_CFLAGS += -D__RHEL6__
# EXTRA_CFLAGS += -D__RHEL7__

NAME			:= imanager

DEPMOD			:= $(shell which depmod)
STRIP			:= $(shell which strip)
DRVPATH			:= /lib/modules/$(shell uname -r)/extra/$(NAME)

# MFD driver is always enabled.
CONFIG_MFD		:= m
CONFIG_GPIO		:= m
CONFIG_I2C		:= m
CONFIG_SENSORS		:= m
CONFIG_BACKLIGHT	:= m
CONFIG_WDT		:= m

# If KERNELRELEASE is defined, we've been invoked from the
# kernel build system and can use its language.
ifneq ($(KERNELRELEASE), )

	ccflags-y += $(EXTRA_CFLAGS)
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
	$(RM) -rf *.bak

install: modules
	$(MAKE) -C $(KDIR) M=$(PWD) INSTALL_MOD_DIR=extra/$(NAME) modules_install
	$(DEPMOD) -a

uninstall:
	$(RM) -r $(DRVPATH)
	$(DEPMOD) -a

