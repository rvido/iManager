################################################################################
# Makefile - Build Advantech iManager Linux kernel modules with Makefile.kbuild
#            recipe (out-of-tree kernel modules)
#
################################################################################

all: clean modules

NAME		:= imanager
DEPMOD		:= $(shell which depmod)
STRIP		:= $(shell which strip)
DRVPATH		:= /lib/modules/$(shell uname -r)/extra/$(NAME)
INCLUDE		:= $(src)

# OS specific switches
# EXTRA_CFLAGS	+= -D__RHEL6__
# EXTRA_CFLAGS	+= -D__RHEL7__

ccflags-y	+= -Wall -O3 $(EXTRA_CFLAGS) -I$(INCLUDE)

ifneq ($(KERNELRELEASE), )
	# We've been invoked from the kernel build system and can
	# use its language.
	include $(src)/Makefile.kbuild
else
	# Otherwise we were called directly from the command line;
	# invoke the kernel build system.
	KDIR	:= /lib/modules/$(shell uname -r)/build
	PWD	:= $(shell pwd)
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
