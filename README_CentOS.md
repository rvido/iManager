## How to Customize RHEL/CentOS 7.x/6.x kernel to Enable GPIOLIB and GPIO_SYSFS

### Prepare Build Environment

- Download the kernel source code package

		$ yumdownloader --source kernel

	**CentOS 6** user will have to manually download the kernel source from http://vault.centos.org

		$ wget -c http://vault.centos.org/6.7/updates/Source/SPackages/kernel-2.6.32-573.18.1.el6.src.rpm

- Unpack kernel source file and change directory

		$ rpm -i kernel-3.10.0-327.10.1.el7.src.rpm
		$ cd rpmbuild/SPEC

- Open kernel configuration file(s) with your favorite text editor (such as 'nano')

		$ nano ../SOURCES/kernel-3.10.0-x86_64.config

- Add below two lines to your kernel configuration file

		CONFIG_GPIOLIB=y
		CONFIG_GPIO_SYSFS=y

	**CentOS 6** users only need to enable **CONFIG_GPIO_SYSFS**

- Prepare kernel build

		$ rpmbuild -bp --target $(uname -m) kernel.spec

	This will most likely generate an error regarding unresolved configuration symbols.  Copy listed CONFIG_* lines into an editor and prefix '# ' and append 'is not set' on each line.  Add those lines then into your current kernel config file below GPIOLIB and GPIO_SYSFS entries.  Below is an example output of CentOS 7.2, kernel 3.10.0-327.10.1.el7.  Append those lines to the kernel configuration file according your architecture.

		# CONFIG_RFKILL_GPIO is not set
		# CONFIG_TI_ST is not set
		# CONFIG_MDIO_GPIO is not set
		# CONFIG_KEYBOARD_GPIO is not set
		# CONFIG_KEYBOARD_GPIO_POLLED is not set
		# CONFIG_KEYBOARD_MATRIX is not set
		# CONFIG_MOUSE_GPIO is not set
		# CONFIG_TOUCHSCREEN_AUO_PIXCIR is not set
		# CONFIG_TOUCHSCREEN_CY8CTMG110 is not set
		# CONFIG_INPUT_GP2A is not set
		# CONFIG_INPUT_GPIO_TILT_POLLED is not set
		# CONFIG_INPUT_GPIO_ROTARY_ENCODER is not set
		# CONFIG_TCG_TIS_I2C_ST33 is not set
		# CONFIG_I2C_CBUS_GPIO is not set
		# CONFIG_I2C_GPIO is not set
		# CONFIG_DEBUG_GPIO is not set
		# CONFIG_GPIO_GENERIC_PLATFORM is not set
		# CONFIG_GPIO_IT8761E is not set
		# CONFIG_GPIO_TS5500 is not set
		# CONFIG_GPIO_SCH is not set
		# CONFIG_GPIO_ICH is not set
		# CONFIG_GPIO_VX855 is not set
		# CONFIG_GPIO_LYNXPOINT is not set
		# CONFIG_GPIO_MAX7300 is not set
		# CONFIG_GPIO_MAX732X is not set
		# CONFIG_GPIO_PCA953X is not set
		# CONFIG_GPIO_PCF857X is not set
		# CONFIG_GPIO_ADP5588 is not set
		# CONFIG_GPIO_AMD8111 is not set
		# CONFIG_GPIO_LANGWELL is not set
		# CONFIG_GPIO_PCH is not set
		# CONFIG_GPIO_ML_IOH is not set
		# CONFIG_GPIO_RDC321X is not set
		# CONFIG_GPIO_MCP23S08 is not set
		# CONFIG_GPIO_VIPERBOARD is not set
		# CONFIG_CHARGER_GPIO is not set
		# CONFIG_SENSORS_GPIO_FAN is not set
		# CONFIG_SENSORS_SHT15 is not set
		# CONFIG_SSB_DRIVER_GPIO is not set
		# CONFIG_BCMA_DRIVER_GPIO is not set
		# CONFIG_UCB1400_CORE is not set
		# CONFIG_MFD_SM501_GPIO is not set
		# CONFIG_TPS65010 is not set
		# CONFIG_MFD_TPS65912 is not set
		# CONFIG_MFD_TIMBERDALE is not set
		# CONFIG_LEDS_GPIO is not set
		# CONFIG_LEDS_LT3593 is not set
		# CONFIG_LEDS_RENESAS_TPU is not set
		# CONFIG_LEDS_TRIGGER_GPIO is not set
		# CONFIG_TOUCHSCREEN_CLEARPAD_TM1217 is not set

	In case you need to build a debug kernel, add the same lines to your debug kernel configuration file.

		$ nano ../SOURCES/kernel-3.10.0-x86_64-debug.config


- Prepare kernel build (again)

		$ rpmbuild -bp --target $(uname -m) kernel.spec

	Verify the two kernel config options have been set.

		$ grep -E 'GPIOLIB|GPIO_SYSFS' ../BUILD/kernel-3.10.0-327.10.1.el7/linux-3.10.0-327.10.1.el7.adv.1.x86_64/.config

	The result should look like:

		CONFIG_ARCH_WANT_OPTIONAL_GPIOLIB=y
		CONFIG_GPIOLIB=y
		CONFIG_GPIO_SYSFS=y

	If no errors are reported and the two options have been set, we are ready to wrap this up.
	In order to distinguish our custom kernel we will add our build ID to the kernel.spec file.

- Open kernel.spec file with your favorite text editor ('nano' is being used here)

		$ nano kernel.spec

	Search for the line which contains '**# % buildid**'

		Original line:
		# % buildid .local

	Modify this to something like (remove space between '%' and 'buildid')

		%buildid .adv.1

	Also add a changelog entry like:

		* Tue Mar 22 2016 John Doe <john.doe@email.com> - 3.10.0-327.10.1.el7.adv.1
		- Enable GPIOLIB and GPIO_SYSFS

- Build kernel source package.

		$ rpmbuild -bs kernel.spec

- Move this new source code package to somewhere safe and clean up.

		$ mv ../SRPMS/kernel-3.10.0-327.10.1.el7.adv.1.src.rpm ~/
		$ cd ~
		$ rm -rf /path/to/rpmbuild

### Kernel Build

- Build new kernel.

	This may take a long time depending on build machine.

		$ rpmbuild --rebuild --target $(uname -m) --without debug --without debuginfo --with headers kernel-3.10.0-327.10.1.el7.adv.1.src.rpm

	**CentOS 6** users will also have to **build a firmware package** in addition to the kernel packages

		$ rpmbuild --rebuild --target noarch --with firmware kernel-2.6.32-573.18.1.el6.adv.1.src.rpm

### Install New Kernel

- Use 'dnf' (CentOS 7) or 'yum' (CentOS 6).

	Avoid using 'rpm' as it would overwrite your kernel unless this is intended.

		$ dnf install kernel-3.10.0-327.10.1.el7.adv.1.x86_64.rpm kernel-headers-3.10.0-327.10.1.el7.adv.1.x86_64.rpm kernel-tools-3.10.0-327.10.1.el7.adv.1.x86_64.rpm kernel-tools-libs-3.10.0-327.10.1.el7.adv.1.x86_64.rpm kernel-devel-3.10.0-327.10.1.el7.adv.1.x86_64.rpm
