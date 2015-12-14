Note
====

This is a set of platform drivers which provide support for multiple
embedded features such as GPIO, I2C/SMBus, Hardware Monitoring, Watchdog,
and Backlight/Brightness control. Those features are available on Advantech
Embedded boards such as SOM, MIO, AIMB, and PCM.
Datasheets of each product line can be downloaded from www.advantech.com

Author:
	Richard Vidal-Dorsch <richard.dorsch@advantech.com>


Kernel driver imanager (MFD)
============================

This is a Multi-Function-Device driver which provides a communication
layer to the Advantech iManager EC firmware which is factory loaded
into ITE IT8518/28 (EC) chips. The type of communication is message
based. The client (driver) requests information from Advantech iManager
and waits for a response (polling, not intterupt driven). If a response
has been sent within an expected timeout, the information is been extracted
from the message and then hand-off to the caller (sub-driver).

  Supported chips:
  * Advantech EC based on ITE IT8518
    Prefix: imanager
    Addresses: 0x029e/0x029f
    Datasheet: Available from ITE upon request
  * Advantech EC based on ITE IT8528
    Prefix: imanager
    Addresses: 0x0299/0x029a
    Datasheet: Available from ITE upon request

This driver depends on mfd-core


Kernel driver imanager_gpio
===========================

This platform driver provides support for 8-bit iManager GPIO
which can be accessed through SYSFS (/sys/class/gpio/).
Kernel option CONFIG_GPIO_SYSFS needs to be enabled.

This driver depends on imanager (mfd).


Kernel driver imanager_i2c
==========================

This platform driver provides support for iManager I2C/SMBus.

This driver depends on imanager (mfd).

Module Parameters
-----------------

* bus_frequency (unsigned short)
Set desired bus frequency. Valid values (kHz) are:
  50  Slow
 100  Standard (default)
 400  Fast


Description
-----------

The Advantech iManager provide up to four SMBus controllers. One of them
is configured for I2C compatibility.


Process Call Support
--------------------

Not supported.


I2C Block Read Support
----------------------

I2C block read is supported.


SMBus 2.0 Support
-----------------

Several SMBus 2.0 features are supported.
No PEC support.


Interrupt Support
-----------------

No PCI interrupt support available


Kernel driver imanager_hwmon
============================

This platform driver provides support for iManager Hardware Monitoring
and FAN control.

This driver depends on imanager (mfd).

Description
-----------

This driver provides support for the Advantech iManager Hardware Monitoring EC.

The Advantech iManager supports up to 3 fan rotation speed sensors,
3 temperature monitoring sources and up to 5 voltage sensors, VID, alarms and
a automatic fan regulation strategy (as well as manual fan control mode).

Temperatures are measured in degrees Celsius and measurement resolution is
1 degC. An Alarm is triggered when the temperature gets higher than the high
limit; it stays on until the temperature falls below the high limit.

Fan rotation speeds are reported in RPM (rotations per minute). An alarm is
triggered if the rotation speed has dropped below a programmable limit. No fan
speed divider support available.

Voltage sensors (also known as IN sensors) report their values in millivolts.
An alarm is triggered if the voltage has crossed a programmable minimum
or maximum limit.

The driver supports automatic fan control mode known as Thermal Cruise.
In this mode, the firmware attempts to keep the measured temperature in a
predefined temperature range. If the temperature goes out of range, fan
is driven slower/faster to reach the predefined range again.

The mode works for fan1-fan3.

sysfs attributes
----------------

pwm[1-3] - this file stores PWM duty cycle or DC value (fan speed) in range:
	   0 (lowest speed) to 255 (full)

pwm[1-3]_enable - this file controls mode of fan/temperature control:
	* 0 Fan control disabled (fans set to maximum speed)
	* 1 Manual mode, write to pwm[1-3] any value 0-255
	* 2 "Fan Speed Cruise" mode

pwm[1-3]_mode - controls if output is PWM or DC level
        * 0 DC output
        * 1 PWM output

Speed Cruise mode (2)
---------------------

This modes tries to keep the fan speed constant.

fan[1-3]_min - Minimum fan speed
fan[1-3]_max - Maximum fan speed


Kernel driver imanager_wdt
==========================

This driver provides support for iManager watchdog.

This driver depends on imanager (mfd).


Kernel driver imanager_bl
=========================

This driver provides support for iManager backlight and brightness
control which can be accessed through SYSFS (/sys/class/backlight).

This driver depends on imanager (mfd).
