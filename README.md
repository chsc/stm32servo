STM32Servo
==========

STM32Servo turns your STM32 micro-controller into a multi-channel servo controller with some additional goodies.
This version was specifically written for the [Blue Pill](https://stm32-base.org/boards/STM32F103C8T6-Blue-Pill.html) but can be easily adopted to other boards.
The firmware is controlled via a simple set of UART commands.

You can find a demonstration video on [YouTube](https://www.youtube.com/watch?v=bqKrrlYUw3U).

This firmware was originally written for a [robot project](https://www.youtube.com/watch?v=5QDXLnl5uDU) but I have never had the time to complete it :(

Basic Features:

* Servo calibration. Sets the PWM length for 0° and 180°. I might change this in the future to make it more flexible!
* Timed PWM interpolation. E.g. you can turn a servo from 45° to 170° within the next 2,5 seconds.
* Each servo has it's own time line. They are completely independent.
* Currently supports up to 24 servos in parallel. This can be easily extended. You can virtually turn every pin of your micro-controller into a PWM generator.
* Enabling/disabling servos on the fly.
* Adoptable to other micro-controller boards. The servo code is controller agnostic and the rest is done using *libopencm3* + *PlatformIO*.

The example directory contains a simple test console and an interface class for the servo controller. Have fun!

