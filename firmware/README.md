# DAQ Firmware

This directory contains the source for the DAQ's firmware which is targeted to
the STM32F407 microcontroller. The STM32F407 contains an ARM Cortex-M4 core
clocked at 168 MHz and provides up to 1 MB of flash and 192 kB of SRAM
(including 64 kB of core coupled memory).

## Requirements

To build the firmware, the following tools are required:

- *[arm-none-eabi-gcc](https://launchpad.net/gcc-arm-embedded/+download)*
