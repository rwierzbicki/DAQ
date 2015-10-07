# DAQ Firmware

This directory contains the source for the DAQ's firmware which is targeted to
the STM32F407 microcontroller. The STM32F407 contains an ARM Cortex-M4 core
clocked at 168 MHz and provides up to 1 MB of flash and 192 kB of SRAM
(including 64 kB of core coupled memory).

## Requirements

To build the firmware, the following tools are required:

- **[arm-none-eabi-gcc](https://launchpad.net/gcc-arm-embedded/+download)** is
  a complete cross toolchain for ARM bare-metal development. This package
  contains a compiler, assembler, linker, debugger, and other tools.

- **[stlink](https://github.com/texane/stlink)** is an open source
  implementation of ST's ST-Link/V2 in-circuit debugger. It provides tools for
  reading and writing to flash as well as a GDB server.

- Optional: **[OpenOCD](http://openocd.org)** is another open source on-chip
  debugging tool compatible with the STM32. It is likely more stable and mature
  than stlink. However, stlink is quite simple and adequate for the purpose of
  this project (at least for now).

## Building

Build the firmware:

    $ make

You can then write the firmware to flash:

    $ make bin
    $ st-flash write build/daq.bin 0x08000000

## Debugging

Debugging with GDB is fairly straightforward. It requires running both the GDB
server provided by stlink and GDB itself provided by the arm-none-eabi-gcc
package.

Start the GDB server in one shell:

    $ st-util

Run GDB in another:

    $ arm-none-eabi-gdb build/daq.elf

In GDB, connect to the target:

    $ target extended-remote :4242

Now you can load and run the firmware:

    $ load
    $ continue

## Libraries

The following libraries are included in the source tree:

- **cmsis**: ARM's Cortex Microcontroller Software Interface Standard provides a
  hardware abstraction layer for Cortex-M microcontrollers. `stm32f407xx.h`
  provides all of the register definitions and includes the appropriate CMSIS
  files for the Cortex-M4 core within the STM32F4.

- **fat**: FAT filesystem library.

- **hal**: ST's hardware abstraction library for the STM32F407. This provides
  relatively simple interfaces to all of the microcontroller's peripherals. Many
  examples are available which makes this library the ideal choice for this
  project.

- **ld**: linker scripts to define the memory spaces, stack and heap sizes, and
  determine which data goes where.

- **usb**: USB device library which provides basic USB functionality as well as
  Communications Device Class (CDC), Device Firmware Update (DFU), and Mass
  Storage Class (MSC) device classes. This is used to configure and retreive
  data from the DAQ and upgrade the firmware.

## References

The following documents will be essential to DAQ firmware development:

- **[STM32F407 Reference Manual][1]**: describes microcontroller core, memory,
  and all peripherals.

- **[STM32F4 Programming Manual][3]**: describes programming model, instruction
  set, and core peripherals.

- **[STM32F407 Errata Sheet][4]**: describes bugs and limitations in the actual
  silicon and generally provides workarounds.

- **[Application Notes][5]**: describe using the microcontroller in specific
  applications (e.g. using the DMA controller or the real-time clock). The
  reference manual provides all of the technical details, but application notes
  provide "tutorials."

- **[NAND Flash Controller Manual][6]**: describes the NAND flash controller and
  provides examples of using memory-mapped file system stored in NAND flash.

[1]: http://www.st.com/web/en/resource/technical/document/reference_manual/DM00031020.pdf
[2]: http://libopencm3.github.io/docs/latest/html/
[3]: http://www.st.com/web/en/resource/technical/document/programming_manual/DM00046982.pdf
[4]: http://www.st.com/web/en/resource/technical/document/errata_sheet/DM00037591.pdf
[5]: http://www.st.com/stonline/stappl/resourceSelector/app?page=fullResourceSelector&doctype=application_note&LineID=11
[6]: http://www.st.com/web/en/resource/technical/document/user_manual/DM00091013.pdf
