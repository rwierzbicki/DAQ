/*
  This file is part of the UVic Formula Motorsports DAQ project.

  Copyright (c) 2015 UVic Formula Motorsports

  This program is free software: you can redistribute it and/or modify it under
  the terms of the GNU General Public License as published by the Free Software
  Foundation, either version 3 of the License, or (at your option) any later
  version.

  This program is distributed in the hope that it will be useful, but WITHOUT
  ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS
  FOR A PARTICULAR PURPOSE.  See the GNU General Public License for more
  details.

  You should have received a copy of the GNU General Public License along with
  this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <libopencm3/stm32/rcc.h>

int
main(void)
{
	/* Initialize */

	/* Set main clock to use external 8 MHz crystal */
	rcc_clock_setup_hse_3v3(&hse_8mhz_3v3[CLOCK_3V3_168MHZ]);

	/* Enable peripheral clocks */

	/* Configure input/output pins */

	// TODO: Configure interrupts

	// TODO: Configure USB

	// TODO: Configure CAN

	// TODO: Configure ADCs

	// TODO: Configure DMA

	// TODO: Configure flash

	// TODO: Configure SD card SPI?

	// TODO: Load configuration

	// TODO: Initialize main state machine

	/* Main event loop */
	for (;;) {

		// TODO: Stuff goes here

	}

	/* If we end up here, we're in big trouble */
	return 0;
}