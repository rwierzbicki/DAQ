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