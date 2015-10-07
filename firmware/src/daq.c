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


#include "daq.h"


int
main(void)
{
	daq_init();

	HAL_GPIO_WritePin(GPIOD, GPIO_PIN_12, GPIO_PIN_SET);

	for (;;) {

	}

	/* If we end up here, we're in big trouble */
	return 0;
}


void
daq_clock_init(void)
{
	RCC_OscInitTypeDef rcc_osc_init;
	RCC_ClkInitTypeDef rcc_clk_init;

	/* Enable power controller peripheral clock */
	__PWR_CLK_ENABLE();

	/* Ensure maximum clock frequency is not limited */
	__HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

	/* Enable HSE oscillator */
	rcc_osc_init.OscillatorType = RCC_OSCILLATORTYPE_HSE;
	rcc_osc_init.HSEState       = RCC_HSE_ON;

	/* Enable PLL, use HSE oscillator as source */
	rcc_osc_init.PLL.PLLState  = RCC_PLL_ON;
	rcc_osc_init.PLL.PLLSource = RCC_PLLSOURCE_HSE;

	/* Configure PLL for 168 MHz system clock */
	rcc_osc_init.PLL.PLLM = 8;
	rcc_osc_init.PLL.PLLN = 336;
	rcc_osc_init.PLL.PLLP = RCC_PLLP_DIV2;
	rcc_osc_init.PLL.PLLQ = 7;

	HAL_RCC_OscConfig(&rcc_osc_init);

	/* Use PLL as system clock source */
	rcc_clk_init.ClockType   = (RCC_CLOCKTYPE_SYSCLK |
		                        RCC_CLOCKTYPE_HCLK |
		                        RCC_CLOCKTYPE_PCLK1 |
		                        RCC_CLOCKTYPE_PCLK2);
	rcc_clk_init.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;

	/* Configure peripheral clock dividers */
	rcc_clk_init.AHBCLKDivider  = RCC_SYSCLK_DIV1;
	rcc_clk_init.APB1CLKDivider = RCC_HCLK_DIV4;
	rcc_clk_init.APB2CLKDivider = RCC_HCLK_DIV2;

	HAL_RCC_ClockConfig(&rcc_clk_init, FLASH_LATENCY_5);

	/* Enable flash prefetch */
	__HAL_FLASH_PREFETCH_BUFFER_ENABLE();
}


void
daq_gpio_init(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Enable GPIO port D peripheral clock */
	__GPIOD_CLK_ENABLE();

	/* Configure LED pin */
	gpio_init.Pin   = GPIO_PIN_12;
	gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull  = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOD, &gpio_init);
}


void
daq_init(void)
{
	daq_clock_init();
	daq_gpio_init();
}