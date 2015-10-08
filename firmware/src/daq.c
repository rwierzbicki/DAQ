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

uint16_t values[2];


int
main(void)
{
	daq_init();

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
	__HAL_RCC_PWR_CLK_ENABLE();

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
	rcc_clk_init.ClockType = (RCC_CLOCKTYPE_SYSCLK |
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
daq_adc_init(void)
{
	/* Enable ADC1 periperhal clock */
	RCC->APB2ENR |= RCC_APB2ENR_ADC1EN;

	HAL_NVIC_SetPriority(ADC_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(ADC_IRQn);

	/* Disable EOC interrupt */
	ADC1->CR1 &= ~ADC_CR1_EOCIE;

	/* Enable scan mode */
	ADC1->CR1 |= ADC_CR1_SCAN;

	/* Enable external trigger on rising edge of TIM2 TRGO */
	ADC1->CR2 |= ADC_CR2_EXTEN_0 | ADC_CR2_EXTSEL_1 | ADC_CR2_EXTSEL_2;

	/* Enable DMA mode, issue requests for every transfer */
	ADC1->CR2 |= ADC_CR2_DMA | ADC_CR2_DDS;

	/* Sample channel 1 and 2 for 28 cycles */
	ADC1->SQR1 |= ADC_SQR1_L_0;
	ADC1->SQR3 = ADC_SQR3_SQ1_0 | ADC_SQR3_SQ2_1;
	ADC1->SMPR2 = ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP2_1;

	/* Power ADC on */
	ADC1->CR2 |= ADC_CR2_ADON;
}


void
daq_dma_init(void)
{
	HAL_NVIC_SetPriority(DMA2_Stream0_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(DMA2_Stream0_IRQn);

	/* Enable DMA2 peripheral clock */
	RCC->AHB1ENR |= RCC_AHB1ENR_DMA2EN;

	/* Select channel 0 (ADC1) for DMA2 stream 0 */
	DMA2_Stream0->CR &= ~DMA_SxCR_CHSEL;

	/* Configure DMA2 stream 0 for circular, half-word, memory-incrementing */
	DMA2_Stream0->CR |= DMA_SxCR_MINC | DMA_SxCR_CIRC | DMA_SxCR_MSIZE_0 | DMA_SxCR_PSIZE_0;

	/* Enable DMA2 stream 0 transfer complete interrupt */
	DMA2_Stream0->CR |= DMA_SxCR_TCIE;

	/* Set DMA2 stream 0 source peripheral address to ADC1 DR */
	DMA2_Stream0->PAR = (uint32_t) &ADC1->DR;
	
	/* Set DMA2 stream 0 target memory address */
	DMA2_Stream0->M0AR = (uint32_t) &values;

	/* Tranfer 2 items per DMA2 stream 0 request */
	DMA2_Stream0->NDTR = 2;

	/* Enable DMA2 stream 0 */
	DMA2_Stream0->CR |= DMA_SxCR_EN;
}


void
daq_gpio_init(void)
{
	GPIO_InitTypeDef gpio_init;

	/* Enable GPIO port D peripheral clock */
	__HAL_RCC_GPIOD_CLK_ENABLE();

	/* Configure LED pin */
	gpio_init.Pin   = GPIO_PIN_12 | GPIO_PIN_13 | GPIO_PIN_14 | GPIO_PIN_15;
	gpio_init.Mode  = GPIO_MODE_OUTPUT_PP;
	gpio_init.Pull  = GPIO_PULLUP;
	gpio_init.Speed = GPIO_SPEED_FAST;

	HAL_GPIO_Init(GPIOD, &gpio_init);

	/* Enable GPIO port A peripheral clock */
	__HAL_RCC_GPIOA_CLK_ENABLE();

	gpio_init.Pin   = GPIO_PIN_1 | GPIO_PIN_2;
	gpio_init.Mode  = GPIO_MODE_ANALOG;
	gpio_init.Pull  = GPIO_NOPULL;

	HAL_GPIO_Init(GPIOA, &gpio_init);
}


void
daq_timer_init(void)
{
	TIM_HandleTypeDef timer_handle;

	HAL_NVIC_SetPriority(TIM1_CC_IRQn, 0, 1);
	HAL_NVIC_EnableIRQ(TIM2_IRQn);

	/* Enabled TIM2 peripheral clock */
	__HAL_RCC_TIM2_CLK_ENABLE();

	timer_handle.Instance = TIM2;
	timer_handle.Init.CounterMode = TIM_COUNTERMODE_UP;
	timer_handle.Init.Period = 500 - 1;
	timer_handle.Init.Prescaler = (uint32_t) ((SystemCoreClock /2) / 10000) - 1;
	timer_handle.Init.ClockDivision = 0;
	timer_handle.Init.RepetitionCounter = 0;

	TIM2->CR2 |= TIM_CR2_MMS_1;

	HAL_TIM_Base_Init(&timer_handle);
	HAL_TIM_Base_Start_IT(&timer_handle);
}


void
daq_init(void)
{
	daq_clock_init();
	daq_gpio_init();
	daq_timer_init();
	daq_dma_init();
	daq_adc_init();
}


extern void
TIM2_IRQHandler(void)
{
	if ((TIM2->SR & TIM_SR_UIF)) {
		
		TIM2->SR &= ~TIM_SR_UIF;

		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_12);
	}
}


extern void
DMA2_Stream0_IRQHandler(void)
{
	if ((DMA2->LISR & DMA_LISR_TCIF0)) {
		HAL_GPIO_TogglePin(GPIOD, GPIO_PIN_13);

		DMA2->LIFCR |= DMA_LIFCR_CTCIF0;

		if (values[0] > 255) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_SET);
		else  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_14, GPIO_PIN_RESET);

		if (values[1] > 255) HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_SET);
		else  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_15, GPIO_PIN_RESET);
	}
}