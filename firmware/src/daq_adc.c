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

	/* Sample channels 1 and 2 for 28 cycles */
	ADC1->SQR1 |= ADC_SQR1_L_0;
	ADC1->SQR3 = ADC_SQR3_SQ1_0 | ADC_SQR3_SQ2_1;
	ADC1->SMPR2 = ADC_SMPR2_SMP1_1 | ADC_SMPR2_SMP2_1;

	/* Power ADC on */
	ADC1->CR2 |= ADC_CR2_ADON;
}