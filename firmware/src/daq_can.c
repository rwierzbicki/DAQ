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
daq_can_init(void)
{
	/* Enable CAN1 peripheral clock */
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;

	/* Exit sleep mode */
	CAN1->MCR &= ~CAN_MCR_SLEEP;

	/* Request initialization of the CAN hardware */
	CAN1->MCR |= CAN_MCR_INRQ;
}