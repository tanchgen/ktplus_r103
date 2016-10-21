//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include "main.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "stm32f10x.h"
#include "my_time.h"


tMesure	r103Mesure;
tR103Stat r103Stat;
uint32_t VlvDevId;			// Иденитификатор контроллера задвижки

RCC_ClocksTypeDef RCC_Clocks;

void thermoProcess( void );

// ----- main() ---------------------------------------------------------------

int main(int argc, char* argv[]) {
	(void)argc;
	(void)argv;
	myTick = 0;
  uxTime = 1472731200;			// Unix Time = 01.09.2016г., четверг

  RCC_GetClocksFreq(&RCC_Clocks);
  // Use SysTick as reference for the delay loops.
  SysTick_Config (SystemCoreClock / TIMER_FREQUENCY_HZ);


  memset((uint8_t *)&r103Mesure, 0, sizeof(r103Mesure));
#warning " !!! Указать, какому контуру принадлежит контроллер (Горячий/Холодный)"
  r103Mesure.coldHot = COLD;

  canInit();
  delayUsInit();
  toInit();
  flowSensInit();

  // Infinite loop
  while (1)
    {
  		timersProcess();
  		canProcess();
//  		thermoProcess();
    }
}

void thermoProcess( void ){
	uint8_t newToData = FALSE;
	int16_t dTo = (r103Mesure.toAdj - r103Mesure.to[TO_OUT]) / 8;

	if( r103Stat.flowStat ){
		if( (dTo > 2) && ((r103Stat.toStat == TO_DOWN) || (r103Stat.toStat == TO_STOP)) ){
			newToData = TRUE;
			r103Stat.toStat = TO_UP;
		}
		if( (dTo < -2) && ((r103Stat.toStat == TO_UP) || (r103Stat.toStat == TO_STOP)) ) {
			newToData = TRUE;
			r103Stat.toStat = TO_DOWN;
		}
		if( newToData ){
			r103Mesure.degAdj = r103Mesure.degCur + 1;
			canSendMsg( VALVE_DEG, r103Mesure.degAdj );
			r103Stat.flowStat = FALSE;
		}
	}

}

// ----------------------------------------------------------------------------
