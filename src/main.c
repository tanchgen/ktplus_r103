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
uint32_t VlvDevId = 0;			// Иденитификатор контроллера задвижки

RCC_ClocksTypeDef RCC_Clocks;

#define TO_INTERVAL  15000

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
  		thermoProcess();
    }
}

void thermoProcess( void ){
	static uint32_t toTout;
	int8_t toDir;
	uint8_t newToData = FALSE;
	int16_t dTo;

	if( toTout > myTick ){
		return;
	}
	toTout += TO_INTERVAL;

	toDir = (r103Mesure.to[TO_OUT] - r103Mesure.toPrev) / 16;
	if( toDir > 1 ) {
		r103Stat.toStat = TO_UP;
	}
	else if ( toDir < -1 ) {
		r103Stat.toStat = TO_DOWN;
	}
	else {
		r103Stat.toStat = TO_STOP;
	}

	r103Mesure.toPrev = r103Mesure.to[TO_OUT];

	dTo = (r103Mesure.toAdj - r103Mesure.to[TO_OUT]) / 16;

	if( r103Stat.flowStat ){
		if( (dTo > 1) && ((r103Stat.toStat == TO_DOWN) || (r103Stat.toStat == TO_STOP)) ){
			newToData = TRUE;
			r103Stat.toStat = TO_UP;
		}
		if( (dTo < -1) && ((r103Stat.toStat == TO_UP) || (r103Stat.toStat == TO_STOP)) ) {
			newToData = TRUE;
			r103Stat.toStat = TO_DOWN;
		}
		if( newToData ){
			int8_t deltaDeg;
			if( r103Mesure.coldHot == COLD  ){
				if( r103Stat.toStat == TO_DOWN ){
					deltaDeg = DELTA_DEG;
				}
				else {
					deltaDeg = -DELTA_DEG;
				}
			}
			else {
				if( r103Stat.toStat == TO_DOWN ){
					deltaDeg = -DELTA_DEG;
				}
				else {
					deltaDeg = DELTA_DEG;
				}
			}
			r103Mesure.degAdj = r103Mesure.degCur + deltaDeg;
			if( r103Mesure.degAdj < 0  ){
				r103Mesure.degAdj = 0;
			}
			else if( r103Mesure.degAdj > 90 ){
				r103Mesure.degAdj = 90;
			}
			canSendMsg( VALVE_DEG, r103Mesure.degAdj );
			r103Stat.flowStat = FALSE;
		}
	}

}

// ----------------------------------------------------------------------------
