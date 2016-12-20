/*
 * flow.c
 *
 *  Created on: 25 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <main.h>
#include "my_time.h"
#include "flow.h"

volatile uint32_t flowCount;

uint16_t ktf[21];					// Таблица поправочных коэффициентов потока на температуру

void flowSensInit(void)
{
	GPIO_InitTypeDef FLOW_SENS_GPIO_InitStructure;
	EXTI_InitTypeDef FLOW_SENS_EXTI_InitStructure;
	NVIC_InitTypeDef FLOW_SENS_NVIC_InitStructure;

	RCC->APB2ENR |= RCC_APB2ENR_IOPBEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

	FLOW_SENS_GPIO_InitStructure.GPIO_Pin = FLOW_SENS_PIN;
	FLOW_SENS_GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
	GPIO_Init( FLOW_SENS_PORT, &FLOW_SENS_GPIO_InitStructure);
	GPIO_EXTILineConfig(FLOW_SENS_PORTSOURCE, FLOW_SENS_PINSOURCE);
	FLOW_SENS_EXTI_InitStructure.EXTI_Line = FLOW_SENS_EXTI_LINE;
	FLOW_SENS_EXTI_InitStructure.EXTI_Mode = EXTI_Mode_Interrupt;
	FLOW_SENS_EXTI_InitStructure.EXTI_Trigger = EXTI_Trigger_Rising_Falling;
	FLOW_SENS_EXTI_InitStructure.EXTI_LineCmd = ENABLE;
	EXTI_Init(&FLOW_SENS_EXTI_InitStructure);
	FLOW_SENS_NVIC_InitStructure.NVIC_IRQChannel = FLOW_SENS_NVIC_IRQCHANNEL;
	FLOW_SENS_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2;
	FLOW_SENS_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	FLOW_SENS_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&FLOW_SENS_NVIC_InitStructure);
	flowCount = 0;
	// TODO: Таблица поправочных коэффициентов
/*	for(uint8_t i=0; i<21; i++) {
		ktf[i] = 1;
	}
*/

}

void flowSecondProcess( void ) {
#define C_H2O				4.187			// Удельная теплоемкость воды
	double sigmh;
	static uint32_t prevFlow;
	static uint32_t fHour;
	static uint32_t dToHour;
	int16_t deltaTo;

	//	Вычисление тепловой энергии за секунду
	deltaTo = ((r103Mesure.to[TO_IN] - r103Mesure.to[TO_OUT]));
	if( deltaTo<0 ){
		deltaTo = -deltaTo;
	}
	dToHour += deltaTo;
	// Вычисляем поток за секунду
	// Эмуляция
//	flowCount += 10;
	r103Mesure.flowSec = (uint32_t)((float)(flowCount - prevFlow) * KFLOW);
	prevFlow = flowCount;
	if( minuteFlag ){
		// Минута окончилась.
		minuteFlag = FALSE;
		// Получаем поток за минуту
		r103Mesure.flowMin = (uint32_t)((float)flowCount * KFLOW);
		flowCount = 0;
		// Высчитываем сигму
		sigmh = deltaTo;
		sigmh *= C_H2O;
		sigmh *= 0.0625;
		r103Mesure.qMin = sigmh * r103Mesure.flowMin;
		canSendMsg( TO_IN_MSG, r103Mesure.to[TO_IN]);
		canSendMsg( TO_OUT_MSG, r103Mesure.to[TO_OUT]);
		canSendMsg( FLOW, r103Mesure.flowMin / 60 );
		canSendMsg( POWER_SEC, r103Mesure.qMin / 60 );
		fHour += r103Mesure.flowMin;
		if( hourFlag ){
			// Час окончился
			hourFlag = FALSE;
			canSendMsg( FLOW_HOUR, fHour );
			fHour = 0;
			canSendMsg(	TO_DELTA_HOUR, dToHour );
			dToHour = 0;
		}
		r103Mesure.qDay += 	r103Mesure.qMin;
		r103Mesure.flowMin = 0;
		r103Mesure.qMin = 0;
	}
	if( dayFlag ){
		// День закончился
		dayFlag =FALSE;
		r103Mesure.qWeek += r103Mesure.qDay;
		r103Mesure.qMonth += r103Mesure.qDay;
		canSendMsg( POWER_DAY, r103Mesure.qDay );
		r103Mesure.qDay = 0;
		if( sysDate.WeekDay == 1 ) {
			canSendMsg( POWER_WEEK, r103Mesure.qWeek );
			r103Mesure.qWeek = 0;
		}
		if( sysDate.Date == 1 ) {
			canSendMsg( POWER_MON, r103Mesure.qMonth );
			r103Mesure.qMonth = 0;
		}
	}
}

// Расчет нужного потока для получения заданной температуры
uint32_t getDemandFlow( void ){

	uint16_t dto;
	uint16_t ndto;
	dto = r103Mesure.to[TO_IN] - r103Mesure.to[TO_OUT];
	ndto = r103Mesure.to[TO_IN] - r103Mesure.toAdj;

	return ( (r103Mesure.flowSec * KFLOW) * dto / ndto );
}


/*
void flowGetVolume(void) {

	if(r103Mesure.flowNow < r103Mesure.flowPrev){
		// Если счетчик flowCount перешел через 0
		r103Mesure.flowNow += (0xFFFFFFFF) - r103Mesure.flowPrev;
	}
	r103Mesure.flowSec = (uint32_t)((float)(r103Mesure.flowNow - r103Mesure.flowPrev) * KFLOW_CORRECT);
	r103Mesure.flowPrev = r103Mesure.flowNow;

}

void flowSecondProcess( void ) {
#define C_H2O				4.187			// Удельная теплоемкость воды
	double sigmh;
	static uint32_t fMin;
	static uint32_t fHour;
	static uint32_t dToHour;
	int16_t deltaTo;

	//	Вычисление тепловой энергии за <i>
	deltaTo = ((r103Mesure.to[TO_IN] - r103Mesure.to[TO_OUT]));
	if(deltaTo < 0){
		deltaTo = -deltaTo;
	}
	if( (r103Mesure.to[TO_IN] == 0x7FF) || (r103Mesure.to[TO_OUT] == 0x7FF) ){
		deltaTo =0;
	}

	// ********** !!! Симуляция !!! ****************
//	deltaTo = 3*16;
//	r103Mesure.flowSec = 20;

	dToHour += deltaTo;
	fMin += r103Mesure.flowSec;

	r103Mesure.qDay += 	r103Mesure.qSec;

	if( minuteFlag ){
		// Минута окончилась.
		minuteFlag = FALSE;

		sigmh = deltaTo;
		sigmh *= C_H2O;
		sigmh /= 16.0;
		r103Mesure.qSec = (sigmh * fMin)/60;
		r103Mesure.qDay += 	r103Mesure.qSec;

		canSendMsg( TIME, uxTime);
		canSendMsg( TO_IN_MSG, r103Mesure.to[TO_IN]);
		canSendMsg( TO_OUT_MSG, r103Mesure.to[TO_OUT]);
		canSendMsg( FLOW, fMin / 60 );
		canSendMsg( POWER_SEC, r103Mesure.qSec );
		fHour += fMin;
		fMin = 0;
		if( hourFlag ){
			// Час окончился
			hourFlag = FALSE;
			canSendMsg( FLOW_HOUR, fHour );
			fHour = 0;
			canSendMsg(	TO_DELTA_HOUR, dToHour/16 );
			dToHour = 0;
			if( dayFlag ){
				// День закончился
				dayFlag =FALSE;
				r103Mesure.qWeek += r103Mesure.qDay;
				r103Mesure.qMonth += r103Mesure.qDay;
				canSendMsg( POWER_DAY, r103Mesure.qDay );
				r103Mesure.qDay = 0;
				if( weekFlag ) {
					// Неделя закончилась
					weekFlag = FALSE;
					canSendMsg( POWER_WEEK, r103Mesure.qWeek );
					r103Mesure.qWeek = 0;
				}
				if( monthFlag ) {
					// Месяц закончился
					monthFlag = FALSE;
					canSendMsg( POWER_MON, r103Mesure.qMonth );
					r103Mesure.qMonth = 0;
				}
			}
		}
	}
}

// Расчет нужного потока для получения заданной температуры
uint32_t getDemandFlow( void ){

	uint16_t dto = (r103Mesure.coldHot)? r103Mesure.to[0] - r103Mesure.to[1] : r103Mesure.to[1] - r103Mesure.to[0];
	uint16_t ndto = (r103Mesure.coldHot)? r103Mesure.to[0] - r103Mesure.toAdj : r103Mesure.toAdj - r103Mesure.to[0];

	return ( r103Mesure.flowSec * dto / ndto );
}
*/

