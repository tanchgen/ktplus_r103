/*
 * flow.c
 *
 *  Created on: 25 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <main.h>
#include "my_time.h"

volatile uint32_t flowCount;
// Коэффициент пересчета имп. в поток (см3 на импульс)
#define KFLOW								(33)
// Коэффициент коррекции определенный опытным путем
#define KFLOW_CORRECT				((float)0.987654321)
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

void flowGetVolume(void) {

	r103Mesure.flowNow = flowCount * KFLOW;
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
	static uint32_t qMin;
	static uint32_t dToHour;
	int16_t deltaTo;

	//	Вычисление тепловой энергии за секунду
	deltaTo = ((r103Mesure.to[r103Mesure.coldHot ^ 0x1] - r103Mesure.to[r103Mesure.coldHot]));
	if( (deltaTo<0) || (r103Mesure.to[TO_IN] == 0x7FF) || (r103Mesure.to[TO_OUT] == 0x7FF) ){
		deltaTo =0;
	}
// ********** !!! Симуляция !!! ****************
	deltaTo = 8;
	r103Mesure.flowSec = 20;

	dToHour += deltaTo;
	sigmh = deltaTo;
	sigmh *= C_H2O;
	sigmh /= 16.0;
	r103Mesure.qSec = sigmh * r103Mesure.flowSec;
	fMin += r103Mesure.flowSec;
	qMin += r103Mesure.qSec;
	if( !(sysTime.Seconds ) ){
		canSendMsg( TO_IN_MSG, r103Mesure.to[TO_IN]);
		canSendMsg( TO_OUT_MSG, r103Mesure.to[TO_OUT]);
		canSendMsg( FLOW, fMin / 60 );
		canSendMsg( POWER_SEC, qMin / 60 );
		fHour += fMin;
		if( !(sysTime.Minutes) ){
			canSendMsg( FLOW_HOUR, fHour );
			fHour = 0;
			canSendMsg(	TO_DELTA_HOUR, dToHour/16 );
			dToHour = 0;
		}
		fMin = 0;
		qMin = 0;
	}

	r103Mesure.qDay += 	r103Mesure.qSec;
	if( !(sysTime.Seconds || sysTime.Minutes || sysTime.Hours) ){
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

	uint16_t dto = (r103Mesure.coldHot)? r103Mesure.to[0] - r103Mesure.to[1] : r103Mesure.to[1] - r103Mesure.to[0];
	uint16_t ndto = (r103Mesure.coldHot)? r103Mesure.to[0] - r103Mesure.toAdj : r103Mesure.toAdj - r103Mesure.to[0];

	return ( r103Mesure.flowSec * dto / ndto );
}



