/*
 * flow.h
 *
 *  Created on: 25 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef FLOW_H_
#define FLOW_H_

#include "stm32f10x.h"

#define FLOW_SENS_PIN 							GPIO_Pin_10
#define FLOW_SENS_PORT	 						GPIOB
#define FLOW_SENS_PORTSOURCE 				GPIO_PortSourceGPIOB
#define FLOW_SENS_PINSOURCE 				GPIO_PinSource10
#define FLOW_SENS_EXTI_LINE 				EXTI_Line10
#define FLOW_SENS_NVIC_IRQCHANNEL 	EXTI15_10_IRQn
#define FLOW_SENS_NVIC_IRQHANDLER 	EXTI15_10_IRQHandler

#define FLOW_READ_TOUT							1000

extern volatile uint32_t flowCount;

void flowSensInit(void);
void flowSensIrqHandler(void);
void flowGetVolume(void);
void flowSecondProcess( void );
uint32_t getDemandFlow( void );
#endif /* FLOW_H_ */
