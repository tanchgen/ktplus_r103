/*
 * flow.h
 *
 *  Created on: 25 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef FLOW_H_
#define FLOW_H_

#include "stm32f10x.h"

#define FLOW_SENS_PIN 							GPIO_Pin_8
#define FLOW_SENS_PORT	 						GPIOA
#define FLOW_SENS_PORTSOURCE 				GPIO_PortSourceGPIOA
#define FLOW_SENS_PINSOURCE 				GPIO_PinSource8
#define FLOW_SENS_EXTI_LINE 				EXTI_Line8
#define FLOW_SENS_NVIC_IRQCHANNEL 	EXTI9_5_IRQn
#define FLOW_SENS_NVIC_IRQHANDLER 	EXTI9_5_IRQHandler

#define FLOW_READ_TOUT							1000

// Коэффициент пересчета имп. в поток (см3 на импульс)
#define KFLOW								(16)
// Коэффициент коррекции определенный опытным путем
#define KFLOW_CORRECT				((float)0.98300213866)

extern volatile uint32_t flowCount;

void flowSensInit(void);
void flowSensIrqHandler(void);
void flowGetVolume(void);
void flowSecondProcess( void );
uint32_t getDemandFlow( void );
#endif /* FLOW_H_ */
