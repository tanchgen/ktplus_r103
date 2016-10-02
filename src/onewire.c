/*
 * onewire.c
 *
 *  Created on: 21 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */
#include <main.h>
#include "stm32f10x.h"
#include "my_time.h"
//#include "onewire.h"


// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[8];

uint8_t owDevNum;
eOwStatus owStatus;
tOwToDev owToDev[ TO_DEV_NUM ]; 			// Массив структур термометров 1-Wire;

uint8_t rxCount;
uint8_t txCount;

uint32_t tmpModerOut, tmpModerAf;			 // Значения регистра MODER для UART и для подтяжки UART_RX к Vdd


#define OW_0	0x00
#define OW_1	0xff
#define OW_R_1	0xff

//static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits);

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
//static uint8_t OW_toByte(uint8_t *ow_bits);

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
//static eOwStatus OW_Reset( void );

// внутренняя процедура. Записывает указанное число бит
//static eOwStatus OW_SendBits(uint8_t num_bits);

// Настройка выводов GPIO на прием 1-Wire
static void toPinSetInput( toSensNum toNum );
// Настройка выводов GPIO на передачу 1-Wire
static void toPinSetOutput( toSensNum toNum );


void toInit( void ){
	// Инициализация выводов для работы с 1-wire To-датчиками

	TO_IN_CLK_ENABLE;
	TO_OUT_CLK_ENABLE;

	owToDev[TO_IN].port = TO_IN_PORT;
	owToDev[TO_IN].pin = TO_IN_PIN;
	owToDev[TO_IN].pinNum = TO_IN_PIN_NUM;
	owToDev[TO_OUT].port = TO_OUT_PORT;
	owToDev[TO_OUT].pin = TO_OUT_PIN;
	owToDev[TO_OUT].pinNum = TO_OUT_PIN_NUM;

	owToDev[TO_IN].mesurAcc = MESUR_ACC;
	owToDev[TO_IN].tMax = TEMPER_MAX;
	owToDev[TO_IN].tMax = TEMPER_MIN;
	owToDev[TO_IN].newErr = FALSE;
	owToDev[TO_OUT].mesurAcc = MESUR_ACC;
	owToDev[TO_OUT].tMax = TEMPER_MAX;
	owToDev[TO_OUT].tMax = TEMPER_MIN;
	owToDev[TO_OUT].newErr = FALSE;

	toPinSetInput( TO_IN );
	toPinSetInput( TO_OUT );
	toReadCount = TO_READ_TOUT;

}

static void toPinSetInput( toSensNum toNum ) {
	uint16_t pin = owToDev[toNum].pin;
	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;

	if (pin < GPIO_Pin_8){
		port->CRL &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << (pinNum * 4));
		// Устанавливаем в Input + PullUp
		port->CRL |= ( 0x8 << (pinNum * 4) );
		port->ODR |= pin;
	}
	else {
		port->CRH &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << ((pinNum - 8) * 4) );
		// Устанавливаем в Input + PullUp
		port->CRH |= ( 0x4 << ((pinNum - 8) * 4) );
		port->ODR |= pin;
	}

}

static void toPinSetOutput( toSensNum  toNum ) {
	uint16_t pin = owToDev[toNum].pin;
	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;


	if (pin < GPIO_Pin_8){
		port->CRL &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << (pinNum * 4));
		// Устанавливаем в Output + PushPull + 2 MHz
		port->CRL |= ( 0x2 << (pinNum * 4) );
	}
	else {
		port->CRH &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << ((pinNum - 8) * 4) );
		// Устанавливаем в Output + PushPull + 2 MHz
		port->CRH |= ( 0x2 << ((pinNum - 8) * 4) );
	}

}

static eOwStatus toOwReset( toSensNum toNum ) {
	uint16_t pin = owToDev[toNum].pin;
//	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;
	eOwStatus status = OW_OK;

	port->BRR |= pin;
	toPinSetOutput( toNum );
	usDelay(1000);
	toPinSetInput(toNum );
	usDelay(120);
	if(port->IDR & pin) {
		status = OW_DEV_ERR;
	}
	else {
		usDelay(500);
		if( !(port->IDR & pin) ) {
			status = OW_DEV_ERR;
		}
		else {
			usDelay(400);
		}
	}
	return owToDev[toNum].devStatus = status;
}

uint8_t toOwRead( toSensNum toNum ){
	uint16_t pin = owToDev[toNum].pin;
//	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;
	uint8_t data=0;

	for (uint8_t i = 0; i < 8; i++)	{
		data >>= 1;
		port->BRR |= pin;
		toPinSetOutput( toNum );
		usDelay(2);
		toPinSetInput( toNum );
		usDelay(28);
		if (port->IDR & pin)
			data |= 0x80;
		port->BSRR |= pin;
		toPinSetOutput( toNum );
		usDelay(100);
	}
	return data;

}

void toOwWrite(toSensNum toNum, uint8_t value) {
	uint16_t pin = owToDev[toNum].pin;
//	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;
//	eOwStatus status = OW_OK;

	port->BSRR |= pin;
	toPinSetOutput( toNum );
	for (uint8_t i= 0; i < 8; i++) {
		port->BRR |= pin;
		usDelay(2);
		if (value & 0x01){
			port->BSRR |= pin;
		}
		value >>= 1;
		usDelay(120);
		port->BSRR |= pin;
		usDelay(2);
	}
}

void toReadTemperature( toSensNum toNum ) {
//	uint8_t sendBuf[11];
	int16_t readBuf;

	uint16_t pin = owToDev[toNum].pin;
//	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;

	if( toOwReset( toNum ) ){
		r103Mesure.to[toNum] = 0xFFFF;
		return;
	}
	toOwWrite( toNum, SCIP_ROM );
	toOwWrite( toNum, TERM_CONVERT );

	// При паразитном питании требуется дополнительная подтяжка провода шины к питанию на >10мс
	// Включаем подтяжку шины 1-Wire к Vdd
	port->BSRR |= pin;										// Выставляем в 1
	toPinSetOutput( toNum );

	// Задержка на пересчет измерения
	// Задержка для данной точности датчиков ( 95, 190, 380 или 760 мс )
	myDelay( MESUR_TIME_9 << (owToDev[toNum].mesurAcc - 9) );

	// Считываем показания датчиков
	toOwReset( toNum );
	toOwWrite( toNum, SCIP_ROM );
	toOwWrite( toNum, MEM_READ );
	readBuf = toOwRead( toNum );
	readBuf |= toOwRead( toNum ) << 8;

	if( readBuf & 0xf800){
		readBuf |= 0xf800;
	}
	r103Mesure.to[toNum] = readBuf;
}

