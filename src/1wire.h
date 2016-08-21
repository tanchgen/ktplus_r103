/*
 * onewire.h
 *
 *  Version 1.0.1
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_

// для разных процессоров потребуется проверить функцию OW_Init
// на предмет расположения ножек USART
#include "stm32f0xx.h"
#include "my_main.h"
// ****************** Установки для UART *****************************
// выбираем, на каком USART находится 1-wire
#define OW_USART1
//#define OW_USART2
//#define OW_USART3
//#define OW_USART4

#ifdef OW_USART1

#undef OW_USART2
#undef OW_USART3
#undef OW_USART4

#define OW_USART 				USART1
#define OW_DMA_CH_RX 		DMA1_Channel3
#define OW_DMA_CH_TX 		DMA1_Channel2
#define OW_DMA_RX_FLAG	DMA1_FLAG_TC3

#define OW_TRANS_TOUT		500

#define OW_PORT					GPIOA
#define OW_TX_PIN				GPIO_Pin_2
#define OW_TX_PIN_NUM		2
#define OW_RX_PIN				GPIO_Pin_3
#define OW_RX_PIN_NUM		3

#define DD_1_PORT				GPIOB
#define DD_1_PIN				GPIO_Pin_0
#define DD_1_PIN_NUM		0

#define DD_2_PORT				GPIOB
#define DD_2_PIN				GPIO_Pin_1
#define DD_2_PIN_NUM		1

#define MAX_TO_DEV_NUM	4

#define TO_DEV_NUM			3				// Количество термометров
#if (MAX_TO_DEV_NUM < TO_DEV_NUM)
#error "Число датчиков температуры превышает максимальное для данной EEPROM (128кБ) для Логов"
#endif

#define DD_DEV_NUM			2				// Количество Датчиков Дверей (DD)
#define OW_DEV_NUM			TO_DEV_NUM

#endif


typedef struct {
	uint64_t addr;						//  Адрес устройства
	uint8_t  mesurAcc;				//  Точность измерения ( 9-10-11-12 бит )
	int16_t tMin;						//  Допустимый максимум температуры
	int16_t tMax;						//  Допустимый минимум температуры
	eErrStatus devStatus;
	uint8_t newErr;					//	Признак новой ошибки
	int16_t	temper;						// Действующее значение температуры
} tOwToDev;

typedef struct {
	uint8_t ddData;					// Действующее значение датчика двери
	uint8_t ddDataPrev;
} tDdDev;

#define DD_READ_TOUT				500					// Таймаут считывания датчиков двери

// Команды 1-Wire
#define SEARCH_ROM			0xF0
#define READ_ROM				0x33
#define MATCH_ROM				0x55
#define SCIP_ROM				0xCC
#define ALARM_SEACH			0xEC
#define TERM_CONVERT		0x44
#define MEM_WRITE				0x4E
#define MEM_READ				0xBE
#define RAM_TO_EEPROM		0x48
#define EPPROM_TO_RAM		0xB8


// первый параметр функции OW_Send
#define OW_SEND_RESET		1
#define OW_NO_RESET			2

#define OW_NO_READ			0xff

#define OW_READ_SLOT		0xff

extern uint8_t owDevNum;
extern eErrStatus owStatus;
extern tOwToDev owToDev[]; 			// Массив структур устройств 1-Wire;
extern tDdDev ddDev[]; 			// Массив структур Датчиков Двери 1-Wire;

extern uint32_t tmpModerOut, tmpModerAf;			 // Значения регистра MODER для UART и для подтяжки UART_RX к Vdd

uint8_t OW_Init();
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_Scan(uint8_t *buf, uint8_t num);

void ddReadDoor( void );

#endif /* ONEWIRE_H_ */
