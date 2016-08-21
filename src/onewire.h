/*
 * onewire.h
 *
 *  Created on: 21 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef ONEWIRE_H_
#define ONEWIRE_H_


#define TO_IN_PIN					GPIO_Pin_11
#define TO_IN_PIN_NUM			11
#define TO_IN_PORT				GPIOA

#define TO_IN_CLK_ENABLE		RCC->APB1ENR |= RCC_APB2ENR_IOPAEN
//#else
//#error "Need define other function for enable clk for this GPIO port"
//#endif

#define TO_OUT_PIN				GPIO_Pin_12
#define TO_OUT_PIN_NUM		12
#define TO_OUT_PORT				GPIOA

#define TO_OUT_CLK_ENABLE		RCC->APB1ENR |= RCC_APB2ENR_IOPAEN
//#else
//#error "Need define other function for enable clk for this GPIO port"
//#endif

typedef enum {
	TO_IN,
	TO_OUT,
} toSensNum ;

typedef enum {
	OW_OK,
	OW_WIRE_ERR,
	OW_DEV_ERR,
	OW_DEV_NUM_ERR,
	OW_ERR
} eOwStatus;


typedef struct {
	GPIO_TypeDef * port;
	uint16_t pin;
	uint8_t pinNum;
	uint64_t addr;						//  Адрес устройства
	uint8_t  mesurAcc;				//  Точность измерения ( 9-10-11-12 бит )
	int16_t tMin;						//  Допустимый максимум температуры
	int16_t tMax;						//  Допустимый минимум температуры
	eOwStatus devStatus;
	uint8_t newErr;					//	Признак новой ошибки
	int16_t	temper;						// Действующее значение температуры
} tOwToDev;

#define MAX_TO_DEV_NUM	4

#define TO_DEV_NUM			3				// Количество термометров
#if (MAX_TO_DEV_NUM < TO_DEV_NUM)
#error "Число датчиков температуры превышает максимальное для данной EEPROM (128кБ) для Логов"
#endif

#define DD_DEV_NUM			2				// Количество Датчиков Дверей (DD)
#define OW_DEV_NUM			TO_DEV_NUM

#endif

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
extern eOwStatus owStatus;
extern tOwToDev owToDev[]; 			// Массив структур устройств 1-Wire;
extern tDdDev ddDev[]; 			// Массив структур Датчиков Двери 1-Wire;

extern uint32_t tmpModerOut, tmpModerAf;			 // Значения регистра MODER для UART и для подтяжки UART_RX к Vdd

uint8_t OW_Init();
uint8_t OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen, uint8_t *data, uint8_t dLen, uint8_t readStart);
uint8_t OW_Scan(uint8_t *buf, uint8_t num);

#endif /* ONEWIRE_H_ */
