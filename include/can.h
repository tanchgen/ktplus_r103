/*
 *
 * can.h
 *  Created on: 30 июля 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef CAN_H_
#define CAN_H_

#include "stm32f10x.h"

#define CAN_RX_PIN 						GPIO_Pin_8
#define CAN_RX_PIN_NUM				8
#define CAN_RX_PORT 					GPIOB

#define CAN_TX_PIN 						GPIO_Pin_9
#define CAN_TX_PIN_NUM				9
#define CAN_TX_PORT 					GPIOB

#define CAN_RCC_GPIOEN 				RCC_APB2ENR_IOPBEN


#define CAN_RCC_CAN 									RCC_APB1Periph_CAN1
#define CAN_CAN 											CAN1

#define CAN1_TX_IRQn  USB_HP_CAN1_TX_IRQn     /*!< USB Device High Priority or CAN1 TX Interrupts       */
#define CAN1_RX0_IRQn  USB_LP_CAN1_RX0_IRQn   /*!< USB Device Low Priority or CAN1 RX0 Interrupts       */

/*

#define CAN_RX0_NVIC_IRQCHANNEL 			CAN1_RX0_IRQn
#define CAN_RX0_NVIC_IRQHANDLER 			CAN1_RX0_IRQHandler
#define CAN_RX1_NVIC_IRQCHANNEL 			CAN1_RX1_IRQn
#define CAN_RX1_NVIC_IRQHANDLER 			CAN1_RX1_IRQHandler
#define CAN_TX_NVIC_IRQCHANNEL 				CAN1_TX_IRQn
#define CAN_TX_NVIC_IRQHANDLER 				CAN1_TX_IRQHandler
#define CAN_SCE_NVIC_IRQCHANNEL 			CAN1_SCE_IRQn
#define CAN_SCE_NVIC_IRQHANDLER 			CAN1_SCE_IRQHandler
*/

#define CAN_RX_MESSAGE_LEN		sizeof(tCanMessage)
#define CAN_RX_BUFFER_SIZE 		20
#define CAN_TX_BUFFER_SIZE 		10
#define CAN_TX_MESSAGE_LEN		sizeof(tCanMessage)

typedef struct CanMessageStruct
{
  uint32_t StdId;
  uint32_t ExtId;
  uint8_t IDE;
  uint8_t RTR;
  uint8_t DLC;
  uint8_t Data[8];
	uint8_t FMI;
} tCanMessage;

#define CUR_ADJ_MASK		(uint32_t)0x10000000
#define MSG_ID_MASK			(uint32_t)(0x7F << 21)		//(uint32_t)0x0FE00000
#define S207_MASK				(uint32_t)0x00100000
#define DEV_ID_MASK			(uint32_t)0x000FFFFF

typedef struct {
	uint8_t adjCur;		// Действующее-задаваемое
	uint8_t msgId;		// Идентификатор сообщения
	uint8_t coldHot;	// Охлаждающий-нагревательный контур
	uint8_t s207;			// Признак S207-контроллера
	uint32_t devId;		// Идентификатор устройства
} tCanId;

typedef struct {
	uint32_t idList;
	uint32_t idMask;
	uint8_t ideList;
	uint8_t ideMask;
	uint8_t rtrList;
	uint8_t rtrMask;
} tFilter;

typedef enum {				// Действующее-Задаваемое
	CUR,
	ADJ
} eCurAdj;


typedef enum {							// Условный номер сообщения
	NULL_MES = 0,
	TO_IN_MSG = 1,						// Входящая температура
	TO_OUT_MSG,								// Выходящая температура
	TO_DELTA_HOUR,						// Сумма разница темпкратур за час
	VALVE_DEG,								// Угол поворота задвижки
	FLOW,											// Значение потока - показания расходомера
	FLOW_HOUR,								// Поток за час
	POWER_SEC,								// Тепловая работа в сек
	POWER_DAY,								// Тепловая работа в сутки
	POWER_WEEK,								// Тепловая работа в неделю
	POWER_MON,								// Тепловая работа в месяц
	VALVE_ID,									// ID контоллера задвижки
	IMP_TOTAL,								// Общее количество импульсов от 0 до 90 гр.
	IMP_EXEC,									// Количество импульсов пройденное с последнего раза

	TIME = 0x0F,							// Дата-Время

// Сообщения от электросчетчика
	AWATT_NOW = 0x10,         // Действующее значение потребляемой мощности
  AENRG_DAY = 0x11,         // Значение потребляемой мощности за сутки
  AENRG_DAY_SUM = 0x12,     // Значение потребляемой мощности за сутки с нарастающим итогом
  AENRG_MON = 0x13,         // Значение потребляемой мощности за месяц
  AENRG_MON_SUM = 0x14,     // Значение потребляемой мощности за месяц с нарастающим итогом
	AWATT_MON_MAX = 0x15,     // Пиковое значение потребляемой мощности за месяц
	// Сообщения от S207
	WGAIN_A = 0x16,           // Коэффициент коррекции мощности - Фаза A
  WGAIN_B = 0x17,           // Коэффициент коррекции мощности - Фаза B
  WGAIN_C = 0x18,           // Коэффициент коррекции мощности - Фаза C
  WGAIN_N = 0x19,           // Коэффициент коррекции мощности - Фаза N
  WOS_A = 0x1A,             // Коэффициент коррекции мощности - Фаза A
  WOS_B = 0x1B,             // Коэффициент коррекции мощности - Фаза B
  WOS_C = 0x1C,             // Коэффициент коррекции мощности - Фаза C
  WOS_N = 0x1D,             // Коэффициент коррекции мощности - Фаза N
	WATT_SEND_TOUT = 0x1E,    // Интервал отправки данных

	// От S207 - задаются параметры, к S207 - превышены значения параметров
	WATT_MAX  = 0x1F,         // Максимум потребляемой мощности
	CUR_MAX = 0x20,           // Максимальный ток в цепи
	VOLT_MAX = 0x21,          // Максимальное напряжение в цепи

// Ошибки в системе
	DEG0_FAULT = 0x31,				// Неисправность концевого датчика 0гр.
	DEG90_FAULT,							// Неисправность концевого датчика 90гр.
	VALVE_SENS_FAULT,					// Неисправность датчика положения задвижки
	VALE_MOT_FAULT,						// Неисправность мотора задвижки
	TO_IN_SENS_FAULT,					// Неисправность датчика температуры вход
	TO_OUT_SENS_FAULT,				// Неисправность датчика температуры выход
	FLOW_SENS_FAULT,					// Неисправность датчика расходомера
	DEV_FAULT,								// Нисправность контроллера

} eMsgId;

enum _eS207 {
	nS207_DEV = 0,			// Поле признака НЕ-S207 - устройство
	S207_DEV	= 1				// Поле признака S207-устройства
};

typedef struct {			// Структура CAN-сообщения
	eCurAdj curAdj;
	eMsgId messId;
	uint32_t devId;			// Идентификационный номер контроллера ( Serial ID MCU )
} tIdStruct;

extern uint32_t selfDevId;

uint8_t CAN_ReceiveMessage(tCanMessage *RxMessage);
uint8_t CAN_TransmitMessage(tCanMessage *TxMessage);


void canInit( void );
void canBspInit( void );
void canFilterInit( void );
void canFilterUpdate( tFilter * filter, uint8_t filterNum );

void canSendMsg( eMsgId msgId, uint32_t data );

void canProcess( void );

void canRx0IrqHandler(void);
void canRx1IrqHandler(void);
void canTxIrqHandler(void);
void canSceIrqHandler(void);

uint32_t setIdList( tCanId *canid );
#endif /* CAN_H_ */
