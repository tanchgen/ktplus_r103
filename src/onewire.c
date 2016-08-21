/*
 * onewire.c
 *
 *  Created on: 21 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */
#include "stm32f10x.h"
#include "onewire.h"


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

static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits);

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static uint8_t OW_toByte(uint8_t *ow_bits);

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
static eOwStatus OW_Reset( void );

// внутренняя процедура. Записывает указанное число бит
static eOwStatus OW_SendBits(uint8_t num_bits);


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

}

void toPinSetInput( uint8_t toNum ) {
	uint16_t pin = owToDev[toNum].pin;
	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;

	if (pin < GPIO_Pin_8){
		port->CRL &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << pinNum);
		// Устанавливаем в Input + PullUp
		port->CRL |= ( 0x8 << pinNum );
		port->ODR |= pin;
	}
	else {
		port->CRH &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << (pinNum - 8) );
		// Устанавливаем в Input + PullUp
		port->CRH |= ( 0x8 << (pinNum - 8) );
		port->ODR |= pin;
	}

}

void toPinSetOutput( toSensNum  toNum ) {
	uint16_t pin = owToDev[toNum].pin;
	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;


	if (pin < GPIO_Pin_8){
		port->CRL &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << pinNum);
		// Устанавливаем в Output + PashPull + 2 MHz
		port->CRL |= ( 0x2 << pinNum );
	}
	else {
		port->CRH &= ~((GPIO_CRL_CNF0|GPIO_CRL_MODE0) << (pinNum - 8) );
		// Устанавливаем в Output + PashPull + 2 MHz
		port->CRH |= ( 0x2 << (pinNum - 8) );
	}

}

static eOwStatus toOwReset( toSensNum toNum ) {
	uint16_t pin = owToDev[toNum].pin;
	uint8_t pinNum = owToDev[toNum].pinNum;
	GPIO_TypeDef * port = owToDev[toNum].port;
	eOwStatus status = OW_OK;

	port->BRR |= pin;
	toPinSetOutput();
	usDelay(500);
	toPinSetInput();
	usDelay(60);
	if(port->IDR & pin) {
		status = OW_DEV_ERR;
	}
	else {
		usDelay(250);
		if( !(port->IDR & pin) ) {
			status = OW_DEV_ERR;
		}
		else {
			usDelay(200);
		}
	}
	return owToDev[toNum].devStatus = status;
}

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------

// внутренняя процедура. Записывает указанное число бит
static int8_t OW_SendBits(uint8_t num_bits) {
	DMA_InitTypeDef DMA_InitStructure;
	uint32_t owTout;

	// DMA на чтение
	DMA_DeInit(OW_DMA_CH_RX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->RDR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
	DMA_InitStructure.DMA_BufferSize = num_bits;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(OW_DMA_CH_RX, &DMA_InitStructure);

	// DMA на запись
	DMA_DeInit(OW_DMA_CH_TX);
	DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t) &(OW_USART->TDR);
	DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t) ow_buf;
	DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
	DMA_InitStructure.DMA_BufferSize = num_bits;
	DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
	DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
	DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
	DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
	DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
	DMA_InitStructure.DMA_Priority = DMA_Priority_Low;
	DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
	DMA_Init(OW_DMA_CH_TX, &DMA_InitStructure);

	// старт цикла отправки
	OW_DMA_CH_RX->CCR |= DMA_CCR_EN;
	OW_DMA_CH_TX->CCR |= DMA_CCR_EN;
	USART_ClearFlag(OW_USART, USART_FLAG_RXNE | USART_FLAG_TXE);
	OW_USART->ICR |= USART_ICR_TCCF; /* Clear transfer complete flag */
	USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, ENABLE);

	owTout = myTick + OW_TRANS_TOUT;
	// Ждем, пока не примем 8 байт
	while (DMA_GetFlagStatus(OW_DMA_RX_FLAG) == RESET) {
		if ( myTick > owTout ) {
			return -1;
		}
	}

	// отключаем DMA
	DMA_Cmd(OW_DMA_CH_TX, DISABLE);
	DMA_Cmd(OW_DMA_CH_RX, DISABLE);
	USART_DMACmd(OW_USART, USART_DMAReq_Tx | USART_DMAReq_Rx, DISABLE);
	return num_bits;
}



//-----------------------------------------------------------------------------
// Данная функция осуществляет сканирование сети 1-wire и записывает найденные
//   ID устройств в массив buf, по 8 байт на каждое устройство.
// переменная num ограничивает количество находимых устройств, чтобы не переполнить
// буфер.
//-----------------------------------------------------------------------------
uint8_t OW_Scan(uint8_t *buf, uint8_t num) {

	uint8_t found = 0;
	uint8_t *lastDevice = buf;
	uint8_t *curDevice = buf;
	uint8_t numBit, lastCollision, currentCollision, currentSelection;

	lastCollision = 0;
	while (found < num) {
		numBit = 1;
		currentCollision = 0;

		// посылаем команду на поиск устройств
		OW_Send(OW_SEND_RESET, (uint8_t*)"\xf0", 1, 0, 0, OW_NO_READ);

		for (numBit = 1; numBit <= 64; numBit++) {
			// читаем два бита. Основной и комплементарный
			OW_toBits(OW_READ_SLOT, ow_buf);
			if (OW_SendBits(2) < 0) {
				return -1;
			}

			if (ow_buf[0] == OW_R_1) {
				if (ow_buf[1] == OW_R_1) {
					// две единицы, где-то провтыкали и заканчиваем поиск
					return found;
				} else {
					// 10 - на данном этапе только 1
					currentSelection = 1;
				}
			} else {
				if (ow_buf[1] == OW_R_1) {
					// 01 - на данном этапе только 0
					currentSelection = 0;
				} else {
					// 00 - коллизия
					if (numBit < lastCollision) {
						// идем по дереву, не дошли до развилки
						if (lastDevice[(numBit - 1) >> 3]
								& 1 << ((numBit - 1) & 0x07)) {
							// (numBit-1)>>3 - номер байта
							// (numBit-1)&0x07 - номер бита в байте
							currentSelection = 1;

							// если пошли по правой ветке, запоминаем номер бита
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						} else {
							currentSelection = 0;
						}
					} else {
						if (numBit == lastCollision) {
							currentSelection = 0;
						} else {
							// идем по правой ветке
							currentSelection = 1;

							// если пошли по правой ветке, запоминаем номер бита
							if (currentCollision < numBit) {
								currentCollision = numBit;
							}
						}
					}
				}
			}

			if (currentSelection == 1) {
				curDevice[(numBit - 1) >> 3] |= 1 << ((numBit - 1) & 0x07);
				OW_toBits(0x01, ow_buf);
			} else {
				curDevice[(numBit - 1) >> 3] &= ~(1 << ((numBit - 1) & 0x07));
				OW_toBits(0x00, ow_buf);
			}
			if (OW_SendBits(1) < 0) {
				return -1;
			}
		}
		found++;
		lastDevice = curDevice;
		curDevice += 8;
		if (currentCollision == 0)
			return found;

		lastCollision = currentCollision;
	}

	return found;
}

//-----------------------------------------------------------------------------
// процедура общения с шиной 1-wire
// sendReset - посылать RESET в начале общения.
// 		OW_SEND_RESET или OW_NO_RESET
// command - массив байт, отсылаемых в шину. Если нужно чтение - отправляем OW_READ_SLOT
// cLen - длина буфера команд, столько байт отошлется в шину
// data - если требуется чтение, то ссылка на буфер для чтения
// dLen - длина буфера для чтения. Прочитается не более этой длины
// readStart - с какого символа передачи начинать чтение (нумеруются с 0)
//		можно указать OW_NO_READ, тогда можно не задавать data и dLen
//-----------------------------------------------------------------------------
eErrStatus OW_Send(uint8_t sendReset, uint8_t *command, uint8_t cLen,
		uint8_t *data, uint8_t dLen, uint8_t readStart) {

	// если требуется сброс - сбрасываем и проверяем на наличие устройств
	if (sendReset == OW_SEND_RESET) {
		if (OW_Reset() == OW_DEV_ERR) {
			return OW_DEV_ERR;
		}
	}

	while (cLen > 0) {

		OW_toBits(*command, ow_buf);
		command++;
		cLen--;

		if (OW_SendBits(8) < 0) {
			return OW_DEV_ERR;
		}

		// если прочитанные данные кому-то нужны - выкинем их в буфер
		if (readStart == 0 && dLen > 0) {
			*data = OW_toByte(ow_buf);
			data++;
			dLen--;
		} else {
			if (readStart != OW_NO_READ) {
				readStart--;
			}
		}
	}

	return OW_OK;
}

//-----------------------------------------------------------------------------
// функция преобразует один байт в восемь, для передачи через USART
// ow_byte - байт, который надо преобразовать
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static void OW_toBits(uint8_t ow_byte, uint8_t *ow_bits) {
	uint8_t i;
	for (i = 0; i < 8; i++) {
		if (ow_byte & 0x01) {
			*ow_bits = OW_1;
		} else {
			*ow_bits = OW_0;
		}
		ow_bits++;
		ow_byte = ow_byte >> 1;
	}
}

//-----------------------------------------------------------------------------
// обратное преобразование - из того, что получено через USART опять собирается байт
// ow_bits - ссылка на буфер, размером не менее 8 байт
//-----------------------------------------------------------------------------
static uint8_t OW_toByte(uint8_t *ow_bits) {
	uint8_t ow_byte, i;
	ow_byte = 0;
	for (i = 0; i < 8; i++) {
		ow_byte = ow_byte >> 1;
		if (*ow_bits == OW_R_1) {
			ow_byte |= 0x80;
		}
		ow_bits++;
	}

	return ow_byte;
}

