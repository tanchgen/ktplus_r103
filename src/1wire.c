/*
 * onewire.c
 *
 *  Version 1.0.3
 */
#include "stm32xx_it.h"
#include "my_main.h"
#include "onewire.h"

// Буфер для приема/передачи по 1-wire
uint8_t ow_buf[8];

uint8_t owDevNum;
eErrStatus owStatus;
tOwToDev owToDev[ TO_DEV_NUM ]; 			// Массив структур термометров 1-Wire;
tDdDev ddDev[ DD_DEV_NUM ]; 			// Массив структур Датчиков Двери 1-Wire;

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
static uint8_t OW_Reset( void );

// внутренняя процедура. Записывает указанное число бит
static int8_t OW_SendBits(uint8_t num_bits);


//-----------------------------------------------------------------------------
// инициализирует USART и DMA
//-----------------------------------------------------------------------------
uint8_t OW_Init() {
	USART_InitTypeDef USART_InitStructure;

		// Clock USART1 enable

	RCC->APB2ENR |= RCC_APB2ENR_USART1EN;
		// Clock GPIOA enable
		RCC->AHBENR |= RCC_AHBENR_GPIOAEN;
		// Clock DMA enable
		RCC->AHBENR |= RCC_AHBENR_DMAEN;
		RCC->APB2ENR |= RCC_APB2ENR_SYSCFGEN;               // включить тактирование SYSCFG

		// USART TX
		OW_PORT->MODER |= 2 << (OW_TX_PIN_NUM*2);					// Выставляем в Alternate Function
		OW_PORT->OTYPER &= ~(OW_TX_PIN);			// PullUp-PullDown
		OW_PORT->OTYPER |= OW_TX_PIN;			        // Open-Drain
		OW_PORT->PUPDR &= ~(3 << (OW_TX_PIN_NUM*2));			// NoPULL
//		OW_PORT->PUPDR |= 1 << (OW_TX_PIN_NUM*2);			// PULLUP
		OW_PORT->OSPEEDR &= ~(3 << (OW_TX_PIN_NUM*2));			// Low Speed
		OW_PORT->AFR[OW_TX_PIN_NUM >> 0x3] |= 1 << ((OW_TX_PIN_NUM & 0x7) * 4);		// AF1

		// USART RX
		OW_PORT->MODER |= 2 << (OW_RX_PIN_NUM*2);					// Выставляем в Alternate Function
		OW_PORT->OTYPER &= ~(OW_RX_PIN);			// PullUp-PullDown
		OW_PORT->PUPDR &= ~(3 << (OW_RX_PIN_NUM*2));			// NoPULL
		OW_PORT->PUPDR |= 1 << (OW_RX_PIN_NUM*2);			// PULLUP
		OW_PORT->OSPEEDR &= ~(3 << (OW_RX_PIN_NUM*2));			// Low Speed
		OW_PORT->AFR[OW_RX_PIN_NUM >> 0x3] |= 1 << ((OW_RX_PIN_NUM & 0x7) * 4);		// AF1

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl = USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;

	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);
	return OW_OK;
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

//-----------------------------------------------------------------------------
// осуществляет сброс и проверку на наличие устройств на шине
//-----------------------------------------------------------------------------
static uint8_t OW_Reset() {
	uint32_t owtout;
	uint8_t ow_presence = 0xf0;
	USART_InitTypeDef USART_InitStructure;

	USART_InitStructure.USART_BaudRate = 9600;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);



	// отправляем 0xf0 на скорости 9600
	OW_USART->TDR = 0xf0;
	while ((OW_USART->ISR & USART_FLAG_TC) == RESET)
	{}

	OW_USART->ICR |= USART_ICR_TCCF; /* Clear transfer complete flag */

	owtout = myTick + OW_TRANS_TOUT;
	// Ждем, пока нe примем байт
	while ( (OW_USART->ISR & USART_ISR_RXNE) == RESET){
		if ( myTick > owtout ) {
			for ( uint8_t i = 0; i < TO_DEV_NUM; i++ ) {
				if( !owToDev[i].devStatus == OW_DEV_OK ){
					owToDev[i].devStatus = OW_DEV_ERR;
					owToDev[i].newErr = TRUE;
				}
			}
		}
	}
	// Сохраняем принятое по RX
	ow_presence = USART_ReceiveData(OW_USART);
	while ( (OW_USART->ISR & USART_ISR_RXNE) == USART_ISR_RXNE)
	{}

	USART_InitStructure.USART_BaudRate = 115200;
	USART_InitStructure.USART_WordLength = USART_WordLength_8b;
	USART_InitStructure.USART_StopBits = USART_StopBits_1;
	USART_InitStructure.USART_Parity = USART_Parity_No;
	USART_InitStructure.USART_HardwareFlowControl =	USART_HardwareFlowControl_None;
	USART_InitStructure.USART_Mode = USART_Mode_Tx | USART_Mode_Rx;
	USART_Init(OW_USART, &USART_InitStructure);
	USART_Cmd(OW_USART, ENABLE);

	if ( ow_presence != 0xf0 ) {
		return OW_OK;
	}

	return OW_DEV_ERR;
}

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

void ddReadDoor( void ){
		ddDev[0].ddData = (DD_1_PORT->IDR & DD_1_PIN) >> DD_1_PIN_NUM;
		ddDev[1].ddData = (DD_2_PORT->IDR & DD_2_PIN) >> DD_2_PIN_NUM;
}

