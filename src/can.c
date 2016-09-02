/*
 * can.c
 *
 *  Created on: 30 июля 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#include <main.h>
#include "stm32f10x_conf.h"
#include "my_time.h"
#include "buffer.h"

	uint32_t selfDevId;

void getIdList( tCanId *canid, uint32_t extId);

void canInit(void)
{
	CAN_InitTypeDef CAN_InitStruct;
	NVIC_InitTypeDef CAN_NVIC_InitStruct;

	RCC->APB1ENR |= RCC_APB1Periph_CAN1;

#define DEV_SIGNATURE			0x1FFFF7E8
	selfDevId = (*(uint32_t *)DEV_SIGNATURE) & 0xFFFFF;
	canBspInit();

	CAN_DeInit(CAN_CAN);
	CAN_InitStruct.CAN_Prescaler = 4;
	CAN_InitStruct.CAN_Mode = CAN_Mode_Normal;
	CAN_InitStruct.CAN_SJW = CAN_SJW_1tq;
	CAN_InitStruct.CAN_BS1 = CAN_BS1_2tq;
	CAN_InitStruct.CAN_BS2 = CAN_BS2_3tq;
	CAN_InitStruct.CAN_TTCM = DISABLE;
	CAN_InitStruct.CAN_ABOM = DISABLE;
	CAN_InitStruct.CAN_AWUM = DISABLE;
	CAN_InitStruct.CAN_NART = DISABLE;
	CAN_InitStruct.CAN_RFLM = DISABLE;
	CAN_InitStruct.CAN_TXFP = DISABLE;
	CAN_Init(CAN_CAN, &CAN_InitStruct);

	canFilterInit();

	CAN_ITConfig(CAN_CAN, CAN_IT_FMP0 | CAN_IT_FMP1 | CAN_IT_TME | CAN_IT_ERR | CAN_IT_BOF, ENABLE);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX0_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 5;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_RX1_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_TX_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);
	CAN_NVIC_InitStruct.NVIC_IRQChannel = CAN1_SCE_IRQn;
	CAN_NVIC_InitStruct.NVIC_IRQChannelPreemptionPriority = 6;
	CAN_NVIC_InitStruct.NVIC_IRQChannelSubPriority = 0;
	CAN_NVIC_InitStruct.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&CAN_NVIC_InitStruct);

	canBufferInit();
}

void canBspInit( void ){
	RCC->APB1ENR |= RCC_APB1ENR_CAN1EN;
	RCC->APB2ENR |= CAN_RCC_GPIOEN;
	RCC->APB2ENR |= RCC_APB2ENR_AFIOEN;

#if ( CAN_RX_PIN_NUM > 7)
	// CAN RX GPIO configuration
	CAN_RX_PORT->CRH &= ~((uint32_t)0xF << ((CAN_RX_PIN_NUM-8)*4));
	CAN_RX_PORT->CRH |= (uint32_t)0x8 << ((CAN_RX_PIN_NUM-8)*4);			// Input PullUp-PullDown

	// CAN TX GPIO configuration
	CAN_RX_PORT->CRH &= ~((uint32_t)0xF << ((CAN_TX_PIN_NUM-8)*4));
	CAN_RX_PORT->CRH |= (uint32_t)0xB << ((CAN_TX_PIN_NUM-8)*4);			// AF Output PullUp-PullDown 50MHz
#else
	// CAN RX GPIO configuration
	CAN_RX_PORT->CRL &= ~((uint32_t)0xF << ((CAN_RX_PIN_NUM)*4));
	CAN_RX_PORT->CRL |= (uint32_t)0x8 << ((CAN_RX_PIN_NUM)*4);			// Input PullUp-PullDown

	// CAN TX GPIO configuration
	CAN_RX_PORT->CRL &= ~((uint32_t)0xF << ((CAN_TX_PIN_NUM)*4));
	CAN_RX_PORT->CRL |= (uint32_t)0xB << ((CAN_TX_PIN_NUM)*4);			// AF Output PullUp-PullDown 50MHz

#endif

	AFIO->MAPR &= ~AFIO_MAPR_CAN_REMAP;
	AFIO->MAPR |= AFIO_MAPR_CAN_REMAP_REMAP2;

}

void canFilterInit( void ){
	tFilter filter;
	tCanId canId;
	uint8_t  filterNum;

// Формируем фильтр для приема пакетов от S207
	canId.adjCur = ADJ;
	canId.coldHot = r103Mesure.coldHot;
	canId.msgId = NULL_MES;
	canId.s207 = S207_DEV;
	canId.devId = selfDevId;
	// Фильтр принимаемых устройств
	filter.idList = setIdList( &canId );
#if CAN_TEST
// Для тестирования в колбцевом режиме - маска = 0x00000000
	filter.idMask = 0;
#else
	filter.idMask = S207_MASK;
#endif

	filter.ideList = 0;
	filter.ideMask = 0;
	filter.rtrList = 0;
	filter.rtrMask = 0;
	filterNum = 0;

	canFilterUpdate( &filter, filterNum );

// Формируем фильтр для приема пакетов от контроллера задвижки
		canId.adjCur = CUR;
		canId.msgId = VALVE_DEG,								// Угол поворота задвижки
		canId.s207 = nS207_DEV;
// TODO: Ввод полученного DevId от контроллера задвижки.
		canId.devId = VlvDevId;
		// Фильтр принимаемых устройств
		filter.idList = setIdList( &canId );
	#if CAN_TEST
	// Для тестирования в колбцевом режиме - маска = 0x00000000
		filter.idMask = 0;
	#else
		filter.idMask = CUR_ADJ_MASK | S207_MASK | COLD_HOT_MASK | MSG_ID_MASK | DEV_ID_MASK;
	#endif

		filter.ideList = 0;
		filter.ideMask = 0;
		filter.rtrList = 0;
		filter.rtrMask = 0;
		filterNum = 1;

		canFilterUpdate( &filter, filterNum );

}

void canFilterUpdate( tFilter * filter, uint8_t filterNum ) {
	CAN_FilterInitTypeDef CAN_FilterInitStruct;
	uint16_t stdId;
	uint32_t extId;

	stdId = (filter->idList & 0x7FF);
	extId = (filter->idList >> 11) & 0x3FFFF;
	CAN_FilterInitStruct.CAN_FilterIdHigh = (stdId << 5) | (extId >> 13);
	CAN_FilterInitStruct.CAN_FilterIdLow = (extId << 3) | (filter->ideList << 2) | (filter->rtrList << 1);
	stdId = (filter->idMask & 0x7FF);
	extId = (filter->idMask >> 11) & 0x3FFFF;
	CAN_FilterInitStruct.CAN_FilterMaskIdHigh = (stdId << 5) | (extId >> 13);
	CAN_FilterInitStruct.CAN_FilterMaskIdLow = (extId << 3) | (filter->ideMask << 2) | (filter->rtrMask << 1);

	CAN_FilterInitStruct.CAN_FilterFIFOAssignment = CAN_Filter_FIFO0;
	CAN_FilterInitStruct.CAN_FilterNumber = filterNum;
	CAN_FilterInitStruct.CAN_FilterMode = CAN_FilterMode_IdMask;
	CAN_FilterInitStruct.CAN_FilterScale = CAN_FilterScale_32bit;
	CAN_FilterInitStruct.CAN_FilterActivation = ENABLE;
	CAN_FilterInit(&CAN_FilterInitStruct);

}


void canRx0IrqHandler(void) {
	CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP0))
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP0);
		CAN_Receive(CAN1, CAN_FIFO0, (CanRxMsg *)&RxMessage);

		writeBuff( &canRxBuf, (uint8_t *)&RxMessage );
	}
}

void canRx1IrqHandler(void) {
	CanRxMsg RxMessage;

	if (CAN_GetITStatus(CAN1, CAN_IT_FMP1))
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_FMP1);
		CAN_Receive(CAN1, CAN_FIFO1, (CanRxMsg *)&RxMessage);

		writeBuff( &canRxBuf, (uint8_t *)&RxMessage );
	}
}

void canTxIrqHandler(void) {
	CanTxMsg TxMessage;

	if ((CAN_GetITStatus(CAN1, CAN_IT_TME)))
	{
		CAN_ClearITPendingBit(CAN1, CAN_IT_TME);
// TODO: Если есть сообщение для отправки - отправить его

		if (  readBuff( &canTxBuf, (uint8_t *)&TxMessage) ) {
			CAN_Transmit(CAN1, &TxMessage);
		}
	}
}

void canSceIrqHandler(void) {
	if (CAN_GetITStatus(CAN1, CAN_IT_BOF)){
		CAN_ClearITPendingBit(CAN1, CAN_IT_BOF);
		canInit();
	}
	if (CAN_GetITStatus(CAN1, CAN_IT_ERR)){
		CAN_ClearITPendingBit(CAN1, CAN_IT_ERR);
		canInit();
	}
}

void canProcess( void ){
	CanRxMsg rxMessage;
  /* Select one empty transmit mailbox */
  if( ((CAN1->TSR&CAN_TSR_TME0) == CAN_TSR_TME0) ||
  		((CAN1->TSR&CAN_TSR_TME1) == CAN_TSR_TME1) ||
			((CAN1->TSR&CAN_TSR_TME2) == CAN_TSR_TME2) ){

  	CanTxMsg txMessage;

//Читаем предназначенные для отправки сообщения, если они есть, и запихиваем его в буфер отправки.
  	if (  readBuff( &canTxBuf, (uint8_t *)&txMessage) ) {
			CAN_Transmit(CAN1, (CanTxMsg *)&txMessage);
		}
  	else {

  	}


  }

  if( readBuff( &canRxBuf, (uint8_t *)&rxMessage) ) {
  	tCanId canid;
  	getIdList( &canid, rxMessage.ExtId );
  	switch( canid.msgId ){
  		case VALVE_DEG:
  			r103Mesure.degCur = (uint8_t)*((uint32_t *)&rxMessage.Data);
  			break;
  		case TIME:
  			uxTime = *((uint32_t *)&rxMessage.Data);
  			xUtime2Tm( &sysDate, &sysTime, uxTime );
  			break;
  		case TO_IN_MSG:
  			r103Mesure.toAdj = *((int16_t *)&rxMessage.Data);
  			break;
  	}
  }

}

void canSendMsg( eMessId msgId, uint32_t data ) {
	CanTxMsg canTxMsg;
	tCanId canId;
	// Формируем структуру canId

	if( msgId == VALVE_DEG ){
		// Если отправляем новое полодение задвижки контроллеру задвижки
		canId.adjCur = ADJ;
// TODO: 	Идентификатор контроллера задвижки
		canId.devId = VlvDevId;
	}
	else {
		canId.adjCur = CUR;
		canId.devId = selfDevId;
	}

	canId.msgId = msgId;
	canId.coldHot = r103Mesure.coldHot;
	canId.s207 = nS207_DEV;

	if ( (msgId == TO_IN_MSG) || (msgId == TO_OUT_MSG) ) {
		// Для температуры - данные 16-и битные со знаком
		*((int16_t *)canTxMsg.Data) = *((int16_t *)&data);
	}
	else {
		// Для всех, кроме температуры, беззнаковое 32-х битное целое
		*((uint32_t *)canTxMsg.Data) = data;
	}

	canTxMsg.ExtId = setIdList( &canId );
	canTxMsg.IDE = CAN_Id_Extended;
	canTxMsg.RTR = 0;
	canTxMsg.StdId = 0;

	writeBuff( &canTxBuf, (uint8_t *)&canTxMsg );
}

uint32_t setIdList( tCanId *canid ){
 return 	( (((canid->adjCur)<<28) & CUR_ADJ_MASK)	|
		 	 	 	 	(((canid->msgId)<<22) & MSG_ID_MASK)		|
						(((canid->coldHot)<<21) & COLD_HOT_MASK)|
						(((canid->s207)<<20) & S207_MASK)				|
						((canid->devId) & DEV_ID_MASK) );
}

void getIdList( tCanId *canid, uint32_t extId){
	canid->adjCur = (extId & CUR_ADJ_MASK) >> 28;
	canid->msgId =  (extId & MSG_ID_MASK) >> 22;
	canid->coldHot = (extId & COLD_HOT_MASK) >> 21;
	canid->s207 = (extId & S207_MASK) >> 20;
	canid->devId = (extId & DEV_ID_MASK);
}
