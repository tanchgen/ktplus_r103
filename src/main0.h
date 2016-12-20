/*
 * main.h
 *
 *  Created on: 25 авг. 2016 г.
 *      Author: Gennady Tanchin <g.tanchin@yandex.ru>
 */

#ifndef MAIN_H_
#define MAIN_H_

#include <flow.h>
#include "can.h"
#include "onewire.h"

// ***************** Какому контуру принадлежит контроллер *******************

#define CIRCUIT						COLD

//****************************************************************************

enum _toStat {
	TO_DOWN,
	TO_STOP,
	TO_UP,
};

typedef struct {
//	uint8_t coldHot;
	int16_t toAdj;				// Требуемое значение температуры
	int16_t to[2];				// Действующее значение температуры Входной[0] и Входной[1]
	uint8_t degCur;						// Дайствующее положение задвижки
	uint8_t degAdj;						// Требуемуе полоджение задвижки
	uint32_t qMin;
	uint32_t qDay;
	uint32_t qWeek;
	uint32_t qMonth;
	uint32_t flowAdj;
	uint32_t flowSec;					// Текущий значение показаний расходомера
	uint32_t flowPrev;				// Предыдущее значение показаний расходомера
	uint32_t flowMin;					// Расход за последнюю секунду
	uint32_t flowDay;					// Расход за последний день
	uint32_t flowWeek;				// Расход за последнюю неделю
	uint32_t flowMonth;				// Расход за последний месяц
	uint32_t timeNow;
	uint32_t timePrev;
} tMesure;

typedef struct {
	enum _toStat toStat;			// Изменение вызодящей температуры: Растет, Падает, Не меняется
	uint8_t flowStat;					// Задвижка установлена в требуемое положение или нет...
} tR103Stat;

extern tMesure r103Mesure;
extern tR103Stat r103Stat;

extern uint32_t VlvDevId;

#endif /* MAIN_H_ */

