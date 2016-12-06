/*
 *
 * NMEA library
 * URL: http://nmea.sourceforge.net
 * Author: Tim (xtimor@gmail.com)
 * Licence: http://www.gnu.org/licenses/lgpl.html
 * $Id: time.c 4 2007-08-27 13:11:03Z xtimor $
 *
 * unix_time.c
 *      Author: Jet <g.tanchin@yandex.ru>
 *  Created on: 08 апр. 2016 г.
 */

#include <main.h>
#include "stm32f10x.h"
#include "my_time.h"
#include "stm32f10x_it.h"
#include "flow.h"
//#include "logger.h"
//#include "thermo.h"

volatile timer_ticks_t timer_delayCount;
volatile time_t uxTime;
volatile uint32_t myTick;
volatile uint32_t usDelFlag;

tDate sysDate;
tTime sysTime;

static uint8_t  secondFlag;

volatile uint8_t minuteFlag = RESET;
volatile uint8_t hourFlag = RESET;
volatile uint8_t	dayFlag = RESET;
volatile uint8_t	weekFlag = RESET;
volatile uint8_t	monthFlag = RESET;

extern RCC_ClocksTypeDef RCC_Clocks;

uint32_t toReadCount;

/*
// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void timeInit( void ) {
	RTC_InitTypeDef rtcInitStruct;
  RTC_DateTypeDef  sdatestructure;
  RTC_TimeTypeDef  stimestructure;

  RTC_StructInit( &rtcInitStruct );
  RTC_Init( &rtcInitStruct );
  //##-1- Configure the Date #################################################
  // Set Date: Wednesday June 1st 2016
  sdatestructure.RTC_Year = 16;
  sdatestructure.RTC_Month = RTC_Month_June;
  sdatestructure.RTC_Date = 1;
  sdatestructure.RTC_WeekDay = RTC_Weekday_Wednesday;

  if(RTC_SetDate( RTC_Format_BIN ,&sdatestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

  stimestructure.RTC_Hours = 0;
  stimestructure.RTC_Minutes = 0;
  stimestructure.RTC_Seconds = 0;

  if(RTC_SetTime( rtcInitStruct.RTC_HourFormat ,&stimestructure ) != SUCCESS)
  {
    // Initialization Error
    genericError( GEN_ERR_HW );
  }

}
*/

// Получение системного мремени
uint32_t getTick( void ) {
	// Возвращает количество тиков
	return myTick;
}

uint32_t sys_now( void ){
	return myTick;
}
#define _TBIAS_DAYS		((70 * (uint32_t)365) + 17)
#define _TBIAS_SECS		(_TBIAS_DAYS * (uint32_t)86400)
#define	_TBIAS_YEAR		0
#define MONTAB(year)		((((year) & 03) || ((year) == 0)) ? mos : lmos)

const int16_t	lmos[] = {0, 31, 60, 91, 121, 152, 182, 213, 244, 274, 305, 335};
const int16_t	mos[] = {0, 31, 59, 90, 120, 151, 181, 212, 243, 273, 304, 334};

#define	Daysto32(year, mon)	(((year - 1) / 4) + MONTAB(year)[mon])

/////////////////////////////////////////////////////////////////////

time_t xTm2Utime( tDate *mdate, tTime *mtime ){
	/* convert time structure to scalar time */
int32_t		days;
int32_t		secs;
int32_t		mon, year;

	/* Calculate number of days. */
	mon = mdate->Month - 1;
	year = mdate->Year - _TBIAS_YEAR;
	days  = Daysto32(year, mon) - 1;
	days += 365 * year;
	days += mdate->Date;
	days -= _TBIAS_DAYS;

	/* Calculate number of seconds. */
	secs  = 3600 * mtime->Hours;
	secs += 60 * mtime->Minutes;
	secs += mtime->Seconds;

	secs += (days * (time_t)86400);

	return (secs);
}

/////////////////////////////////////////////////////////////////////

void xUtime2Tm( tDate * mdate, tTime *mtime, time_t secsarg){
	uint32_t		secs;
	int32_t		days;
	int32_t		mon;
	int32_t		year;
	int32_t		i;
	const int16_t *	pm;

	#ifdef	_XT_SIGNED
	if (secsarg >= 0) {
			secs = (uint32_t)secsarg;
			days = _TBIAS_DAYS;
		} else {
			secs = (uint32_t)secsarg + _TBIAS_SECS;
			days = 0;
		}
	#else
		secs = secsarg;
		days = _TBIAS_DAYS;
	#endif

		/* days, hour, min, sec */
	days += secs / 86400;
	secs = secs % 86400;
	mtime->Hours = secs / 3600;
	secs %= 3600;
	mtime->Minutes = secs / 60;
	mtime->Seconds = secs % 60;

	mdate->WeekDay = (days + 1) % 7;

	/* determine year */
	for (year = days / 365; days < (i = Daysto32(year, 0) + 365*year); ) { --year; }
	days -= i;
	mdate->Year = year + _TBIAS_YEAR;

		/* determine month */
	pm = MONTAB(year);
	for (mon = 12; days < pm[--mon]; );
	mdate->Month = mon + 1;
	mdate->Date = days - pm[mon] + 1;
}

void timersHandler( void ) {

	// Decrement to zero the counter used by the delay routine.
  if (timer_delayCount != 0u) {
    --timer_delayCount;
  }

/*	// Таймаут для логгирования температуры
	if ( toLogCount > 1) {
		toLogCount--;
	}=
*/
	// Таймаут для считывания температуры
	if ( toReadCount > 1) {
		toReadCount--;
	}

	// Секундный таймер
	if ( !(myTick % 1000) ) {
		secondFlag = SET;
		uxTime++;
		xUtime2Tm( &sysDate, & sysTime, uxTime );
		// Выставляем флаги минут, часов, дней, недель и месяцев
		if( sysTime.Seconds == 0 ){
			minuteFlag = SET;
			if( sysTime.Minutes == 0){
				hourFlag = SET;
				if( sysTime.Hours == 0){
					dayFlag = SET;
					if( sysDate.WeekDay == 1){
						weekFlag = SET;
					}
					if( sysDate.Date == 1){
						monthFlag = SET;
					}
				}
			}
		}

		// Запоминаем счетчик потока, чтоб было точно
//		r103Mesure.flowNow = flowCount * KFLOW;
	}


}

void timersProcess( void ) {
/*
	// Таймаут для логгирования температуры
	if ( toLogCount == 1 ) {
		toLogCount = toLogTout+1;
		toLogWrite();
	}
*/

	// Секундный таймер
	if ( secondFlag ) {
		secondFlag = RESET;
		// Симуляция водомера
//		flowGetVolume();
		flowSecondProcess();
	}

	// Таймаут для считывания температуры
	if ( toReadCount == 1 ) {
		int16_t tmpTo;
		toReadCount += TO_READ_TOUT;
		toReadTemperature( TO_IN );
		// Симуляция измерения температуры
		//r103Mesure.to[TO_IN] = 50*16;
		tmpTo = r103Mesure.to[TO_OUT];
		// Датчика выходящей температуры нет, поэтому имитируем показания
		toReadTemperature( TO_OUT );
		// Симуляция измерения температуры
		//r103Mesure.to[TO_OUT] = 40*16;
		if( (tmpTo == 0) || (tmpTo == 0x7FF) ){
			tmpTo = r103Mesure.to[TO_OUT];
		}
		if( tmpTo < (r103Mesure.to[TO_OUT] - 4) ){
			r103Stat.toStat = TO_UP;
		}
		else if( tmpTo > (r103Mesure.to[TO_OUT] + 4)){
			r103Stat.toStat = TO_DOWN;
		}
		else {
				r103Stat.toStat = TO_STOP;
		}
	}
}

// Инициализация таймера микросекундных задержек
void delayUsInit( void ) {

	RCC->APB1ENR |= RCC_APB1ENR_TIM4EN;

	TIM_DeInit(DELAY_TIM);
	DELAY_TIM->CR1 &= ~TIM_CR1_CEN;
	// Выставляем счетчик на 0,5 мкс
	DELAY_TIM->PSC = (RCC_Clocks.PCLK2_Frequency/2000000) - 1;
	DELAY_TIM->ARR = 0xFFFF;
	DELAY_TIM->CR1 &= ~TIM_CR1_CKD;
	DELAY_TIM->CR1 &= ~TIM_CR1_DIR;		// Считаем на возрастание

/*
	TIM_ITConfig(DELAY_TIM, TIM_IT_Update, ENABLE);
	DELAY_NVIC_InitStructure.NVIC_IRQChannel = DELAY_NVIC_IRQCHANNEL;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
	DELAY_NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
	NVIC_Init(&DELAY_NVIC_InitStructure);
*/
}

// Задержка в 0,5 мкс

void usDelay( uint32_t usDel ){

	DELAY_TIM->ARR = usDel-1;
	DELAY_TIM->CNT = 0;
	DELAY_TIM->CR1 |= TIM_CR1_CEN;

	while( !(DELAY_TIM->SR & TIM_SR_UIF) )
	{}
	DELAY_TIM->CR1 &= ~TIM_CR1_CEN;
	DELAY_TIM->SR &= ~TIM_SR_UIF;
}

// Задержка в мс
void myDelay( uint32_t del ){
	uint32_t finish = myTick + del;
	while ( myTick < finish)
	{}
}

