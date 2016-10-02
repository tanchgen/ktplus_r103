/*
 * my_time.h
 *
 *  Created on: 08 апр. 2016 г.
 *      Author: Jet <g.tanchin@yandex.ru>
 */

#ifndef MY_TIME_H_
#define MY_TIME_H_

#include <stdint.h>
#include "stm32f10x.h"
#include <time.h>


#define DELAY_RCC_TIM 					RCC_APB1Periph_TIM4
#define DELAY_TIM 							TIM4
#define DELAY_NVIC_IRQCHANNEL 	TIM4_IRQn
#define DELAY_NVIC_IRQHANDLER 	TIM4_IRQHandler

void DELAY_Init(void);
void DELAY_Count_mS(uint16_t mS);
void DELAY_Count_uS(uint16_t uS);
void DELAY_TimeOut_mS(uint16_t mS);
void DELAY_TimeOut_uS(uint16_t mS);
uint8_t DELAY_Check(void);

#define TIMER_FREQUENCY_HZ (1000u)
typedef uint32_t timer_ticks_t;


#if defined(USE_HAL_DRIVER)
void HAL_IncTick(void);
#endif

extern volatile timer_ticks_t timer_delayCount;
extern void timer_start (void);
extern void timer_sleep (timer_ticks_t ticks);

#define TIMEZONE_MSK			(+3)

	// DEF: standard signed format
	// UNDEF: non-standard unsigned format
	#define	_XT_SIGNED


#ifdef	_XT_SIGNED

//typedef	int32_t                           time_t;

#else
	typedef	uint32                          time_t;
#endif


typedef struct {
  uint8_t Hours;    /*!< Specifies the RTC Time Hour.
                        This parameter must be set to a value in the 0-12 range
                        if the HourFormat_12 is selected or 0-23 range if
                        the HourFormat_24 is selected. */
  uint8_t Minutes;  /*!< Specifies the RTC Time Minutes.
                        This parameter must be set to a value in the 0-59 range. */
  uint8_t Seconds;  /*!< Specifies the RTC Time Seconds.
                        This parameter must be set to a value in the 0-59 range. */
  uint8_t H12;      /*!< Specifies the RTC AM/PM Time.
                        This parameter can be a value of @ref AM_PM_Definitions */
} tTime;

typedef struct {
  uint8_t WeekDay; /*!< Specifies the RTC Date WeekDay.
                        This parameter can be a value of @ref WeekDay_Definitions */
  uint8_t Month;   /*!< Specifies the RTC Date Month (in BCD format).
                        This parameter can be a value of @ref Month_Date_Definitions */
  uint8_t Date;     /*!< Specifies the RTC Date.
                        This parameter must be set to a value in the 1-31 range. */
  uint8_t Year;     /*!< Specifies the RTC Date Year.
                        This parameter must be set to a value in the 0-99 range. */
} tDate;


extern volatile time_t uxTime;

extern tDate sysDate;
extern tTime sysTime;


extern volatile uint32_t myTick;
extern volatile uint32_t usDelFlag;

extern uint32_t toReadCount;

// *********** Инициализация структуры ВРЕМЯ (сейчас - системное ) ************
void timeInit( void );
// Получение системного мремени
uint32_t getTick( void );
void xUtime2Tm( tDate * mdate, tTime *mtime, time_t secsarg);
time_t xTm2Utime( tDate *mdate, tTime *mtime );
void setRtcTime( time_t xtime );
time_t getRtcTime( void );
void timersProcess( void );
void timeToStr( time_t ut, uint8_t *str );

void timersHandler( void );

void delayUsInit( void );
void myDelay( uint32_t del );
void usDelay( uint32_t usDel );

#endif /* UNIX_TIME_H_ */
