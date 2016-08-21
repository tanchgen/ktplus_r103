//
// This file is part of the GNU ARM Eclipse distribution.
// Copyright (c) 2014 Liviu Ionescu.
//

// ----------------------------------------------------------------------------

#include <stdio.h>
#include <stdlib.h>
#include "stm32f10x.h"
#include "diag/Trace.h"

#include "Timer.h"

RCC_ClocksTypeDef RCC_Clocks;


// ----- main() ---------------------------------------------------------------

int
main(int argc, char* argv[])
{
	myTick = 0;
	usTick = 0;

  timer_start();

  RCC_GetClocksFreq(&RCC_Clocks);


  // Infinite loop
  while (1)
    {


    }
  // Infinite loop, never return.
}

// ----------------------------------------------------------------------------
