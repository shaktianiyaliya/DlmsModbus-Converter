/******************************************************************************
* Description : Contains functions to generate delays and timing base using systick timer.
*
*	Author			: Rohan Patil
*
*	Version 		: 1.0
******************************************************************************/
#ifndef  SYSTICK_H
#define SYSTICK_H

#include "LPC17xx.h"


#define MillisecondsIT 1000
#define SYSTICK_INTERVAL_MS 10

void init_systick(void);
uint32_t get_systick_count(void);
uint32_t get_elapsed_time(uint32_t time1, uint32_t time2);

#endif

