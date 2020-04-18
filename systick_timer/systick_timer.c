/******************************************************************************
* Description : Contains functions to generate delays and timing base using Systick timer.
*
*	Author			: Rohan Patil
*
*	Version 		: 1.0
******************************************************************************/
#include "systick_timer.h"

/*  systick_count holds the value of systick interrupt counts, it is in use to 
    keep track of time for system */
static volatile uint32_t systick_count = 0;
/****************************************************************************/


/*
* returns value of systick_count
*/
uint32_t get_systick_count(void)
{

    return systick_count;
}


/*
*   init_systick() initialises the systick timer to generate interrupt repeatedly
*   on every 10ms. The reload value is taken from CALIB register which has 
*   factory programmed value in it for precise 10ms delays using cpu clock.
*/
void init_systick(void)
{
	//SystemCoreClockUpdate(); //Update SystemCoreClock variable to current clock speed

	    // disable timer if already running and clear countflag
	    SysTick->CTRL &= ~( (1 << 0) | (1 << 16));
	    // select core clock as clock
	    SysTick->CTRL |= (1 << 2);
	    // load current value as 0;
	    SysTick->VAL = 0UL;
	    // load reload value
	    //SysTick->LOAD = (uint32_t)((SystemCoreClock/MillisecondsIT) - 1UL);
	    SysTick->LOAD = (uint32_t)(12000 - 1UL);
	    /* Enable SysTick IRQ and SysTick Timer */
	    SysTick->CTRL  = SysTick_CTRL_CLKSOURCE_Msk |
	                     SysTick_CTRL_TICKINT_Msk |
	                     SysTick_CTRL_ENABLE_Msk;
	    return;
}


/*
*   returns time difference between two systick time_stamps in milliseconds.
*/
uint32_t get_elapsed_time(uint32_t time1, uint32_t time2)
{
    if(time2 >= time1)
    {
        return ((time2-time1) /* SYSTICK_INTERVAL_MS*/);
    }
    else
    {
        return (((0xffffffff - time1)+ time2) /* SYSTICK_INTERVAL_MS*/);
    }
}


/*
* SysTick_Handler() is isr for systick timer.
*/

#if 1
void SysTick_Handler(void)
{
    // disable irq to make section atomic
    __disable_irq();
    systick_count++;
    __enable_irq();
}
#endif
