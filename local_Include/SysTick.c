/*
 * File         :   SysTick.c
 *
 * Description  :   Contains functions to configure SysTick register
 *                  and handles SysTick interrupts.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-23
 */
#include "SysTick.h"
#include "LED.h"

// global variables and externs

// Function definitions.

void SysTick_IntHandler(void)
{
    SysFlag_Set(SYSFLAG_SYS_TICK);
}

void SysTick_Init(void)
{
    SysTickPeriodSet(SYSTICK_COUNTER_TIMER);
    SysTickIntRegister(SysTick_IntHandler);
    SysTickIntEnable();
    SysTickEnable();
}
