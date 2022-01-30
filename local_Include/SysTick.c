/*
 * File         :   SysTick.c
 *
 * Description  :   Contains functions to configure SysTick register
 *                  and handles SysTick interrupts.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-23
 */
#include "local_include/SysFlag.h"
#include "local_include/SysTick.h"
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

void SYSTICK_Delay( uint32_t uiDelay )
{
#ifdef DEBUG
        UARTprintf("SYSTICK: Waiting for %d ms\n", uiDelay);
#endif
    while( uiDelay )
    {
        while( !SysFlag_Check( SYSFLAG_SYS_TICK ) );
        SysFlag_Clear( SYSFLAG_SYS_TICK );
        uiDelay--;
    }

    return;
}
