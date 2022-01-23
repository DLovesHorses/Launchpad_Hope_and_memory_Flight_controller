#include "SysTick.h"
#include "LED.h"

void SysTick_IntHandler(void){
    SysFlag_Set(SYSFLAG_SYS_TICK);
}



void SysTick_Init(void)
{
    SysTickPeriodSet(SYSTICK_COUNTER_TIMER);
    SysTickIntRegister(SysTick_IntHandler);
    SysTickIntEnable();
    SysTickEnable();
}
