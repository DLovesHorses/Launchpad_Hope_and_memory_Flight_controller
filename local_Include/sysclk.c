/*
 * File         :   sysclk.c
 *
 * Description  :   Contains function to configure System Clock.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-23
 */

// Includes

#include "sysclk.h"

// global variables and externs

// Function definitions
void SysClk_Init(void)
{
    //SysCtlClockSet(SYSCTL_SYSDIV_1 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN);

    // 80 MHz
    SysCtlClockSet(SYSCTL_SYSDIV_2_5 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN |
    SYSCTL_XTAL_16MHZ);

    //IntMasterEnable();
}
