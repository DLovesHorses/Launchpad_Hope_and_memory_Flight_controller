#include "local_include/global.h"
#include "local_include/led.h"
#include "local_include/onBoardSwitch.h"
#include "local_include/sysclk.h"
#include "local_include/SysTick.h"
#include "local_include/SysFlag.h"
#include "local_include/uart.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

#ifdef DEBUG
void
__error__(char *pcFilename, uint32_t ui32Line)
{
    while(1);
}
#endif

//*****************************************************************************
//
// SystemInitialize :   Initialize all peripheral devices.
//
//*****************************************************************************
uint32_t resetCause;

void SystemInitialize(void)
{
    resetCause = SysCtlResetCauseGet();
    SysCtlResetCauseClear(resetCause);

    SysClk_Init();
    SysTick_Init();
    SysFlag_Init();
    LED_LEDInit();
    SWITCH_Init();
    //UART0_Init();
    UART0_STDIO_Init();

}

//*****************************************************************************
//
// main :   Main firmware logic.
//
//*****************************************************************************

int main(void)
{
    SystemInitialize();

    char UART0_strToRec[5];
    const char strToTr[5] = "Hello";
    // Loop forever.
    //
    while (1)
    {
        if (SysFlag_Check(SYSFLAG_SYS_TICK))
        {
            // systick is expired
            SysFlag_Clear(SYSFLAG_SYS_TICK);

            if (SysFlag_Check(SYSFLAG_UART0_RX))
            {
                SysFlag_Clear(SYSFLAG_UART0_RX);
                //UART0_charRec = UARTCharGetNonBlocking(UART0_BASE);
            }

            if (SysFlag_Check(SYSFLAG_UART0_TX))
            {
                SysFlag_Clear(SYSFLAG_UART0_TX);
                //UARTCharPutNonBlocking(UART0_BASE, UART0_charRec);
            }


            if (SWITCH_SW2_Pressed())
            {
                LED_LED2(ON);
                //UARTprintf("Enter string: ");
                //UARTgets(UART0_strToRec, 5);
                //UARTprintf("Entered String: %c", UART_strToRec[4]);
                UARTwrite(strToTr, 5);
            }
        }

        else
        {
            // systick is not expired
            //LED_ALL(OFF);

        }
    }
}

