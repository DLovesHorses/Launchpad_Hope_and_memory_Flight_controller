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
    char strToTr[50] = "Hello, How are you?";
    unsigned char charRec = '\0';
    UARTprintf("Hello, I am D, and I made this program!");
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
                charRec = HWREG(UART0_BASE + UART_O_DR);
                UARTprintf("%c", charRec);

                if (charRec == '\r')
                {
                    UARTprintf("\n");
                }

                if (charRec == 'H')
                {
                    //UARTprintf("Secret String: %s", strToTr);
                    UARTprintf("Hello, I am D, and I made this program! %d", 39485);

                    //UARTwrite("Hello, I am D, and I made this program!", 40);
                    char charBuffer[800];
                    /*sprintf(charBuffer,
                            "Hello, I am D, and I made this program!\r\nThis is what a dream feels like.\r\nRandom number: %d\r\n", 234231);
                    int32_t count = 0;
                    while (charBuffer[count] != NULL)
                    {
                        UARTprintf("%c", charBuffer[count]);
                        count++;
                    }
                    */
                    //UARTFlushTx(false);
                }
                //UARTFlushTx(true);
                //UARTFlushRx();
            }

            if (SysFlag_Check(SYSFLAG_UART0_TX))
            {
                SysFlag_Clear(SYSFLAG_UART0_TX);
            }

            if (SWITCH_SW2_Pressed())
            {

            }

        }

        else
        {
            // systick is not expired
            //LED_ALL(OFF);

        }
    }
}

