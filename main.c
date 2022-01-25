#include "local_include/global.h"
#include "local_include/led.h"
#include "local_include/onBoardSwitch.h"
#include "local_include/sysclk.h"
#include "local_include/SysTick.h"
#include "local_include/SysFlag.h"
#include "local_include/uart.h"
#include "local_include/i2c.h"

//external sensors
#include "local_include/MPU9250.h"

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
    I2C0_Init();
    MPU9250_Init();
    PCF8574A_Init();

}

//*****************************************************************************
//
// main :   Main firmware logic.
//
//*****************************************************************************

// Todo: Enable interrupts on GPIO PF0 (SW2) and PF4 (SW1);

int main(void)
{
    SystemInitialize();
    unsigned char charRec = '\0';
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
                charRec = UARTgetc();
                //UARTprintf("%c", charRec);

                switch (charRec)
                {

                case 'A':
                {
                    MPU9250_Write(GYRO_CONFIG, 0x18);

                    break;
                }

                case 'G':
                {
                    MPU9250_Write(GYRO_CONFIG, 0x18);

                    break;
                }

                case 'R':
                {
                    uint32_t receivedData;
                    PCF8574A_Read( PCF8574A_SA, &receivedData);

                    bool pb4State = receivedData & PCF8574A_PB4;
                    bool pb5State = receivedData & PCF8574A_PB5;
                    bool pb6State = receivedData & PCF8574A_PB6;
                    UARTprintf("\nPB4 state: %d ", pb4State);
                    UARTprintf("PB5 state: %d ", pb5State);
                    UARTprintf("PB6 state: %d", pb6State);
                    break;
                }
                case '1':
                {

                    PCF8574A_Write( PCF8574A_SA, 0xA7);
                    UARTprintf("PCF8575A written with %X", 0xA7);
                    break;
                }

                case '2':
                {
                    PCF8574A_Write( PCF8574A_SA, 0x47);
                    UARTprintf("PCF8575A written with %X", 0x47);
                    break;
                }

                case '\x03': // CTRL + C
                {
                    uint8_t i = 0;
                    for (i = 0; i < 100; i++)
                    {
                        UARTprintf("\r\n");
                    }
                    break;
                }

                case '\r':
                {
                    UARTprintf("\r\n");

                    break;
                }

                default:
                {

                }
                }

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

