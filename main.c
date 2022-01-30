/*
 * File         :   main.c
 *
 * Purpose      :   Glues everything together
 *
 * Description  :   Starting point of firmware.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-22
 */

// Includes
#include "local_include/global.h"

#include "local_include/SysFlag.h"
#include "local_include/led.h"
#include "local_include/onBoardSwitch.h"
#include "local_include/sysclk.h"
#include "local_include/SysTick.h"
#include "local_include/SysFlag.h"
#include "local_include/uart.h"
#include "local_include/i2c.h"

//#include "local_include/MPU9250.h"
#include "local_include/BMX160.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

// global variables and externs

#ifdef DEBUG
void __error__(char *pcFilename, uint32_t ui32Line)
{
    UARTprintf(
            "Internal code:      File:   %s      , Line:     %d      , status:   ERROR   ",
            pcFilename, ui32Line);
    while (1)
        ;
}
#endif

// Function definitions.

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

    PCF8574A_Init();
    //MPU9250_Init();
    BMX160_Init();

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

            static uint16_t sensorDataSampleTimeCounter = 0;
            if (sensorDataSampleTimeCounter == 332) // 2 seconds
            {
                BMX160_showData();
                sensorDataSampleTimeCounter = 0;
            }

            sensorDataSampleTimeCounter++;

            if (SysFlag_Check(SYSFLAG_UART0_RX))
            {
                SysFlag_Clear(SYSFLAG_UART0_RX);
                charRec = UARTgetc();
                //UARTprintf("%c", charRec);

                switch (charRec)
                {

                case 'a':
                {

                    break;
                }

                case 'b':
                {
                    uint8_t testReg = 0x00;
                    uint8_t knownValue = 0xD8;
                    I2C_AddressBruteForcer(testReg, knownValue);
                    break;
                }
                case 'm':
                {
#ifdef DEBUG
                    static int reg = 0;
                    uint8_t gyroData[1];
                    int count;
                    for (count = reg; count < reg + 10; count++)
                    {
                        SysCtlDelay(100000);
                        //MPU9250_MultiByteRead(count, 1, gyroData);
                        //I2C_ReadByte(MPU9250_SA, count, gyroData);
                        UARTprintf("Reg: %d (0x%X), \t value: 0x%X \n", count,
                                   count, gyroData[0]);
                    }

                    reg = reg + 10;
#endif
                    break;
                }

                case 'w':
                {

                    // wake the BMX160
                    BMX160_wakeUp();
#ifdef DEBUG

                    /*
                     uint8_t dataToRec;

                     UARTprintf("\n MPU9250: WHO_AM_I command sending...\n");
                     I2C_ReadByte(MPU9250_SA, MPU9250_WHO_AM_I, &dataToRec);

                     UARTprintf("MPU9250: WHO_AM_I command response: %X\n",
                     dataToRec);

                     UARTprintf("\n AK8963: WHO_AM_I command sending...\n");
                     I2C_ReadByte(AK8963_SA, AK8963_WIA, &dataToRec);

                     UARTprintf("AK_8963: WHO_AM_I command response: %X\n",
                     dataToRec);

                     UARTprintf("\n BMP180: WHO_AM_I command sending...\n");
                     I2C_ReadByte(BMP280_SA, BMP280_CHIP_ID, &dataToRec);

                     UARTprintf("BMP180: WHO_AM_I command response: %X\n",
                     dataToRec);
                     */
#endif
                    break;

                }
                case 'A':
                {

                    break;
                }

                case 'G':
                {

                    break;
                }

                case 'R':
                {
#ifdef DEBUG
                    UARTprintf("Reading state of PCF switches/n/n");
                    uint32_t receivedData;
                    PCF8574A_Read( PCF8574A_SA, &receivedData);

                    bool pb4State = receivedData & PCF8574A_PB4;
                    bool pb5State = receivedData & PCF8574A_PB5;
                    bool pb6State = receivedData & PCF8574A_PB6;
                    UARTprintf("\nPB4 state: %d ", pb4State);
                    UARTprintf("PB5 state: %d ", pb5State);
                    UARTprintf("PB6 state: %d/n/n", pb6State);

                    UARTprintf("Reading all registers of MPU9250/n/n");
                    uint8_t gyroData[1];
                    int count;
                    for (count = 0; count < 256; count++)
                    {
                        //MPU9250_MultiByteRead(count, 1, gyroData);
                        //I2C_ReadByte(MPU9250_SA, count, gyroData);
                        UARTprintf("Reg: %d (0x%X), \t value: 0x%X \n", count,
                                   count, gyroData[0]);
                        SysCtlDelay(10000);
                    }
#endif
                    break;
                }
                case '1':
                {
#ifdef DEBUG
                    PCF8574A_Write( PCF8574A_SA, 0xA7);
                    UARTprintf("PCF8575A written with %X\n", 0xA7);
                    break;
#endif
                }

                case '2':
                {
#ifdef DEBUG
                    PCF8574A_Write( PCF8574A_SA, 0x47);
                    UARTprintf("PCF8575A written with %X", 0x47);
                    break;
#endif
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

            static bool showData = true;
            if (SWITCH_SW2_Pressed())
            {
#ifdef DEBUG

                if (showData == true)
                {
                    BMX160_showData();
                    showData = false;
                }

#endif
            }

            else
            {
                // if Switch 2 is not pressed

                showData = true;
            }

        }
        else
        {
// systick is not expired
//LED_ALL(OFF);

        }
    }
}

