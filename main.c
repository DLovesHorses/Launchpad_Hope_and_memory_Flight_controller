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
#include "local_include/BUZZER/buzzer.h"
#include "local_include/BMX160/BMX160.h"
#include "local_include/BMP388/BMP388.h"
#include "local_include/OrangeRX/OrangeRX.h"
#include "local_include/PWM/PWM.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
extern uint8_t motorSelect;

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
    //I2C0_Init();

    // PCF8574A_Init();
    // MPU9250_Init(); // don't use
    // BMX160_Init();
    // init_Buzzer();
    //BMP388_Init();

    OrangeRX_Init();
    PWM_Init();

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

    UARTprintf("System Clock: %d Hz\n", SysCtlClockGet());
    // Loop forever.
    //
    while (1)
    {

        if (SysFlag_Check(SYSFLAG_SYS_TICK))
        {
            // systick is expired
            SysFlag_Clear(SYSFLAG_SYS_TICK);

            BUZZ_SM(BUZZ_SM_CALLED_FROM_MAIN); // don't remove. It is necessary for Buzzer state machine

            /*
             *
             *
             *
             *
             *
             *
             *
             *
             *
             * Operation to do if cPPM channel data received.
             */

            if (SysFlag_Check(Orange_RX_INT))
            {
                SysFlag_Clear(Orange_RX_INT);

                // Channel order:
                // <CH_1> - <CH_2> - <CH_3> - <CH_4> - <CH_5> - <CH_6> - <Frame_GAP> - Repeat

                // Each signal has 2 (+ve) going edge.
                // Therefore, one Frame will have:      2 * 7 / 2 = 7 (+ve edges)

                /*
                 * Signal                   :               (+ve) edge. no
                 *
                 *
                 * Frame_GAP                :               1 - 2
                 * CH_1                     :               2 - 3
                 * CH_2                     :               3 - 4
                 * CH_3                     :               4 - 5
                 * CH_4                     :               5 - 6
                 * CH_5                     :               6 - 7
                 * CH_6                     :               7 - 1
                 *
                 *
                 *
                 */

                // channel to store frequency value to.
                // NOTE: The first data will be invalid. Therefore, we need to wait until the second +ve edge is detected.
                static uint8_t channelSelect = 0;

                static uint32_t firstPulse_y = 0;
                static uint32_t secondPulse_y = 0;

                firstPulse_y = secondPulse_y;
                secondPulse_y = TimerValueGet(WTIMER5_BASE, TIMER_A);

                double firstPulse_x = (double) (1
                        - (double) firstPulse_y / (double) TM4C_CLK_RATE);
                double secondPulse_x = (double) (1
                        - (double) secondPulse_y / (double) TM4C_CLK_RATE);

                double period = secondPulse_x - firstPulse_x;

                if (firstPulse_x > secondPulse_x)
                {
                    period = (1.0f - firstPulse_x) + secondPulse_x;
                }

                double frequency = 1 / period;

                switch (channelSelect)
                {

                case FRAME_GAP_SELECT:
                {
                    rx_data.ch_period[0] = period;
                    rx_data.ch_freq[0] = frequency;
                    break;
                }

                case CH_1_SELECT:
                {
                    rx_data.ch_period[1] = period;
                    rx_data.ch_freq[1] = frequency;

                    break;
                }
                case CH_2_SELECT:
                {
                    rx_data.ch_period[2] = period;
                    rx_data.ch_freq[2] = frequency;
                    break;
                }
                case CH_3_SELECT:
                {
                    rx_data.ch_period[3] = period;
                    rx_data.ch_freq[3] = frequency;
                    break;
                }
                case CH_4_SELECT:
                {
                    rx_data.ch_period[4] = period;
                    rx_data.ch_freq[4] = frequency;
                    break;
                }
                case CH_5_SELECT:
                {
                    rx_data.ch_period[5] = period;
                    rx_data.ch_freq[5] = frequency;
                    break;
                }
                case CH_6_SELECT:
                {
                    rx_data.ch_period[6] = period;
                    rx_data.ch_freq[6] = frequency;
                    break;
                }

                default:
                {
#ifdef DEBUG
                    UARTprintf(
                            "Failed: Reading channel data. Invalid Channel to store value to.\n");
#endif
                    break;
                }
                }

                // increment channelSelect to prepare it for next call.
                channelSelect++;

                if (channelSelect == INVALID_CHANNEL)
                {

                    OrangeRX_receivedChannelReorganizer();
                    channelSelect = FRAME_GAP_SELECT; // Store content of next frame starting from FRAME_GAP for next round.

                }

                /*                                                      // <- Working! very good.



                 static uint32_t firstPulse_y = 0;
                 static uint32_t secondPulse_y  = 0;

                 firstPulse_y  = secondPulse_y ;
                 secondPulse_y  = TimerValueGet(WTIMER5_BASE, TIMER_A);

                 double firstPulse_x = (double) (1 - (double)firstPulse_y / (double) TM4C_CLK_RATE);
                 double secondPulse_x = (double) (1 - (double)secondPulse_y / (double) TM4C_CLK_RATE);


                 double period = secondPulse_x - firstPulse_x;

                 if(firstPulse_x > secondPulse_x){
                 period = (1.0f - firstPulse_x) + secondPulse_x;
                 }

                 double frequency = 1/period;

                 char cBuffer[100];
                 //sprintf(cBuffer, "first -> %6f    second -> %6f     \n period -> %10f      frequency -> %10f\n\n", firstPulse_x, secondPulse_x,  period, frequency);
                 sprintf(cBuffer, "Freq: %10f\n", frequency);
                 UARTprintf("%s", cBuffer);

                 */
                /*
                 static uint32_t count = 1;

                 static uint32_t firstPulse_y = 0;
                 static uint32_t secondPulse_y = 0;
                 static uint32_t thirdPulse_y = 0;

                 static double firstPulse_x = 0;
                 static double secondPulse_x = 0;
                 static double thirdPulse_x = 0;

                 switch (count)
                 {
                 case 1:
                 {
                 // save the first pulse value
                 firstPulse_y = TimerValueGet(WTIMER5_BASE, TIMER_A);
                 firstPulse_x = (double) (1
                 - (double) firstPulse_y / (double) TM4C_CLK_RATE);
                 count++;
                 break;
                 }
                 case 2:
                 {
                 // save the first pulse value
                 secondPulse_y = TimerValueGet(WTIMER5_BASE, TIMER_A);
                 secondPulse_x = (double) (1
                 - (double) secondPulse_y / (double) TM4C_CLK_RATE);
                 count++;
                 break;
                 }
                 case 3:
                 {
                 // save the first pulse value
                 thirdPulse_y = TimerValueGet(WTIMER5_BASE, TIMER_A);
                 thirdPulse_x = (double) (1
                 - (double) thirdPulse_y / (double) TM4C_CLK_RATE);
                 count++;
                 break;
                 }

                 case 4:
                 {
                 // display data

                 double period =
                 (thirdPulse_x - firstPulse_x) > 0 ?
                 (thirdPulse_x - firstPulse_x) :
                 (firstPulse_x - thirdPulse_x);
                 double duty =
                 (secondPulse_x - firstPulse_x) > 0 ?
                 (secondPulse_x - firstPulse_x) :
                 (firstPulse_x - secondPulse_x);
                 duty = duty / period;

                 char cBuffer[100];
                 sprintf(cBuffer,
                 "first -> %10f     second -> %10f     third -> %10f.  period -> %10f, duty -> %10.f \n\n",
                 firstPulse_x, secondPulse_x, thirdPulse_x, period,
                 duty);
                 UARTprintf("%s", cBuffer);
                 count = 1;
                 break;
                 }

                 default:
                 {
                 // do nothing
                 break;
                 }
                 }
                 */

                /*              static bool callState = 0;          // <- working, but bad
                 callState = callState ^ 0x01;

                 static uint32_t firstMeasure = 0;
                 static uint32_t secondMeasure = 0;
                 int32_t difference = 0;

                 if (callState == 1)
                 {
                 firstMeasure = TimerValueGet(WTIMER5_BASE, TIMER_A);
                 }
                 else
                 {

                 //firstMeasure = secondMeasure;
                 secondMeasure = TimerValueGet(WTIMER5_BASE, TIMER_A);

                 #ifdef DEBUG
                 //BUZZ_BUZZER(BUZZ_DUR_LONG, BUZZ_REP_TIME_4,
                 // BUZZ_PAUSE_TIME_100);

                 difference = (int32_t) firstMeasure
                 - (int32_t) secondMeasure;

                 double period = (double) difference / (double)TM4C_CLK_RATE;
                 double frequency = (double) 1/period;

                 char cBuffer[100];
                 //sprintf(cBuffer, "Time between edges:  %d  ->  %f Seconds \n\n", difference, us_timeScale);

                 sprintf(cBuffer,
                 "first -> %10d second -> %10d        difference -> %10d     period -> %10f    frequency -> %10f\n\n",
                 firstMeasure, secondMeasure, difference, period, frequency);
                 UARTprintf("%s", cBuffer);

                 #endif
                 }
                 LED_LED1(callState);



                 */
            }
            /*
             *
             *
             *
             *
             * Operation to do if BMP388 Data-Ready int was sent.
             */
            if (SysFlag_Check(BMP388_DRDY_INT))
            {
                // clear the flag
                SysFlag_Clear(BMP388_DRDY_INT);
                BMP388_showData();
#ifdef DEBUG
                BUZZ_BUZZER(BUZZ_DUR_SHORT, BUZZ_REP_TIME_1,
                BUZZ_PAUSE_TIME_100);
#endif
            }

            static uint16_t sensorDataSampleTimeCounter = 0;
            if (sensorDataSampleTimeCounter == 100) // 1 seconds
            {
                // BMX160_showData();
                // BMP388_showData();
                // OrangeRX_showRawData();
                // OrangeRX_showActData();

                // for debug only
                uint8_t data[8] = { 0 };
                uint8_t reg_addr = 0x11;
                uint8_t length = 1; // 0x11
                BMP388_get_regs(reg_addr, data, length);
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

                case '+':
                {
                    static uint8_t duty = 5;
                    Motor_setDuty(MOTOR_ONE, duty);     // PA6
                    Motor_setDuty(MOTOR_TWO, duty);     // PA7
                    Motor_setDuty(MOTOR_THREE, duty);   // PB6
                    Motor_setDuty(MOTOR_FOUR, duty);    // PB7

                    if (duty < 99)
                    {
                        duty++;
                    }

                    break;
                }

                case '-':
                {
                    static uint8_t duty = 95;
                    Motor_setDuty(MOTOR_ONE, duty);     // PA6
                    Motor_setDuty(MOTOR_TWO, duty);     // PA7
                    Motor_setDuty(MOTOR_THREE, duty);   // PB6
                    Motor_setDuty(MOTOR_FOUR, duty);    // PB7

                    if (duty != 3)
                    {
                        duty--;
                    }

                    break;
                }

                case 'a':
                {
                    OrangeRX_showActData();
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

                case 'p':
                {

                    motorSelect++;
                    if (motorSelect == (MOTOR_ALL + 1))
                    {
                        motorSelect = MOTOR_ONE; // wrap around;
                    }

                    UARTprintf("Motor selected \t: ");
                    switch (motorSelect)
                    {
                    case MOTOR_ONE:
                    {
                        UARTprintf("MOTOR_ONE \t \t -> \t Motor_1_Fine_Tune\n");
                        break;
                    }

                    case MOTOR_TWO:
                    {
                        UARTprintf("MOTOR_TWO \t \t -> \t Motor_2_Fine_Tune\n");
                        break;
                    }

                    case MOTOR_THREE:
                    {
                        UARTprintf(
                                "MOTOR_THREE \t \t -> \t Motor_3_Fine_Tune\n");
                        break;
                    }

                    case MOTOR_FOUR:
                    {
                        UARTprintf(
                                "MOTOR_FOUR  \t \t -> \t Motor_4_Fine_Tune\n");
                        break;
                    }

                    case MOTOR_ONE_THREE:
                    {
                        UARTprintf("MOTOR_ONE_THREE \t -> \t Forward \n");
                        break;
                    }

                    case MOTOR_TWO_FOUR:
                    {
                        UARTprintf("MOTOR_TWO_FOUR \t -> \t Backward\n");
                        break;
                    }

                    case MOTOR_ONE_TWO:
                    {
                        UARTprintf("MOTOR_ONE_TWO \t -> \t Move Left \n");
                        break;
                    }

                    case MOTOR_THREE_FOUR:
                    {
                        UARTprintf("MOTOR_THREE_FOUR \t -> \t Move Right  \n");
                        break;
                    }

                    case MOTOR_ONE_FOUR:
                    {
                        UARTprintf("MOTOR_ONE_FOUR \t -> \t Turn Left  \n");
                        break;
                    }

                    case MOTOR_TWO_THREE:
                    {
                        UARTprintf("MOTOR_TWO_THREE \t -> \t Turn Right  \n");
                        break;
                    }

                    case MOTOR_ALL:
                    {
                        UARTprintf("MOTOR_ALL \t \t -> \t Go up/down  \n");
                        break;
                    }

                    default:
                    {
                        motorSelect = MOTOR_ONE;
                        UARTprintf("MOTOR_ONE \n");
                    }
                    }

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

            if (SWITCH_SW1_Pressed())
            {
                //BUZZ_BUZZ(ON);
                BUZZ_BUZZER(BUZZ_DUR_MEDIUM, BUZZ_REP_TIME_3,
                BUZZ_PAUSE_TIME_1000);
            }

            if (SWITCH_SW2_Pressed())
            {
                //BUZZ_BUZZ(OFF);
                BUZZ_BUZZER(BUZZ_DUR_LONG_LONG, BUZZ_REP_TIME_5,
                BUZZ_PAUSE_TIME_500);
            }

          /*    // test individual degree of freedoms.
               // Press 'p' to change DOF to test.
            OrangeRX_extractData();
            uint8_t duty = (uint8_t) (rx_data.ch_nom_data[3]);
            Motor_setDuty(motorSelect, duty);
          */



        }
        else
        {
// systick is not expired
//LED_ALL(OFF);

        }
    }
}

