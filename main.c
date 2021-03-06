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
#include "local_include/BLUETOOTH/BLUETOOTH.h"
#include "local_include/PWM/PWM.h"
#include "local_include/PID/PID.h"
#include "local_include/SONAR/SONAR.h"
#include "local_include/MotorControl/MotorControl.h"
#include "local_include/PPM/PPM.h"

#include "utils/uartstdio.h"
#include "utils/uartstdio.c"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
extern uint8_t motorSelect;
extern uint8_t curSelectedESC;
extern MOTOR_DUTY_TRACKER dutyOf;

extern float base_altitude;

extern float kp_tune;
extern float ki_tune;

extern float MOTOR_DUTY_LIMIT_MULTIPLIER;

//int32_t w_time_stamp = 0;    // timestamp for bsx_lite

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
    BLUETOOTH_Init();
    I2C0_Init();

    PCF8574A_Init();
    // MPU9250_Init(); // don't use
    BMX160_Init();
    init_Buzzer();
    BMP388_Init();

    OrangeRX_Init();
    PWM_Init();
    PPM_Init();

    // SONAR_Init();

    // BUZZ_BUZZER(BUZZ_DUR_LONG, BUZZ_REP_TIME_2, BUZZ_PAUSE_TIME_500 );

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


            if(SysFlag_Check(SYSFLAG_BLUETOOTH_PRESSURE_SENSOR_CALIBRATE_CMD_START)){

                SysFlag_Clear(SYSFLAG_BLUETOOTH_PRESSURE_SENSOR_CALIBRATE_CMD_START);
                BMP388_calibrate();

                char charBuffer[80];
                sprintf(charBuffer, "Pressure Sensor Calibrated. Base Altitude: %f \n", base_altitude);
                BLUETOOTHprintf(charBuffer);
                charBuffer[0] = '\0';
            }


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
                    // calculate actual period.
                    // this is necessary in order to get the correct period if the
                    // timer value is reset before receiving second pulse and after
                    // receiveing first pulse.
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
            }

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
             *
             *
             * Operation to do if BMP388 Data-Ready int was sent.
             */

            if (SysFlag_Check(BMP388_DRDY_INT))
            {
                // clear the flag
                SysFlag_Clear(BMP388_DRDY_INT);
                BMP388_showData();
                UARTprintf("\n\n");
#ifdef DEBUG
                BUZZ_BUZZER(BUZZ_DUR_SHORT, BUZZ_REP_TIME_1,
                BUZZ_PAUSE_TIME_100);
#endif
            }

            static uint8_t ESC_PPM_PERIOD = 0;
            if (ESC_PPM_PERIOD == ESC_REFRESH_PERIOD_MS){

                ESC_PPM_PERIOD = 0;

                switch (curSelectedESC)
                {

                case ESC_1:
                {
                    PPM_ESC_HighLevelControl(ESC_1, dutyOf.motor_one_ppm);
                    break;
                }
                case ESC_2:
                {
                    PPM_ESC_HighLevelControl(ESC_2, dutyOf.motor_two_ppm);
                    break;
                }
                case ESC_3:
                {
                    PPM_ESC_HighLevelControl(ESC_3, dutyOf.motor_three_ppm);
                    break;
                }
                case ESC_4:
                {
                    PPM_ESC_HighLevelControl(ESC_4, dutyOf.motor_four_ppm);
                    break;
                }

                default:
                {

                    break;
                }

                }


            }
            ESC_PPM_PERIOD++;


            static uint16_t sensorDataSampleTimeCounter = 0;
            if (sensorDataSampleTimeCounter == SAMPLE_TIME_MS) // 1 seconds
            {
                //w_time_stamp +=  10;
                // BMX160_showData();
                // BMP388_showData();
                // OrangeRX_showStatus();
                // OrangeRX_showRawData();
                // OrangeRX_showActData();
                // GetEStopState();
                // GetFlightMode();

                //  Motor_ManMixer();
                // PID_altitude_adjust();
                //flightControl();
                //MOTOR_showDuty();
                BMX160_updateData(); // Enable this so that the Uuser can see the roll and pitch of the system even when the automatic controller is not running
                flightControl_SM();

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
                    MOTOR_DUTY_LIMIT_MULTIPLIER += 0.1;

                    char charBuffer[80];
                    sprintf(charBuffer, "Motor Duty Multiplier : %f\n", MOTOR_DUTY_LIMIT_MULTIPLIER);
                            UARTprintf("%s", charBuffer);
                    //kp_tune += 0.01;
                    break;
                }

                case '-':
                {

                    MOTOR_DUTY_LIMIT_MULTIPLIER -= 0.1;

                    char charBuffer[80];
                    sprintf(charBuffer, "Motor Duty Multiplier : %f\n", MOTOR_DUTY_LIMIT_MULTIPLIER);
                            UARTprintf("%s", charBuffer);
                    //kp_tune -= 0.01;
                    break;
                }

                case '*':
                {

                    ki_tune += 0.0001;
                    break;
                }

                case '/':
                {

                    ki_tune -= 0.0001;
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

                case 'c':
                {
                    BMP388_calibrate();
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

                case 's':
                {
                    // Display State of the devices

                    I2C_showState();
                    BMX160_showState();
                    BMP388_showState();
                    BUZZER_showState();
                    PWM_showState();
                    ORANGE_showState();

                    break;

                }

                case 'w':
                {

                    // wake the BMX160
                    // BMX160_wakeUp();
                    uint8_t reg = 0x44; //
                    uint8_t data = 0;
                    BMX160_readReg(reg, &data, 1);
                    UARTprintf("BMX160: Reg: 0x%x \t -> \t Data: 0x%x", reg,
                               data);
                    UARTprintf("\n");
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

                case 'P':
                {
                    PWM_Init();
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
                    UARTprintf("PB6 state: %d\n\n", pb6State);


#endif
                    break;
                }

                case 'T':
                {
                    static bool trigState = HIGH;   // First call : High
                    SONAR_Trigger(trigState, TRIG_TIMER_DIV_1_S);

                    UARTprintf("Trig State: %d \n", trigState);
                    //trigState ^= 0x01;
                    break;
                }

                case 'U':
                {
                    static bool trigState = HIGH;   // First call : High
                    SONAR_Trigger(trigState, TRIG_TIMER_DIV_100_US);

                    UARTprintf("Trig State: %d \n", trigState);
                    //trigState ^= 0x01;
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

                case '3':
                {
#ifdef DEBUG
                    static bool state = 0;
                    state ^= 0x01;
                    LED_LED2(state);
                    BUZZ_BUZZ(state);
                    break;
#endif
                }

                case '5':
                {
#ifdef DEBUG
                    //UARTCharPut(UART5_BASE, '5');
                    char buffer[200];
                    int a = 2;
                    int b = 3;
                    sprintf(buffer, "UART sending test %d + %d = %f", a, b,
                            (float) a + b);
                    BLUETOOTHprintf(buffer);
                    break;
#endif
                }

                case '6':
                {

#ifdef DEBUG

                    break;

#endif
                }

                case '7':
                {

#ifdef DEBUG
                    BMP388_calibrate();

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

            /*   // test individual degree of freedoms.
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

