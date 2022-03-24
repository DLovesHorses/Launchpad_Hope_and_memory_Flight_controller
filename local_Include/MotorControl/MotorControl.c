/*
 * File         :   MotorControl.c
 *
 * Description  :   Main flight control logic.
 *                  Contains logic for E-Stop and Flight Controller mode selection.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-19
 */

// Includes
#include "MotorControl.h"
#include "local_Include/led.h"
#include "local_Include/BUZZER/buzzer.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"

#include "local_Include/PWM/PWM.h"
#include "local_Include/OrangeRX/OrangeRX.h"
#include "local_Include/PID/PID.h"
#include "local_Include/BMP388/BMP388.h"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;
extern MOTOR_VARIABLES mixer;
extern bool BMP388_calibState;

// Function definitions.
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
 *
 */
void flightControl(void)
{
    bool eStopState = GetEStopState();
    bool rxIsConnected = OrangeRX_isConnected();

    if (eStopState == ENGAGED || rxIsConnected == NOT_CONNECTED)
    {
        // show RED light (onboard).
        LED_Drive(LED_ALL, OFF);
        LED_Drive(RED, ON);

        //LED_ALL(OFF);
        //LED_LED1(ON);

        uint8_t count;
        for (count = 0; count < 4; count++)
        {
            //check if the current duty is already 0
            if (mixer.final_duty[count] == 0)
            {
                // drone is landed.
            }
            else
            {
                // drone is in air. Decrement by a predefined step.
                mixer.final_duty[count] -= E_STOP_DECREMENT_STEP;
            }

            // send the duty to pwm controller
            Motor_setDuty(count, mixer.final_duty[count]);
        }

#ifdef DEBUG
        UARTprintf("E-Stop:\t  ON \n\n");
#endif
    }

    else
    {
        // e-stop is not engaged.

        //Select flight controller.
        uint8_t flightMode = GetFlightMode();

        if (flightMode == CALIBRATION)
        {

            // turn on Blue Light
            LED_Drive(LED_ALL, OFF);
            LED_Drive(BLUE, ON);
            //LED_ALL(OFF);
            //LED_LED2(ON);

            // call calibration unit
            BMP388_calibrate();
        }

        if (flightMode == MANUAL)
        {

            // turn on GREEN Light
            LED_Drive(LED_ALL, OFF);
            LED_Drive(GREEN, ON);
            //LED_ALL(OFF);
            //LED_LED3(ON);

            // call Manual Mixer.
            Motor_ManMixer();
        }

        if (flightMode == AUTO)
        {

            // turn on BLUE+GREEN Light
            LED_Drive(LED_ALL, OFF);
            LED_Drive(BLUE | GREEN, ON);
            //LED_ALL(OFF);
            //LED_LED2(ON);
            //LED_LED3(ON);

            // call Auto Mixer.
            //PID_altitude_adjust();
        }

#ifdef DEBUG
        UARTprintf("E-Stop:\t  OFF \n");
        if (flightMode == CALIBRATION)
        {
            UARTprintf("Mode :\t  Calibration. \n");
        }
        if (flightMode == MANUAL)
        {
            UARTprintf("Mode :\t  Manual. \n");
        }
        if (flightMode == AUTO)
        {
            UARTprintf("Mode :\t  Automatic. \n");
        }

#endif

    }

    return;
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
 */

uint8_t GetFlightMode(void)
{

    uint8_t flightMode = CALIBRATION;
    bool eStopState = GetEStopState();

    if (rx_data.dataOfCh[MODE_SELECTOR] < 10 || eStopState == ENGAGED)
    {
        flightMode = CALIBRATION;
    }

    else if (rx_data.dataOfCh[MODE_SELECTOR] > 30
            && rx_data.dataOfCh[MODE_SELECTOR] < 60)
    {
        flightMode = MANUAL;
    }

    else if (rx_data.dataOfCh[MODE_SELECTOR] > 70)
    {
        flightMode = AUTO;
    }

    else
    {
        // just for safety.
        flightMode = CALIBRATION;
    }

#ifdef DEBUG_D

    if (flightMode == MANUAL)
    {
        UARTprintf("Flight Mode : \t Manual Controller \n");
    }
    else if (flightMode == AUTO)
    {
        UARTprintf("Flight Mode : \t Automatic Controller \n");
    }
    else
    {
        UARTprintf("Flight Mode : \t Calibrating. \n");
    }

#endif

    return flightMode;
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
 */

bool GetEStopState(void)
{
    bool eStopState = NOT_ENGAGED;
    bool rxIsConnected = OrangeRX_isConnected();
    if (rx_data.dataOfCh[E_STOP] > 50 || rxIsConnected == NOT_CONNECTED)
    {
        eStopState = ENGAGED;

    }

#ifdef DEBUG_D
    if (rxIsConnected == NOT_CONNECTED) { UARTprintf("RX state : \t Not Connected \n"); }
    if (eStopState == ENGAGED) { UARTprintf("EStop State : \t ON \n"); }
    if (eStopState == NOT_ENGAGED) { UARTprintf("EStop State : \t OFF \n"); }

#endif

    return eStopState;

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
 */

void flightControl_SM(void)
{
    static uint8_t curState = IDLE;
    /*
     if(OrangeRX_isConnected() == NOT_CONNECTED){
     // bypass the state machine. Turn Solid Red.
     LED_Drive(RED, ON);
     return;
     }
     */
    if (GetEStopState() == ENGAGED)
    {
        curState = IDLE;
        BMP388_calibState = BMP388_NOT_CALIBRATED;

    }

    if (OrangeRX_isConnected() == NOT_CONNECTED)
    {
        // bypass the state machine. Turn Solid Red.

        curState = RX_NOT_CONNECTED;
        BMP388_calibState = BMP388_NOT_CALIBRATED;
        //LED_Drive(RED, ON);
    }

    switch (curState)
    {

    case RX_NOT_CONNECTED:
    {

        static unsigned int timer = 0;
        if (timer == 0)
        {
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            LED_Drive(RED, ON);
            timer++;
        }

        if (GetEStopState() == ENGAGED)
        {
            curState = IDLE;
            timer = 0;
        }

        // RX Connection Lost motor duty change logic.
        flightSafeLand();

        break;
    }
    case IDLE:
    {
        static unsigned int timer = 0;
        static bool ledState = 0;

        // TURN off all LEDs if transition from other state
        if (timer == 0)
        {
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
        }
        if (timer == IDLE_STATE_LED_LEVEL_STABLE_TIME)
        {
            ledState ^= 0x01;
            LED_Drive(RED, ledState);
            BUZZ_BUZZ(ledState);

            timer = 1;
        }
        else
        {
            timer++;
        }

        // transition logic
        uint8_t eStopState = GetEStopState();
        uint8_t rxState = OrangeRX_isConnected();
        uint8_t flightMode = GetFlightMode();
        if (eStopState == NOT_ENGAGED && rxState == CONNECTED
                && flightMode == CALIBRATION)
        {
            curState = CALIBRATION;
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            timer = 0;
            ledState = 0;
        }
        else
        {
            curState = IDLE;
        }

        // E-stop logic
        flightSafeLand();

#ifdef DEBUG_D
       UARTprintf("E-Stop:\t  ON \n\n");
#endif

        break;
    }

    case CALIBRATION:
    {
        static unsigned int timer = 0;
        static bool ledState = 0;

        // TURN off all LEDs if transition from other state
        if (timer == 0)
        {
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);

        }
        if (BMP388_calibState == BMP388_NOT_CALIBRATED)
        {

            // if BMP388 is not calibrated, flash Blue light while calibrating
            if (timer == CALIB_STATE_LED_LEVEL_CALIB_NOT_COMPLETED)
            {
                ledState ^= 0x01;
                LED_Drive(BLUE, ledState);

                timer = 1;
            }
            else
            {
                timer++;
            }

            // Calibration logic.
            BMP388_calibrate_SM();

        }
        else
        {

            // if BMP388 is already calibrated, flash with period of 1 sec
            if (timer == CALIB_STATE_LED_LEVEL_CALIB_COMPLETED)
            {
                ledState ^= 0x01;
                LED_Drive(BLUE, ledState);

                timer = 1;
            }
            else
            {
                timer++;
            }

            if (GetEStopState() == ENGAGED)
            {
                curState = IDLE;
                LED_Drive(LED_ALL, OFF);
                BUZZ_BUZZ(OFF);
                timer = 0;
                ledState = 0;
            }

            // transition logic
            if (GetEStopState() == NOT_ENGAGED
                    && OrangeRX_isConnected() == CONNECTED
                    && GetFlightMode() == MANUAL)
            {
                curState = MANUAL;
                LED_Drive(LED_ALL, OFF);
                BUZZ_BUZZ(OFF);
                timer = 0;
                ledState = 0;
            }

            if (GetEStopState() == NOT_ENGAGED
                    && OrangeRX_isConnected() == CONNECTED
                    && GetFlightMode() == AUTO)
            {
                curState = AUTO;
                LED_Drive(LED_ALL, OFF);
                BUZZ_BUZZ(OFF);
                timer = 0;
                ledState = 0;
            }

        }

        break;
    }

    case MANUAL:
    {

        static unsigned int timer = 0;
        static bool ledState = 0;

        // TURN off all LEDs if transition from other state
        if (timer == 0)
        {
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);

            timer++;
        }
        else
        {
            //LED_Drive(GREEN, ON);

            // follow this sequence: GREEN - GREEN, BLUE - BLUE, ...
            if (timer == MANUAL_STATE_LED_LEVEL)
            {
                static uint8_t led_select = GREEN;
                static uint8_t led_flash_count = 0;

                if (led_flash_count == 4)
                {
                    led_select = BLUE;
                }
                if (led_flash_count == 0)
                {
                    led_select = GREEN;
                }

                ledState ^= 0x01;
                LED_Drive(led_select, ledState);

                led_flash_count++;

                if (led_flash_count == 9)
                {
                    led_flash_count = 0;
                }
                timer = 1;
            }
            else
            {
                timer++;
            }
        }

        if (GetEStopState() == ENGAGED)
        {
            curState = IDLE;
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            timer = 0;
        }

        // transition logic
        if (GetEStopState() == NOT_ENGAGED
                && OrangeRX_isConnected() == CONNECTED
                && GetFlightMode() == AUTO)
        {
            curState = AUTO;
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            timer = 0;
        }
        else
        {
            //curState = MANUAL;
        }

        // Manual controller logic

        // call Manual Mixer.
        Motor_ManMixer();

        break;
    }

    case AUTO:
    {

        static unsigned int timer = 0;
        static bool ledState = 0;

        // TURN off all LEDs if transition from other state
        if (timer == 0)
        {
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);

        }
        if (timer == AUTO_STATE_LED_LEVEL_STABLE_TIME)
        {
            ledState ^= 0x01;
            LED_Drive(GREEN, ledState);

            timer = 1;
        }
        else
        {
            timer++;
        }

        if (GetEStopState() == ENGAGED)
        {
            curState = IDLE;
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            timer = 0;
            ledState = 0;
        }

        // transition logic
        if (GetEStopState() == NOT_ENGAGED
                && OrangeRX_isConnected() == CONNECTED
                && GetFlightMode() == MANUAL)
        {
            curState = MANUAL;
            LED_Drive(LED_ALL, OFF);
            BUZZ_BUZZ(OFF);
            timer = 0;
            ledState = 0;
        }

        // Automatic controller logic

        // call Auto Mixer.
        //PID_altitude_adjust();
        //PID_roll_adjust();
        PID_master_pid();

        break;
    }

    }
}

/*
 *
 *
 *+
 *
 *
 *
 *
 *
 *
 */

void flightSafeLand(void)
{

    // Safely reduce duty of motors when
    // E-Stop is pressed or RX loses the connection from TX.

    uint8_t count;
    for (count = 0; count < 4; count++)
    {
        //check if the current duty is already 0
        if (mixer.final_duty[count] == 0)
        {
            // drone is landed.
        }
        else
        {
            // drone is in air. Decrement by a predefined step.
            mixer.final_duty[count] -= E_STOP_DECREMENT_STEP;
        }

        // send the duty to pwm controller
        Motor_setDuty(count, mixer.final_duty[count]);
    }
}
