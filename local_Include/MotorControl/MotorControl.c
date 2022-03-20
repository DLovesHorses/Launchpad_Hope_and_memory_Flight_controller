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
#include "local_Include/uart.h"
#include "local_Include/i2c.h"


#include "local_Include/PWM/PWM.h"
#include "local_Include/OrangeRX/OrangeRX.h"
#include "local_Include/PID/PID.h"
#include "local_Include/BMP388/BMP388.h"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;
extern MOTOR_VARIABLES mixer;

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
       if (flightMode == CALIBRATION) { UARTprintf("Mode :\t  Calibration. \n"); }
       if (flightMode == MANUAL) { UARTprintf("Mode :\t  Manual. \n"); }
       if (flightMode == AUTO) { UARTprintf("Mode :\t  Automatic. \n"); }

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

#ifdef DEBUG

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

#ifdef DEBUG
    if (rxIsConnected == NOT_CONNECTED) { UARTprintf("RX state : \t Not Connected \n"); }
    if (eStopState == ENGAGED) { UARTprintf("EStop State : \t ON \n"); }
    if (eStopState == NOT_ENGAGED) { UARTprintf("EStop State : \t OFF \n"); }

#endif

    return eStopState;

}
