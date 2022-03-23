/*
 * File         :   MotorControl
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for MotorControl.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-19
 */


// Includes

#ifndef MOTOR_CONTROL_LOCK
#define MOTOR_CONTROL_LOCK

#include "local_Include/global.h"





// Defines

#define E_STOP_DECREMENT_STEP               1       // decrement motor by 1 if e-stop is pressed while flying

#define IDLE_STATE_LED_LEVEL_STABLE_TIME    250     // 250 ms

#define CALIB_STATE_LED_LEVEL_STABLE_TIME    1000     // 1000 ms
#define AUTO_STATE_LED_LEVEL_STABLE_TIME     1000     // 1000 ms



// Macros

enum ESTOP_STATES{
    NOT_ENGAGED = 0,
    ENGAGED = 1
};

enum FLIGHT_MODES{
    CALIBRATION = 0,        // Switch A -> UP
    MANUAL,                 // Switch A -> Middle
    AUTO,                    // Switch A -> Down
    IDLE                     // IDLE State
};





// Function prototypes

void flightControl(void);
bool GetEStopState(void);
uint8_t GetFlightMode(void);
void flightControl_SM(void);

#endif /* MOTOR_CONTROL_LOCK */
