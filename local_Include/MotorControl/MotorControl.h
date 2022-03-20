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

#define E_STOP_DECREMENT_STEP   1       // decrement motor by 1 if e-stop is pressed while flying



// Macros

enum ESTOP_STATES{
    NOT_ENGAGED = 0,
    ENGAGED = 1
};

enum FLIGHT_MODES{
    CALIBRATION = 0,        // Switch A -> UP
    MANUAL,                 // Switch A -> Middle
    AUTO                    // Switch A -> Down
};




// Function prototypes

void flightControl(void);
bool GetEStopState(void);
uint8_t GetFlightMode(void);

#endif /* MOTOR_CONTROL_LOCK */
