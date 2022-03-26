/*
 * File         :   PID.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for PID.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-11
 */


// Includes

#ifndef PID_LOCK_
#define PID_LOCK_

#include "local_Include/global.h"

// Defines

#define ROLL_KP     0.1000f
#define ROLL_KI     0.0001f

#define PITCH_KP    0.2000f
#define PITCH_KI    0.0010f



// Macros


typedef struct {
    float kp;
    float ki;
}PID_TUNE_PARAMETER;

typedef struct {
    PID_TUNE_PARAMETER roll;
    PID_TUNE_PARAMETER pitch;
    PID_TUNE_PARAMETER yaw;
    PID_TUNE_PARAMETER alt;
}PID_VAR;



// Function prototypes

void PID_altitude_adjust(void);
void PID_roll_adjust(void);
void PID_master_pid(void);
void changeMotorDuty(uint8_t motor, float movementError );


#endif /* PID_LOCK_ */
