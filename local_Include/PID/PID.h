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





// Macros





// Function prototypes

void PID_altitude_adjust(void);
void PID_roll_adjust(void);
void PID_master_pid(void);


#endif /* PID_LOCK_ */
