/*
 * File         :   Bluetooth
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for BLUETOOTH.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-25
 */


// Includes

#ifndef BLUETOOTH_LOCK_
#define BLUETOOTH_LOCK_

#include "local_Include/global.h"



// Defines

#define BLUETOOTH_BAUD_RATE     9600

#define ROLL_KP_PLUS            '1'
#define ROLL_KP_MINUS           '2'
#define ROLL_KI_PLUS            '3'
#define ROLL_KI_MINUS           '4'

#define PITCH_KP_PLUS           '5'
#define PITCH_KP_MINUS          '6'
#define PITCH_KI_PLUS           '7'
#define PITCH_KI_MINUS          '8'

#define HEIGHT_KP_PLUS          '9'
#define HEIGHT_KP_MINUS         'A'
#define HEIGHT_KI_PLUS          'B'
#define HEIGHT_KI_MINUS         'C'





// Macros





// Function prototypes

void BLUETOOTH_Init(void);
void BLUETOOTH_IntHandler(void);

void BLUETOOTHprintf(char* string);

#endif /* BLUETOOTH_LOCK */
