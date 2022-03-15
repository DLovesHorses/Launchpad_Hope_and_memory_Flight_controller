/*
 * File         :   filename
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for filename.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */


// Includes

#include "local_Include/global.h"

// Defines


#define TRIG_TIMER_DIV_1_S       1.0f      // 10 sec : 1/10; 2 sec : 1/2; 5 sec : 1/5
#define TRIG_TIMER_DIV_1_MS      1000.0f
#define TRIG_TIMER_DIV_100_US    10000.0f
#define TRIG_TIMER_DIV_10_US     100000.0f

#define TRIG_LOW_TIME            1.0f       // this will never be used in calculation, but it is required for calling the trigger function




// Macros

enum TRIG_STATES{
    LOW = 0,
    HIGH
};



// Function prototypes
void SONAR_TrigIntHandler(void);
void SONAR_EchoIntHandler(void);
void SONAR_Init(void);

void SONAR_Trigger(bool trigState, float trigTime);
