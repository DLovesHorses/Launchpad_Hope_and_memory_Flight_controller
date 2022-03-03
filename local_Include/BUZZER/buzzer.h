/*
 * File         :   buzzer.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for filename.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-06
 */


// Includes

#include "local_Include/global.h"


// Defines
#define BUZZ_SM_CALLED_FROM_MAIN    0
#define BUZZ_SM_CALLED_FROM_BUZZ    1

// different Buz Duration period
#define BUZZ_DUR_SHORT             100     // 100 ms BUZ
#define BUZZ_DUR_MEDIUM            300     // 300 ms BUZ
#define BUZZ_DUR_LONG              700     // 700 ms BUZ
#define BUZZ_DUR_LONG_LONG         1000    // 1000 ms BUZ

// different Buz repeat counts
#define BUZZ_REP_TIME_1             1
#define BUZZ_REP_TIME_2             3
#define BUZZ_REP_TIME_3             4
#define BUZZ_REP_TIME_4             5
#define BUZZ_REP_TIME_5             6
#define BUZZ_REP_TIME_6             7
#define BUZZ_REP_TIME_7             8
#define BUZZ_REP_TIME_8             9

// different Buz Pause intervals
#define BUZZ_PAUSE_TIME_100         100     // 100 ms
#define BUZZ_PAUSE_TIME_500         500     // 500 ms
#define BUZZ_PAUSE_TIME_1000        1000    // 1000 ms
#define BUZZ_PAUSE_TIME_1500        1500    // 1500 ms
#define BUZZ_PAUSE_TIME_2000        2000    // 2000 ms
#define BUZZ_PAUSE_TIME_3000        3000    // 3000 ms







// Macros
enum BUZZER_SM_STATES{
    BUZZ_OFF = 0,
    BUZZ_ON,
    BUZZ_PAUSE
};




// Function prototypes
void init_Buzzer(void);

//Low level driver functions
void BUZZ_BUZZ(bool buzzTone);
void BUZZ_BUZZER(uint16_t duration, uint8_t repTime, uint16_t pauseIntervalTime);

void BUZZ_SM(bool callerID);

