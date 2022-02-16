/*
 * File         :   PWM.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for PWM.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-15
 */

// Includes
#include "local_Include/global.h"

// Defines

/*
 * calculate period:
 * required frequency: 50k
 * therefore, 1/50k = 20uS.
 * PWM clock: SystClock / 2 : 40 MHz.
 * therefore, clock ticks in 20 uS if 40 MClock ticks occur in 1 second : 800 clock Ticks
 *
 */
#define PWM_FREQ                50000       // 50kHz
#define PWM_GEN_PERIOD_TICK     800         // see the above calculation

#define MOTOR_ONE_INIT_DUTY     80          // change it to 0 on final version
#define MOTOR_TWO_INIT_DUTY     80          // change it to 0 on final version
#define MOTOR_THREE_INIT_DUTY   80          // change it to 0 on final version
#define MOTOR_FOUR_INIT_DUTY    80          // change it to 0 on final version


// Macros

enum MOTOR_ID{
    MOTOR_ONE = 0,
    MOTOR_TWO,
    MOTOR_THREE,
    MOTOR_FOUR,
    MOTOR_ALL,
    MOTOR_ONE_THREE,
    MOTOR_TWO_FOUR
};


// Function prototypes
void PWM_Init(void);

void Motor_setDuty(uint8_t motorID, uint8_t duty);

