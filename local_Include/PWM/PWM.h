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

#define RC_STICK_LOW_RANGE_THRESHOLD        49
#define RC_STICK_HIGH_RANGE_THRESHOLD       53



// Macros

enum MOTOR_GROUP{
    MOTOR_ONE = 0,
    MOTOR_TWO,
    MOTOR_THREE,
    MOTOR_FOUR,
    MOTOR_ONE_THREE,            // Front two motors
    MOTOR_TWO_FOUR,             // back two motors
    MOTOR_ONE_TWO,              // Left two motors
    MOTOR_THREE_FOUR,           // Right two motors
    MOTOR_ONE_FOUR,             // Clock-wise motor pair
    MOTOR_TWO_THREE,            // CCW motor pair
    MOTOR_ALL                   // All motors
};

typedef struct{
    uint32_t motor_duty[4];
    uint32_t motor_one;      // keeps track of the duty of motor one
    uint32_t motor_two;      // keeps track of the duty of motor two
    uint32_t motor_three;    // keeps track of the duty of motor three
    uint32_t motor_four;     // keeps track of the duty of motor four

}MOTOR_DUTY_TRACKER;



enum MOTOR_MOTIONS{
    MOVE_UP = 0,    // 0
    MOVE_RIGHT,     // 1
    MOVE_FWD,       // 2
    YAW_RIGHT,      // 3
    MOVE_DOWN,      // 4
    MOVE_LEFT,      // 5
    MOVE_BKD,       // 6
    YAW_LEFT        // 7
};

enum MOTORS_AVAILABLE{
    M1 = 0,     // Motor 1
    M2,         // Motor 2
    M3,         // Motor 3
    M4          // Motor 4
};

typedef struct{
    uint8_t numOfDeductions[4]; // This holds the number of deductions to be done on individual motor throttle values for calculation of actual net duty
                                // one variable for each motor
} MOTOR_VARIABLES;


// Function prototypes
void PWM_Init(void);

void Motor_setDuty(uint8_t motorID, uint8_t duty);

void Motor_ManMixer(void);

