/*
 * File         :   PWM.c
 *
 * Description  :   <Short Discription of the functionality provided.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-15
 */

// Includes
#include "PWM.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"
#include "local_include/OrangeRX/OrangeRX.h"

// global variables and externs
extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
uint8_t motorSelect = MOTOR_ALL;

MOTOR_DUTY_TRACKER dutyOf;

const bool subtract_Motor[8][4] = { { 0, 0, 0, 0 },      // MOVE_UP [0]
        { 0, 0, 1, 1 },      // MOVE_RIGHT      [1]
        { 1, 0, 1, 0 },      // MOVE_FWD        [2]
        { 1, 0, 0, 1 },      // YAW_RIGHT       [3]
        { 1, 1, 1, 1 },      // MOVE_DOWN       [4]
        { 1, 1, 0, 0 },      // MOVE_LEFT       [5]
        { 0, 1, 0, 1 },      // MOVE_BKD        [6]
        { 0, 1, 1, 0 }       // YAW_LEFT        [7]

};

float Channel_Tune_Variable[4] = { 1.0f,     // Aileron  (K2)
        1.0f,     // Elevator (K3)
        1.0f,     // Throttle (K1)
        1.0f      // Rudder   (K4)
        };

bool selected_Motion[8] = { NOT_SELECTED,  // MOVE_UP      [0]
        NOT_SELECTED,  // MOVE_RIGHT   [1]
        NOT_SELECTED,  // MOVE_FWD     [2]
        NOT_SELECTED,  // YAW_RIGHT    [3]
        NOT_SELECTED,  // MOVE_DOWN    [4]
        NOT_SELECTED,  // MOVE_LEFT    [5]
        NOT_SELECTED,  // MOVE_BKD     [6]
        NOT_SELECTED,  // YAW_LEFT     [7]
        };

char *motion_string[8] = { "UP", "RIGHT", "Forward", "Rotate Right", "DOWN",
                           "LEFT", "Backward", "Rotate Left" };

MOTOR_VARIABLES mixer;
// Function definitions.

void PWM_Init(void)
{

    SysCtlPWMClockSet(SYSCTL_PWMDIV_2);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM0);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_PWM1);

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOA);
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOB);

    GPIOPinConfigure(GPIO_PA6_M1PWM2);
    GPIOPinConfigure(GPIO_PA7_M1PWM3);
    GPIOPinConfigure(GPIO_PB6_M0PWM0);
    GPIOPinConfigure(GPIO_PB7_M0PWM1);

    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTA_BASE, GPIO_PIN_7);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_6);
    GPIOPinTypePWM(GPIO_PORTB_BASE, GPIO_PIN_7);

// PWM Module 1, Gen 1 controls M1PWM2 (PA6) and M1PWM3 (PA7) , see datasheet of TM4C, table 20-1, pg: 1233.
    PWMGenConfigure(PWM1_BASE, PWM_GEN_1,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

// PWM Module 0, Gen 0 controls M0PWM0 (PB6) and M0PWM1 (PB7) , see datasheet of TM4C, table 20-1, pg: 1233.
    PWMGenConfigure(PWM0_BASE, PWM_GEN_0,
    PWM_GEN_MODE_DOWN | PWM_GEN_MODE_NO_SYNC);

    PWMGenPeriodSet(PWM1_BASE, PWM_GEN_1, PWM_GEN_PERIOD_TICK); // PA6 & PA7 period
    PWMGenPeriodSet(PWM0_BASE, PWM_GEN_0, PWM_GEN_PERIOD_TICK); // PB6 & PB7 period

    Motor_setDuty(MOTOR_ONE, 10);   // 10% duty on PA6
    Motor_setDuty(MOTOR_TWO, 10);   // 10% duty on PA7
    Motor_setDuty(MOTOR_THREE, 10); // 10% duty on PB6
    Motor_setDuty(MOTOR_FOUR, 10);  // 10% duty on PB7

    /*
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, MOTOR_ONE_INIT_DUTY);     // PA6 duty (Motor 1, CW)
     PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, MOTOR_TWO_INIT_DUTY);     // PA7 duty (Motor 2, CCW)
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, MOTOR_THREE_INIT_DUTY);   // PB6 duty (Motor 3, CCW)
     PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, MOTOR_FOUR_INIT_DUTY);    // PB7 duty (Motor 4, CW)
     */
// M1PWM2 : Generator 1 -> PWM_OUT_2 -> PA6
// M1PWM3 : Generator 1 -> PWM_OUT_3 -> PA7
// M0PWM0 : Generator 0 -> PWM_OUT_0 -> PB6
// M0PWM1 : Generator 0 -> PWM_OUT_1 -> PB6
    PWMOutputState(PWM1_BASE, PWM_OUT_2_BIT | PWM_OUT_3_BIT, true);
    PWMOutputState(PWM0_BASE, PWM_OUT_0_BIT | PWM_OUT_1_BIT, true);

// Gen 1 of Module 1 controls PA6 and PA7
// Gen 0 of Module 0 controls PB6 and PB7
    PWMGenEnable(PWM1_BASE, PWM_GEN_1);
    PWMGenEnable(PWM0_BASE, PWM_GEN_0);

    UARTprintf("PWM 0 clock: 0x%x\n", SysCtlPWMClockGet());
    UARTprintf("PWM 0 Gen 0 Period : %d\n",
               PWMGenPeriodGet(PWM0_BASE, PWM_GEN_0));
    UARTprintf("PWM 0 Gen 0 Pulse width : %d\n",
               PWMPulseWidthGet(PWM0_BASE, PWM_OUT_0));

}

/*
 *
 *
 *
 *
 * Motor    -> PWM pin assignment:
 *
 * 1        ->  PA6
 * 2        ->  PA7
 * 3        ->  PB6
 * 4        ->  PB7
 *
 *
 *
 *
 */
void Motor_setDuty(uint8_t motorID, uint8_t duty)
{

// detect and correct invalid input duty
    if (duty > 98)
    {
        duty = 98;
    }

    if (duty < 1)
    {
        duty = 1;
    }

    uint32_t convDuty = (uint32_t) ((duty * PWM_GEN_PERIOD_TICK) / 100);

    switch (motorID)
    {
    case MOTOR_ONE:
    {
        dutyOf.motor_one = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)

        break;
    }

    case MOTOR_TWO:
    {
        dutyOf.motor_two = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)

        break;
    }

    case MOTOR_THREE:
    {
        dutyOf.motor_three = convDuty;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)

        break;
    }

    case MOTOR_FOUR:
    {
        dutyOf.motor_four = convDuty;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)

        break;
    }

    case MOTOR_ONE_THREE:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_three = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)

        break;
    }

    case MOTOR_TWO_FOUR:
    {
        dutyOf.motor_two = convDuty;
        dutyOf.motor_four = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)

        break;
    }

    case MOTOR_ONE_TWO:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_two = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)

        break;
    }

    case MOTOR_THREE_FOUR:
    {
        dutyOf.motor_three = convDuty;
        dutyOf.motor_four = convDuty;

        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)

        break;
    }

    case MOTOR_ONE_FOUR:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_four = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)

        break;
    }

    case MOTOR_TWO_THREE:
    {
        dutyOf.motor_two = convDuty;
        dutyOf.motor_three = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)

        break;
    }

    case MOTOR_ALL:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_two = convDuty;
        dutyOf.motor_three = convDuty;
        dutyOf.motor_four = convDuty;

        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)

        break;
    }
    default:
    {
        // do nothing
        break;
    }
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
 *
 *
 */

void Motor_ManMixer(void)
{

// get Channle Data
    OrangeRX_extractData();

// initially no motion is selected
    uint8_t count = 0;
    for (count = 0; count < 8; count++)
    {
        selected_Motion[count] = NOT_SELECTED;
    }

    int8_t throttle_value = rx_data.dataOfCh[THROTTLE];
    int8_t aileron_value = rx_data.dataOfCh[AILERON];
    int8_t elevator_value = rx_data.dataOfCh[ELEVATOR];
    int8_t rudder_value = rx_data.dataOfCh[RUDDER];


    if (subtract_Motor[0][0] == 0)
    {
        UARTprintf("");
    }

    // if throttle value is 0, shutdown
    if (throttle_value < 3)
    {
        dutyOf.motor_one = 0;
        dutyOf.motor_two = 0;
        dutyOf.motor_three = 0;
        dutyOf.motor_four = 0;
    }

    else
    {
        // throttle value is in the low range
        selected_Motion[MOVE_UP] = SELECTED;

        dutyOf.motor_one = throttle_value;
        dutyOf.motor_two = throttle_value;
        dutyOf.motor_three = throttle_value;
        dutyOf.motor_four = throttle_value;

        // check Aileron_value and decide motion: Left, Right or None

        if (aileron_value > RC_STICK_HIGH_RANGE_THRESHOLD)
        {
            selected_Motion[MOVE_RIGHT] = SELECTED;
        }
        else if (aileron_value < RC_STICK_LOW_RANGE_THRESHOLD)
        {
            selected_Motion[MOVE_LEFT] = SELECTED;
        }
        else
        {
            // nothing.
        }

        // check Elevator_value and decide motion: FWD, BKD or None

        if (elevator_value > RC_STICK_HIGH_RANGE_THRESHOLD)
        {
            selected_Motion[MOVE_FWD] = SELECTED;
        }
        else if (elevator_value < RC_STICK_LOW_RANGE_THRESHOLD)
        {
            selected_Motion[MOVE_BKD] = SELECTED;
        }
        else
        {
            // nothing.
        }

        // check Rudder_value and decide motion: YAW_Right, YAW_LEFT or None

        if (rudder_value > RC_STICK_HIGH_RANGE_THRESHOLD)
        {
            selected_Motion[YAW_RIGHT] = SELECTED;
        }
        else if (rudder_value < RC_STICK_LOW_RANGE_THRESHOLD)
        {
            selected_Motion[YAW_LEFT] = SELECTED;
        }
        else
        {
            // nothing.
        }

        /* For each motor, determine the number of individual DOF deductions
         and point them to appropriate channel values
         */

        // initialize number of deductions for each motor to 0
        mixer.numOfDeductions[M1] = 0;
        mixer.numOfDeductions[M2] = 0;
        mixer.numOfDeductions[M3] = 0;
        mixer.numOfDeductions[M4] = 0;

        for (count = 0; count < 4; count++)
        {
            uint8_t dof = 0;
            for (dof = 0; dof < 8; dof++)
            {
                if ((selected_Motion[dof] == SELECTED)
                        && (subtract_Motor[dof][count] == SELECTED))
                {
                    mixer.numOfDeductions[count]++;
                }
            }
        }

    }

#ifdef DEBUG
    UARTprintf("Throttle: \t %d , \n", throttle_value);
    UARTprintf("Aileron: \t %d , \n", aileron_value);
    UARTprintf("Elevator: \t%d , \n", elevator_value);
    UARTprintf("Rudder: \t %d , \n", rudder_value);
    UARTprintf("\n\n");

    UARTprintf("Motion Selected:\n ");
    for (count = 0; count < 8; count++)
    {
        if (selected_Motion[count] == SELECTED)
        {
            UARTprintf("%s, ", motion_string[count]);
        }
    }
    UARTprintf("\n\n");

    UARTprintf("Number of deductions: \n");
    for (count = 0; count < 4; count++){
        UARTprintf("Motor %d: \t %d \n", (count+1), mixer.numOfDeductions[count]);
    }

    UARTprintf("\n\n");
#endif

}

