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

float MOTOR_DUTY_LIMIT_MULTIPLIER = 0.2f;   // default : 0.2f

bool  PWM_state = NOT_INITIALIZED;
uint8_t motorSelect = MOTOR_ALL;

MOTOR_VARIABLES mixer;

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

// NOTE :
float Channel_Tune_Variable[4] = { 0.2f,     // this is 'a' // Aileron  [0] (K2)   // K2 = a% of k1
        0.2f,     // this is 'e' // Elevator   [1] (K3)        // K3 = e% of k1
        0.2f,     // this is 'r' // Rudder     [2] (K4)        // K4 = r% of k1
        1.0f      // Throttle   [3] (K1)
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

// Function definitions.

void PWM_Init(void)
{

#ifdef PWM_ENABLED
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

    PWM_state = INITIALIZED;


#endif      // PWM_ENABLED

    mixer.channelConstant[K1] = Channel_Tune_Variable[K1];  // Throttle constant
    mixer.channelConstant[K2] = Channel_Tune_Variable[K2] * Channel_Tune_Variable[K1];   // Aileron constant (in terms of percentage of throttle_constant)
    mixer.channelConstant[K3] = Channel_Tune_Variable[K3] * Channel_Tune_Variable[K1];  // Elevator constant (in terms of percentage of throttle_constant)
    mixer.channelConstant[K4] = Channel_Tune_Variable[K4] * Channel_Tune_Variable[K1];   // Rudder constant (in terms of percentage of throttle_constant)

    dutyOf.motor_one_ppm = (TM4C_CLK_RATE_FOR_PWM / 1000);
    dutyOf.motor_two_ppm = (TM4C_CLK_RATE_FOR_PWM / 1000);
    dutyOf.motor_three_ppm = (TM4C_CLK_RATE_FOR_PWM / 1000);
    dutyOf.motor_four_ppm = (TM4C_CLK_RATE_FOR_PWM / 1000);

    // define default motor gains here
    dutyOf.motor_one_ppm_gain   = 0.998f;
    dutyOf.motor_two_ppm_gain   = 0.987f;
    dutyOf.motor_three_ppm_gain = 0.999f;
    dutyOf.motor_four_ppm_gain  = 0.996f;


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
 */
void PWM_showState(void){

    if( PWM_state  == INITIALIZED){
        UARTprintf("PWM  initialized. \n");
    }
    else{
        UARTprintf("PWM not initialized. \n");
    }

    return;
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

    uint32_t convDuty = (uint32_t) (((duty * PWM_GEN_PERIOD_TICK) / 100) * MOTOR_DUTY_LIMIT_MULTIPLIER);


    switch (motorID)
    {
    case MOTOR_ONE:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_one_ppm = ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) + ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) * duty * MOTOR_DUTY_LIMIT_MULTIPLIER / 100));
        dutyOf.motor_one_ppm *= dutyOf.motor_one_ppm_gain;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
#endif

        break;
    }

    case MOTOR_TWO:
    {
        dutyOf.motor_two = convDuty;
        dutyOf.motor_two_ppm = ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) + ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) * duty * MOTOR_DUTY_LIMIT_MULTIPLIER / 100));
        dutyOf.motor_two_ppm *= dutyOf.motor_two_ppm_gain;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
#endif

        break;
    }

    case MOTOR_THREE:
    {
        dutyOf.motor_three = convDuty;
        dutyOf.motor_three_ppm = ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) + ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) * duty * MOTOR_DUTY_LIMIT_MULTIPLIER / 100));
        dutyOf.motor_three_ppm *= dutyOf.motor_three_ppm_gain;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
#endif

        break;
    }

    case MOTOR_FOUR:
    {
        dutyOf.motor_four = convDuty;
        dutyOf.motor_four_ppm = ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) + ((SysCtlClockGet() / MOTOR_DUTY_PPM_DIVISOR) * duty * MOTOR_DUTY_LIMIT_MULTIPLIER / 100));
        dutyOf.motor_four_ppm *= dutyOf.motor_four_ppm_gain;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)
#endif

        break;
    }

    case MOTOR_ONE_THREE:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_three = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
#endif

        break;
    }

    case MOTOR_TWO_FOUR:
    {
        dutyOf.motor_two = convDuty;
        dutyOf.motor_four = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)
#endif

        break;
    }

    case MOTOR_ONE_TWO:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_two = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
#endif

        break;
    }

    case MOTOR_THREE_FOUR:
    {
        dutyOf.motor_three = convDuty;
        dutyOf.motor_four = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)
#endif

        break;
    }

    case MOTOR_ONE_FOUR:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_four = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)
#endif

        break;
    }

    case MOTOR_TWO_THREE:
    {
        dutyOf.motor_two = convDuty;
        dutyOf.motor_three = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
#endif

        break;
    }

    case MOTOR_ALL:
    {
        dutyOf.motor_one = convDuty;
        dutyOf.motor_two = convDuty;
        dutyOf.motor_three = convDuty;
        dutyOf.motor_four = convDuty;

#ifdef PWM_ENABLED
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_2, dutyOf.motor_one); // PA6 duty (Motor 1, CW)
        PWMPulseWidthSet(PWM1_BASE, PWM_OUT_3, dutyOf.motor_two); // PA7 duty (Motor 2, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_0, dutyOf.motor_three); // PB6 duty (Motor 3, CCW)
        PWMPulseWidthSet(PWM0_BASE, PWM_OUT_1, dutyOf.motor_four); // PB7 duty (Motor 4, CW)
#endif
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
 * To really understand this code, please read
 * the report and the docs for this function.
 *
 *
 * This function divides the operation of channel mixing
 * into many small tasks. Many tasks involve tracking the
 * values of the modified channel content to assist in the
 * final calculation of the deduction values for each motor.
 *
 * Two types of values are tracked:
 * 1. The modified Channel content for each DOF.
 * 2. The associated channel constants for each DOF.
 *
 * Pre-mix tasks:
 *
 * Task 1:  Acquire the data from the received signal
 *
 * Task 2:  Make sure that initially no motion is selected.
 *
 * Task 3:  Check the value of Throttle channel.
 *          If it is very low (less than 3) make duty of every motor '0' and exit.
 *
 *          It it is not low, enable the mixing algorithm.
 *
 *
 * Mixing Algorithm tasks:
 *
 * Task 1:  Initialize the duty of each motor <- Throttle Value.
 *
 * Task 2:  Using the Motor_deduction table ( subtract_Motor[][])
 *          and DOF selection table (selected_Motion[]),
 *          note the motion that will be activated based on the
 *          channel values.
 *
 * Task 3:  For each motor, make sure that initially the number of
 *          deductions = '0'.
 *
 *
 * Task 4:  Make sure that each tracking pointer is set to NULL
 *          before continuing.
 *
 * Task 5:  For each motor, count and store the number of deductions
 *          that will be done on the final duty of that motor.
 *          These values will be stored in mixer.numOfDeductions[M#]
 *          where '#' is motor number (1, 2, 3 or 4).
 *
 *          See notes to see it pictorially.
 *
 *
 * Task 6:  For each motor and selected motion, track the
 *          channel value using mixer.pChValue[][] pointers.
 *
 *          For each motor and selected motion, track the
 *          channel multiplier constants using mixer.pChConstant[][] pointers.
 *
 *
 * Task 7:  For each motor and noOfDeduction, calculate
 *          absolute difference from mid-position ( |50 - *pChValue| )
 *          and store it in mixer.abs_diff[][] array.
 *
 * Task 8:  For each motor, track each abs_diff value in
 *          descending order. The tracker pointers are in
 *          mixer.descending_abs_diff[][].
 *
 *          For each motor, track each channel Multiplier constant in
 *          order that corresponds the descending order of
 *          the abs_diff values. The tracker pointers are in
 *          mixer.descending_constant_matrix[][].
 *
 *          See notes to understand it better.
 *
 *
 * Task 9:  For each motor, calculate the value that will be
 *          deducted and the final duty that will be the output.
 *
 *          Use the information of mixer.noOfDeduction to determine
 *          which equation to use, and which intermediate variables
 *          will determine the final deduction value.
 *
 * Task 10: For each motor, send the final calculated value to its output
 *          (Set PWM duty).
 *
 *
 *
 */


void Motor_ManMixer(void)
{
    if(OrangeRX_isConnected() == NOT_CONNECTED){
        return;
    }

    // Pre-Mix tasks

// get Channle Data
    OrangeRX_extractData();

    // loop variables
    uint8_t count = 0;
    uint8_t i;

// initially no motion is selected
    for (count = 0; count < 8; count++)
    {
        selected_Motion[count] = NOT_SELECTED;
    }

    // assign decoded channel values [0-100%] to local variables.
    int8_t throttle_value = rx_data.dataOfCh[THROTTLE];
    int8_t aileron_value = rx_data.dataOfCh[AILERON];
    int8_t elevator_value = rx_data.dataOfCh[ELEVATOR];
    int8_t rudder_value = rx_data.dataOfCh[RUDDER];

    // make sure that subtract_Motor is initialized.
    if (subtract_Motor[0][0] == 0)
    {
        UARTprintf(""); // this code does nothing, but is essential for debugging purposes.
    }

    // if throttle value is 0, shutdown
    if (throttle_value < 3)
    {
        dutyOf.motor_one = 0;
        dutyOf.motor_two = 0;
        dutyOf.motor_three = 0;
        dutyOf.motor_four = 0;

        for (count = 0; count < 4; count++)
        {
            mixer.final_duty[count] = 0;
        }
    }

    else
    {
        // Mixig Algorithm

        // throttle value is active
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

        // initialize pointer to individual DOF to null.
        for (count = 0; count < 4; count++)
        {
            for (i = 0; i < 3; i++)
            {
                mixer.pChValue[count][i] = NULL;
                mixer.pChConstant[count][i] = NULL;
                mixer.descending_abs_diff[count][i] = NULL;
                mixer.marked_slot[count][i] = NULL;
            }
        }

        // count and point individual DOF for each motor
        for (count = 0; count < 4; count++)
        {
            uint8_t dof = 0;
            for (dof = 0; dof < 8; dof++)
            {
                if ((selected_Motion[dof] == SELECTED)
                        && (subtract_Motor[dof][count] == SELECTED))
                {
                    switch (dof)
                    {

                    case MOVE_UP:
                    case MOVE_DOWN:
                    {
                        // point to throttle value
                        mixer.pChValue[count][mixer.numOfDeductions[count]] =
                                &throttle_value;

                        // point to throttle constant
                        mixer.pChConstant[count][mixer.numOfDeductions[count]] =
                                &(mixer.channelConstant[K1]);
                        break;
                    }

                    case MOVE_RIGHT:
                    case MOVE_LEFT:
                    {
                        // point to aileron
                        mixer.pChValue[count][mixer.numOfDeductions[count]] =
                                &aileron_value;

                        // point to aileron constant
                        mixer.pChConstant[count][mixer.numOfDeductions[count]] =
                                &(mixer.channelConstant[K2]);

                        break;
                    }

                    case MOVE_FWD:
                    case MOVE_BKD:
                    {
                        // point to elevator
                        mixer.pChValue[count][mixer.numOfDeductions[count]] =
                                &elevator_value;

                        // point to elevator constant
                        mixer.pChConstant[count][mixer.numOfDeductions[count]] =
                                &(mixer.channelConstant[K3]);

                        break;
                    }

                    case YAW_RIGHT:
                    case YAW_LEFT:
                    {
                        // point to rudder
                        mixer.pChValue[count][mixer.numOfDeductions[count]] =
                                &rudder_value;

                        // point to rudder constant
                        mixer.pChConstant[count][mixer.numOfDeductions[count]] =
                                &(mixer.channelConstant[K4]);

                        break;
                    }

                    }

                    mixer.numOfDeductions[count]++;
                }
            }
        }

        // for each motor, and for each valid DOF,
        // calcultate |50 - DOF|

        for (count = 0; count < 4; count++)
        {
            for (i = 0; i < mixer.numOfDeductions[count]; i++)
            {
                mixer.abs_diff[count][i] = abs(
                        50 - *(mixer.pChValue[count][i]));

            }
        }

        // point the decending_abs_diff pointer to each DOF from max to min value of |50 - abs|

        // for each motor, and until each dof is marked, sort in decending order.
        for (count = 0; count < 4; count++)
        {

            uint8_t timesToLoop = mixer.numOfDeductions[count];
            uint8_t descending_tracker = 0;

            while (timesToLoop > 0)
            {
                timesToLoop--;
                int8_t dummy = 0;
                float dummy_f = 0.0f;
                int8_t *curMax = &dummy;
                float *curConstantMax = &dummy_f;

                uint8_t motor_number = count;
                uint8_t row_number = 0;

                for (i = 0; i < mixer.numOfDeductions[count]; i++)
                {
                    if ((mixer.abs_diff[count][i] > *curMax)
                            && (mixer.marked_slot[count][i] == NULL))
                    {
                        curMax = &(mixer.abs_diff[count][i]);
                        curConstantMax = (mixer.pChConstant[count][i]);
                        row_number = i;
                    }
                }

                mixer.descending_abs_diff[count][descending_tracker] = curMax;
                mixer.descending_constant_matrix[count][descending_tracker] =
                        curConstantMax;
                mixer.marked_slot[motor_number][row_number] = SELECTED;
                descending_tracker++;
            }

        }

        // For each motor, calculate the value to be deducted.

        for (count = 0; count < 4; count++)
        {

            //get no. of deduction
            uint8_t deductionNumber = mixer.numOfDeductions[count];

            // Multipliers
            float throttle_multiplier = mixer.channelConstant[K1];
            float multiplier_0 = 0.0f;
            float multiplier_1 = 0.0f;
            float multiplier_2 = 0.0f;
            // intermediate deduction variables
            int32_t m = 0;
            int32_t n = 0;
            int32_t p = 0;
            int8_t final_deduction = 0;

            if (deductionNumber == 1)
            {

                // calculate m
                m = *(mixer.descending_abs_diff[count][0]);
                m *= 2;
                m *= throttle_value;
                m /= 100;

                // assign multiplier from descending_constant matrix.
                multiplier_0 = *(mixer.descending_constant_matrix[count][0]);

                // calculate final_deduction
                final_deduction = m * multiplier_0;

            }

            if (deductionNumber == 2)
            {

                // calculate m
                m = *(mixer.descending_abs_diff[count][0]);
                m *= 2;
                m *= throttle_value;
                m /= 100;

                // calculate n

                n = *(mixer.descending_abs_diff[count][1]);
                n *= 2;
                n *= (throttle_value - m);
                n /= 100;
                n += m;

                // assign multiplier from descending_constant matrix.
                multiplier_0 = *(mixer.descending_constant_matrix[count][0]);
                multiplier_1 = *(mixer.descending_constant_matrix[count][1]);

                // calculate final_deduction
                final_deduction = (m * multiplier_0);
                final_deduction += ((n - m) * multiplier_1);
            }

            if (deductionNumber == 3)
            {

                // calculate m
                m = *(mixer.descending_abs_diff[count][0]);
                m *= 2;
                m *= throttle_value;
                m /= 100;

                // calculate n

                n = *(mixer.descending_abs_diff[count][1]);
                n *= 2;
                n *= (throttle_value - m);
                n /= 100;
                n += m;

                // calculate p

                p = *(mixer.descending_abs_diff[count][2]);
                p *= 2;
                p *= (throttle_value - n);
                p /= 100;
                p += n;

                // assign multiplier from descending_constant matrix.
                multiplier_0 = *(mixer.descending_constant_matrix[count][0]);
                multiplier_1 = *(mixer.descending_constant_matrix[count][1]);
                multiplier_2 = *(mixer.descending_constant_matrix[count][2]);

                // calculate final_deduction
                final_deduction = (m * multiplier_0);
                final_deduction += ((n - m) * multiplier_1);
                final_deduction += ((p - n) * multiplier_2);


            }

            mixer.final_duty[count] = (throttle_value * throttle_multiplier) - final_deduction;
        }

    }

    // modify the output.

    for (count = 0; count < 4; count++)
    {
        Motor_setDuty(count, mixer.final_duty[count]);
    }

#ifdef DEBUG_DISABLED
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
    for (count = 0; count < 4; count++)
    {
        UARTprintf("Motor %d: \t %d \n", (count + 1),
                   mixer.numOfDeductions[count]);
    }

    UARTprintf("\n\n");

    for (count = 0; count < 4; count++)
    {
        for (i = 0; i < 3; i++)
        {
            if (mixer.pChValue[count][i] != NULL)
            {
                UARTprintf(
                        "Motor %d \t deduction %d -> %d \t Absolute: %d \t Descending: %d\n",
                        (count + 1), i, *(mixer.pChValue[count][i]),
                        mixer.abs_diff[count][i],
                        *(mixer.descending_abs_diff[count][i]));
            }
        }
    }

    //UARTprintf("\n\n");

    UARTprintf("Final Output: \n");
    for (count = 0; count < 4; count++)
    {

        UARTprintf("Motor %d : \t %d\n", (count + 1), mixer.final_duty[count]);
    }

    UARTprintf("\n\n");

#endif

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
 */

void MOTOR_showDuty(void){

    uint8_t count = 0;
    for(count = 0; count < 4 ; count++){
        UARTprintf(" Motor : \t %d \t -> \t %d . \n", ( count + 1 ) , mixer.final_duty[count]);
    }
    UARTprintf("\n\n");
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
 *
 *
 */





