/*
 * File         :   PID.c
 *
 * Description  :   This file contains all the PID code for the controller
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-11
 */

// Includes
#include "PID.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"

#include "local_include/PWM/PWM.h"
#include "local_include/BMX160/BMX160.h"
#include "local_include/BMP388/BMP388.h"
#include "local_include/OrangeRX/OrangeRX.h"

#include "local_include/BUZZER/buzzer.h"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
extern MOTOR_VARIABLES mixer;
extern float base_altitude;

extern int roll_filtered; // this will contain filtered value of Roll. Call BMX160_updateData() to update this data.
extern int pitch_filtered; // this will contain filtered value of Pitch. Call BMX160_updateData() to update this data.
extern int yaw_filtered; // this will contain filtered value of Yaw. Call BMX160_updateData() to update this data.

int16_t pv_height = 0;
int16_t pv_roll = 0;

// tuning variables
float kp_tune = 0.0f;   // depreciated
float ki_tune = 0.0f;   // depreciated

float roll_kp_tune = 0.0f;
float roll_ki_tune = 0.0f;

float pitch_kp_tune = 0.0f;
float pitch_ki_tune = 0.0f;

float yaw_kp_tune = 0.0f;
float yaw_ki_tune = 0.0f;

float alt_kp_tune = 0.0f;
float alt_ki_tune = 0.0f;

PID_VAR tune;



// Function definitions.

void PID_altitude_adjust(void)
{

    // enable following code when BMP388 is connected
    // get process variable for height
    float curReading = BMP388_readAltitude();

    float curBaselineReading = curReading - base_altitude;
    int16_t alt_process_variable = (int16_t) (100 * curBaselineReading);

    // enable following code when BMP388 is NOT connected
    /*
     float curReading = 0.5f;


     float curBaselineReading = curReading - base_altitude;
     int16_t alt_process_variable = (int16_t) (100 * curBaselineReading);

     */

    // Height process variable conditioning.
    // lower limit  : 0 cm
    // upper limit  : 100 cm
    pv_height = alt_process_variable; // Just for Debug. Use sensor fusion to make sure that there is no drift.

    /*
     if(alt_process_variable >= 0 && alt_process_variable <= 100){
     pv_height = alt_process_variable;
     }
     else{
     if (alt_process_variable < 0){
     pv_height = 0;
     }
     else if(alt_process_variable > 100){
     pv_height = 100;
     }
     }

     */
    // get throttle value and set it as setpoint
    OrangeRX_extractData();

    int16_t sp_height;

    if (OrangeRX_isConnected())
    {
        sp_height = (int16_t) (rx_data.dataOfCh[THROTTLE]);

    }
    else
    {
        sp_height = 0;
    }

    // Reverse action of P Controller
    float error = sp_height - pv_height;
    float kp = 0.4;
    float alt_error = kp * error;

    uint8_t count = 0;
    for (count = 0; count < 4; count++)
    {
        float rangeChecker = mixer.final_duty[count] + alt_error;
        if (rangeChecker < 3)
        {
            mixer.final_duty[count] = 3;
        }

        else if (rangeChecker > 97)
        {
            mixer.final_duty[count] = 97;
        }
        else
        {
            mixer.final_duty[count] += alt_error;
        }
        //mixer.final_duty[count] = mixer.final_duty[count] < 3 ? 3 : mixer.final_duty[count] ;
        //mixer.final_duty[count] = mixer.final_duty[count] > 97 ? 97 : mixer.final_duty[count] ;

        Motor_setDuty(count, mixer.final_duty[count]);

    }

    // debug
#ifdef DEBUG
    char cBuffer[80];
    sprintf(cBuffer, " Altitude Adjust : \t PV: %d cm, \t SP: %d cm", pv_height,
            sp_height);
    UARTprintf("%s", cBuffer);

    cBuffer[0] = '\0';
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
 *
 *
 *
 *
 *
 *
 *
 *
 */
void PID_roll_adjust(void)
{

    // get Aileron value and set it as setpoint
    OrangeRX_extractData();

    // update sensor data
    BMX160_updateData();

    // variables to store appropriate data
    int16_t chData;
    int16_t sp_roll;
    int16_t pv_roll = roll_filtered;

    if (OrangeRX_isConnected())
    {
        chData = (int16_t) rx_data.dataOfCh[AILERON];
        sp_roll = chData - AILERON_MID;

    }
    else
    {
        sp_roll = 0;
    }

#ifdef DEBUG

    char cBuffer[80];
    sprintf(cBuffer,
            " Roll Adjust : \t Channel Data: %d \t PV: %d degree, \t SP: %d degree",
            chData, pv_roll, sp_roll);
    UARTprintf("%s", cBuffer);

    cBuffer[0] = '\0';
    UARTprintf("\n");

#endif

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
 *
 *
 *
 *
 */
void PID_master_pid(void)
{

    // get Throttle, Aileron and Elevator values and set it as setpoint variables
    OrangeRX_extractData();

    // update BMX160 sensor data (for roll, pitch & yaw)
    BMX160_updateData();

    // get process variable for height
    float AltReading = BMP388_readAltitude();
    AltReading -= base_altitude;
    int16_t pv_height = (int16_t) (100 * AltReading);

    // get process variables for roll, pitch and yaw
    int16_t pv_roll = roll_filtered;
    int16_t pv_pitch = (-1) *pitch_filtered;        // multiplied by -1 so that the sp and pv aligns in direction.
    int16_t pv_yaw = yaw_filtered;

    // variables to store appropriate data
    int16_t chData_throttle;
    int16_t chData_aileron;
    int16_t chData_elevator;
    int16_t chData_rudder;

    int16_t sp_roll;
    int16_t sp_pitch;
    int16_t sp_yaw;
    int16_t sp_height;

    // set tuning variables roll
    tune.roll.kp = roll_kp_tune;    // save this tuned value in ROLL_KP;
    tune.roll.ki = roll_ki_tune;    // save this tunde value in ROLL_KI;

    // set tuning variables roll
    tune.pitch.kp = pitch_kp_tune;    // save this tunde value in PITCH_KP;
    tune.pitch.ki = pitch_ki_tune;    // save this tunde value in PITCH_KI;

    // set tuning variables roll
    tune.yaw.kp = yaw_kp_tune;    // save this tunde value in YAW_KP;
    tune.yaw.ki = yaw_ki_tune;    // save this tunde value in YAW_KI;

    // set tuning variables roll
    tune.alt.kp = alt_kp_tune;    // save this tunde value in ALT_KP;
    tune.alt.ki = alt_ki_tune;    // save this tunde value in ALT_KI;

    if (OrangeRX_isConnected())
    {

        // roll setpoint
        chData_aileron = (int16_t) rx_data.dataOfCh[AILERON];
        sp_roll = chData_aileron - AILERON_MID;

        // pitch setpoint
        chData_elevator = (int16_t) rx_data.dataOfCh[ELEVATOR];
        sp_pitch = chData_elevator - ELEVATOR_MID;

        // yaw setpoint
        chData_rudder = (int16_t) rx_data.dataOfCh[RUDDER];
        sp_yaw = chData_rudder - RUDDER_MID;

        // height setpoint
        chData_throttle = (int16_t) rx_data.dataOfCh[THROTTLE];
        sp_height = chData_throttle;

    }
    else
    {
        sp_roll = 0;
        sp_pitch = 0;
        sp_yaw = 0;
        sp_height = 0;
    }


    /*
    // Enable PID for height only if the Aileron, Elevator and Rudder are not changed. (add some buffer to let the TX, RX trimming possible).
    // Priority:
    // Roll -> Pitch -> Height
    if ((sp_roll > -3 && sp_roll < 3) && (sp_pitch > -3 && sp_pitch < 3)
            && (sp_yaw > -3 && sp_yaw < 3))
    {

        // Reverse action of P Controller
        float error = sp_height - pv_height;
        float kp = 0.4;
        float height_error = kp * error;

        uint8_t count = 0;
        for (count = 0; count < 4; count++)
        {
            switch (count)
            {

            case M1:
            {
                changeMotorDuty(M1, height_error);
                break;
            }

            case M2:
            {
                changeMotorDuty(M2, height_error);
                break;
            }

            case M3:
            {
                changeMotorDuty(M3, height_error);
                break;
            }

            case M4:
            {
                changeMotorDuty(M4, height_error);
                break;
            }

            }

        }
        UARTprintf("Height Changed. \n");

    }


*/

    // if Aileron (Roll is changed), Force the drone to go to requested direction
    if (!(sp_roll > -3 && sp_roll < 3))//else if (!(sp_roll > -3 && sp_roll < 3))
    {

        // Reverse action of P Controller
        float error = sp_roll - pv_roll;
        float kp = tune.roll.kp;

        float ki = tune.roll.ki;

        static float intigral_error_dt = 0;
        intigral_error_dt += error;

        float p_term = kp * error;
        float i_term = ki * intigral_error_dt;

        float roll_error = (p_term + i_term) ;


        if (sp_roll <= -3)
        {
            // if requested to go left, slow motor 1 and motor 2

            uint8_t count = 0;
            for (count = 0; count < 4; count++)
            {
                switch (count)
                {

                case M1:
                {
                    changeMotorDuty(M1, roll_error);
                    break;
                }

                case M2:
                {
                    changeMotorDuty(M2, roll_error);
                    break;
                }

                case M3:
                {

                    break;
                }

                case M4:
                {

                    break;
                }

                }

            }

            UARTprintf("Roll Left. \n\n\n");

        }

        else    //(sp_roll >= 3)
        {
            // if requested to go right, slow motor 3 and motor 4

            uint8_t count = 0;
            for (count = 0; count < 4; count++)
            {
                switch (count)
                {

                case M1:
                {
                    break;
                }

                case M2:
                {
                    break;
                }

                case M3:
                {
                    changeMotorDuty(M3, (-1) * roll_error);
                    break;
                }

                case M4:
                {
                    changeMotorDuty(M4, (-1) * roll_error);
                    break;
                }

                }

            }

            UARTprintf("Roll Right. \n\n\n");

        }

    }




    // if Elevator (Pitch is changed), Force the drone to go to Forward / Backward
    if (!(sp_pitch > -3 && sp_pitch < 3))
    {

        // Reverse action of P Controller
        float error = sp_pitch - pv_pitch;
        float kp = tune.pitch.kp;
        float ki = tune.pitch.ki;

        static float intigral_error_dt = 0;
        intigral_error_dt += error;

        float p_term = kp * error;
        float i_term = ki * intigral_error_dt;

        float pitch_error = (p_term + i_term) ;

        if (sp_pitch <= -3)
        {
            // if requested to go backward, slow motor 2 and motor 4

            uint8_t count = 0;
            for (count = 0; count < 4; count++)
            {

                switch (count)
                {

                case M1:
                {
                    break;
                }

                case M2:
                {
                    changeMotorDuty(M2, pitch_error);
                    break;
                }

                case M3:
                {

                    break;
                }

                case M4:
                {
                    changeMotorDuty(M4, pitch_error);
                    break;
                }

                }

            }

            UARTprintf("Go Forward. \n\n\n");

        }

        else
        {
            // if requested to go forward, slow motor 1 and motor 3

            uint8_t count = 0;
            for (count = 0; count < 4; count++)
            {
                switch (count)
                {

                case M1:
                {
                    changeMotorDuty(M1, (-1) * pitch_error);
                    break;
                }

                case M2:
                {
                    break;
                }

                case M3:
                {
                    changeMotorDuty(M3, (-1) * pitch_error);
                    break;
                }

                case M4:
                {

                    break;
                }

                }

            }

            UARTprintf("Go Backward. \n\n\n");
        }
    }


/*
    else{
        UARTprintf("Yaw Changed. \n");
    }

    */
    //MOTOR_showDuty();

#ifdef DEBUG

    UARTprintf("Variable \t -> \t SP \t , \t PV\n");
    UARTprintf("Height \t \t -> \t %d \t , \t %d \n", sp_height, pv_height);
    UARTprintf("Roll \t \t -> \t %d \t , \t %d \n", sp_roll, pv_roll);
    UARTprintf("Pitch \t \t -> \t %d \t , \t %d \n", sp_pitch, pv_pitch);
    UARTprintf("Yaw \t \t -> \t %d \t , \t %d \n", sp_yaw, pv_yaw);

    char cBuffer[80];
    sprintf(cBuffer,
            " Tuning Variables: \t Kp : \t %f \t Ki : \t %f.",
            kp_tune, ki_tune);
    UARTprintf("%s", cBuffer);

    cBuffer[0] = '\0';


    UARTprintf("\n\n");

#endif

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
 */

void changeMotorDuty(uint8_t motor, float movementError)
{
    float rangeChecker = mixer.final_duty[motor] + movementError;
    if (rangeChecker < 3)
    {
        mixer.final_duty[motor] = 3;
    }

    else if (rangeChecker > 97)
    {
        mixer.final_duty[motor] = 97;
    }
    else
    {
        mixer.final_duty[motor] += movementError;
    }

    Motor_setDuty(motor, mixer.final_duty[motor]);
}

