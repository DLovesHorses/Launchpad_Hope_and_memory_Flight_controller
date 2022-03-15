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
#include "local_include/BMP388/BMP388.h"
#include "local_include/OrangeRX/OrangeRX.h"

#include "local_include/BUZZER/buzzer.h"

// global variables and externs

extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
extern MOTOR_VARIABLES mixer;
extern float base_altitude;
int16_t pv_height = 0;

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
