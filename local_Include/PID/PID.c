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


#include "local_include/BMP388/BMP388.h"
#include "local_include/OrangeRX/OrangeRX.h"

#include "local_include/BUZZER/buzzer.h"


// global variables and externs

extern Orange_RX_Channel_Data rx_data;    // Received channel frequency content.
extern float base_altitude;
int16_t pv_height = 0;


// Function definitions.

void PID_altitude_adjust(void){

    /*  // enable following code when BMP388 is connected
        // get process variable for height
        float curReading = BMP388_readAltitude();


        float curBaselineReading = curReading - base_altitude;
        int16_t alt_process_variable = (int16_t) (100 * curBaselineReading);
     */

     // enable following code when BMP388 is NOT connected

        float curReading = 0.5f;


        float curBaselineReading = curReading - base_altitude;
        int16_t alt_process_variable = (int16_t) (100 * curBaselineReading);




        // Height process variable conditioning.

        // lower limit  : 0 cm
        // upper limit  : 100 cm

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

        // get throttle value and set it as setpoint
        OrangeRX_extractData();

        int16_t sp_height = (int16_t) (rx_data.dataOfCh[THROTTLE]);





        // debug
#ifdef DEBUG
        char cBuffer[80];
        sprintf(cBuffer, " Altitude Adjust : \t PV: %d cm, \t SP: %d cm", pv_height, sp_height);
        UARTprintf("%s", cBuffer);

        cBuffer[0] = '\0';
        UARTprintf("\n\n");
#endif
}
