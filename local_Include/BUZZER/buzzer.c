/*
 * File         :   buzzer.c
 *
 * Description  :   <Short Discription of the functionality provided.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-06
 */

// Includes
#include "buzzer.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"

// global variables and externs

/*
 *
 *
 *
 *
 *
 * BUZZER -> PC_7
 *
 *
 */
void init_Buzzer(void)
{

    // enable GPIO port C system peripheral
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    //
    // Check if the peripheral access is enabled.
    //
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    GPIOPadConfigSet(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_STRENGTH_4MA, GPIO_PIN_TYPE_STD);
    // configure PC7 as output pin
    GPIOPinTypeGPIOOutput(GPIO_PORTC_BASE, GPIO_PIN_7);

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
 */
void BUZZ_BUZZ(bool buzzTone){

    if(buzzTone == ON){
        // turn on Buzzer.
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, GPIO_PIN_7);
    }
    else{
        GPIOPinWrite(GPIO_PORTC_BASE, GPIO_PIN_7, OFF);
    }

}
// Function definitions.
