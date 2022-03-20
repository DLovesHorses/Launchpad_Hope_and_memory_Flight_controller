/*
 * File         :   led.c
 *
 * Description  :   Houses LED Initialization and driver.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-24
 */


// Includes

#include "led.h"

// global variables and externs


// Function definitions.

void LED_LEDInit(void)
{
    //
    // Enable the GPIO port that is used for the on-board LED.
    //
    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOF);

    //
    // Check if the peripheral access is enabled.
    //
    while (!SysCtlPeripheralReady(SYSCTL_PERIPH_GPIOF))
    {
    }

    //
    // Enable the GPIO pin for the LED (PF1, PF2, PF3).  Set the direction as output, and
    // enable the GPIO pin for digital function.
    //
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_1);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_2);
    GPIOPinTypeGPIOOutput(GPIO_PORTF_BASE, GPIO_PIN_3);

}

void LED_LED1(bool state)
{
    if (state == ON)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, GPIO_PIN_1);
    }

    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_1, 0x00);
    }
}

void LED_LED2(bool state)
{
    if (state == ON)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, GPIO_PIN_2);
    }

    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_2, 0x00);
    }
}

void LED_LED3(bool state)
{
    if (state == ON)
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, GPIO_PIN_3);
    }

    else
    {
        GPIOPinWrite(GPIO_PORTF_BASE, GPIO_PIN_3, 0x00);
    }
}

void LED_ALL_FUNCTION(bool command)
{
    if (command == OFF)
    {
        LED_LED1(OFF);
        LED_LED2(OFF);
        LED_LED3(OFF);
    }
    else
    {
        LED_LED1(ON);
        LED_LED2(ON);
        LED_LED3(ON);
    }

}

/*
 *
 *
 *
 *
 *
 *
 * LED_Drive() : Top level API for driving any set of LEDs.
 *
 * Inputs:
 *      leds : LED to select. (eg: RED, BLUE, RED | GREEN, RED | BLUE | GREEN, etc.)
 *      state: State of led to drive to (ON, OFF).
 *
 */
void LED_Drive(uint8_t leds, bool state){
    bool red = ((leds & RED) == RED) ? LED_SELECTED : LED_NOT_SELECTED;
    bool blue = ((leds & BLUE) == BLUE) ? LED_SELECTED : LED_NOT_SELECTED;
    bool green = ((leds & GREEN) == GREEN) ? LED_SELECTED : LED_NOT_SELECTED;
    bool all = ((leds & LED_ALL) == LED_ALL) ? LED_SELECTED : LED_NOT_SELECTED;

    if(red == LED_SELECTED){ LED_LED1(state); }
    if(blue == LED_SELECTED){ LED_LED2(state); }
    if(green == LED_SELECTED){ LED_LED3(state); }
    if(all == LED_SELECTED){ LED_ALL_FUNCTION(state); }


    return;
}
