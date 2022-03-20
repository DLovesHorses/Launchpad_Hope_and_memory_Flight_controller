/*
 * File         :   led.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for led.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-24
 */

// Includes
#ifndef LED_LOCK_
#define LED_LOCK_

#include "local_include/global.h"

// Defines

#define RED     0x02    // PF1
#define BLUE    0x04    // PF2
#define GREEN   0x08    // PF3
#define LED_ALL 0x0E    // ALL leds.

// Macros

enum LED_SELECT{
    LED_NOT_SELECTED = 0,
    LED_SELECTED
};

// Function prototypes

void LED_LEDInit(void);
void LED_LED1(bool);
void LED_LED2(bool);
void LED_LED3(bool);

void LED_ALL_FUNCTION(bool);
void LED_Drive(uint8_t leds, bool state);


#endif /* LED_LOCK */
