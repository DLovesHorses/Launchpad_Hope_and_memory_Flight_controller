/*
 * File         :   SysTick.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for SysTick.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-23
 */

// Includes
#include "local_include/global.h"

// Defines

#define SYSTICK_COUNTER_TIMER   80000

// Macros

// Function prototypes

void SysTick_IntHandler(void);
void SysTick_Init(void);
void SYSTICK_Delay( uint32_t );
