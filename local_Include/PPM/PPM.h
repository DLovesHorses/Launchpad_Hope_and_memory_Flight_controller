/*
 * File         :   filename
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for filename.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-28
 */


// Includes

#ifndef PPM_LOCK
#define PPM_LOCK

#include "local_Include/global.h"



// Defines

#define TM4C_CLK_RATE                   79999759



// Macros

enum ESC_ID{
    ESC_1 = 1,
    ESC_2,
    ESC_3,
    ESC_4
};



// Function prototypes



void PPM_Init(void);
void PPM_IntHandler(void);
void PPM_SignalOutPin_Init(void);

void PPM_ESC_Control(uint8_t escID, bool state);
void PPM_ESC_HighLevelControl(uint8_t escID, uint32_t loadValue);

#endif /* SAMPLE_LOCK */
