/*
 * File         :   OrangeRX
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for OrangeRX.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-11
 */


// Includes

#include "local_Include/global.h"

// Defines

#define TM4C_CLK_RATE   79999759 // 80Mhz


// structures

typedef struct
{
    /**
     * @ Frequency of individual channels
     */
    /**@{*/
    double ch1_freq;
    double ch2_freq;
    double ch3_freq;
    double ch4_freq;
    double ch5_freq;
    double ch6_freq;
    double frame_gap_freq;

    /**@}*/
}Orange_RX_Channel_Frequency_Data;

enum channelSelectorEnum{
    FRAME_GAP_SELECT = 1,
    CH_1_SELECT,
    CH_2_SELECT,
    CH_3_SELECT,
    CH_4_SELECT,
    CH_5_SELECT,
    CH_6_SELECT,
    INVALID_CHANNEL
};




// Function prototypes

void OrangeRX_Init(void);

void OrangeRX_IntHandler(void);

void OrangeRX_showData(void);
