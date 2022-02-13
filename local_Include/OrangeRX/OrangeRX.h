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

    double ch_freq[7];      // [0] -> Frame_gap, [1] -> CH_1 ... so on
    double ch_period[7];    // same as above

    double *pFreq_Ch[7];
    double *pPeriod_Ch[7];  // pointe to logical channels.

    /* Use only pPerCh variable whenever you want to access the data.
     *
     * pointer[0] will always point to the channel representing FRAME_GAP
     * pointer[1] will always point to the channel representing CH_1
     * pointer[2] will always point to the channel representing CH_2
     * pointer[3] will always point to the channel representing CH_3
     * pointer[4] will always point to the channel representing CH_4
     * pointer[5] will always point to the channel representing CH_5
     * pointer[6] will always point to the channel representing CH_6
     */

    /**@}*/
} Orange_RX_Channel_Frequency_Data;

enum channelSelectorEnum
{
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

void OrangeRX_receivedChannelReorganizer(void);
