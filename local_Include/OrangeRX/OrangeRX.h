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

#define TM4C_CLK_RATE                   79999759    // 80Mhz

// these are the result of measurements done during extracting data content from individual channels

// LOW  -> -100 %
// HIGH -> +100 %

#define ORANGE_RX_CH_0_LOW_TIME         0.01329f    // Low time for Frame Gap (don't use it because it is a function of other channels, obviously.
#define ORANGE_RX_CH_1_LOW_TIME         0.001015f   // Low time for Channel 1
#define ORANGE_RX_CH_2_LOW_TIME         0.001015f   // Low time for Channel 2
#define ORANGE_RX_CH_3_LOW_TIME         0.001018f   // Low time for Channel 3
#define ORANGE_RX_CH_4_LOW_TIME         0.001020f   // Low time for Channel 4
#define ORANGE_RX_CH_5_LOW_TIME         0.001021f   // Low time for Channel 5
#define ORANGE_RX_CH_6_LOW_TIME         0.001015f   // Low time for Channel 6

#define ORANGE_RX_CH_0_HIGH_TIME        0.01329f     // again, don't use it!
#define ORANGE_RX_CH_1_HIGH_TIME        0.002055f    // High time for Channel 1
#define ORANGE_RX_CH_2_HIGH_TIME        0.002055f    // High time for Channel 2
#define ORANGE_RX_CH_3_HIGH_TIME        0.002057f    // High time for Channel 3
#define ORANGE_RX_CH_4_HIGH_TIME        0.002061f    // High time for Channel 4
#define ORANGE_RX_CH_5_HIGH_TIME        0.002061f    // High time for Channel 5
#define ORANGE_RX_CH_6_HIGH_TIME        0.002055f    // High time for Channel 6




// structures

typedef struct
{
    /**
     * @ Frequency of individual channels
     */
    /**@{*/

    double ch_low_time[7];      // -100 %
    double ch_high_time[7];     // 100 %
    double ch_nom_data[7];      // normalized data [0-100%]     ; element [0] is not used.
    double ch_act_data[7];      // actual data [-100% - 100%]   ; element [0] is not used.
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
} Orange_RX_Channel_Data;

typedef struct {
    char *ch_text[7];
    char *ch_text_alt[7];
} Orange_RX_Channel_Text;



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

void OrangeRX_extractData(void);
void OrangeRX_showActData(void);       // displays normalized and actual channel data extracted from period of the channel;
void OrangeRX_showRawData(void);    // show's frequency and period information

void OrangeRX_receivedChannelReorganizer(void);
