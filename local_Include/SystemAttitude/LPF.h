/*
 * File         :   LPF.h
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for LPF.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-03-15
 */


// Includes

#ifndef LPF_LOCK_
#define LPF_LOCK_

#include "local_Include/global.h"

// Defines

#define PI      3.14159265358979323846f



// Macros

typedef struct {
    float prevOutput; // Previous output
    float Fc;  // and cutoff frequency in Hz
} LOW_PASS_FILTER;




// Function prototypes
float applyLPF(LOW_PASS_FILTER *low_pass, float input, float dt);


#endif /* LPF_LOCK */
