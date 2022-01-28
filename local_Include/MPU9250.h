/*
 * File         :   MPU9250.c
 *
 * Description  :   Provides defines, Macros and
 *                  function prototypes for MPU9250.c
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */


// Includes
#include "local_include/global.h"

#define MPU9250_SA      0x68
#define AK8963_SA       9830

// Defines

// For MPU9250 (MPU6500)
#define MPU9250_CONFIG          0x1A
#define MPU9250_GYRO_CONFIG     0x1B
#define MPU9250_ACCEL_CONFIG    0x1C
#define MPU9250_SMPLRT_DIV      0x25
#define MPU9250_INT_ENABLE      0x38
#define MPU9250_PWR_MGMT_1      0x6B
#define MPU9250_WHO_AM_I        0x75


#define AK8963_WIA                      0x00
#define AK8963_WIA_ID                   0x48

#define BMP280_SA                       0x76        // confirmed
#define BMP280_CHIP_ID                  0xD0





// Macros




// Function prototypes

void MPU9250_Init(void);
void MPU9250_WriteOld(uint8_t regAddr, uint8_t dataToWr);                       // working (old method)
void MPU9250_WriteNew(uint8_t regAddr, uint8_t dataToWr);                       // working
void MPU9250_SingleRead(uint8_t regAddr, uint8_t *dataToSend);                  // not-working (good docs)
void MPU9250_SingleByteRead(uint8_t regAddr, uint8_t *dataToSend);              // not-working
void MPU9250_ReadReg(uint8_t regAddr, uint8_t *dataToSend);                     // not-working
void MPU9250_MultiByteRead(char regAddr, int byteCount, char *data);            // not-working
void MPU9250_Read(uint8_t slaveAddr, uint32_t regAddr, uint8_t *puiData);       // working





