#include "local_include/global.h"

#define I2C_RUN     0x00000001
#define I2C_START   0x00000002
#define I2C_STOP    0x00000004
#define I2C_ACK     0x00000008


void I2C0_IntHandler(void);
void I2C0_Init(void);

void I2C_ReadByte(uint8_t slaveAddr, uint32_t regAddr, uint8_t *puiData);

