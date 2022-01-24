#include "local_include/global.h"


void I2C0_IntHandler(void);
void I2C0_Init(void);

void I2C0_Write(uint8_t slaveAddr, uint8_t regAddr, uint8_t dataToWr);
