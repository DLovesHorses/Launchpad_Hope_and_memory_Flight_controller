
/*
 * File         :   MPU9250.c
 *
 * Description  :   Houses core drivers for BMX160 module.
 *              :
 *                  It provides Initialization of module,
 *                  Gyro config,
 *                  Magnetometer config,
 *                  Accelerometer config,
 *                  interrupt config,
 *                  power level config,
 *
 *                  Gyro raw value reading,
 *                  Mag. "      "       " ,
 *                  Acc. "      "       " ,
 *
 *                  Gyro calculated reading,
 *                  Mag.    "           "   ,
 *                  Acc.    "           "
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-28
 */

// Includes

#include "MPU9250.h"
#include "i2c.h"


// global variables and externs

// Function definitions.
void MPU9250_Init(void)
{
    /*
     * Slave address    : 110100X
     *      X           : Level of SA0 pin on the breakout
     *                  : 0 -> current config
     *
     * Current SA (Hex) : 68
     */

    //
    /*
    MPU9250_WriteNew(MPU9250_CONFIG, 0x00);
    MPU9250_WriteNew(MPU9250_GYRO_CONFIG, 0x18);
    MPU9250_WriteNew(MPU9250_ACCEL_CONFIG, 0x00);
    MPU9250_WriteNew(MPU9250_SMPLRT_DIV, 0x07);
    MPU9250_WriteNew(MPU9250_INT_ENABLE, 0x01);
    MPU9250_WriteNew(MPU9250_PWR_MGMT_1, 0x01);
    MPU9250_WriteNew(MPU9250_WHO_AM_I, 0x71);
*/

#ifdef DEBUG


    uint8_t dataToRec;

    UARTprintf("\n MPU9250: WHO_AM_I command sending...\n");
    I2C_ReadByte(MPU9250_SA, MPU9250_WHO_AM_I, &dataToRec);

    UARTprintf("MPU9250: WHO_AM_I command response: %X\n", dataToRec);

#endif

}




void MPU9250_WriteNew(uint8_t regAddr, uint8_t dataToWr)
{

    uint32_t errorStatus;
#ifdef DEBUG
    UARTprintf("Writing MPU9250: Register %X with Data %X\n", regAddr,
               dataToWr);
#endif

// append write command at the end of slave address
// (handled in I2CMasterSlaveAddrSet())
//slaveAddr = (slaveAddr << 1) | 0x00 ;       // 0x00 -> write;
// 0x01 -> read.

// set I2C0 slave address

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
    false);         // false -> write to slave;
                    // true -> read from slave

// put address of register to be written into slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

// send register address

    I2CMasterControl(I2C0_BASE,
    I2C_START | I2C_RUN);

// wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    errorStatus = I2CMasterErr(I2C0_BASE);

    if (errorStatus == I2C_MASTER_ERR_NONE)
    {
#ifdef DEBUG
        UARTprintf("MPU9250 on the bus. \n", regAddr, dataToWr);
#endif

// put data to be transmitted to slave

        I2CMasterDataPut(I2C0_BASE, dataToWr);

// send data

        I2CMasterControl(I2C0_BASE,
        I2C_STOP | I2C_RUN);

        while (I2CMasterBusBusy(I2C0_BASE))
        {

        }

        errorStatus = I2CMasterErr(I2C0_BASE);

        if (errorStatus == I2C_MASTER_ERR_DATA_ACK)
        {
#ifdef DEBUG
            UARTprintf("MPU9250 on the bus, but data error.\n");
#endif
        }
        else
        {
#ifdef DEBUG
            UARTprintf("MPU9250 written: Register %X with Data: %X\n", regAddr,
                       dataToWr);
#endif
        }
    }

    else if (errorStatus == I2C_MASTER_ERR_ADDR_ACK)
    {
#ifdef DEBUG
        UARTprintf("MPU9250 not on the bus.\n");
#endif
    }

    else
    {
#ifdef DEBUG
        UARTprintf("MPU9250 error: UNKNOWN\n");
#endif
    }

}

/*
 void MPU9250_SingleRead(uint8_t regAddr, uint8_t *dataToSend)
 {

 uint32_t command;
 uint32_t errorStatus;
 UARTprintf("Reading MPU9250: Register %X\n", regAddr);

 // Single Byte read cycle consists of two phases:

 // Phase 1  :
 // MSTR     :   |   S   |   SA+W    |       |   RegAddr |       |   << Phase 2  >>
 // Slave    :   |       |           |   ACK |           |   ACK |   << Phase 2  >>

 // Phase 2  :
 // MSTR     :   |   S   |   SA+R    |       |           |   NACK|   P   |
 // Slave    :   |       |           |   ACK |   DATA    |       |       |

 // Phase 1 Start

 // Step 1: SA+W
 // set I2C0 slave address with Write bit
 I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
 false);         // false -> write to slave;
 // true -> read from slave

 // Step 2: Set RA (RegAddr from which the data will be read)
 // put address of register to be read from the slave
 I2CMasterDataPut(I2C0_BASE, regAddr);

 // send register address and read bit
 // Current Master State : Master IDLE
 // Next State           : Master Transmit (To transmit the SA+R signal)
 // Signal to send: Write    :   R/S : 0
 // Desired state: Multiple transmit     : RUN + START

 command = I2C_RUN | I2C_START;
 I2CMasterControl(I2C0_BASE, command); // enable master and initiate read cycle

 // wait until Master is not busy (wait for acknowledgment)
 while (I2CMasterBusy(I2C0_BASE))
 {

 }

 // check if Slave address acknowledged
 errorStatus = I2CMasterErr(I2C0_BASE);

 if (errorStatus != I2C_MASTER_ERR_ADDR_ACK)
 {
 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 1 : Success  : Chip on the bus\n");

 // check if Reg Addr was sent successfully
 if (errorStatus != I2C_MASTER_ERR_DATA_ACK)
 {
 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 1 : Success :     Reg :   %X  : Acknoledged  \n",
 regAddr);
 // Phase 1 complete.

 // Phase 2 Start

 // Step 1: Set up SA+R
 I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
 true);         // false -> write to slave;
 // true -> read from slave

 // Step 2: Send read command to SA + Start signal
 command = 0x03;
 I2CMasterControl(I2C0_BASE, command); // enable master and send second START signal

 // wait until Master is not busy (wait for acknowledgment)
 while (I2CMasterBusy(I2C0_BASE))
 {

 }

 // Get error status
 errorStatus = I2CMasterErr(I2C0_BASE);

 // proceed only if no error has occurred
 if (errorStatus == I2C_MASTER_ERR_NONE)
 {

 // wait if master bus is busy (receiver is putting the data)

 while (I2CMasterBusy(I2C0_BASE))
 {

 }

 // Send STOP bit
 command = 0x04;
 I2CMasterControl(I2C0_BASE, command); // Send STOP condition + NACK

 while (I2CMasterBusy(I2C0_BASE))
 {

 }

 // Get the data
 uint32_t dataReceived;
 dataReceived = I2CMasterDataGet(I2C0_BASE);

 // Send the data
 *dataToSend = dataReceived;

 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 2 : Success :     Reg :   %X  : Data Received :   %X  \n",
 regAddr, dataReceived);

 }

 else
 {
 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 2 : Error: SA+R not acknowledged\n");
 }
 }
 else
 {
 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 1 : Error :     Reg :   %X  : Not Acknowledged  \n",
 regAddr);
 }
 }

 else
 {
 UARTprintf(
 "MPU9250 :   Single Byte Read Cycle  :   Phase 1 : Error: Chip not on the bus\n");
 }

 return;

 }
 */

/* This is not optimized funciton.
 * More optimized funciton is implemented in this file.
 */

/*
 void MPU9250_WriteOld(uint8_t regAddr, uint8_t dataToWr)
 {

 uint32_t errorStatus;
 UARTprintf("Writing MPU9250: Register %X with Data %X\n", regAddr,
 dataToWr);

 // append write command at the end of slave address
 // (handled in I2CMasterSlaveAddrSet())
 //slaveAddr = (slaveAddr << 1) | 0x00 ;       // 0x00 -> write;
 // 0x01 -> read.

 // set I2C0 slave address

 I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
 false);         // false -> write to slave;
 // true -> read from slave

 // put address of register to be written into slave

 I2CMasterDataPut(I2C0_BASE, regAddr);

 // send register address

 I2CMasterControl(I2C0_BASE,
 I2C_MASTER_CMD_SINGLE_SEND);

 // wait until Master is not busy
 while (I2CMasterBusy(I2C0_BASE))
 {

 }

 errorStatus = I2CMasterErr(I2C0_BASE);

 if (errorStatus == I2C_MASTER_ERR_NONE)
 {
 UARTprintf("MPU9250 on the bus. \n", regAddr, dataToWr);

 // put data to be transmitted to slave

 I2CMasterDataPut(I2C0_BASE, dataToWr);

 // send data

 I2CMasterControl(I2C0_BASE,
 I2C_MASTER_CMD_SINGLE_SEND);

 while (I2CMasterBusBusy(I2C0_BASE))
 {

 }

 errorStatus = I2CMasterErr(I2C0_BASE);

 if (errorStatus == I2C_MASTER_ERR_DATA_ACK)
 {
 UARTprintf("MPU9250 on the bus, but data error.\n");
 }
 else
 {
 UARTprintf("MPU9250 written: Register %X with Data: %X\n", regAddr,
 dataToWr);
 }
 }

 else if (errorStatus == I2C_MASTER_ERR_ADDR_ACK)
 {
 UARTprintf("MPU9250 not on the bus.\n");
 }

 else
 {
 UARTprintf("MPU9250 error: UNKNOWN\n");
 }

 }

*/


