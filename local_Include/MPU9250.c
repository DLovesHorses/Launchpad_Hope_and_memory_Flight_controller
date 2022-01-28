#include "MPU9250.h"
#include "i2c.h"

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
    MPU9250_WriteNew(CONFIG, 0x00);
    MPU9250_WriteNew(GYRO_CONFIG, 0x18);
    MPU9250_WriteNew(ACCEL_CONFIG, 0x00);
    MPU9250_WriteNew(SMPLRT_DIV, 0x07);
    MPU9250_WriteNew(INT_ENABLE, 0x01);
    MPU9250_WriteNew(PWR_MGMT_1, 0x01);





    uint8_t dataToRec;

    UARTprintf("WHO_AM_I command sending...\n");
    I2C_ReadByte(MPU9250_SA, WHO_AM_I, &dataToRec);

    UARTprintf("WHO_AM_I command response: %X\n", dataToRec);



}

void MPU9250_MultiByteRead(char regAddr, int byteCount, char *data)
{
    char error;
    int slaveAddr = MPU9250_SA;
    uint32_t ui32Base = I2C0_BASE;

    if (byteCount <= 0)
        return; /* no read was performed */

    /* send slave address and starting address */
    HWREG(ui32Base + I2C_O_MSA) = slaveAddr << 1;
    HWREG(ui32Base + I2C_O_MDR) = regAddr;
    HWREG(ui32Base + I2C_O_MCS) = 0x03; /* S-(saddr+w)-ACK-maddr-ACK */

    // wait until Master is not busy
    while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
    {

    }

    /* change bus from write to read, send restart with slave addr */
    HWREG(ui32Base + I2C_O_MSA) = (slaveAddr << 1) + 1; /* restart: -R-(saddr+r)-ACK */

    if (byteCount == 1) /* if last byte, don't ack */
        HWREG(ui32Base + I2C_O_MCS) = 0x03; /* -data-NACK-P */
    else
        /* else ack */
        HWREG(ui32Base + I2C_O_MCS) = 0x0B; /* -data-ACK- */

    // wait until Master is not busy
    while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
    {

    }

    *data++ = HWREG(ui32Base + I2C_O_MDR); /* store the data received */

    if (--byteCount == 0) /* if single byte read, done */
    {
        while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSBSY)
        {

        }; /* wait until bus is not busy */
        return; /* no error */
    }

    /* read the rest of the bytes */
    while (byteCount > 1)
    {
        HWREG(ui32Base + I2C_O_MCS) = 0x09; /* -data-ACK- */
        // wait until Bus is not busy
        while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
        {

        }

        byteCount--;
        *data++ = HWREG(ui32Base + I2C_O_MDR); /* store data received */
    }

    HWREG(ui32Base + I2C_O_MCS) = 0x04; /* -data-NACK-P */
    // wait until Master is not busy
    while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
    {

    }

    *data = HWREG(ui32Base + I2C_O_MDR); /* store data received */
    // wait until Bus is not busy
    while (HWREG(ui32Base + I2C_O_MCS) & I2C_MCS_BUSY)
    {

    } /* wait until bus is not busy */

    return; /* no error */
}

void MPU9250_ReadReg(uint8_t regAddr, uint8_t *dataToSend)
{
    uint32_t command;

    // Master is in IDLE state.
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA,
    false);         // false -> write to slave;
                    // true -> read from slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

    command = (I2C_START | I2C_STOP);
    I2CMasterControl(I2C0_BASE, command); /* S-(SA+W)-ACK-RegAddr-ACK */

    // wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // Master is in TRANSMIT state now
    I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA, true); // false -> write to slave;
                                                        // true -> read from slave

    // Send a repeated Start condition and transition master to RECEIVE state.
    command = (I2C_START | I2C_STOP);
    I2CMasterControl(I2C0_BASE, command); /* -*S-(SA+R)-ACK-DATA-NACK */

    // wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // Master has RECEIVED the data. Send STOP signal.
    // Transition master into IDLE state.

    I2CMasterSlaveAddrSet(I2C0_BASE, MPU9250_SA, true); // false -> write to slave;
                                                        // true -> read from slave
    command = I2C_STOP;
    I2CMasterControl(I2C0_BASE, command); /*  -P  */

    // wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // Save data if no error occurred.
    if (I2CMasterErr(I2C0_BASE) == I2C_MASTER_ERR_NONE)
    {
        *dataToSend = I2CMasterDataGet(I2C0_BASE);
        UARTprintf("MPU9250 register: %d contains %x.\n", regAddr,
                   I2CMasterDataGet(I2C0_BASE));
    }
    else
    {
        UARTprintf("MPU9250 register: %x cannot be read\n", regAddr);
    }

    return;

}

void MPU9250_WriteNew(uint8_t regAddr, uint8_t dataToWr)
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
    I2C_START | I2C_RUN);

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
        I2C_STOP | I2C_RUN);

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
// For Debug only
void PCF8574A_Init(void)
{
    uint8_t uiData;

// Configure LEDs and switches
    uiData = 0xE7;  // default

    uiData = 0x47;  //0xFF & 0x07;
    PCF8574A_Write( PCF8574A_SA, uiData);

    return;
}

void PCF8574A_Write(uint8_t slaveAddr, uint8_t dataToWr)
{

// Write slave address and R/W = 0
    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
// true -> read from slave

// check if i2c Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {
// wait if bus is busy
    }

// Write data to transmit register
    I2CMasterDataPut(I2C0_BASE, dataToWr);
//HWREG( I2C0_BASE + I2C_O_MDR ) = dataToWr;

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

// Wait until the controller is no longer busy
// wait until Master is not busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    UARTprintf("PCF written\n");

    return;
}

void PCF8574A_Read(uint8_t slaveAddr, uint32_t *puiData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    true);         // false -> write to slave;
// true -> read from slave

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_RECEIVE);

// Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

// Read data from receive register
    uint32_t receivedData;
    receivedData = I2CMasterDataGet(I2C0_BASE);

    *puiData = receivedData;

    return;
}

void MPU9250_Read(uint8_t slaveAddr, uint32_t regAddr, uint8_t *puiData)
{

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    false);         // false -> write to slave;
// true -> read from slave

    I2CMasterDataPut(I2C0_BASE, regAddr);

// Initiate I2C transaction
    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_SEND);

// Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    I2CMasterSlaveAddrSet(I2C0_BASE, slaveAddr,
    true);  // false -> write to slave;
            // true -> read from slave

    I2CMasterControl(I2C0_BASE,
    I2C_MASTER_CMD_SINGLE_RECEIVE);

    // Wait until the controller is no longer busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

// Read data from receive register
    uint32_t receivedData;
    receivedData = I2CMasterDataGet(I2C0_BASE);

    *puiData = receivedData;

    return;
}
