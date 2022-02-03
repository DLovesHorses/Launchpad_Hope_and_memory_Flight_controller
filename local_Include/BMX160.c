/*
 * File         :   BMX160.c
 *
 * Description  :   Custom library for BMX160.
 *                  Provides functionality for:
 *
 *                  Initialization,
 *                  Power management,
 *                  interrupt management,
 *                  configuration, and
 *                  data access methods for
 *
 *                  Accelerometer,
 *                  Gyrometer,
 *                  Magnetometer.
 *
 *
 *
 * Written By   :   The one and only D!
 * Date         :   2022-01-29
 */

// Includes
#include "BMX160.h"

// global variables and externs

const uint8_t int_mask_lookup_table[13] = {
BMX160_INT1_SLOPE_MASK,
                                            BMX160_INT1_SLOPE_MASK,
                                            BMX160_INT2_LOW_STEP_DETECT_MASK,
                                            BMX160_INT1_DOUBLE_TAP_MASK,
                                            BMX160_INT1_SINGLE_TAP_MASK,
                                            BMX160_INT1_ORIENT_MASK,
                                            BMX160_INT1_FLAT_MASK,
                                            BMX160_INT1_HIGH_G_MASK,
                                            BMX160_INT1_LOW_G_MASK,
                                            BMX160_INT1_NO_MOTION_MASK,
                                            BMX160_INT2_DATA_READY_MASK,
                                            BMX160_INT2_FIFO_FULL_MASK,
                                            BMX160_INT2_FIFO_WM_MASK };

// Variables

#define ACCEL_SENSITIVITY_SELECT    BMX160_ACCEL_MG_LSB_2G
#define GYRO_SENSITIVITY_SELECT     BMX160_GYRO_SENSITIVITY_125DPS

float accelRange = ACCEL_SENSITIVITY_SELECT; // * 9.8;
float gyroRange = GYRO_SENSITIVITY_SELECT;

sBmx160Dev_t *Obmx160;

sBmx160SensorData_t *Omagn;
sBmx160SensorData_t *Oaccel;
sBmx160SensorData_t *Ogyro;

// Function definitions.

void BMX160_Init(void)
{
#ifdef DEBUG
    UARTprintf("Initializing BMX160.\n");
#endif

    Obmx160 = (sBmx160Dev_t*) malloc(sizeof(sBmx160Dev_t));
    Oaccel = (sBmx160SensorData_t*) malloc(sizeof(sBmx160SensorData_t));
    Ogyro = (sBmx160SensorData_t*) malloc(sizeof(sBmx160SensorData_t));
    Omagn = (sBmx160SensorData_t*) malloc(sizeof(sBmx160SensorData_t));
    if ((Obmx160 == NULL) || (Oaccel == NULL) || (Ogyro == NULL)
            || (Omagn == NULL))
    {
#ifdef DEBUG
        UARTprintf("Cannot allocate enough space for data to store.\n");
#endif
    }
    else
    {

        if (BMX160_begin() == false)
        {
#ifdef DEBUG
            UARTprintf("BMX160 cannot be initializad\n");
#endif
        }
        else
        {
#ifdef DEBUG
            UARTprintf("BMX160 Initialized...\n");
#endif
        }
    }
}

bool BMX160_scan(void)
{
    // Call Bruteforcer.

    uint8_t knownReg = BMX160_CHIP_ID_ADDR;
    uint8_t knownValue = BMX160_CHIP_ID;

    if (I2C_AddressBruteForcer(knownReg, knownValue) == 1)
    {
        return true;
    }
    return false;
}

bool BMX160_begin(void)
{

#ifdef DEBUG
    UARTprintf("Scanning for BMX160 on the bus.\n");
#endif

    if (BMX160_scan() == true)
    {

#ifdef DEBUG
        UARTprintf("BMX160 is on the bus.\n");
#endif
        // same logic as BMX160_wakeUp
        // perform a soft-reset (without overhead of POR)
        BMX160_softReset();

        // wait for sometime
        SYSTICK_Delay(BMX160_SOFT_RESET_DELAY_MS);

        /* Set acc to normal mode */
        // Send command acc_set_pmu_mode: 0b0001 00nn
        // nn = 00->suspend;
        //      01->normal;
        //      10->low power
        BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
        BMX160_ACC_PMU_MODE_NORMAL_CMD);

        // Delay for atleast 3.8 ms after setting accelerometer power mode
        SYSTICK_Delay(BMX160_ACC_DELAY_MS);

        /* Set gyro to normal mode */
        // Send command gyr_set_pmu_mode: 0b0001 01nn
        // nn = 00  ->  suspend;
        //      01  ->  normal;
        //      10  ->  reserved
        //      11  ->  Fast Start-Up
        BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
        BMX160_GYRO_PMU_MODE_NORMAL_CMD);

        // Delay for atleast 80 ms after setting gyro. power mode
        SYSTICK_Delay(BMX160_GYRO_DELAY_MS);

        /* Set mag INTERFACE to normal mode */
        // Send command mag_if_set_pmu_mode: 0b0001 10nn
        // nn = 00  ->  suspend;
        //      01  ->  normal;
        //      10  ->  low power
        //      11  ->  not used
        BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
        BMX160_MAG_PMU_MODE_NORMAL_CMD);

        // Delay for atleast 0.5 ms after setting mag. power mode
        SYSTICK_Delay(BMX160_MAG_DELAY_MS);

        BMX160_setMagnConf();
        return true;
    }
    else
#ifdef DEBUG
        UARTprintf("BMX160 cannot be located on the bus.\n");
#endif
    return false;
}

void BMX160_setLowPower(void)
{
    //perform soft-reset and wait for sometime.
    BMX160_softReset();
    SYSTICK_Delay(BMX160_SOFT_RESET_DELAY_MS);

    // Configure Mag. meter registers (direct access).
    BMX160_setMagnConf();
    SYSTICK_Delay(BMX160_MAG_DELAY_MS); // wait for atleast 1 ms ( param: tw_up,m ; pg.10 of datasheet)

    /* Set acc to Low-Power mode */
    // Send command acc_set_pmu_mode: 0b0001 00nn
    // nn = 00->suspend;
    //      01->normal;
    //      10->low power
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACC_PMU_MODE_LOW__CMD);
    SYSTICK_Delay(BMX160_ACC_DELAY_MS);

    /* Set gyro to fast-startup mode */
    // Send command gyr_set_pmu_mode: 0b0001 01nn
    // nn = 00  ->  suspend;
    //      01  ->  normal;
    //      10  ->  reserved
    //      11  ->  Fast Start-Up
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
    BMX160_GYRO_PMU_MODE_FAST_STARTUP_CMD);
    SYSTICK_Delay(BMX160_GYRO_DELAY_MS);

    /* Set mag INTERFACE to Low-Power mode */
    // Send command mag_if_set_pmu_mode: 0b0001 10nn
    // nn = 00  ->  suspend;
    //      01  ->  normal;
    //      10  ->  low power
    //      11  ->  not used
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_MAG_PMU_MODE_LOW__CMD); // library error: It was : 0x1B, it should be 0x1A.
    SYSTICK_Delay(BMX160_MAG_DELAY_MS);
}

void BMX160_wakeUp(void)
{
    BMX160_softReset();
    SYSTICK_Delay(100);
    BMX160_setMagnConf();
    SYSTICK_Delay(100);

    /* Set acc to normal mode */
    // Send command acc_set_pmu_mode: 0b0001 00nn
    // nn = 00->suspend;
    //      01->normal;
    //      10->low power
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACC_PMU_MODE_NORMAL_CMD);
    SYSTICK_Delay(BMX160_ACC_DELAY_MS);

    /* Set gyro to Normal mode */
    // Send command gyr_set_pmu_mode: 0b0001 01nn
    // nn = 00  ->  suspend;
    //      01  ->  normal;
    //      10  ->  reserved
    //      11  ->  Fast Start-Up
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
    BMX160_GYRO_PMU_MODE_NORMAL_CMD);
    SYSTICK_Delay(BMX160_GYRO_DELAY_MS);

    /* Set mag INTERFACE to Normal mode */
    // Send command mag_if_set_pmu_mode: 0b0001 10nn
    // nn = 00  ->  suspend;
    //      01  ->  normal;
    //      10  ->  low power
    //      11  ->  not used
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACC_PMU_MODE_NORMAL_CMD);
    SYSTICK_Delay(BMX160_MAG_DELAY_MS);
}

bool BMX160_softReset(void)
{

#ifdef DEBUG
    UARTprintf("BMX160 : softReset IN \n");
#endif

    int8_t rslt = BMX160_OK;
    if (Obmx160 == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }
    rslt = BMX160_softResetWorker(Obmx160);
    if (rslt == 0)
    {
#ifdef DEBUG
        UARTprintf("BMX160 : softReset OK \n");
#endif
        return true;
    }
    else
    {
#ifdef DEBUG
        UARTprintf("BMX160 : softReset FAIL \n");
#endif
    }
    return false;
}

int8_t BMX160_softResetWorker(sBmx160Dev_t *dev)
{
    int8_t rslt = BMX160_OK;

    if (dev == NULL)
    {
        rslt = BMX160_E_NULL_PTR;
    }

    // Send command softreset to command register: 0x7E <- 0xB6
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_SOFT_RESET_CMD);
    SYSTICK_Delay(BMX160_SOFT_RESET_DELAY_MS);

    if (rslt == BMX160_OK)
    {
        BMX160_defaultParamSettg(dev);
    }
    return rslt;
}

void BMX160_defaultParamSettg(sBmx160Dev_t *dev)
{
    // Initializing accel and gyro params with
    dev->gyroCfg.bw = BMX160_GYRO_BW_NORMAL_MODE;
    dev->gyroCfg.odr = BMX160_GYRO_ODR_100HZ;
    dev->gyroCfg.power = BMX160_GYRO_SUSPEND_MODE;
    dev->gyroCfg.range = BMX160_GYRO_RANGE_125_DPS;        //Gyro range: 125 DPS  ; Sensitivity: BMX160_GYRO_SENSITIVITY_125DPS

    dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
    dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
    dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
    dev->accelCfg.range = BMX160_ACCEL_RANGE_2G;            //Accel range: 2G       ; Sensitivity : BMX160_ACCEL_MG_LSB_2G

    dev->prevMagnCfg = dev->magnCfg;
    dev->prevGyroCfg = dev->gyroCfg;
    dev->prevAccelCfg = dev->accelCfg;
}

void BMX160_setMagnConf(void)
{
#ifdef DEBUG
    UARTprintf("BMX160: Configuring Magnetometer...\n");
#endif
    // NOTE!: BMX160 uses BMM150 magnetometer on its dye.
    // the registers of BMM150 can be accessed using Mag. Interface Registers (0x4C - 0x4F).

    // the indirect register map can be found here: https://www.bosch-sensortec.com/media/boschsensortec/downloads/datasheets/bst-bmm150-ds001.pdf
    // this function configures mag. using indirect addressing.
    //
    // see page. 25 of datasheet for example.

    // set magnetometer INTERFACE to setup mode (Manual config. )
    // pg. 23 of datasheet
    BMX160_writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x80);
    SYSTICK_Delay(50);
    // Configure Mag. sensor (not the interface) to Sleep mode
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x01);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4B);

    // set Number of repetation for x/y axis to 9: (1+2(0x04)): (0x51 <- 04)
    // REPXY regular preset (pg. 50 of BMM150 datasheet)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x04);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);

    // set Number of repetation for z axis to 15: (1+1(0x0E)): (0x51 <- 04)
    // REPZ regular preset (pg. 51 of BMM150 datasheet)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x0E);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);

    // setup Mag interfaces to change from setup to data mode.
    // see section 4.2.4 in BMM150 datasheet (pg. 12)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
    BMX160_writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);

    // Set output datarate to 100 Hz.
    // see BMX160 datasheet (pg. 67)
    BMX160_writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x08);

    // Burst read operation of 8 bytes- in Data mode
    BMX160_writeBmxReg(BMX160_MAGN_IF_0_ADDR, 0x03);
    SYSTICK_Delay(50);

#ifdef DEBUG
    UARTprintf("BMX160: Magnetometer configured.\n");
#endif
}

void BMX160_setGyroRange(eGyroRange_t bits)
{
    switch (bits)
    {
    case eGyroRange_125DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_125DPS;
        break;
    case eGyroRange_250DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
        break;
    case eGyroRange_500DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_500DPS;
        break;
    case eGyroRange_1000DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_1000DPS;
        break;
    case eGyroRange_2000DPS:
        gyroRange = BMX160_GYRO_SENSITIVITY_2000DPS;
        break;
    default:
        gyroRange = BMX160_GYRO_SENSITIVITY_250DPS;
        break;
    }
}

void BMX160_setAccelRange(eAccelRange_t bits)
{
    switch (bits)
    {
    case eAccelRange_2G:
        accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
        break;
    case eAccelRange_4G:
        accelRange = BMX160_ACCEL_MG_LSB_4G * 10;
        break;
    case eAccelRange_8G:
        accelRange = BMX160_ACCEL_MG_LSB_8G * 10;
        break;
    case eAccelRange_16G:
        accelRange = BMX160_ACCEL_MG_LSB_16G * 10;
        break;
    default:
        accelRange = BMX160_ACCEL_MG_LSB_2G * 10;
        break;
    }
}

void BMX160_getAllData(sBmx160SensorData_t *magn, sBmx160SensorData_t *gyro,
                       sBmx160SensorData_t *accel, rawData *magRaw, rawData *gyroRaw, rawData *accRaw)
{

    uint8_t data[23] = { 0 };
    int16_t x = 0;
    int16_t y = 0;
    int16_t z = 0;

    BMX160_readReg(BMX160_MAG_DATA_ADDR, data, 23);
    if (magn)
    {
        x = (int16_t) (((uint16_t) data[1] << 8) | data[0]);
        y = (int16_t) (((uint16_t) data[3] << 8) | data[2]);
        z = (int16_t) (((uint16_t) data[5] << 8) | data[4]);
        magn->x = x * BMX160_MAGN_UT_LSB; // resolution: 0.3 uT / LSB (pg.11 of datasheet)
        magn->y = y * BMX160_MAGN_UT_LSB;
        magn->z = z * BMX160_MAGN_UT_LSB;

        magRaw->x = x;
        magRaw->y = y;
        magRaw->z = z;
    }
    if (gyro)
    {
        x = (int16_t) (((uint16_t) data[9] << 8) | data[8]);
        y = (int16_t) (((uint16_t) data[11] << 8) | data[10]);
        z = (int16_t) (((uint16_t) data[13] << 8) | data[12]);
        gyro->x = x * gyroRange;
        gyro->y = y * gyroRange;
        gyro->z = z * gyroRange;

        gyroRaw->x = x;
        gyroRaw->y = y;
        gyroRaw->z = z;
    }
    if (accel)
    {
        x = (int16_t) (((uint16_t) data[15] << 8) | data[14]);
        y = (int16_t) (((uint16_t) data[17] << 8) | data[16]);
        z = (int16_t) (((uint16_t) data[19] << 8) | data[18]);
        accel->x = x * accelRange;
        accel->y = y * accelRange;
        accel->z = z * accelRange;

        accRaw->x = x;
        accRaw->y = y;
        accRaw->z = z;
    }
}

void BMX160_writeBmxReg(uint8_t reg, uint8_t value)
{
#ifdef DEBUG
    UARTprintf("BMX160 : writing register : 0x%x with value: 0x%x\n", reg,
               value);
#endif
    //uint8_t buffer[1] = { value };
    uint8_t _addr = BMX160_SA;
    // Call I2C_writeSingleReg()
    I2C_writeSingleReg(_addr, reg, value);
    //writeReg(reg, buffer, 1);
}

void BMX160_readReg(uint8_t reg, uint8_t *pBuf, uint16_t len)
{
    uint8_t _addr = BMX160_SA;

    // Call I2C_readReg()
    I2C_readReg(_addr, reg, pBuf, len);

}

void BMX160_showData(void)
{
#ifdef DEBUG
    // get data from BMX160;
    sBmx160SensorData_t magData;
    sBmx160SensorData_t gyroData;
    sBmx160SensorData_t accData;

    rawData accRaw;
    rawData gyroRaw;
    rawData magRaw;


    // Get data
    BMX160_getAllData(&magData, &gyroData, &accData, &magRaw, &gyroRaw, &accRaw);


    char printBuffer[252] = "";

    UARTprintf("\n\n Data from BMX160: \n");
    sprintf(printBuffer, "Accel: ( %d, %d, %d ) -> ( %6.3f, %6.3f, %6.3f ).\n\n", accRaw.x, accRaw.y, accRaw.z, accData.x, accData.y, accData.z );
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    /*
    UARTprintf("Accel.  X:  %d\n", (int)accData.x);
    UARTprintf("Accel.  Y:  %d\n", (int)accData.y);
    UARTprintf("Accel.  Z:  %d\n", (int)accData.z);
    */

    sprintf(printBuffer, "Gyro : ( %d, %d, %d ) -> ( %6.3f, %6.3f, %6.3f ).\n\n", gyroRaw.x, gyroRaw.y, gyroRaw.z, gyroData.x, gyroData.y, gyroData.z );
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    /*
    UARTprintf("Gyro.   X:  %d\n", (int)gyroData.x);
    UARTprintf("Gyro.   Y:  %d\n", (int)gyroData.y);
    UARTprintf("Gyro.   Z:  %d\n", (int)gyroData.z);
    */

    sprintf(printBuffer, "Mag   : ( %d, %d, %d ) -> ( %6.3f, %6.3f, %6.3f ).\n\n", magRaw.x, magRaw.y, magRaw.z, magData.x, magData.y, magData.z );
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    /*
    UARTprintf("Mag.   X:  %d\n", (int)magData.x);
    UARTprintf("Mag.   Y:  %d\n", (int)magData.y);
    UARTprintf("Mag.   Z:  %d\n", (int)magData.z);
    */


    // convert the float into integral and fraction part.
 /*   signed int result[18];
    uint16_t precision = 1000;
    float storedValue[9];

    storedValue[0] = accData.x;
    storedValue[1] = accData.y;
    storedValue[2] = accData.z;

    storedValue[3] = gyroData.x;
    storedValue[4] = gyroData.y;
    storedValue[5] = gyroData.z;

    storedValue[6] = magData.x;
    storedValue[7] = magData.y;
    storedValue[8] = magData.z;

    uint8_t count = 0;
    for (count = 0; count < 9; count++)
    {
        static uint8_t resultCount = 0;
        if (storedValue[count] > 0)
        {
            // normal floor operation
            result[resultCount] = (signed int) floor(storedValue[count]);
            resultCount++;
            result[resultCount] = (storedValue[count] - result[resultCount - 1])
                    * precision;
            resultCount++;
        }

        else
        {
            // add one to integral part after flooring
            result[resultCount] = (signed int) floor(storedValue[count]) + 1;
            resultCount++;
            result[resultCount] = ((-1 * storedValue[count])
                    - (-1 * result[resultCount - 1])) * precision;
            resultCount++;
        }
    }

    //result[0] = (signed int) floor(accData.x); // Int. part of mag.x
    //result[1] = (accData.x - result[0]) * precision; // Fract. part of max.x
*/


/*
    for (count = 0; count < 9; count++)
    {
        switch (count)
        {
        case 0:
        {
            UARTprintf("Accelerometer X: %d.%d\n", result[0], result[1]);
            break;
        }
        case 1:
        {
            UARTprintf("Accelerometer Y: %d.%d\n", result[2], result[3]);
            break;
        }
        case 2:
        {
            UARTprintf("Accelerometer Z: %d.%d\n", result[4], result[5]);
            break;
        }
        case 3:
        {
            UARTprintf("Gyrometer X: %d.%d\n", result[6], result[7]);
            break;
        }
        case 4:
        {
            UARTprintf("Gyrometer Y: %d.%d\n", result[8], result[9]);
            break;
        }
        case 5:
        {
            UARTprintf("Gyrometer X: %d.%d\n", result[10], result[11]);
            break;
        }
        case 6:
        {
            UARTprintf("Magnetometer X: %d.%d\n", result[12], result[13]);
            break;
        }
        case 7:
        {
            UARTprintf("Magnetometer Y: %d.%d\n", result[14], result[15]);
            break;
        }
        case 8:
        {
            UARTprintf("Magnetometer Z: %d.%d\n", result[16], result[17]);
            break;
        }

        }
    }
*/
    UARTprintf("\n\n");

#endif

    return;
}
