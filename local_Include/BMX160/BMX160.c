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
#include "local_Include/BMP388/BMP388.h"
#include "local_Include/SystemAttitude/LPF.h"

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

    //uint8_t knownReg = BMX160_CHIP_ID_ADDR;
    //uint8_t knownValue = BMX160_CHIP_ID;

    uint8_t ReceivedChipId = 0;
    I2C_ReadByte(BMX160_SA, BMX160_CHIP_ID_ADDR, &ReceivedChipId);
    //BMP388_user_i2c_read(BMX160_SA, BMX160_CHIP_ID_ADDR, ReceivedChipId, 1);

    //if (I2C_AddressBruteForcer(knownReg, knownValue) == 1)
    if (ReceivedChipId == BMX160_CHIP_ID)
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
    dev->gyroCfg.range = BMX160_GYRO_RANGE_125_DPS; //Gyro range: 125 DPS  ; Sensitivity: BMX160_GYRO_SENSITIVITY_125DPS

    // gyro power mode to normal mode
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR,
    BMX160_GYRO_PMU_MODE_NORMAL_CMD);
    SYSTICK_Delay(BMX160_GYRO_DELAY_MS);

    //gyro filter to Normal mode with 3200 Hz sampling rate
    BMX160_writeBmxReg(BMX160_GYRO_CONFIG_ADDR, 0x2D);

    // gyro range to 125 dps. If this is changed, also change the GYRO_SENSITIVITY_SELECT
    BMX160_writeBmxReg(BMX160_GYRO_RANGE_ADDR, BMX160_GYRO_RANGE_125_DPS);

    dev->accelCfg.bw = BMX160_ACCEL_BW_NORMAL_AVG4;
    dev->accelCfg.odr = BMX160_ACCEL_ODR_100HZ;
    dev->accelCfg.power = BMX160_ACCEL_SUSPEND_MODE;
    dev->accelCfg.range = BMX160_ACCEL_RANGE_2G; //Accel range: 2G       ; Sensitivity : BMX160_ACCEL_MG_LSB_2G

    // accel power mode to normal mode
    BMX160_writeBmxReg(BMX160_COMMAND_REG_ADDR, BMX160_ACC_PMU_MODE_NORMAL_CMD);
    SYSTICK_Delay(BMX160_ACC_DELAY_MS);

    //accel filter to Normal mode (acc_us = 0, acc_bwp = 010) with 1600 Hz sampling rate (acc_odr = 0x0C)
    BMX160_writeBmxReg(BMX160_ACCEL_CONFIG_ADDR, 0x2C);

    // acc range to 2G . If this is changed, also change the ACCEL_SENSITIVITY_SELECT
    BMX160_writeBmxReg(BMX160_ACCEL_RANGE_ADDR, BMX160_ACCEL_RANGE_2G);

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

    // new - switch to active mode, ODR - 20 Hz - (0010 1000)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x28);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);

    // set Number of repetation for x/y axis to 47: (1+2(0x17)): (0x51 <- 17)
    // REPXY regular preset (pg. 13 and 30 of BMM150 datasheet)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x17);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x51);

    // set Number of repetation for z axis to 83: (1+1(0x52)): (0x52 <- 52)
    // REPZ regular preset (pg. 51 of BMM150 datasheet)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x52);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x52);

    // setup Mag interfaces to change from setup to data mode.
    // see section 4.2.4 in BMM150 datasheet (pg. 12)
    BMX160_writeBmxReg(BMX160_MAGN_IF_3_ADDR, 0x02);
    BMX160_writeBmxReg(BMX160_MAGN_IF_2_ADDR, 0x4C);
    BMX160_writeBmxReg(BMX160_MAGN_IF_1_ADDR, 0x42);

    // Set output datarate to 12.5 Hz.
    // see BMX160 datasheet (pg. 67)
    BMX160_writeBmxReg(BMX160_MAGN_CONFIG_ADDR, 0x05);

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
                       sBmx160SensorData_t *accel, rawData *magRaw,
                       rawData *gyroRaw, rawData *accRaw)
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

        // apply LPF on raw values and save it in same variable
        static LOW_PASS_FILTER mag_low_pass[3] = { { .Fc = 53.05f }, // Cut-off freq for X axis
                { .Fc = 53.05f },   // Cut-off freq for Y axis
                { .Fc = 53.05f },   // Cut-off freq for Z axis
                };

        float dt = 1.0f / (5 * 2.0f * PI * mag_low_pass[0].Fc);
        //float dt = 0.000600f;
        magn->x_LPF = applyLPF(&mag_low_pass[0], magRaw->x, dt) * BMX160_MAGN_UT_LSB; // filtered magnetic field X_axis

        dt = 1.0f / (5 * 2.0f * PI * mag_low_pass[1].Fc);
        magn->y_LPF = applyLPF(&mag_low_pass[1], magRaw->y, dt) * BMX160_MAGN_UT_LSB; // filtered magnetic field X_axis

        dt = 1.0f / (5 * 2.0f * PI * mag_low_pass[2].Fc);
        magn->z_LPF = applyLPF(&mag_low_pass[2], magRaw->z, dt) * BMX160_MAGN_UT_LSB; // filtered magnetic field X_axis

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

        // apply LPF on raw values and save it in same variable
        static LOW_PASS_FILTER gyro_low_pass[3] = { { .Fc = 53.05f }, // Cut-off freq for X axis
                { .Fc = 53.05f },   // Cut-off freq for Y axis
                { .Fc = 53.05f },   // Cut-off freq for Z axis
                };

        float dt = 1.0f / (5 * 2.0f * PI * gyro_low_pass[0].Fc);
        //float dt = 0.000600f;
        gyro->x_LPF = applyLPF(&gyro_low_pass[0], gyroRaw->x, dt) * gyroRange; // filtered rotational acceleration X_axis

        dt = 1.0f / (5 * 2.0f * PI * gyro_low_pass[1].Fc);
        gyro->y_LPF = applyLPF(&gyro_low_pass[1], gyroRaw->y, dt) * gyroRange; // filtered rotational acceleration X_axis

        dt = 1.0f / (5 * 2.0f * PI * gyro_low_pass[2].Fc);
        gyro->z_LPF = applyLPF(&gyro_low_pass[2], gyroRaw->z, dt) * gyroRange; // filtered rotational acceleration X_axis

    }
    if (accel)
    {
        x = (int16_t) (((uint16_t) data[15] << 8) | data[14]);      // raw X
        y = (int16_t) (((uint16_t) data[17] << 8) | data[16]);      // raw Y
        z = (int16_t) (((uint16_t) data[19] << 8) | data[18]);      // raw Z

        accel->x = x * accelRange;  // unfiltered converted X
        accel->y = y * accelRange;  // unfiltered converted Y
        accel->z = z * accelRange;  // unfiltered converted Z

        accRaw->x = x;
        accRaw->y = y;
        accRaw->z = z;

        // apply LPF on raw values and save it in same variable
        static LOW_PASS_FILTER acc_low_pass[3] = { { .Fc = 53.05f }, // Cut-off freq for X axis
                { .Fc = 53.05f },   // Cut-off freq for Y axis
                { .Fc = 53.05f },   // Cut-off freq for Z axis
                };

        float dt = 1.0f / (5 * 2.0f * PI * acc_low_pass[0].Fc);
        //float dt = 0.000600f;
        accel->x_LPF = applyLPF(&acc_low_pass[0], accRaw->x, dt) * accelRange; // filtered acceleration X_axis

        dt = 1.0f / (5 * 2.0f * PI * acc_low_pass[1].Fc);
        accel->y_LPF = applyLPF(&acc_low_pass[1], accRaw->y, dt) * accelRange; // filtered acceleration X_axis

        dt = 1.0f / (5 * 2.0f * PI * acc_low_pass[2].Fc);
        accel->z_LPF = applyLPF(&acc_low_pass[2], accRaw->z, dt) * accelRange; // filtered acceleration X_axis

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
    BMX160_getAllData(&magData, &gyroData, &accData, &magRaw, &gyroRaw,
                      &accRaw);

    char printBuffer[252] = "";

    // all data
    UARTprintf("\n\nData from BMX160: \n\n");
    UARTprintf(
            "-Device-   -Raw Data (Field Measurement)-   -Unfiltered calculated data- \t -Filtered Data-\n\n");
    sprintf(printBuffer,
            "Accel: ( %5d, %5d, %5d ) -> \t ( %6.3f, %6.3f, %6.3f ) \t -> \t ( %6.3f, %6.3f, %6.3f ) \n",
            accRaw.x, accRaw.y, accRaw.z, accData.x, accData.y, accData.z,
            accData.x_LPF, accData.y_LPF, accData.z_LPF);
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    sprintf(printBuffer,
            "Gyro : ( %5d, %5d, %5d ) -> \t ( %6.3f, %6.3f, %6.3f ) \t -> \t ( %6.3f, %6.3f, %6.3f ) \n",
            gyroRaw.x, gyroRaw.y, gyroRaw.z, gyroData.x, gyroData.y, gyroData.z,
            gyroData.x_LPF, gyroData.y_LPF, gyroData.z_LPF);
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    sprintf(printBuffer,
            "Mag  : ( %5d, %5d, %5d ) -> \t ( %6.3f, %6.3f, %6.3f ) -> \t ( %6.3f, %6.3f, %6.3f ) \n",
            magRaw.x, magRaw.y, magRaw.z, magData.x, magData.y, magData.z,
            magData.x_LPF, magData.y_LPF, magData.z_LPF);
    UARTprintf("%s", printBuffer);
    printBuffer[0] = '\0';

    UARTprintf("\n\n");

    /*// Raw and filtered
     UARTprintf("\n\nData from BMX160: \n\n");
     sprintf(printBuffer, "Accel: ( %6.3f, %6.3f, %6.3f ) \t -> \t ( %6.3f, %6.3f, %6.3f ).\n", accData.x, accData.y, accData.z, accData.x_LPF, accData.y_LPF, accData.z_LPF );
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';


     sprintf(printBuffer, "Gyro : ( %6.3f, %6.3f, %6.3f ) \t -> \t ( %6.3f, %6.3f, %6.3f ).\n", gyroData.x, gyroData.y, gyroData.z, gyroData.x_LPF, gyroData.y_LPF, gyroData.z_LPF );
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';



     sprintf(printBuffer, "Mag  : ( %6.3f, %6.3f, %6.3f ) \t -> \t ( %6.3f, %6.3f, %6.3f ).\n\n", magData.x, magData.y, magData.z, magData.x_LPF, magData.y_LPF, magData.z_LPF );
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';

     UARTprintf("\n\n");

     */

    /*
     // just acc
     sprintf(printBuffer, "( %6.3f, %6.3f, %6.3f ).\n", accData.x, accData.y,
     accData.z);
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';
     */

    /*
     // just gyro
     sprintf(printBuffer, "( %6.3f, %6.3f, %6.3f ).\n", gyroData.x, gyroData.y, gyroData.z );
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';
     */

    /*
     // just mag

     float heading_angle = 90 - (atan(magData.y/ magData.x) * 180 / (2 * acos(0.0)));
     sprintf(printBuffer, "( %6.3f, %6.3f, %6.3f ) -> %6.3f .\n", magData.x, magData.y, magData.z, heading_angle );
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';
     */

    /*
     // calculate heading angle

     float heading_angle = 90 - (atan(magData.y/ magData.x) * 180 / (2 * acos(0.0)));
     sprintf(printBuffer, "%3.1f\n", heading_angle);
     UARTprintf("%s", printBuffer);
     printBuffer[0] = '\0';
     */

#endif

    return;
}
