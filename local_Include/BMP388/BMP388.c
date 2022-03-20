/*
 * File         :   BMP388.c
 *
 * Description  :   This file contains the implementation of drivers
 *                  to interface BMP388 Pressure + Temperature Sensor
 *                  to TM4C123GH6PM.
 *
 * Written By   :   The one and only D!
 * Date         :   2022-02-04
 */

// Includes
#include "BMP388.h"
#include "local_include/BUZZER/buzzer.h"
#include "local_Include/led.h"
#include "local_Include/uart.h"
#include "local_Include/i2c.h"
#include "local_Include/SysTick.h"
#include "local_Include/SysFlag.h"
#include "local_Include/SystemAttitude/LPF.h"

// global variables and externs
struct bmp3_dev dev;
uint8_t BMP388_addr;

float base_altitude = 0.0f;
bool  BMP388_state = NOT_INITIALIZED;

/*
 * settings to configure:
 *
 * dev.settings.op_mode                         // Power mode which user wants to set
 * dev.settings.press_en                        // Enable pressure sensor?
 * dev.settings.temp_en                         // Enable Temperature sensor?
 *
 * dev.settings.odr_filter.press_os             // Over sampling for Pressure sensor
 * dev.settings.odr_filter.temp_os              // Over sampling for Temperature sensor
 * dev.settings.odr_filter.iir_filter           // Filter coefficient for IIR filter
 * dev.settings.odr_filter.odr                  // Output data rate (subdivision selection) (pg.37)
 *
 * dev.settings.int_settings.output_mode        // Configure output: 1 -> open-drain  , 0 -> push-pull
 * dev.settings.int_settings.level              // level of INT pin: 1 -> active_high , 0 -> active_low
 * dev.settings.int_settings.latch              // Latching of interrupts for
 *                                              // INT pin and INT_STATUS register: 1 -> enabled     , 0 -> disabled
 *
 * dev.settings.int_settings.drdy_en            // enable temperature / pressure
 *                                              // data ready interrupt for INT pin and INT_STATUS
 *
 * dev.settings.adv_settings.i2c_wdt_en         // i2c watch dog enable
 * dev.settings.adv_settings.i2c_wdt_sel        // i2c watch dog select
 *
 *
 *
 */

// Function definitions.
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_Int_Handler(void)
{
    // read the interrupt status register

    int8_t rslt = BMP3_E_NULL_PTR;
    uint8_t status;   // slave address
    rslt = BMP388_get_regs(BMP3_INT_STATUS_REG_ADDR, &status, 1);

    /* Proceed if everything is fine until now */
    if (rslt != BMP3_OK)
    {

#ifdef DEBUG
        UARTprintf("FAILED: Reading INT_STATUS reg from BMP388\n");
#endif

        return;
    }

    // Set DRDY flag is data-ready interrupt is set.
    if (status && BMP3_INT_DRDY_SET)
    {
        SysFlag_Set(BMP388_DRDY_INT);

    }

    return;

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * Configures PC6 as BMP388 Interrupt line.
 *
 *
 */

void BMP388_Int_Configure(void)
{

    // enable GPIO port C

    SysCtlPeripheralEnable(SYSCTL_PERIPH_GPIOC);

    // set-up pin 6 as input

    GPIOPinTypeGPIOInput(GPIO_PORTC_BASE, GPIO_PIN_6);

    // set interrupt type to detect BOTH_EDGES

    GPIOIntTypeSet(GPIO_PORTC_BASE, GPIO_PIN_6, GPIO_HIGH_LEVEL);

    // register BMP388_Int_Handler() as interrupt handler

    GPIOIntRegister(GPIO_PORTC_BASE, BMP388_Int_Handler);

    // enable GPIO interrupts for PC6

    GPIOIntEnable(GPIO_PORTC_BASE, GPIO_INT_PIN_6);

}
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
void BMP388_Init(void)
{

    BMP388_set_i2c_addr(BMP3_I2C_ADDR);

    // initialize the structure.
    // this won't be used to set up a interface with
    // the sensor via Bosche's API.
    // This is just for tracking purposes.

    dev.intf = BMP3_I2C_INTF;
    dev.read = BMP388_user_i2c_read;
    dev.write = BMP388_user_i2c_write;
    dev.delay_ms = BMP388_user_delay_ms;

    // Initialize the structure. Wire.begin();

    // Call begin() function and check the result
    if (BMP388_begin() != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("BMP388 cannot be initializad\n");
#endif
    }
    else
    {

        BMP388_calibrate();
        BMP388_state  = INITIALIZED;
#ifdef DEBUG
        UARTprintf("BMP388 Initialized...\n");

        char buffer[80];
        sprintf(buffer, "Base Altitude: %f", base_altitude);
        UARTprintf("%s\n", buffer);
        BUZZ_BUZZER(BUZZ_DUR_SHORT, BUZZ_REP_TIME_3, BUZZ_PAUSE_TIME_100);
#endif
    }

    return;
}
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_calibrate(void){

    /*
    // take minimum of 100 altitude reading
    // and store it in base altitude variable.

    UARTprintf("BMP388 Calibrating\n");

    float intermediate_result = BMP388_readAltitude();
    uint8_t count = 0;
    float rslt = intermediate_result;
    for (count = 0; count < 100; count++)
    {
        SYSTICK_Delay(100);
        rslt = BMP388_readAltitude();

        intermediate_result =
                (rslt < intermediate_result) ? rslt : intermediate_result;
    }

    base_altitude = intermediate_result;

    UARTprintf("BMP388 Calibrated.\n");
    */



    // exponentially weighted moving average algorithm

    UARTprintf("BMP388 Calibrating\n");

    uint8_t count = 0;
    char buffer[80];


    // apply LPF on raw values and save it in same variable
    static LOW_PASS_FILTER BMP388_calib;
    BMP388_calib.Fc = 53.05f;       // cut-off freq. for altitude change.

    float dt = 1.0f / (5 * 2.0f * PI * BMP388_calib.Fc);

    float actual_Altitude = 0.0f;
    float filtered_Altitude = 0.0f;

    // initialize the previous value so that the exponentially weighted moving average will be more accurate
    actual_Altitude = BMP388_readAltitude();
    BMP388_calib.prevOutput = actual_Altitude; // <- Initialize the previous value here.

    for (count = 0; count < 100; count++)
    {
        SYSTICK_Delay(100);
        actual_Altitude = BMP388_readAltitude();
        filtered_Altitude = applyLPF(&BMP388_calib, actual_Altitude, dt);

#ifdef DEBUG
        sprintf(buffer, "Actual Altitude: \t %f \t  filtered_altitude : \t %f \n", actual_Altitude, filtered_Altitude);
        UARTprintf("%s", buffer);
        buffer[0] = '\0';
#endif

    }

    base_altitude = filtered_Altitude;

    sprintf(buffer, "BMP388 Calibrated. \t Base Altitude : \t %f \n", filtered_Altitude);
    UARTprintf("%s", buffer);
    buffer[0] = '\0';
    return;
}
/*
 *
 *
 *
 *
 *
 *
 */
void BMP388_set_i2c_addr(const uint8_t addr)
{
    BMP388_addr = addr; //= BMP3_I2C_ADDR
}

/*
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_begin(void)
{

    int8_t rslt;
    uint8_t chip_id = 0;

    /* Read the chip-id of bmp3 sensor */
    dev.dev_id = BMP388_addr;   // slave address
    rslt = BMP388_get_regs(BMP3_CHIP_ID_ADDR, &chip_id, 1);

    /* Proceed if everything is fine until now */
    if (rslt != BMP3_OK)
    {

#ifdef DEBUG
        UARTprintf("FAILED: Reading CHIP_ID from BMP388\n");
#endif

        return BMP3_E_GET_REG_DATA_FAILED;
    }

#ifdef DEBUG
    UARTprintf("SUCCESS: Reading CHIP_ID from BMP388\n");
#endif

    /* Check for chip id validity */
    if (chip_id != BMP3_CHIP_ID)
    {
        //dev.chip_id = chip_id;

#ifdef DEBUG
        UARTprintf("FAILED: BMP388 chip not Recognized. %X\n");
#endif
        return BMP3_E_CHIP_NOT_RECOGNIZED;
    }

    dev.chip_id = chip_id;

#ifdef DEBUG
    UARTprintf("SUCCESS: BMP388 Chip Recognized: ID: %X\n", dev.chip_id);
#endif

    // next call is for debug purpose only
    // BMP388_read_all_regs();

    /* Reset the sensor */
    rslt = BMP388_reset();

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Resetting BMP388\n");
#endif
        return BMP3_E_RESET_FAILED;
        /* Read the calibration data */

    }

#ifdef DEBUG
    UARTprintf("SUCCESS: Resetting BMP388 completed.\n");
#endif

// get calibration data
    rslt = BMP388_get_calib_data();

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Getting Calibration data from BMP388\n");
#endif
        return BMP3_E_CALIB_DATA_RET_FAILED;

    }
#ifdef DEBUG
    UARTprintf("SUCCESS: Calibration data acquisition completed.\n");
#endif

    // configure device

    rslt = BMP388_set_config();

    // set-up the interrupt system
    BMP388_Int_Configure();

    // next call is for debug purpose only: to ensure config is properly set
    BMP388_read_all_regs();

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Configuring BMP388\n");
#endif
        return BMP3_E_DEVICE_CONFIG_FAILED;
    }

#ifdef DEBUG
    UARTprintf("SUCCESS: Configuration of BMP388 completed.\n");
#endif

    return rslt;

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
void BMP388_showState(void){

    if( BMP388_state  == INITIALIZED){
        UARTprintf("BMP388 initialized. \n");
    }
    else{
        UARTprintf("BMP388 not initialized. \n");
    }

    return;
}
/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
/* wrong implementation
 int8_t BMP388_set_config(void)
 {
 int8_t rslt;

 //  Used to select the settings user needs to change
 uint16_t settings_sel;

 //  Select the pressure and temperature sensor to be enabled
 dev.settings.press_en = BMP3_ENABLE;
 dev.settings.temp_en = BMP3_ENABLE;

 //  Select the output data rate and oversampling settings for pressure and temperature
 dev.settings.odr_filter.press_os = BMP3_NO_OVERSAMPLING;
 dev.settings.odr_filter.temp_os = BMP3_NO_OVERSAMPLING;
 dev.settings.odr_filter.odr = BMP3_ODR_200_HZ;

 //  Assign the settings which needs to be set in the sensor
 settings_sel = BMP3_PRESS_EN_SEL | BMP3_TEMP_EN_SEL | BMP3_PRESS_OS_SEL
 | BMP3_TEMP_OS_SEL | BMP3_ODR_SEL;
 rslt = BMP388_set_sensor_settings(settings_sel);

 if (rslt != BMP3_OK)
 {
 #ifdef DEBUG
 UARTprintf("FAILED: Setting BMP388 settings.\n");
 #endif
 return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
 }

 #ifdef DEBUG
 UARTprintf("SUCCESS: BMP388 Settings configured properly.\n");
 #endif

 // Set the power mode to normal mode
 dev.settings.op_mode = BMP3_NORMAL_MODE;
 rslt = BMP388_set_op_mode();

 if (rslt != BMP3_OK)
 {
 #ifdef DEBUG
 UARTprintf("FAILED: Setting operation mode of BMP388.\n");
 #endif
 return BMP3_E_SET_OP_MODE_FAILED;
 }

 #ifdef DEBUG
 UARTprintf("SUCCESS: BMP3888 operation mode set to : %x\n",
 dev.settings.op_mode);
 #endif

 return rslt;
 }

 */

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
int8_t BMP388_set_config(void)
{

    int8_t rslt = BMP3_E_NULL_PTR;
    uint8_t reg_addr;

    // write pwr_ctrl reg data
    reg_addr = BMP3_PWR_CTRL_ADDR;
    uint8_t pwr_ctrl_data = 0x33;
    rslt = BMP388_set_regs(&reg_addr, &pwr_ctrl_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Setting Power Control register [0x%x] of BMP388.\n",
                   reg_addr);
#endif
        return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
    }

    // write config reg data
    reg_addr = BMP3_CONFIG_ADDR;
    uint8_t config_data = 0x0E;     // IIR Filter coefficient = 127 (Max Filtering).
    rslt = BMP388_set_regs(&reg_addr, &config_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Setting Config register [0x%x] of BMP388.\n",
                   reg_addr);
#endif
        return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
    }

    // write output sampeling reg (osr) reg data
    reg_addr = BMP3_OSR_ADDR;
    uint8_t osr_data = 0x03;
    rslt = BMP388_set_regs(&reg_addr, &osr_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: OSR register [0x%x] of BMP388.\n", reg_addr);
#endif
        return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
    }

    // write output data rate (odr) reg data
    reg_addr = BMP3_ODR_ADDR;
    uint8_t odr_data = 0x02;
    rslt = BMP388_set_regs(&reg_addr, &odr_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: ODR register [0x%x] of BMP388.\n", reg_addr);
#endif
        return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
    }

    // configure interrupts

    reg_addr = BMP3_INT_CTRL_ADDR;
    uint8_t int_ctrl_data = 0x42;
    rslt = BMP388_set_regs(&reg_addr, &int_ctrl_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: INT_CTRL register [0x%x] of BMP388.\n", reg_addr);
#endif
        return BMP3_E_SET_SENSOR_SETTINGS_FAILED;
    }

    return rslt;

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

float BMP388_readTemperature(void)
{
    int8_t rslt;

    uint8_t sensor_comp;
    sensor_comp = BMP3_TEMP;
    struct bmp3_data data;

    // get the sensor data.
    rslt = BMP388_get_sensor_data(sensor_comp, &data);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Reading temperature from BMP388.\n");
#endif
        return BMP3_E_SENSOR_DATA_READ_FAILED;
    }

    return data.temperature; // return only temperature
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
void BMP388_showData(void)
{
    char charBuffer[80];
    charBuffer[0] = '\0';

    UARTprintf("Data from BMP388: \n");
    /*
     UARTprintf("Data from BMP388: \n\n");
     // data from BMP388_readPressure
     sprintf(charBuffer, "Pressure: \t %7.3f \t (readPressure)\n",
     BMP388_readPressure());
     UARTprintf("%s", charBuffer);
     charBuffer[0] = '\0';

     // data from BMP388_readTemperature
     sprintf(charBuffer, "Temperature: \t %7.3f \t (readTemperature)\n",
     BMP388_readTemperature());
     UARTprintf("%s", charBuffer);
     charBuffer[0] = '\0';

     // data from BMP388_readAltitude
     sprintf(charBuffer, "Alt.: \t \t  %7.3f \t (readAltitude) \t Base Altitude : %f\n",
     BMP388_readAltitude() - base_altitude, base_altitude);
     UARTprintf("%s", charBuffer);
     charBuffer[0] = '\0';

     // data from BMP388_readCalibratedAltitude
     sprintf(charBuffer, "Calib. Alt.: \t %7.3f \t (readCalibratedAltitude)\n",
     BMP388_readCalibratedAltitude(1031.25));
     UARTprintf("%s", charBuffer);
     charBuffer[0] = '\0';

     // data from BMP388_readSeaLevel
     sprintf(charBuffer, "Sea Level: \t %7.3f \t (readSeaLevel)\n\n",
     BMP388_readSeaLevel(102));
     UARTprintf("%s", charBuffer);
     charBuffer[0] = '\0';

     */

    // Display Just Altitude. (and altitude process variable)
    static float prevReading = 0.0f;

    float curReading = BMP388_readAltitude();
    float curBaselineReading = curReading - base_altitude;
    int16_t alt_process_variable = (int16_t) (100 * curBaselineReading);

    float high_limit;
    float low_limit;

    if(prevReading > 0.05f){
        high_limit  = prevReading + 0.05;
        low_limit   = prevReading - 0.05;
    }
    else if(prevReading < -0.05f){
        high_limit  = prevReading + 0.05;
        low_limit   = prevReading - 0.05;
    }
    else{
        high_limit    = 0.05f;
        low_limit     = -0.05f;
    }


    if (curBaselineReading <= low_limit
            || curBaselineReading >= high_limit)
    {
        prevReading = curBaselineReading;
    }


    // Height process variable conditioning.

    int16_t pv_height = 0;

    // lower limit  : 0 cm
    //pv_height = alt_process_variable < 0 ? 0 : alt_process_variable;

    pv_height = alt_process_variable; // Just for debug. Remove it in final production code.
    /*
    if(alt_process_variable >= 0 && alt_process_variable <= 100){
        pv_height = alt_process_variable;
    }
    else{
        if (alt_process_variable < 0){
            pv_height = 0;
        }
        else if(alt_process_variable > 100){
            pv_height = 100;
        }
    }
    */





    sprintf(charBuffer, "\nAlt.(drifted): %4.2f, \t (stable): %4.2f, \t Height: %d cm (PV: %d cm)\n",
            curBaselineReading, prevReading, alt_process_variable, pv_height);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';

    UARTprintf("\n\n");





    return;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

float BMP388_readPressure(void)
{

    int8_t rslt;
    uint8_t sensor_comp;
    sensor_comp = BMP3_PRESS;
    struct bmp3_data data;

    // get the sensor data.
    rslt = BMP388_get_sensor_data(sensor_comp, &data);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Reading pressure from BMP388.\n");
#endif
        return BMP3_E_SENSOR_DATA_READ_FAILED;
    }

    return data.pressure;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_get_regs(uint8_t reg_addr, uint8_t *reg_data, uint16_t len)
{

    int8_t rslt = BMP3_OK;
    //-----I2C-----
    if (dev.intf == BMP3_I2C_INTF)
    {
        rslt = dev.read(dev.dev_id, reg_addr, reg_data, len);

        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf("FAILED: Reading register %x from BMP388.\n", reg_addr);
#endif
            return BMP3_E_REG_READ_FAILED;
        }
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_set_regs(uint8_t *reg_addr, const uint8_t *reg_data, uint8_t len)
{
    int8_t rslt = BMP3_E_NULL_PTR;

    uint8_t dataToWrite = *reg_data;
    // Check for arguments validity
    if ((reg_addr != NULL) && (reg_data != NULL) && (len != 0))
    {

        rslt = dev.write(dev.dev_id, reg_addr[0], &dataToWrite, len);

        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf("FAILED: Writing register(s) %x - %x from BMP388.\n",
                       reg_addr, reg_addr + len);
#endif
            return BMP3_E_REG_WRITE_FAILED;
        }

    }
    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_set_sensor_settings(uint32_t desired_settings)
{
    int8_t rslt = BMP3_E_NULL_PTR;

    // if desired_settings has POWER_CNTL bit enabled
    if (POWER_CNTL & desired_settings)
    {
        /* Set the power control settings */
        rslt = BMP388_set_pwr_ctrl_settings(desired_settings);
        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf("FAILED: Setting power control setting for BMM388\n");
#endif
            return BMP3_E_POWER_CNTL_SET_FAILED;
        }
    }
    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_set_op_mode(void)
{
    int8_t rslt;

    rslt = BMP388_write_power_mode();

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf("FAILED: Setting power mode for BMM388\n");
#endif
        return BMP3_E_POWER_MODE_SET_FAILED;
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_reset(void)
{
    int8_t rslt;

    uint8_t reg_addr = BMP3_CMD_ADDR;
    /* 0xB6 is the soft reset command */
    uint8_t soft_rst_cmd = 0xB6;
    uint8_t cmd_rdy_status;
    uint8_t cmd_err_status;

    /* Check for command ready status (Command decoder is ready to accept a new command*/
    rslt = BMP388_get_regs(BMP3_SENS_STATUS_REG_ADDR, &cmd_rdy_status, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Reading Command Register for Ready status for Reset.\n");
#endif
        return BMP3_E_REG_READ_FAILED;
    }

    /* check if command decoder is ready to accept new command */
    if ((cmd_rdy_status & BMP3_CMD_RDY))
    {
        /* Write the soft reset command in the sensor */
        rslt = BMP388_set_regs(&reg_addr, &soft_rst_cmd, 1);

        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf(
                    "FAILED: Sending sending Soft-Reset command to BMM388.\n");
#endif
            return BMP3_E_REG_WRITE_FAILED;
        }

        /* Proceed if everything is fine until now */

        /* Wait for 2 ms */
        dev.delay_ms(2);

        /* Read for command error status */
        rslt = BMP388_get_regs(BMP3_ERR_REG_ADDR, &cmd_err_status, 1);
        /* check for command error status */
        if ((cmd_err_status & BMP3_CMD_ERR) || (rslt != BMP3_OK))
        {
            /* Command not written hence return
             error */
#ifdef DEBUG
            UARTprintf("FAILED: Command execution for soft-reset failed.\n");
#endif
            rslt = BMP3_E_CMD_EXEC_FAILED;
        }

    }
    else
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Soft-Reset failed because command decoder of BMP388 is busy.\n");
#endif
        rslt = BMP3_E_CMD_EXEC_FAILED;
    }
    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_get_sensor_data(uint8_t sensor_comp, struct bmp3_data *comp_data)
{
    int8_t rslt;

    /* Array to store the pressure and temperature data read from
     the sensor */
    uint8_t reg_data[BMP3_P_T_DATA_LEN] = { 0 };
    struct bmp3_uncomp_data uncomp_data = { 0 };

    if ((comp_data != NULL))
    {

        /* Read the pressure and temperature data from the sensor */
        rslt = BMP388_get_regs(BMP3_DATA_ADDR, reg_data, BMP3_P_T_DATA_LEN);

        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf(
                    "FAILED: Sensor Data register read failed (BMP388_get_sensor_data).\n");
#endif
            return BMP3_E_REG_READ_FAILED;
        }

        /* Parse the read data from the sensor */
        BMP388_parse_sensor_data(reg_data, &uncomp_data);



        /* Compensate the pressure/temperature/both data read
         from the sensor */

        rslt = BMP388_compensate_data(sensor_comp, &uncomp_data, comp_data,
                                      &dev.calib_data);

        if (rslt != BMP3_OK)
        {
#ifdef DEBUG
            UARTprintf(
                    "FAILED: Compensating data failed : (BMP388_get_sensor_data).\n");
#endif
            return BMP3_E_DATA_COMPENSATION_FAILED;
        }

        //debug display for uncompensated (parsed) and compensated from BMP388.
#ifdef DEBUG_DIS

        // Uncompensated Data

        UARTprintf("Uncompensated Data: \t Pressure : \t %d \t Temp. : \t %d \n", uncomp_data.pressure, uncomp_data.temperature);
        UARTprintf("\n\n");

        char charBuffer [80] = {0};
        sprintf(charBuffer, "Compensated   Data: \t Pressure : \t %f \t Temp. : \t %f \n", comp_data->pressure, comp_data->temperature);
        UARTprintf("%s", charBuffer);
        charBuffer[0] = '\0';
        UARTprintf("\n\n");

#endif

    }

    else
    {
        rslt = BMP3_E_NULL_PTR;
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_write_power_mode(void)
{
    int8_t rslt;
    uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
    uint8_t op_mode = dev.settings.op_mode; // see BMP388_set_config().

    /* Temporary variable to store the value read from op-mode register */
    uint8_t op_mode_reg_val;

    /* Read the power mode register */
    rslt = BMP388_get_regs(reg_addr, &op_mode_reg_val, 1);
    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Reading mode from Power Control register (BMP388_write_power_mode).\n");
#endif
        return BMP3_E_REG_READ_FAILED;
    }

    /* Clear the current power mode and
     * Set the desired power mode (op_mode)*/

    op_mode_reg_val = BMP3_SET_BITS(op_mode_reg_val, BMP3_OP_MODE, op_mode);

    /* Write the power mode in the register */
    rslt = BMP388_set_regs(&reg_addr, &op_mode_reg_val, 1);
    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Writing new power mode to Power Control register (BMP388_write_power_mode).\n");
#endif
        return BMP3_E_REG_WRITE_FAILED;
    }
    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_get_calib_data(void)
{
    int8_t rslt = BMP3_E_NULL_PTR;
    uint8_t reg_addr = BMP3_CALIB_DATA_ADDR;

    /* Array to store calibration data */
    uint8_t calib_data[BMP3_CALIB_DATA_LEN] = { 0 };

    /* Read the calibration data from the sensor */

    rslt = BMP388_get_regs(reg_addr, calib_data, BMP3_CALIB_DATA_LEN);
    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Reading mode from Calibration data (%x) from BMP388. (BMP388_get_calib_data).\n",
                reg_addr);
#endif
        return BMP3_E_REG_READ_FAILED;
    }

    /* Parse calibration data and store it in device structure */
    BMP388_parse_calib_data(calib_data);

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_set_pwr_ctrl_settings(uint32_t desired_settings)
{
    int8_t rslt;
    uint8_t reg_addr = BMP3_PWR_CTRL_ADDR;
    uint8_t reg_data = 0;

    rslt = BMP388_get_regs(reg_addr, &reg_data, 1);
    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Reading POWER_CTRL reg (%x) from BMP388. (BMP388_set_pwr_ctrl_settings).\n",
                reg_addr);
#endif
        return BMP3_E_REG_READ_FAILED;
    }

    // if we want to enable pressure sensor
    if (desired_settings & BMP3_PRESS_EN_SEL)
    {
        /* Set the pressure enable settings in the
         register variable without changing any other
         settings inside the POWER_CTRL register*/
        reg_data = BMP3_SET_BITS_POS_0(reg_data, BMP3_PRESS_EN,
                                       dev.settings.press_en);
    }

    // if we want to enable temperature sensor
    if (desired_settings & BMP3_TEMP_EN_SEL)
    {
        /* Set the temperature enable settings in the
         register variable */
        reg_data = BMP3_SET_BITS(reg_data, BMP3_TEMP_EN, dev.settings.temp_en);
    }

    /* Write the power control settings in the register */
    rslt = BMP388_set_regs(&reg_addr, &reg_data, 1);
    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Writing POWER_CTRL reg (%x) from BMP388. (BMP388_set_pwr_ctrl_settings).\n",
                reg_addr);
#endif
        return BMP3_E_REG_WRITE_FAILED;
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_parse_sensor_data(const uint8_t *reg_data,
                              struct bmp3_uncomp_data *uncomp_data)
{
    /* Temporary variables to store the sensor data */
    uint32_t data_xlsb = 0;
    uint32_t data_lsb = 0;
    uint32_t data_msb = 0;

    /* Store the parsed register values for pressure data */
    data_xlsb = (uint32_t) reg_data[0];
    data_lsb = (uint32_t) reg_data[1] << 8;
    data_msb = (uint32_t) reg_data[2] << 16;

    uncomp_data->pressure = data_msb | data_lsb | data_xlsb;

    /* Prepare temp. variables to store temperature data */
    data_xlsb = 0;
    data_lsb = 0;
    data_msb = 0;

    /* Store the parsed register values for temperature data */
    data_xlsb = (uint32_t) reg_data[3];
    data_lsb = (uint32_t) reg_data[4] << 8;
    data_msb = (uint32_t) reg_data[5] << 16;

    uncomp_data->temperature = data_msb | data_lsb | data_xlsb;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_compensate_data(uint8_t sensor_comp,
                              const struct bmp3_uncomp_data *uncomp_data,
                              struct bmp3_data *comp_data,
                              struct bmp3_calib_data *calib_data)
{
    int8_t rslt = BMP3_OK;

    // check for NULL pointer references before continuing.
    if ((uncomp_data == NULL) || (comp_data == NULL) || (calib_data == NULL))
    {

#ifdef DEBUG
        UARTprintf(
                "FAILED: BMP388_compensate_data() called with NULL pointers.\n");
#endif

        return BMP3_E_NULL_PTR;

    }

    /* If pressure or temperature component is selected.
     * We need to compensate temperature before compensating
     * pressure. That is why the following IF statement will
     * get executed if sensor_comp = BMP3_PRESS or sensor_comp = BMP3_TEMP
     *
     * (See datasheet: section 9: Appx. pg. 54
     */

    if (sensor_comp & (BMP3_PRESS | BMP3_TEMP))
    {
        /* Compensate the temperature data */
        comp_data->temperature = BMP388_compensate_temperature(uncomp_data,
                                                               calib_data);
    }

    // compensate pressure only if it needs to be compensated
    if (sensor_comp & BMP3_PRESS)
    {
        /* Compensate the pressure data */
        comp_data->pressure = BMP388_compensate_pressure(uncomp_data,
                                                         calib_data);
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_parse_calib_data(const uint8_t *reg_data)
{
    /* Temporary variable to store the aligned trim data */

    struct bmp3_reg_calib_data *reg_calib_data = &dev.calib_data.reg_calib_data;

    struct bmp3_quantized_calib_data *quantized_calib_data =
            &dev.calib_data.quantized_calib_data;

    /* Temporary variable */
    double temp_var = 0;

    // Calculate PAR_T1 (16-bits)
    temp_var = BMP3_CALIB_PAR_T1_DIVISOR_VALUE;
    reg_calib_data->par_t1 = BMP3_CONCAT_BYTES(reg_data[1], reg_data[0]);
    quantized_calib_data->par_t1 = ((double) reg_calib_data->par_t1 / temp_var);

    // Calculate PAR_T2 (16-bits)
    reg_calib_data->par_t2 = BMP3_CONCAT_BYTES(reg_data[3], reg_data[2]);
    temp_var = BMP3_CALIB_PAR_T2_DIVISOR_VALUE;
    quantized_calib_data->par_t2 = ((double) reg_calib_data->par_t2 / temp_var);

    // Calculate PAR_T3 (8-bits)
    reg_calib_data->par_t3 = (int8_t) reg_data[4];
    temp_var = BMP3_CALIB_PAR_T3_DIVISOR_VALUE;
    quantized_calib_data->par_t3 = ((double) reg_calib_data->par_t3 / temp_var);

    // Calculate PAR_P1 (16-bits)
    reg_calib_data->par_p1 = (int16_t) BMP3_CONCAT_BYTES(reg_data[6],
                                                         reg_data[5]);
    temp_var = BMP3_CALIB_PAR_P1_DIVISOR_VALUE;
    quantized_calib_data->par_p1 = ((double) (reg_calib_data->par_p1 - (16384))
            / temp_var);

    // Calculate PAR_P2 (16-bits)
    reg_calib_data->par_p2 = (int16_t) BMP3_CONCAT_BYTES(reg_data[8],
                                                         reg_data[7]);
    temp_var = BMP3_CALIB_PAR_P2_DIVISOR_VALUE;
    quantized_calib_data->par_p2 = ((double) (reg_calib_data->par_p2 - (16384))
            / temp_var);

    // Calculate PAR_P3 (8-bits)
    reg_calib_data->par_p3 = (int8_t) reg_data[9];
    temp_var = BMP3_CALIB_PAR_P3_DIVISOR_VALUE;
    quantized_calib_data->par_p3 = ((double) reg_calib_data->par_p3 / temp_var);

    // Calculate PAR_P4 (8-bits)
    reg_calib_data->par_p4 = (int8_t) reg_data[10];
    temp_var = BMP3_CALIB_PAR_P4_DIVISOR_VALUE;
    quantized_calib_data->par_p4 = ((double) reg_calib_data->par_p4 / temp_var);

    // Calculate PAR_P5 (16-bits)
    reg_calib_data->par_p5 = BMP3_CONCAT_BYTES(reg_data[12], reg_data[11]);
    temp_var = BMP3_CALIB_PAR_P5_DIVISOR_VALUE;
    quantized_calib_data->par_p5 = ((double) reg_calib_data->par_p5 / temp_var);

    // Calculate PAR_P6 (16-bits)
    reg_calib_data->par_p6 = BMP3_CONCAT_BYTES(reg_data[14], reg_data[13]);
    temp_var = BMP3_CALIB_PAR_P6_DIVISOR_VALUE;
    quantized_calib_data->par_p6 = ((double) reg_calib_data->par_p6 / temp_var);

    // Calculate PAR_P7 (8-bits)
    reg_calib_data->par_p7 = (int8_t) reg_data[15];
    temp_var = BMP3_CALIB_PAR_P7_DIVISOR_VALUE;
    quantized_calib_data->par_p7 = ((double) reg_calib_data->par_p7 / temp_var);

    // Calculate PAR_P8 (8-bits)
    reg_calib_data->par_p8 = (int8_t) reg_data[16];
    temp_var = BMP3_CALIB_PAR_P8_DIVISOR_VALUE;
    quantized_calib_data->par_p8 = ((double) reg_calib_data->par_p8 / temp_var);

    // Calculate PAR_P9 (16-bits)
    reg_calib_data->par_p9 = (int16_t) BMP3_CONCAT_BYTES(reg_data[18],
                                                         reg_data[17]);
    temp_var = BMP3_CALIB_PAR_P9_DIVISOR_VALUE;
    quantized_calib_data->par_p9 = ((double) reg_calib_data->par_p9 / temp_var);

    // Calculate PAR_P10 (8-bits)
    reg_calib_data->par_p10 = (int8_t) reg_data[19];
    temp_var = BMP3_CALIB_PAR_P10_DIVISOR_VALUE;
    quantized_calib_data->par_p10 =
            ((double) reg_calib_data->par_p10 / temp_var);

    // Calculate PAR_P11 (8-bits)
    reg_calib_data->par_p11 = (int8_t) reg_data[20];
    temp_var = BMP3_CALIB_PAR_P11_DIVISOR_VALUE;
    quantized_calib_data->par_p11 =
            ((double) reg_calib_data->par_p11 / temp_var);

    return;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * This funciton is based on reference function in section 9.2 (Temperature compensation) in datasheet
 *
 *
 */

double BMP388_compensate_temperature(const struct bmp3_uncomp_data *uncomp_data,
                                     struct bmp3_calib_data *calib_data)
{
    uint32_t uncomp_temp = uncomp_data->temperature;
    double partial_data1;
    double partial_data2;

    partial_data1 = (double) (uncomp_temp
            - calib_data->quantized_calib_data.par_t1);

    partial_data2 = (double) (partial_data1
            * calib_data->quantized_calib_data.par_t2);

    /* Update the compensated temperature in calib structure since this is
     needed for pressure calculation */

    calib_data->quantized_calib_data.t_lin = partial_data2
            + (partial_data1 * partial_data1)
                    * calib_data->quantized_calib_data.par_t3;

    /* Return compensated temperature */
    return calib_data->quantized_calib_data.t_lin;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

double BMP388_compensate_pressure(const struct bmp3_uncomp_data *uncomp_data,
                                  const struct bmp3_calib_data *calib_data)
{
    /* Variable to store the compensated pressure */
    float comp_press;

    /* Temporary variables used for compensation */
    float partial_data1;
    float partial_data2;
    float partial_data3;
    float partial_data4;
    float partial_out1;
    float partial_out2;

    /* Calibration data */
    // for out 1
    partial_data1 = calib_data->quantized_calib_data.par_p6
            * calib_data->quantized_calib_data.t_lin;

    partial_data2 = calib_data->quantized_calib_data.par_p7
            * (calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin);

    partial_data3 = calib_data->quantized_calib_data.par_p8
            * (calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin);

    partial_out1 = calib_data->quantized_calib_data.par_p5 + partial_data1
            + partial_data2 + partial_data3;

    // for out 2
    partial_data1 = calib_data->quantized_calib_data.par_p2
            * calib_data->quantized_calib_data.t_lin;

    partial_data2 = calib_data->quantized_calib_data.par_p3
            * (calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin);

    partial_data3 = calib_data->quantized_calib_data.par_p4
            * (calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin
                    * calib_data->quantized_calib_data.t_lin);

    partial_out2 = (float) uncomp_data->pressure
            * (calib_data->quantized_calib_data.par_p1 + partial_data1
                    + partial_data2 + partial_data3);

    partial_data1 = (float) uncomp_data->pressure
            * (float) uncomp_data->pressure;

    partial_data2 = calib_data->quantized_calib_data.par_p9
            + calib_data->quantized_calib_data.par_p10
                    * calib_data->quantized_calib_data.t_lin;

    partial_data3 = partial_data1 * partial_data2;

    partial_data4 = partial_data3
            + ((float) uncomp_data->pressure * (float) uncomp_data->pressure
                    * (float) uncomp_data->pressure)
                    * calib_data->quantized_calib_data.par_p11;

    comp_press = partial_out1 + partial_out2 + partial_data4;

    return comp_press;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *  Online calc         : https://keisan.casio.com/exec/system/1224585971
 *
 *  Refer to article    : Relationship Between Altitude and Pressure
 *  source (accurate)   : https://www.mide.com/air-pressure-at-altitude-calculator
 *
 *  Source (secondary)  : https://www.weather.gov/epz/wxcalc_pressurealtitude
 *                      : (https://www.weather.gov/media/epz/wxcalc/pressureAltitude.pdf)
 *
 *  Main formula        : where, h = hb + (Tb/Lb) * [ (P/Pb)^{-(R*Lb)/(g*M)} - 1]

 *  P    = Atmospheric pressure read from the sensor
 *  Pb   = static pressure (pressure at sea level) [Pa]
 *  Tb   = standard temperature (temperature at sea level) [K]
 *  Lb   = standard temperature lapse rate [K/m] = -0.0065 [K/m]
 *  h    = height about sea level [m]
 *  hb   = height at the bottom of atmospheric layer [m]
 *  R    = universal gas constant = 8.31432
 *  g   = gravitational acceleration constant = 9.80665
 *  M    = molar mass of Earth’s air = 0.0289644 [kg/mol]
 *
 *
 *  precalculated values:   -(R*Lb)/(g*M)   = -(8.31432 * -0.0065)/(9.80665 * 0.0289644)
 *                                          = 0.1902632365
 *
 *                                    Tb    = 15 degree C.
 *                                          = 288.15 K.
 *
 *                                 (Tb/Lb)  = 288.15 / -0.0065
 *                                          = -1.872947
 *
 *
 *
 */

float BMP388_readCalibratedAltitude(float seaLevel)
{

    float pressure = BMP388_readPressure();
    return (1.0 - pow((float) pressure / seaLevel, 0.1902632365)) * 288.15
            / 0.0065;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 * source : https://keisan.casio.com/exec/system/1224575267
 *
 *
 *
 */

float BMP388_readSeaLevel(float altitude)
{
    float pressure = BMP388_readPressure();
    return (pressure / pow(1.0 - (altitude / 44330.7), 5.255));
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * Same as BMP388_readCalibratedAltitude(), just with no external
 * parameters.
 *
 *
 */

float BMP388_readAltitude(void)
{
    float pressure = BMP388_readPressure();
    float temperature = BMP388_readTemperature();

    float interResult = 0.0f;
    interResult = pow(pressure / 101325, 0.190284);
    interResult = 1 - interResult;
    interResult *=  (temperature + 273.15);

    interResult /= 0.0065;


    float alt = interResult ;

#ifdef DEBUG

    char charBuffer [80] = {0};
    sprintf(charBuffer, "Read Altitude : \t Pressure : \t %f \t Temp. : \t %f \t Alt. : \t %f \n", pressure, temperature, alt);
    UARTprintf("%s", charBuffer);
    charBuffer[0] = '\0';
    UARTprintf("\n\n");

#endif
    return alt;
    //return (1.0 - pow(pressure / 101325, 0.190284)) * 288.15 / 0.0065;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

int8_t BMP388_INTEnable(uint8_t config)
{
    int8_t rslt = BMP3_E_NULL_PTR;
    uint8_t reg_data = 0x00; //
    uint8_t reg_addr = BMP3_INT_CTRL_ADDR;

    // if FIFO water-mark interrupt needs to be enabled
    if (config & BMP3_INT_FIFO_WM)
    {
        reg_data |= BMP3_INT_FIFO_WM;
    }

    // if FIFO FULL interrupt needs to be enabled
    if (config & BMP3_INT_FIFO_FULL)
    {
        reg_data |= BMP3_INT_FIFO_FULL;
    }

    // if Data Ready interrupt needs to be enabled
    if (config & BMP3_INT_DATA_RDY)
    {
        reg_data |= BMP3_INT_DATA_RDY;
    }

    rslt = BMP388_set_regs(&reg_addr, &reg_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Writing INT_CTRL register failed (BMP388_INTEnable).\n");
#endif
        return BMP3_E_REG_WRITE_FAILED;
    }

    return rslt;
}

int8_t BMP388_INTDisable(uint8_t config)
{
    int8_t rslt = BMP3_E_NULL_PTR;
    uint8_t reg_data = 0x00;
    uint8_t reg_addr = BMP3_INT_CTRL_ADDR;

    //read the current status of Int Ctrl Reg.
    rslt = BMP388_get_regs(reg_addr, &reg_data, 1);
    if (rslt != BMP3_OK)
    {

#ifdef DEBUG
        UARTprintf(
                "FAILED: Reading INT_CTRL register from BMP388 (BMP388_INTDisable)\n");
#endif

        return BMP3_E_GET_REG_DATA_FAILED;
    }

    // reg_data now contains the current status of
    // enabled interrupts

    // if FIFO_Water mark interrupt needs to be disabled
    if (config & BMP3_INT_FIFO_WM)
    {
        reg_data &= ~BMP3_INT_FIFO_WM;
    }

    // if FIFO FULL interrupt needs to be disabled
    if (config & BMP3_INT_FIFO_FULL)
    {
        reg_data &= ~BMP3_INT_FIFO_FULL;
    }

    // if Data Ready interrupt needs to be disabled
    if (config & BMP3_INT_DATA_RDY)
    {
        reg_data &= ~BMP3_INT_DATA_RDY;
    }

    rslt = BMP388_set_regs(&reg_addr, &reg_data, 1);

    if (rslt != BMP3_OK)
    {
#ifdef DEBUG
        UARTprintf(
                "FAILED: Writing INT_CTRL register failed. (BMP388_INTDisable).\n");
#endif
        return BMP3_E_REG_WRITE_FAILED;
    }

    return rslt;

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_user_delay_ms(uint32_t num)
{
    SYSTICK_Delay(num);
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */

void BMP388_read_all_regs(void)
{
    uint8_t reg_addr = BMP3_CHIP_ID_ADDR;
    uint8_t length = 32; // 32

    uint8_t data[32] = { 0 };

    //breakpoint at next line,
    // step-over the program in debug mode
    // read the contents of returned values in debug variables tab.
    reg_addr = 0x19;
    length = 6; // 9, A, B, C, D
    BMP388_get_regs(reg_addr, data, length);

    return;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *  From pg. 40
 *
 *  Writing is done by sending the slave address in write mode (RW = ‘0’), resulting in slave address 111011X0 (‘X’ is determined
 *  by state of SDO pin. Then the master sends pairs of register addresses and register data. The transaction is ended by a stop
 *  condition. This is depicted in Figure 15.
 *
 *
 */

int8_t BMP388_user_i2c_write(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                             uint16_t len)
{

    if (len != 1)
    {

#ifdef DEBUG
        UARTprintf(
                "FAILED: I2C write failed for BMP388: Invalid write length.\n");
#endif

        return BMP3_E_I2C_INVALIDE_WRITE_LENGTH;
    }

    // set slave address in write mode
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_id, false);

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }
    // master is in idle state right now
    I2CMasterDataPut(I2C0_BASE, reg_addr);

    // send START condition followed by TRANSMIT operation (master goes
    // to the Master Transmit state). TM4C datasheet: Table 16-5, pg. 1023.
    I2CMasterControl(I2C0_BASE, I2C_START | I2C_RUN);

    // master is in Master Transmit state right now.

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // send data that will be written to the first
    // addressed register

    I2CMasterDataPut(I2C0_BASE, *data);

    // TRANSMIT followed by STOP condition (master goes
    // to Idle state).
    I2CMasterControl(I2C0_BASE, I2C_STOP | I2C_RUN);

    // wait if bus is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    uint32_t error;
    int8_t rslt = BMP3_OK;
    error = I2CMasterErr(I2C0_BASE);

    if (error != I2C_MASTER_ERR_NONE)
    {
#ifdef DEBUG
        UARTprintf("FAILED: I2C write failed for BMP388: Operation failed\n");
#endif
        rslt = BMP3_E_I2C_WRITE_ERROR;
    }

    return rslt;
}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 */
int8_t BMP388_user_i2c_MultiByteWrite(uint8_t dev_id, uint8_t *reg_addr,
                                      uint8_t *data, uint16_t len)
{

    uint32_t error;
    int8_t rslt = BMP3_OK;
    if (len <= 1)
    {

#ifdef DEBUG
        UARTprintf(
                "FAILED: I2C write failed for BMP388: Invalid write length.\n");
#endif

        return BMP3_E_I2C_INVALIDE_WRITE_LENGTH;
    }

    // slave address in write mode
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_id, false);

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // master is in idle state right now
    I2CMasterDataPut(I2C0_BASE, reg_addr[0]);

    // send START condition followed by TRANSMIT operation (master goes
    // to the Master Transmit state). TM4C datasheet: Table 16-5, pg. 1023.
    I2CMasterControl(I2C0_BASE, I2C_START | I2C_RUN);

    // master is in Master Transmit state right now.

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    //send the first data-byte.
    I2CMasterDataPut(I2C0_BASE, data[0]);

    //TRANSMIT operation (master remains in Master
    // Transmit state).
    I2CMasterControl(I2C0_BASE, I2C_RUN);

    // wait if bus is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // send remaining reg-addr and data combinations.
    uint8_t count = 0;
    for (count = 1; count < len; count++)
    {

        I2CMasterDataPut(I2C0_BASE, reg_addr[count]);

        // TRANSMIT operation (master remains in Master
        // Transmit state).
        I2CMasterControl(I2C0_BASE, I2C_RUN);

        // wait if Master is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }

        //send the  data-byte.
        I2CMasterDataPut(I2C0_BASE, data[count]);

        if (count != len - 1)
        {
            //TRANSMIT operation (master remains in Master
            // Transmit state).
            I2CMasterControl(I2C0_BASE, I2C_RUN);
        }
        else
        {
            //TRANSMIT followed by STOP condition (master goes
            // to Idle state)
            I2CMasterControl(I2C0_BASE, I2C_RUN | I2C_STOP);

        }
        // wait if bus is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }

        error = I2CMasterErr(I2C0_BASE);

        if (error != I2C_MASTER_ERR_NONE)
        {
#ifdef DEBUG
            UARTprintf(
                    "FAILED: I2C write failed for BMP388: Operation failed\n");
#endif
            rslt = BMP3_E_I2C_WRITE_ERROR;
            return rslt;
        }

    }

    return rslt;

}

/*
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 *
 * From BMP388 datasheet section 5.2.2
 *
 * To be able to read registers, first the register address must be sent in write mode (slave address 111011X0). Then either a
 stop or a repeated start condition must be generated. After this the slave is addressed in read mode (RW = ‘1’) at address
 111011X1, after which the slave sends out data from auto-incremented register addresses until a NOACKM and stop
 condition occurs. This is depicted in Figure 16, where two bytes are read from register 0xF6 and 0xF7.
 *
 *
 *
 */

int8_t BMP388_user_i2c_read(uint8_t dev_id, uint8_t reg_addr, uint8_t *data,
                            uint16_t len)
{

    int8_t rslt = BMP3_OK;
    uint32_t error;
    uint32_t receivedData;

    I2CMasterSlaveAddrSet(I2C0_BASE, dev_id, false); // write

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // master is in idle state right now
    I2CMasterDataPut(I2C0_BASE, reg_addr);

    // Send START condition followed by TRANSMIT (master goes
    // to the Master Transmit state).
    I2CMasterControl(I2C0_BASE, I2C_START | I2C_RUN);

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    // At this point, following transaction finishied
    // -> (START)-(SL.ADDR+W)-(ACK)-(REG.ADDR)-(ACK)

    // Put slave address in read mode
    I2CMasterSlaveAddrSet(I2C0_BASE, dev_id, true); // read mode

    // wait if Master is busy
    while (I2CMasterBusy(I2C0_BASE))
    {

    }

    if (len == 1)
    {
        // Repeated START condition followed by a RECEIVE
        // operation with a negative ACK (master goes to Master
        // Receive state

        I2CMasterControl(I2C0_BASE, I2C_START | I2C_RUN);

        // wait if Master is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }

        // part 2 finished:
        // -(Rep. Start)-(SL.ADDR+R)-(ACK)-(DATA)-NACK-

        // stop the transaction
        // STOP condition (master goes to Idle state).b
        I2CMasterControl(I2C0_BASE, I2C_STOP);

        // wait if Master is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }

        // check for error

        error = I2CMasterErr(I2C0_BASE);

        if (error != I2C_MASTER_ERR_NONE)
        {
#ifdef DEBUG
            UARTprintf("FAILED: I2C Read failed for BMP388: Register: %x\n",
                       reg_addr);
#endif
            rslt = BMP3_E_I2C_READ_ERROR;
            return rslt;
        }

        // get the data-byte
        receivedData = I2CMasterDataGet(I2C0_BASE);
        data[0] = receivedData;

    }

    else if (len > 1)
    {
        // Repeated START condition followed by RECEIVE
        // (master goes to Master Receive state).
        I2CMasterControl(I2C0_BASE, I2C_ACK | I2C_START | I2C_RUN);

        // wait if Master is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }

        uint8_t count = 0;
        for (count = 0; count < len; count++)
        {

            // check for error
            error = I2CMasterErr(I2C0_BASE);

            if (error != I2C_MASTER_ERR_NONE)
            {
#ifdef DEBUG
                UARTprintf("FAILED: I2C Read failed for BMP388: Register: %x\n",
                           reg_addr);
#endif
                rslt = BMP3_E_I2C_READ_ERROR;
                return rslt;
            }

            // receive data
            receivedData = I2CMasterDataGet(I2C0_BASE);
            data[count] = receivedData;

            if (count != (len - 1))
            {
                // RECEIVE operation (master remains in Master Receive
                // state).
                I2CMasterControl(I2C0_BASE, I2C_ACK | I2C_RUN);

            }
            else
            {
                // RECEIVE operation with negative ACK (master remains
                // in Master Receive state)
                I2CMasterControl(I2C0_BASE, I2C_RUN);
            }

            // wait if Master is busy
            while (I2CMasterBusy(I2C0_BASE))
            {

            }

            error = I2CMasterErr(I2C0_BASE);

            if (error != I2C_MASTER_ERR_NONE)
            {
#ifdef DEBUG
                UARTprintf("FAILED: I2C Read failed for BMP388: Register: %x\n",
                           reg_addr);
#endif
                rslt = BMP3_E_I2C_READ_ERROR;
                return rslt;
            }

        }

        // send a stop signal
        // STOP condition (master goes to Idle state)
        I2CMasterControl(I2C0_BASE, I2C_STOP);

        // wait if Master is busy
        while (I2CMasterBusy(I2C0_BASE))
        {

        }
    }

    return rslt;
}
