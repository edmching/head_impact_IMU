//-------------------------------------------
// Name: adxl372_test.c
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This file initiates SPI communication
// with the ADXL372 accelerometer, performs write and read tests,
// and then prints out X, Y and Z acceleration values.
// Note: the ADXL372 is intended to record high-g accelerations,
// so running this code in a low-g environment (where the sensor is less accurate)
// will result in error. The point of this code is to test the functionality
// of the sensor more than to capture accurate results.
// Note: see the adxl372 datasheet for more information on the FIFO buffer
//-------------------------------------------

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_delay.h"

//adxl372 driver
#include "adxl372.h" 
#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

#define NUM_SAMPLES 127 // the number of samples to be logged after read/write tests

#define READ_DONE 0
#define NOT_READ  -1

#define TEST_FIFO // test the accelerometer using the FIFO buffer
//#define TEST_REGULAR // test the accelerometer without using the FIFO buffer

static void log_init(void);

int main (void)
{
    // Initialize.
    spi_init();
    log_init();

    int8_t ret = 0;
    // The accelerometer's internal registers contain three different device IDs:
    uint8_t device_id; // the Analog Devices Inc. ID (0xAD)
    uint8_t mst_devid; // the Analog Devices MEMS ID (0x1D)
    uint8_t devid; // the unqiue accelerometer ID (0xFA)

    NRF_LOG_INFO("-----------------------------");
    NRF_LOG_INFO("adxl372 test measurement mode");
    nrf_delay_ms(100);

    adxl372_reset();

    // The read test below attemps to read the three device ID
    // registers and ensure that each of the three device IDs
    // return the correct value - if not, the read test fails.
    // Note: if a read test occurs on the breadboard platform, check the wiring!

    //--------------------READ TEST---------------------//
    device_id = adxl372_get_dev_ID(); // check the Analog Devices Inc. ID (0xAD)
    ret |= adxl372_read_reg( ADI_ADXL372_MST_DEVID, &mst_devid); // check the Analog Devices MEMS ID (0x1D)
    ret |= adxl372_read_reg(ADI_ADXL372_DEVID, &devid); // check the unqiue accelerometer ID (0xFA)
    if (ret < 0) // if negative value is returned, the SPI connection has failed
    {
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
    }

    NRF_LOG_INFO("adi device id = 0x%x (0xAD)", device_id);
    if(device_id != ADI_ADXL372_ADI_DEVID_VAL) // if device ID does not match, read test fails
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
    }
    NRF_LOG_INFO("mst device id = 0x%x (0x1D)", mst_devid); 
    if(mst_devid != ADI_ADXL372_MST_DEVID_VAL) // if device ID does not match, read test fails
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
    }
    NRF_LOG_INFO("accelerometer id = 0x%x (0xFA)", devid);
    if(devid != ADI_ADXL372_DEVID_VAL) // if device ID does not match, read test fails
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
    }
    else{ // if all three device IDs are read correctly, read test passes
        NRF_LOG_INFO("ADXL READ TEST PASS");
    }
    
    // The write test below attemps to write three known values
    // to the device's power control register and read them back
    // if there is a mismatch, the write test fails

    //--------------------WRITE TEST---------------------//
    uint8_t p_reg; // a variable to store the read value of the register
    uint8_t lpf_val; // a variable to store lpf write value
    uint8_t hpf_val; // a variable to store hpf write value
    uint8_t op_val; // a variable to store op write value
    adxl372_set_op_mode(STAND_BY); // writing the known op value
    adxl372_set_lpf_disable(true); // writing the known lpf value
    adxl372_set_hpf_disable(true); // writing the known hpf value
    ret |= adxl372_read_reg(ADI_ADXL372_POWER_CTL, &p_reg);
    if (ret < 0) // if negative value is returned, the SPI connection has failed
    {
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
    }
    lpf_val = (p_reg >> PWRCTRL_LPF_DISABLE_POS) & 0x1; // extracting the lpf value from read register
    hpf_val = (p_reg >> PWRCTRL_HPF_DISABLE_POS) & 0x1; // extracting the hpf value from read register
    op_val = (p_reg >> PWRCTRL_OPMODE_POS) & 0x3; // extracting the op value from read register
    NRF_LOG_INFO("lpf val = %d (expected: 1)", lpf_val);
    NRF_LOG_INFO("hpf val = %d (expected: 1)", hpf_val);
    NRF_LOG_INFO("op val = %d (expected: 0)", op_val);
    if(lpf_val != 1 || hpf_val != 1 || op_val !=0) // if any values are read back improperly, write test fails
    {
        NRF_LOG_ERROR("ADXL WRITE TEST FAIL");
        while(1);
    }
    else{ // if all three values are read back as expected, write test passes
        NRF_LOG_INFO("ADXL WRITE TEST PASS");
    }
    

    //------------------MEASURMENT TEST-------------------//

#ifdef TEST_REGULAR // if the FIFO buffer is not being used
    adxl372_default_init(); // initialize the accelerometer in default mode
#endif
#ifdef TEST_FIFO // if the FIFO buffer is being used
    struct adxl372_device dev;
    adxl372_default_init_fifo_mode(&dev, NUM_SAMPLES); // initialize the accelerometer in FIFO mode
    adxl372_accel_data_t sample_set[NUM_SAMPLES/3]; // declare an instance of a custom struct to hold the data
    int8_t read_fifo_done = NOT_READ;

    while(1)
    {
        read_fifo_done = adxl372_get_fifo_data(&dev, sample_set); // obtain the data using the FIFO buffer

        if(read_fifo_done == READ_DONE)
        {
            for(int i = 0; i < NUM_SAMPLES/3; ++i) // print X, Y and Z acceleration values
                NRF_LOG_INFO("sample: %d, X accel = %d mG, Y accel = %d mG, Z accel = %d mG", i,
                            sample_set[i].x, sample_set[i].y, sample_set[i].z);
            adxl372_reset(); // reset the accelerometer
            while(1);
        }
    
    }
#endif
#ifdef TEST_REGULAR // if the FIFO buffer is not being used
    adxl372_accel_data_t accel_data; // declare an instance of a custom struct to hold the data
    while (1)
    {
        adxl372_get_accel_data(&accel_data); // obtain the data // print X, Y and Z acceleration values
        NRF_LOG_INFO("X accel = %d mG, Y accel = %d mG, Z accel = %d mG",
                        accel_data.x, accel_data.y, accel_data.z);
        nrf_delay_ms(1000);
    }
#endif
    return 0;
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
