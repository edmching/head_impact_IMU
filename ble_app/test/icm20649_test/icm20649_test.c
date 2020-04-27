//-------------------------------------------
// Name: icm20649_test.c
// Author: Edmond Ching
// Description: This file initiates SPI communication
// with the ICM20649 gyroscope (and built-in accelerometer), performs write
// and read tests, and then prints out X, Y and Z rotational velocities and 
// low-g acceleration values.
//-------------------------------------------

#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_delay.h"

#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

// The custom data struct for the sensor
// Holds acceleration and gyroscope data
typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

// Function declarations
static void log_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);



/**
 * @brief Main function that initializes peripheral, runs write and read tests and then prints data
 */
int main (void)
{
    // Initialize.
    spi_init(); // initializes the SPI
    log_init(); // initializes the logging function

    NRF_LOG_INFO(" ICM20649 TEST measurement mode");
    nrf_delay_ms(10);

    // The ICM20649's internal register at address 0x00 contains
    // the device ID (0xE1). If this value can correctly be read,
    // then the read test passes

    // If this test fails, check the device connections

    /*********TEST READ******************/
    uint8_t who_am_i = 0x0; // 1 byte variable to store the read byte
    icm20649_read_reg(0x0, &who_am_i); // reads the register at address 0x00
    NRF_LOG_INFO("who_am_i = 0x%x (0xE1)", who_am_i );
    if(who_am_i == 0xE1) // if the read byte matches the known device ID, then the read test passes
    {
        NRF_LOG_INFO("READ SUCCESSFUL");
    }
    else // otherwise, the read test fails
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/

    // Writing to the ICM20649's internal register at address 0x06
    // allows the user to set-up the power management functions.
    // For the write test, we write 0x1 to this register and read
    // it back - if it matches, the write test passes

    // If this test fails, check the device connections
    
    /*********TEST WRITE******************/

    uint8_t write_read; // variable to hold the read value
    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1); // write the value 0x1 to address 0x06

    icm20649_read_reg(0x06, &write_read); // read the value back

    NRF_LOG_INFO("write_read = 0x%x (0x1)", write_read );
    if(write_read == 0x1) // if the values match, the write test passes
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else // otherwise the write test fails
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/

    // The following write commands configure the gyroscope

    //USER CTRL disable all
    icm20649_write_reg(0x03, 0x0);

    //LP_CONFIG disable duty cycle mode
    icm20649_write_reg(0x05, 0x0);

    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1);

    //PWR_MGMT 2 enable accel & gyro
    icm20649_write_reg(0x07, 0x0);
    
    //REG BANK SEL select userbank 2
    icm20649_write_reg(0x7F, 0x20);

    //GYRO_CONFIG_1 bypass gyro DLPF, 2000dps
    icm20649_write_reg(0x1, 0x4);

    //GYRO_CONFIG_2 disable self test, no avging
    icm20649_write_reg(0x2, 0x0);

    //GYRO_CONFIG_2 disable self test, no avging
    icm20649_write_reg(0x14, 0x6);

    //REG BANK SEL select userbank 0
    icm20649_write_reg(0x7F, 0x0);

    icm20649_data_t data; // intantiation of the gyroscope data struct
    float deg2rad = 3.1415/180.0; // degree to radian conversion constant
    nrf_delay_ms(10);
    while(1)
    {

        icm20649_read_gyro_accel_data(&data); // reads the address containing the data
        data.accel_x = ((float) data.accel_x/1024.0)*1000; // acquires the acceleration data and converts it to mgs
        data.accel_y = ((float) data.accel_y/1024.0)*1000; // acquires the acceleration data and converts it to mgs
        data.accel_z = ((float) data.accel_z/1024.0)*1000; // acquires the acceleration data and converts it to mgs
        data.gyro_x = ((float) data.gyro_x / 32767.0) * 2000.0 * deg2rad; // acquires the rotational data and converts it to mrad/s
        data.gyro_y = ((float) data.gyro_y / 32767.0) * 2000.0 * deg2rad; // acquires the rotational data and converts it to mrad/s
        data.gyro_z = ((float) data.gyro_z / 32767.0) * 2000.0 * deg2rad; // acquires the rotational data and converts it to mrad/s

        NRF_LOG_INFO("accel x = %d mg, accel y = %d mg, accel z = %d mg, gyro x = %d mrad/s, gyro y = %d mrad/s, gyro z = %d mrad/s", 
        data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z ); // prints all six values to serial
    
        nrf_delay_ms(1000);
    }

    return 0;
}

/**
 * @brief Function for writing one byte to a single register address
 */
int8_t icm20649_write_reg(uint8_t address, uint8_t data)
{
    uint8_t tx_msg[2];
    uint8_t rx_buf[2];
    tx_msg[0] = address;
    tx_msg[1] = data;

    return spi_write_and_read(SPI_ICM20649_CS_PIN, tx_msg, 2, rx_buf, 2 ); // send 2 bytes (address and data)
}


/**
 * @brief Function for reading one byte from a single register address
 */
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data)
{
    uint8_t reg_addr;
    int8_t ret;
    uint8_t rx_buf[2];

    reg_addr = (uint8_t)  ( address | 0x80 ); //set 1st bit for reads
    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &reg_addr, 1, rx_buf, 2);

    *reg_data = rx_buf[1];

    return ret; // return the read byte
}


/**
 * @brief Function for reading up to 256 consecutive bytes from the internal registers
 */
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes) 
{
    uint8_t read_addr;
    uint8_t buf[257]; 
    int8_t ret;
    
    if(num_bytes > 256) // if number of bytes is greater than 256, return an error
        return -1;

    read_addr = reg_addr | 0x80; //set MSB to 1 for read
    memset( buf, 0x00, num_bytes + 1);

    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &read_addr, 1, buf, num_bytes + 1 );
    if (ret < 0)
        return ret;
    
    memcpy(reg_data, &buf[1], num_bytes);

    return ret;
}

/**
 * @brief Function for reading the gyroscope's data (linear and rotational)
 */
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data)
{
    uint8_t rx_buf[12] = {0};

    //REG BANK SEL select userbank 0
    icm20649_write_reg(0x7F, 0x0);

    icm20649_multibyte_read_reg( 0x2D, rx_buf, 12); // performs a multibyte read

    // arranges the read data bytes for each measurement to proper values
    icm20649_data->accel_x = rx_buf[0]<<8 | rx_buf[1];
    icm20649_data->accel_y = rx_buf[2]<<8 | rx_buf[3];
    icm20649_data->accel_z = rx_buf[4]<<8 | rx_buf[5];
    icm20649_data->gyro_x = rx_buf[6]<<8 | rx_buf[7];
    icm20649_data->gyro_y = rx_buf[8]<<8 | rx_buf[9];
    icm20649_data->gyro_z = rx_buf[10]<<8 | rx_buf[11];
    

}

/**
 * @brief Function for initializing the nrf log
 */
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
