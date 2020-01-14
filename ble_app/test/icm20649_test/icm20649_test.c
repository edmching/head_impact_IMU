#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"

#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

static void log_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);

int main (void)
{
    // Initialize.
    spi_init();
    log_init();

    NRF_LOG_INFO(" ICM20649 TEST measurement mode");
    nrf_delay_ms(10);

    /*********TEST READ******************/
    uint8_t who_am_i = 0x0;
    icm20649_read_reg(0x0, &who_am_i);
    NRF_LOG_INFO("who_am_i = 0x%x (0xE1)", who_am_i );
    if(who_am_i == 0xE1)
    {
        NRF_LOG_INFO("READ SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/

    
    /*********TEST WRITE******************/

    uint8_t write_read;
    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1);

    icm20649_read_reg(0x06, &write_read);

    NRF_LOG_INFO("write_read = 0x%x (0x1)", write_read );
    if(write_read == 0x1)
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/

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

    icm20649_data_t data;
    float deg2rad = 3.1415/180.0;
    nrf_delay_ms(10);
    while(1)
    {

        icm20649_read_gyro_accel_data(&data);
        data.accel_x = ((float) data.accel_x/1024.0)*1000;
        data.accel_y = ((float) data.accel_y/1024.0)*1000;
        data.accel_z = ((float) data.accel_z/1024.0)*1000;
        data.gyro_x = ((float) data.gyro_x / 32767.0) * 2000.0 * deg2rad;

        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mg, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
        data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z );
    
        nrf_delay_ms(1000);
    }

    return 0;
}

int8_t icm20649_write_reg(uint8_t address, uint8_t data)
{
    uint8_t tx_msg[2];
    uint8_t rx_buf[2];
    tx_msg[0] = address;
    tx_msg[1] = data;

    return spi_write_and_read(SPI_ICM20649_CS_PIN, tx_msg, 2, rx_buf, 2 ); // send 2 bytes
}

int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data)
{
    uint8_t reg_addr;
    int8_t ret;
    uint8_t rx_buf[2];

    reg_addr = (uint8_t)  ( address | 0x80 ); //set 1st bit for reads
    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &reg_addr, 1, rx_buf, 2);

    *reg_data = rx_buf[1];

    return ret;
}

int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes) 
{
    uint8_t read_addr;
    uint8_t buf[257]; 
    int8_t ret;
    
    if(num_bytes > 256)
        return -1;

    read_addr = reg_addr | 0x80; //set MSB to 1 for read
    memset( buf, 0x00, num_bytes + 1);

    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &read_addr, 1, buf, num_bytes + 1 );
    if (ret < 0)
        return ret;
    
    memcpy(reg_data, &buf[1], num_bytes);

    return ret;
}

void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data)
{
    uint8_t rx_buf[12] = {0};

    //REG BANK SEL select userbank 0
    icm20649_write_reg(0x7F, 0x0);

    icm20649_multibyte_read_reg( 0x2D, rx_buf, 12);

    icm20649_data->accel_x = rx_buf[0]<<8 | rx_buf[1];
    icm20649_data->accel_y = rx_buf[2]<<8 | rx_buf[3];
    icm20649_data->accel_z = rx_buf[4]<<8 | rx_buf[5];
    icm20649_data->gyro_x = rx_buf[6]<<8 | rx_buf[7];
    icm20649_data->gyro_y = rx_buf[8]<<8 | rx_buf[9];
    icm20649_data->gyro_z = rx_buf[10]<<8 | rx_buf[11];
    

}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}