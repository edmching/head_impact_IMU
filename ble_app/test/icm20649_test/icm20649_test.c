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

void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);

int main (void)
{
    // Initialize.
    spi_init();
    log_init();

    NRF_LOG_INFO(" ICM20649 TEST measurement mode");

    /*********TEST READ******************/
    nrf_delay_ms(100);
    volatile int8_t ret;
    uint8_t who_am_i[2];

    uint8_t reg_addr;
    reg_addr = (uint8_t) ( (0x0 >> 1) | 1 << 7 );

    ret = spi_write_and_read(&reg_addr, 1, who_am_i, 2);
    NRF_LOG_INFO("who_am_i = 0x%x (0xE1)", who_am_i[1] );
    if(who_am_i[1] == 0xE1)
    {
        NRF_LOG_INFO("READ SUCCESSFUL");
    }
    else if(ret < 0)
    {
        NRF_LOG_INFO("SPI READ FAIL"); 
        while(1); //trap
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR"); 
        while(1); //trap
    }

    /********************************************/

    uint8_t tx_buf[2];
    uint8_t rx_buf[2];

    /*********TEST WRITE******************/

    //PWR_MGMT 1 select best clk and disable everything else
    tx_buf[0] = 0x6 >> 1;
    tx_buf[1] = 0x1;
    ret = spi_write_and_read(tx_buf, 1, rx_buf, 1);


    uint8_t write_read[2];
    reg_addr = (uint8_t) ( (0x6 >> 1) | 1<< 7 );
    spi_write_and_read(&reg_addr, 1, write_read, 2);
    NRF_LOG_INFO("write_read = 0x%x (0x1)", write_read[1] );
    if(write_read[1] == 0x1)
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else if (ret < 0)
    {
        NRF_LOG_INFO("SPI WRITE FAIL");
    }
    else{
        NRF_LOG_INFO("VAL ERROR");
        while(1);//trap
    }
    /********************************************/

    //USER CTRL disable all
    tx_buf[0] = (0x3 >> 1);
    tx_buf[1] = 0x0;

    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //LP_CONFIG disable duty cycle mode
    tx_buf[0] = 0x5 >> 1;
    tx_buf[1] = 0x0;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //PWR_MGMT 1 select best clk and disable everything else
    tx_buf[0] = 0x6 >> 1;
    tx_buf[1] = 0x1;
    ret = spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //PWR_MGMT 2 enable accel & gyro
    tx_buf[0] = 0x7 >> 1;
    tx_buf[1] = 0x0;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);
    
    //REG BANK SEL select userbank 2
    tx_buf[0] = 0x7F >> 1;
    tx_buf[1] = 0x2;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //GYRO_CONFIG_1 bypass gyro DLPF, 2000dps
    tx_buf[0] = 0x1 >> 1;
    tx_buf[1] = 0x4;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //GYRO_CONFIG_2 disable self test, no avging
    tx_buf[0] = 0x2 >> 1;
    tx_buf[1] = 0x0;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //GYRO_CONFIG_2 disable self test, no avging
    tx_buf[0] = 0x14 >> 1;
    tx_buf[1] = 0x6;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    //REG BANK SEL select userbank 0
    tx_buf[0] = 0x7F >> 1;
    tx_buf[1] = 0x0;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    icm20649_data_t data;

    while(1)
    {

        icm20649_read_gyro_accel_data(&data);

        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d, gyro x = %d, gyro y = %d, gyro z = %d", 
        data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z  );
    
        nrf_delay_ms(1000);
    }

    return 0;
}

void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data)
{
    uint8_t tx_buf[2];
    uint8_t rx_buf[12];
    uint8_t reg_addr;

    //REG BANK SEL select userbank 0
    tx_buf[0] = 0x7F >> 1;
    tx_buf[1] = 0x0;
    spi_write_and_read(tx_buf, 1, rx_buf, 1);

    reg_addr = (uint8_t) ( (0x2D >> 1) | 0x1 );
    
    spi_write_and_read(&reg_addr, 1, rx_buf, 12);

    icm20649_data->accel_x = rx_buf[0] | rx_buf[1];
    icm20649_data->accel_y = rx_buf[2] | rx_buf[3];
    icm20649_data->accel_z = rx_buf[4] | rx_buf[5];
    icm20649_data->gyro_x = rx_buf[6] | rx_buf[7];
    icm20649_data->gyro_x = rx_buf[8] | rx_buf[9];
    icm20649_data->gyro_x = rx_buf[10] | rx_buf[11];
    
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
