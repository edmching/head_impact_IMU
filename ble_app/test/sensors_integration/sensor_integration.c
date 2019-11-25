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

//I2C driver library
#include "nrf_drv_twi.h"

#include "sensors_integration.h"

uint16_t prox_val;

#define PROX_THRESHOLD 10000

int main (void)
{
    // Initialize.
    log_init();
    spi_init();
    twi_init();

    NRF_LOG_INFO(" Sensor test");
    nrf_delay_ms(100);

    icm20649_read_test();
    icm20649_write_test();
    icm20649_init();

    vcnl_config();


    icm20649_data_t data;
    while(1)
    {   
        nrf_delay_ms(1000);
        read_sensor_data();
        if(prox_val >= PROX_THRESHOLD)
        {
            icm20649_read_gyro_accel_data(&data);
            icm20649_convert_data(&data);

            NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mg's, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
            data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z );
        }
    }

    return 0;
}

/********************ICM FUNCTIONS***************************/
void icm20649_read_test(void)
{
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
}

void icm20649_write_test(void)
{
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
}

void icm20649_init(void)
{
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
}

void icm20649_convert_data(icm20649_data_t * data)
{
    float deg2rad = 3.1415/180.0;

    data->accel_x = ((float) data->accel_x/1024.0)*1000;
    data->accel_y = ((float) data->accel_y/1024.0)*1000;
    data->accel_z = ((float) data->accel_z/1024.0)*1000;
    data->gyro_x = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_y = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_z = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
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
/**************************************************/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/************************VCNL FUNCTIONS ****************************/
/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
void vcnl_config(void)
{	
	uint8_t reg1[3] = {VCNL4040_PS_CONF3, ps_conf3_data, ps_ms_data};
    nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);
	
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg2, sizeof(reg2), false);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint16_t prox)
{
    NRF_LOG_INFO("Proximity: %d", prox);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                prox_val = m_sample;
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&vcnl_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&vcnl_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
static void read_sensor_data()
{
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    
    m_xfer_done = false;

    uint8_t reg3[2] = {VCNL4040_PS_DATA, VCNL4040_ADDR};
    uint8_t reg4[2] = {m_sample_lsb, m_sample_msb};
    uint8_t *p_ret = reg4;
    nrf_drv_twi_xfer_desc_t const vcnl_desc = {NRFX_TWI_XFER_TXRX, VCNL4040_ADDR, sizeof(reg3), sizeof(reg4), reg3, p_ret};
    nrf_drv_twi_xfer(&vcnl_twi, &vcnl_desc, false);
    while(nrf_drv_twi_is_busy(&vcnl_twi) == true);

    m_sample_lsb = *p_ret;
    p_ret++;
    m_sample_msb = *p_ret;

    m_sample = (((m_sample_msb) << 8) | (m_sample_lsb));
    prox_val = m_sample;
    NRF_LOG_INFO("Proximity: %d", m_sample);
}
