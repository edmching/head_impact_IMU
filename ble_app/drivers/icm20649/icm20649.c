#include "icm20649.h"

const nrf_drv_spi_t gyro_spi = NRF_DRV_SPI_INSTANCE(GYRO_SPI_INSTANCE);  

nrf_drv_spi_config_t const gyro_spi_config = {
        .ss_pin       = SPI_GYRO_CS_PIN,
        .miso_pin     = SPI_GYRO_MISO_PIN,
        .mosi_pin     = SPI_GYRO_MOSI_PIN,
        .sck_pin      = SPI_GYRO_SCK_PIN,
        .irq_priority = SPI_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_1M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
};

void icm20649_default_init(void)
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

int8_t icm20649_write_reg(uint8_t address, uint8_t data)
{
    uint8_t tx_msg[2];
    uint8_t rx_buf[2];
    tx_msg[0] = address;
    tx_msg[1] = data;

    return spi_write_and_read(&gyro_spi, SPI_GYRO_CS_PIN, tx_msg, 2, rx_buf, 2 ); // send 2 bytes
}

int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data)
{
    uint8_t reg_addr;
    int8_t ret;
    uint8_t rx_buf[2];

    reg_addr = (uint8_t)  ( address | 0x80 ); //set 1st bit for reads
    ret = spi_write_and_read(&gyro_spi, SPI_GYRO_CS_PIN, &reg_addr, 1, rx_buf, 2);

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

    ret = spi_write_and_read(&gyro_spi, SPI_GYRO_CS_PIN, &read_addr, 1, buf, num_bytes + 1 );
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

void icm20649_convert_data(icm20649_data_t * data)
{
    float deg2rad = 3.1415/180.0;

    data->accel_x = ((float) data->accel_x/1024.0)*1000;
    data->accel_y = ((float) data->accel_y/1024.0)*1000;
    data->accel_z = ((float) data->accel_z/1024.0)*1000;
    data->gyro_x = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_y = ((float) data->gyro_y / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_z = ((float) data->gyro_z / 32767.0) * 2000.0 * deg2rad *1000;
}


void icm20649_test(void)
{
    NRF_LOG_INFO(" ICM20649 TEST measurement mode");

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
        while(1)
        {
          __WFE();
        }
        
    }
    /********************************************/

}