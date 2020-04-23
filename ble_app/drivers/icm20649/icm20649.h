#ifndef ICM20649_H_
#define ICM20649_H_

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h> 
#include <nrf_drv_spi.h>

#define GYRO_SPI_INSTANCE  0 /**< SPI instance index. */

const nrf_drv_spi_t gyro_spi = NRF_DRV_SPI_INSTANCE(gyro_SPI_INSTANCE);  

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

typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

void log_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);

#endif /* ICM20649_H_ */
