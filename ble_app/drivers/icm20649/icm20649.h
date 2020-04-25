#ifndef ICM20649_H
#define ICM20649_H

#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h> 
#include "nrf_drv_spi.h"
#include "spi_driver.h"
#include "nrf_log.h"


#define GYRO_SPI_INSTANCE 2 /**< SPI instance index. */

extern const nrf_drv_spi_t gyro_spi;

extern nrf_drv_spi_config_t const gyro_spi_config;

typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

void icm20649_default_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);
void icm20649_convert_data(icm20649_data_t * data);
void icm20649_test(void);

#endif /* ICM20649_H */
