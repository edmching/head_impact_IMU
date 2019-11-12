#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stdint.h"
#include "nrf_drv_spi.h"
#include "nrf_delay.h"
#include "app_error.h"
#include "spi_driver.h"
#include "sdk_errors.h"

void spi_init (void);
uint8_t* spi_write_and_read ( uint8_t* tx_msg, uint16_t length);


#endif //SPI_DRIVER_H