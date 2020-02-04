#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stdint.h"
#include <string.h>
#include "nrf_delay.h"
#include "app_error.h"
#include "sdk_errors.h"


#ifndef USE_IMU_PCB_REV1
#define SPI_ICM20649_CS_PIN     29
#define SPI_ADXL372_CS_PIN      30
#define SPI_MT25QL256ABA_CS_PIN 27
#define SPI_MOSI_PIN            4
#define SPI_MISO_PIN            28
#define SPI_SCK_PIN             3
#endif

#ifdef USE_IMU_PCB_REV1
#define SPI_SCK_PIN             12
#define SPI_MISO_PIN            13
#define SPI_MOSI_PIN            14
#define SPI_ICM20649_CS_PIN     5
#define SPI_MT25QL256ABA_CS_PIN 8
#define SPI_ADXL372_CS_PIN      30
#endif

// <i> Priorities 0,2 (nRF51) and 0,1,4,5 (nRF52) are reserved for SoftDevice
// <0=> 0 (highest) 
// <1=> 1 
// <2=> 2 
// <3=> 3 
// <4=> 4 
// <5=> 5 
// <6=> 6 
// <7=> 7 
#define SPI_IRQ_PRIORITY 6

void spi_init (void);
int8_t spi_write_and_read ( uint8_t cs_pin, uint8_t* tx_msg, uint8_t tx_length, uint8_t* rx_msg, uint8_t rx_length);


#endif //SPI_DRIVER_H