#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stdint.h"
#include "string.h"
#include "boards.h"


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