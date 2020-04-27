#ifndef SPI_DRIVER_H
#define SPI_DRIVER_H

#include "stdint.h"
#include "string.h"
#include "boards.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "sdk_errors.h"
//currently unused 
//#include "nrf_delay.h"
//#include "app_error.h"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"


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

void spi_cfg_cs_pins(uint8_t cs_pin);
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *  p_context);
int8_t spi_write_and_read (const nrf_drv_spi_t* spi, uint8_t cs_pin, uint8_t* tx_msg, uint8_t tx_length, uint8_t* rx_msg, uint8_t rx_length);


#endif //SPI_DRIVER_H