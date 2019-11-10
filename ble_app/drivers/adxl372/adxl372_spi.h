#include "stdint.h"

void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *  p_context);
void spi_init (void);
uint8_t* spi_write_and_read ( uint8_t* tx_msg, uint32_t length );