//-------------------------------------------
// Title: spi_driver.c
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This is refactored spi drivers located in spi_pcb folder
// the spi folder is now depreciated and is used for running
// breadboard test programs
// Please use this verison for PCB rev2
// This requires setting up spi_init in your program's main file
//-------------------------------------------
#include "spi_driver.h"

static volatile bool m_transfer_completed = true; /**< A flag to inform about completed transfer. */

/**
 * @brief SPI event handler.
 * 
 * @param event
 * @
 */
void spi_event_handler(nrf_drv_spi_evt_t const * p_event, void *  p_context)
{
    switch(p_event->type){
        case NRF_DRV_SPI_EVENT_DONE:
            m_transfer_completed = true;
            break;
        default:
            break;
    }
}

/* 
 * @brief for configuration single master multiple slaves with different cs pins
 */
void spi_cfg_cs_pins(uint8_t cs_pin)
{
    nrf_gpio_cfg_output(cs_pin);
    nrf_gpio_pin_clear(cs_pin);
    nrf_gpio_pin_set(cs_pin);
}


int8_t spi_write_and_read (nrf_drv_spi_t const* spi, uint8_t cs_pin, uint8_t* tx_msg, uint8_t tx_length, uint8_t* rx_msg, uint8_t rx_length)
{
    m_transfer_completed = false;

    nrf_gpio_pin_clear(cs_pin);
    ret_code_t err_code = nrf_drv_spi_transfer(spi, tx_msg , tx_length, rx_msg, rx_length);
    if (err_code != NRF_SUCCESS)
        return -1;

    while(!m_transfer_completed)
    {
        __WFE();
    }
    nrf_gpio_pin_set(cs_pin);

    return 0;
}

