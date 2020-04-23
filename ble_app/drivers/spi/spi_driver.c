#include "spi_driver.h"
#include "nrf_drv_spi.h"
#include "nrf_gpio.h"
#include "sdk_errors.h"

//currently unused 
//#include "nrf_delay.h"
//#include "app_error.h"
//#include "nrf_log.h"
//#include "nrf_log_ctrl.h"
//#include "nrf_log_default_backends.h"

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

void spi_cfg_cs_pins(uint8_t cs_pin)
{
    nrf_gpio_cfg_output(cs_pin);
    nrf_gpio_pin_clear(cs_pin);
    nrf_gpio_pin_set(cs_pin);
}

void spi_uninit(void)
{
    ret_code_t err_code = nrf_drv_spi_uninit(&spi0);
    APP_ERROR_CHECK(err_code);
}


int8_t spi_write_and_read (uint8_t cs_pin, uint8_t* tx_msg, uint8_t tx_length, uint8_t* rx_msg, uint8_t rx_length)
{
    m_transfer_completed = false;

    nrf_gpio_pin_clear(cs_pin);
    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_msg , tx_length, rx_msg, rx_length);
    if (err_code != NRF_SUCCESS)
        return -1;

    while(!m_transfer_completed)
    {
        __WFE();
    }
    nrf_gpio_pin_set(cs_pin);

    return 0;
}

