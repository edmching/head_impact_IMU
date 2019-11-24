#include "spi_driver.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"
#include <string.h>

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /* could set this up in main (?) */

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
            //NRF_LOG_INFO("Transfer completed.");
            m_transfer_completed = true;
            break;
        default:
            break;
    }
}

void spi_init (void)
{
    nrf_drv_spi_config_t const spi_config = {
        .ss_pin       = SPI_SS_PIN,
        .miso_pin     = SPI_MISO_PIN,
        .mosi_pin     = SPI_MOSI_PIN,
        .sck_pin      = SPI_SCK_PIN,
        .irq_priority = SPI_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_500K, //due to poor wiring must use slower SCK
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    ret_code_t err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
    NRF_LOG_INFO("SPI INITIALIZED.");
}

int8_t spi_write_and_read ( uint8_t* tx_msg, uint8_t tx_length, uint8_t* rx_msg, uint8_t rx_length)
{
    m_transfer_completed = false;

    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_msg , tx_length, rx_msg, rx_length);
    if (err_code != NRF_SUCCESS)
        return -1;

    while(!m_transfer_completed)
    {
        __WFE();
    }

    return 0;
}
