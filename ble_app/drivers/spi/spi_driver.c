#include "spi_driver.h"

#define TX_RX_BUF_LENGTH 1024u  /**< SPI transaction buffer length. */

#define SPI_INSTANCE  0 /**< SPI instance index. */
static const nrf_drv_spi_t spi = NRF_DRV_SPI_INSTANCE(SPI_INSTANCE);  /* could set this up in main (?) */

// Data buffers.
static uint8_t m_rx_buf[TX_RX_BUF_LENGTH] = {0}; /**< A buffer for incoming data. */
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

void spi_init (void)
{
    nrf_drv_spi_config_t const spi_config = {
        .ss_pin       = SPI_SS_PIN,
        .miso_pin     = SPI_MISO_PIN,
        .mosi_pin     = SPI_MOSI_PIN,
        .sck_pin      = SPI_SCK_PIN,
        .irq_priority = SPI_DEFAULT_CONFIG_IRQ_PRIORITY,
        .orc          = 0xFF,
        .frequency    = NRF_DRV_SPI_FREQ_4M,
        .mode         = NRF_DRV_SPI_MODE_0,
        .bit_order    = NRF_DRV_SPI_BIT_ORDER_MSB_FIRST,
    };
    ret_code_t err_code = nrf_drv_spi_init(&spi, &spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

}

uint8_t* spi_write_and_read ( uint8_t* tx_msg, uint16_t length)
{
    m_transfer_completed = false;

    ret_code_t err_code = nrf_drv_spi_transfer(&spi, tx_msg , length, m_rx_buf, length);
    APP_ERROR_CHECK(err_code);

    while(!m_transfer_completed)
    {
        __WFE();
    }

    return m_rx_buf;
}