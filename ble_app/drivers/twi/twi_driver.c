#include "twi_drivers.h"

/* Indicates if operation on I2C has ended. */
volatile bool m_xfer_done = false;

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            NRF_LOG_INFO("\r\nDATA NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            NRF_LOG_INFO("\r\nADDRESS NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        default:
            break;
    }
}

/**
 * @brief initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&twi, &twi_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&twi);
}
