#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"	//I2C driver library
#include "nrf_delay.h"

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// I2C pin assignment
#define I2C_SDA 17
#define I2C_SCL 18

// TWI instance ID
#define TWI_INSTANCE_ID     0

// Proximity sensor address
#define VCNL4040_ADDR 0x60 //U >> 1

// Proximity sensor command codes
#define VCNL4040_PS_CONF1 0x03 //Lower
#define VCNL4040_PS_CONF2 0x03 //Upper
#define VCNL4040_PS_CONF3 0x04 //Lower
#define VCNL4040_PS_MS 0x04 //Upper
#define VCNL4040_PS_DATA 0x08

/* Mode for LM75B. */
#define NORMAL_MODE 0U

// Proximity sensor configuration register values
static uint8_t ps_conf1_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (0 << 0);
static uint8_t ps_conf2_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_conf3_data =	(0 << 7) | (0 << 6) | (0 << 5) | (1 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_ms_data =		(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
 
/* Indicates if operation on I2C has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t vcnl_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint16_t m_sample;

/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
void vcnl_config(void)
{
    ret_code_t err_code;
	
	uint8_t reg1[3] = {VCNL4040_PS_CONF3, ps_conf3_data, ps_ms_data};
    err_code = nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg1, sizeof(reg1), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
	
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    err_code = nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg2, sizeof(reg2), false);
    APP_ERROR_CHECK(err_code);
    while (m_xfer_done == false);
}

/**
 * @brief Function for handling data from temperature sensor.
 *
 * @param[in] temp          Temperature in Celsius degrees read from sensor.
 */
__STATIC_INLINE void data_handler(uint16_t prox)
{
    NRF_LOG_INFO("Proximity: %d", prox);
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                data_handler(m_sample);
            }
            m_xfer_done = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = I2C_SCL,
       .sda                = I2C_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&vcnl_twi, &twi_lm75b_config, twi_handler, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&vcnl_twi);
}

/**
 * @brief Function for reading data from temperature sensor.
 */
// static void read_sensor_data()
// {
//     m_xfer_done = false;

//     // Read 2 byte from the specified address
//     ret_code_t err_code = nrf_drv_twi_rx(&vcnl_twi, LM75B_ADDR, &m_sample, sizeof(m_sample));
//     APP_ERROR_CHECK(err_code);
// }

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nTWI sensor example started.");
    NRF_LOG_FLUSH();
    twi_init();
	
	while(1){
		vcnl_config();
		nrf_delay_ms(1000);
	}

    while (true)
    {
        nrf_delay_ms(500);

        do
        {
            __WFE();
        }while (m_xfer_done == false);

        //read_sensor_data();
        NRF_LOG_FLUSH();
    }
}

/** @} */
