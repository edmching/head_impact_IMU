//-------------------------------------------
// Name: vcnl_4040.c
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This file initiates I2C communication
// with the vcnl4040 proximity sensor, configures the device,
// and then prints out the relative proximity value serially.
//
// Referenced code: https://github.com/sparkfun/SparkFun_VCNL4040_Arduino_Library/blob/master/src/SparkFun_VCNL4040_Arduino_Library.cpp
//-------------------------------------------

#include <stdio.h>
#include "boards.h"
#include "app_util_platform.h"
#include "app_error.h"
#include "nrf_drv_twi.h"	// I2C driver library
#include "nrf_delay.h"
#include "nrf52832_mdk.h"   // Board (pin mapping) header file for breadboard setup

#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

// TWI instance ID
#define TWI_INSTANCE_ID     0

// Proximity sensor address (for I2C)
#define VCNL4040_ADDR 0x60

// Proximity sensor register addresses
// Note: The VCNL4040 uses 2-byte command registers,
// with one upper and one lower byte. The datasheet
// distinguishes between upper and lower bytes by
// giving each byte a different name (i.e. CONF1 and CONF2)
// for the 2 bytes located at 0x03. This convention is followed
// here (see datasheet for more info).
#define VCNL4040_PS_CONF1 0x03 //Lower
#define VCNL4040_PS_CONF2 0x03 //Upper
#define VCNL4040_PS_CONF3 0x04 //Lower
#define VCNL4040_PS_MS 0x04 //Upper
#define VCNL4040_PS_DATA 0x08

// Proximity sensor configuration register values.
// These byte-wide data values are specified bit-by-bit for
// writing one byte at a time to their corresponding
// byte-wide register (see the data sheet for register-specific bit meaning)
// All four of these bytes need to be written to their appropriate
// registers in order to properly configure the device
static uint8_t ps_conf1_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (0 << 0);
static uint8_t ps_conf2_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_conf3_data =	(0 << 7) | (0 << 6) | (0 << 5) | (1 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_ms_data =		(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
 
/* Indicates if operation on I2C has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from proximity sensor. */
static uint8_t m_sample_lsb;
static uint8_t m_sample_msb;
static uint16_t m_sample;

/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
void vcnl_config(void)
{	

    NRF_LOG_INFO("Configuring VCNL...");

    // Prepares target register (PS_CONF3, i.e 0x04) for a 2 byte write.
    // This entails writing the lower byte of data (ps_conf3_data)
    // followed by the upper byte of data (ps_ms_data)
	uint8_t reg1[3] = {VCNL4040_PS_CONF3, ps_conf3_data, ps_ms_data};
    nrf_drv_twi_tx(&twi, VCNL4040_ADDR, reg1, sizeof(reg1), false); // initates I2C transfer to VCNL4040
    while (m_xfer_done == false); // waits for I2C transfer to finish
	
    // Prepares target register (PS_CONF1, i.e 0x03) for a 2 byte write.
    // This entails writing the lower byte of data (ps_conf1_data)
    // followed by the upper byte of data (ps_conf2_data)
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    nrf_drv_twi_tx(&twi, VCNL4040_ADDR, reg2, sizeof(reg2), false); // initates I2C transfer to VCNL4040
    while (m_xfer_done == false); // waits for I2C transfer to finish
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
 
    // Should specific errors occur in I2C communication (such as slave NACK),
    // corresponding error message is printed to serial

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
 * @brief TWI initialization.
 */
void twi_init (void)
{
    ret_code_t err_code;

    // Initializes the I2C connection to 400 kHz,
    // using the SDA and SCL pins for the board
    // (see README to change which board/platform is in use)

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

/**
 * @brief Function for reading data from proximity sensor.
 * The proximity sensor gives relative data - that is, it
 * records a qualitative proximity magnitude compared to the
 * proximity it detects immediately when it is configured.
 * The proximity value:
 * - increases if an object is currently nearer to it than on configuration
 * - decreases if an object is currently farther from it than on configuration
 * 
 * In practice, this means that the device should start with nothing near the
 * sensor, and when it is in use/inserted, the proximity value increases and
 * a the threshold is met.
 */
static void read_sensor_data()
{
     do
    {
        __WFE();
    }while (m_xfer_done == false);
    
    m_xfer_done = false;

    uint8_t reg3[2] = {VCNL4040_PS_DATA, VCNL4040_ADDR};
    uint8_t reg4[2] = {m_sample_lsb, m_sample_msb};
    uint8_t *p_ret = reg4;

    // The descriptor below is used to properly handle the VCNL4040's read protocol
    // The device address is selected, the (16-bit) register to read is passed and the (16-bit) data is returned
    // See the nrf SDK reference guide for more information on nrf_drv_twi_xfer_desc_t
    nrf_drv_twi_xfer_desc_t const vcnl_desc = {NRFX_TWI_XFER_TXRX, VCNL4040_ADDR, sizeof(reg3), sizeof(reg4), reg3, p_ret};
    nrf_drv_twi_xfer(&twi, &vcnl_desc, false); // initiates transfer using I2C descriptor above
    while(nrf_drv_twi_is_busy(&twi) == true); // waits for the transfer to complete

    m_sample_lsb = *p_ret; // the LSB of the returned data is stored
    p_ret++;               // the returned data pointer is incremented to point to the MSB data
    m_sample_msb = *p_ret; // the MSB of the returned data is stored

    m_sample = (((m_sample_msb) << 8) | (m_sample_lsb)); // the LSB and MSB of the data are concatenated to give full 16-bit data
    NRF_LOG_INFO("Proximity: %d", m_sample);
}

/**
 * @brief Function for main application entry.
 */
int main(void)
{
    APP_ERROR_CHECK(NRF_LOG_INIT(NULL));
    NRF_LOG_DEFAULT_BACKENDS_INIT();

    NRF_LOG_INFO("\r\nProximity sensor test code started.");
    NRF_LOG_FLUSH();
    twi_init(); // initializes I2C
	vcnl_config(); // configures the VCNL4040

    while (true)
    {
        nrf_delay_ms(10);
        read_sensor_data(); // reads the 16-bit proximity value recorded
        NRF_LOG_FLUSH();
    }
}

/** @} */
