#ifndef SENSOR_INTEGRATION_H
#define SENSOR_INTEGRATION_H

//standard variable types
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"

#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

//I2C driver library
#include "nrf_drv_twi.h"

//adxl372 driver
#include "adxl372.h"

//app_timer
#include "app_timer.h"
#include "nrf_drv_clock.h"

//flash driver
#include "mt25ql256aba.h"

#define PROX_THRESHOLD 10000
#define IMPACT_DURATION 100 //in milliseconds
#define IMPACT_G_THRESHOLD 30000 //in mili-g's
#define MAX_SAMPLE_BUF_LENGTH 502

// I2C pin assignment
#define I2C_SDA 17
#define I2C_SCL 18

// TWI instance ID
#define TWI_INSTANCE_ID     1

// Proximity sensor address
#define VCNL4040_ADDR 0x60U //U >> 1

// Proximity sensor command codes
#define VCNL4040_PS_CONF1 0x03U //Lower
#define VCNL4040_PS_CONF2 0x03U //Upper
#define VCNL4040_PS_CONF3 0x04U //Lower
#define VCNL4040_PS_MS 0x04U //Upper
#define VCNL4040_PS_DATA 0x08U

/* Mode for LM75B. */
#define NORMAL_MODE 0U

// Proximity sensor configuration register values
static uint8_t ps_conf1_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (1 << 2) | (1 << 1) | (0 << 0);
static uint8_t ps_conf2_data =	(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (1 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_conf3_data =	(0 << 7) | (0 << 6) | (0 << 5) | (1 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
static uint8_t ps_ms_data =		(0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (1 << 2) | (1 << 1) | (1 << 0);
//static uint8_t blank =		    (0 << 7) | (0 << 6) | (0 << 5) | (0 << 4) | (0 << 3) | (0 << 2) | (0 << 1) | (0 << 0);
 
/* Indicates if operation on I2C has ended. */
static volatile bool m_xfer_done = false;

/* TWI instance. */
static const nrf_drv_twi_t vcnl_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample_lsb;
static uint8_t m_sample_msb;
static uint16_t m_sample;


typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

typedef struct{
    icm20649_data_t icm_data;
    adxl372_accel_data_t adxl_data;
}impact_sample_set_t;

uint16_t prox_val;
adxl372_accel_data_t g_high_G_buf[MAX_SAMPLE_BUF_LENGTH];
icm20649_data_t g_low_G_buf[MAX_SAMPLE_BUF_LENGTH];
impact_sample_set_t g_sample_set_buf[MAX_SAMPLE_BUF_LENGTH];
impact_sample_set_t g_flash_output_buf[MAX_SAMPLE_BUF_LENGTH];
uint32_t g_buf_index = 0;

bool g_measurement_done = false;
APP_TIMER_DEF(m_measurement_timer_id);/**< Handler for measurement timer 
                                         used for the impact duration */ 

void icm20649_read_test(void);
void icm20649_write_test(void);
void icm20649_convert_data(icm20649_data_t * data);
void icm20649_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);

void sample_impact_data(adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data);
void serial_output_impact_data(void);
void mt25ql256aba_store_samples(uint8_t* flash_addr_ptr);
void mt25ql256aba_retrieve_samples(void);
void serial_output_flash_data(void);

void vcnl_config(void);
__STATIC_INLINE void data_handler(uint16_t prox);
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init (void);
void read_sensor_data();

void adxl372_init(void);

#endif //SENSOR_INTEGRATION_H