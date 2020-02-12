#ifndef SENSOR_INTEGRATION_H
#define SENSOR_INTEGRATION_H

//ds1388 definitions
#include "ds1388.h"

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
 
/* Indicates if operation on VCNL4040 I2C has ended. */
static volatile bool m_xfer_done_vc = false;

/* TWI instance for VCNL4040. */
static const nrf_drv_twi_t vcnl_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);

/* Buffer for samples read from temperature sensor. */
static uint8_t m_sample_lsb;
static uint8_t m_sample_msb;
static uint16_t m_sample;

// Data structure for RTC data
typedef struct{
    uint8_t year;
    uint8_t month;
    uint8_t date;
    uint8_t day;
    uint8_t hour;
    uint8_t minute;
    uint8_t second;
    uint8_t hundreth;
} ds1388_data_t;

/* RTC variables. */
static uint8_t byte;
static uint8_t time_format =  HOUR_MODE_24; //select either 12-HOUR FORMAT or 24-HOUR FORMAT, if 12-HOUR FORMAT, use together with AM, PM eg: HOUR_MODE_12 | PM
static uint8_t init_time[8] = {20, 2, 11, 2, 7, 26, 10, 0};
//init_time[0] = 20; (year)
//init_time[1] = 2;  (month)
//init_time[2] = 1;  (date)
//init_time[3] = 6;  (day of the week, MONDAY: 1, TUESDAY: 2, ... SUNDAY: 7)
//init_time[4] = 4;  (hour) 
//init_time[5] = 25; (minutes) 
//init_time[6] = 0;  (seconds)
//init_time[7] = 0;  (hundredth of second)
 
/* Indicates if operation on DS1388 I2C has ended. */
static volatile bool m_xfer_done_ds = false;

/* TWI instance for DS1388. */
static const nrf_drv_twi_t ds_twi = NRF_DRV_TWI_INSTANCE(TWI_INSTANCE_ID);


typedef struct{
    int16_t accel_x;
    int16_t accel_y;
    int16_t accel_z;
    int16_t gyro_x;
    int16_t gyro_y;
    int16_t gyro_z;
} icm20649_data_t;

static void log_init(void);
void icm20649_read_test(void);
void icm20649_write_test(void);
void icm20649_convert_data(icm20649_data_t * data);
void icm20649_init(void);
int8_t icm20649_write_reg(uint8_t address, uint8_t data);
int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data);
int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes);
void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data);

void vcnl_config(void);
__STATIC_INLINE void data_handler(uint16_t prox);
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context);
void twi_init (void);
void read_sensor_data();

void adxl372_init(void);

uint8_t get_time(ds1388_data_t* date);

#endif //SENSOR_INTEGRATION_H