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

#include "sensors_integration.h"

//adxl372 driver
#include "adxl372.h"

//app_timer
#include "app_timer.h"
#include "nrf_drv_clock.h"

#define PROX_THRESHOLD 10000
#define IMPACT_DURATION 100 //in milliseconds
#define IMPACT_G_THRESHOLD 30000 //in mili-g's

uint16_t prox_val;
adxl372_accel_data_t g_high_G_buf[1024];
icm20649_data_t g_low_G_buf[1024];
ds1388_data_t g_rtc_buf[1024];
uint32_t g_buf_index = 0;

bool g_measurement_done = false;
APP_TIMER_DEF(m_measurement_timer_id);/**< Handler for measurement timer 
                                         used for the impact duration */ 

static void lfclk_request(void);
static void create_timers();
void record_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_g_data);
void impact_data_output (void);

/**@brief Timeout handler for the measurement timer.
 */
static void measurement_timer_handler(void * p_context)
{
    g_measurement_done = true;
}

int main (void)
{
    // Initialize.
    log_init();
    spi_init();
#ifdef USE_PROX
    twi_init_vcnl_4040();
#endif

    //for app_timer
    lfclk_request();

    NRF_LOG_INFO("Sensor test");

    icm20649_read_test();
    icm20649_write_test();

    //init sensors
    icm20649_init();
    adxl372_init();
    twi_init();
    ds_config();

#ifdef USE_PROX
    vcnl_config();
#endif

    app_timer_init();
    create_timers();
    icm20649_data_t low_g_gyro_data;
    adxl372_accel_data_t high_g_data;
    ds1388_data_t rtc_g_data;

    while(1)
    {   
#ifdef USE_PROX
        read_sensor_data();
        if(prox_val >= PROX_THRESHOLD)
        {
#endif
            adxl372_get_accel_data(&high_g_data);

            if(high_g_data.x >= IMPACT_G_THRESHOLD|| high_g_data.y >= IMPACT_G_THRESHOLD
                    || high_g_data.z >= IMPACT_G_THRESHOLD)
            {
                app_timer_start(m_measurement_timer_id,
                                 APP_TIMER_TICKS(IMPACT_DURATION),
                                 measurement_timer_handler);
                while(g_measurement_done == false)
                {
                    record_impact_data(&high_g_data, &low_g_gyro_data, &rtc_g_data);
                }
                app_timer_stop(m_measurement_timer_id);
                //reset for next impact
                g_measurement_done = false;
                impact_data_output();
            }
#ifdef USE_PROX
        }
#endif
    }

    return 0;
}

/**@brief Function starting the internal LFCLK oscillator.
 *
 * @details This is needed by RTC1 which is used by the Application Timer
 *          (When SoftDevice is enabled the LFCLK is always running and this is not needed).
 */
static void lfclk_request(void)
{
    ret_code_t err_code = nrf_drv_clock_init();
    APP_ERROR_CHECK(err_code);
    nrf_drv_clock_lfclk_request(NULL);
}

/**@brief Create timers.
 */
static void create_timers()
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_measurement_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT, 
                                measurement_timer_handler);
    APP_ERROR_CHECK(err_code);
}

void record_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_g_data)
{
    adxl372_get_accel_data(high_g_data);
    icm20649_read_gyro_accel_data(low_g_gyro_data);
    icm20649_convert_data(low_g_gyro_data);
    get_time(rtc_g_data);
    if (g_buf_index < 1024)
    {
        g_high_G_buf[g_buf_index] = *high_g_data;
        g_low_G_buf[g_buf_index] = *low_g_gyro_data;
        g_rtc_buf[g_buf_index] = *rtc_g_data;
        g_buf_index++;
    }
}

void impact_data_output (void)
{
    NRF_LOG_INFO("\r\n===================IMPACT DATA OUTPUT===================");
    for (int i = 0; i < g_buf_index; ++i)
    {
    NRF_LOG_INFO("id=%d, X accel = %d, Y accel = %d,  Z accel = %d mG",
                    i, g_high_G_buf[i].x, g_high_G_buf[i].y, g_high_G_buf[i].z);
    NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mg's, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
                    g_low_G_buf[i].accel_x, g_low_G_buf[i].accel_y, g_low_G_buf[i].accel_z,
                    g_low_G_buf[i].gyro_x, g_low_G_buf[i].gyro_y, g_low_G_buf[i].gyro_z);
    NRF_LOG_INFO("Date: %d, Day:    %d, Hour:   %d, Minute: %d, Second: %d, Hundreth: %d",
                    g_rtc_buf[i].date, g_rtc_buf[i].day,
                    g_rtc_buf[i].hour, g_rtc_buf[i].minute, g_rtc_buf[i].second, g_rtc_buf[i].hundreth);       
    }
    NRF_LOG_INFO("")
    g_buf_index = 0; //reset buf index
    memset(g_high_G_buf, 0x00, sizeof(g_high_G_buf));
    memset(g_low_G_buf, 0x00, sizeof(g_low_G_buf));
    memset(g_rtc_buf, 0x00, sizeof(g_rtc_buf));
    NRF_LOG_INFO("\r\n====================DATA OUTPUT FINISH==================");
    // TRANSFER DATA
    // after 100ms exit and check if perform a page read
}

void adxl372_init(void)
{
    /*
    GPIO for INT1 pin and INT2 pin,
    gpio_init();
    */

    //initialize device settings
    /* set up measurement mode */
    adxl372_reset();
    adxl372_set_op_mode(STAND_BY);
    //Please refer to figure 36 User offset trim profile for more info
    //For ADXL372 Vs=3.3V, x_offset = 0, y_offset=2, z_offset=5 
    adxl372_set_x_offset(0);
    adxl372_set_y_offset(2); //+10 LSB
    adxl372_set_z_offset(5); //+35 LSB
    adxl372_set_hpf_disable(true);
    adxl372_set_lpf_disable(true);
    adxl372_set_bandwidth(BW_3200HZ);
    adxl372_set_odr(ODR_6400HZ);
    adxl372_set_filter_settle(FILTER_SETTLE_16);
    adxl372_set_instaon_threshold(ADXL_INSTAON_HIGH_THRESH); //sets to 30g
    adxl372_write_mask(ADI_ADXL372_MEASURE, MEASURE_LOW_NOISE_MASK, MEASURE_LOW_NOISE_POS, LOW_NOISE);
    adxl372_set_op_mode(INSTANT_ON);

}

/********************ICM FUNCTIONS***************************/
void icm20649_read_test(void)
{
     /*********TEST READ******************/
    uint8_t who_am_i = 0x0;
    icm20649_read_reg(0x0, &who_am_i);
    NRF_LOG_INFO("who_am_i = 0x%x (0xE1)", who_am_i );
    if(who_am_i == 0xE1)
    {
        NRF_LOG_INFO("READ SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/
}

void icm20649_write_test(void)
{
    /*********TEST WRITE******************/

    uint8_t write_read;
    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1);

    icm20649_read_reg(0x06, &write_read);

    NRF_LOG_INFO("write_read = 0x%x (0x1)", write_read );
    if(write_read == 0x1)
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR"); 
       // while(1);
    }

    /********************************************/
}

void icm20649_init(void)
{
    //USER CTRL disable all
    icm20649_write_reg(0x03, 0x0);

    //LP_CONFIG disable duty cycle mode
    icm20649_write_reg(0x05, 0x0);

    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1);

    //PWR_MGMT 2 enable accel & gyro
    icm20649_write_reg(0x07, 0x0);
    
    //REG BANK SEL select userbank 2
    icm20649_write_reg(0x7F, 0x20);

    //GYRO_CONFIG_1 bypass gyro DLPF, 2000dps
    icm20649_write_reg(0x1, 0x4);

    //GYRO_CONFIG_2 disable self test, no avging
    icm20649_write_reg(0x2, 0x0);

    //GYRO_CONFIG_2 disable self test, no avging
    icm20649_write_reg(0x14, 0x6);

    //REG BANK SEL select userbank 0
    icm20649_write_reg(0x7F, 0x0);
}

void icm20649_convert_data(icm20649_data_t * data)
{
    float deg2rad = 3.1415/180.0;

    data->accel_x = ((float) data->accel_x/1024.0)*1000;
    data->accel_y = ((float) data->accel_y/1024.0)*1000;
    data->accel_z = ((float) data->accel_z/1024.0)*1000;
    data->gyro_x = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_y = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_z = ((float) data->gyro_x / 32767.0) * 2000.0 * deg2rad *1000;
}

int8_t icm20649_write_reg(uint8_t address, uint8_t data)
{
    uint8_t tx_msg[2];
    uint8_t rx_buf[2];
    tx_msg[0] = address;
    tx_msg[1] = data;

    return spi_write_and_read(SPI_ICM20649_CS_PIN, tx_msg, 2, rx_buf, 2 ); // send 2 bytes
}

int8_t icm20649_read_reg(uint8_t address, uint8_t * reg_data)
{
    uint8_t reg_addr;
    int8_t ret;
    uint8_t rx_buf[2];

    reg_addr = (uint8_t)  ( address | 0x80 ); //set 1st bit for reads
    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &reg_addr, 1, rx_buf, 2);

    *reg_data = rx_buf[1];

    return ret;
}

int8_t icm20649_multibyte_read_reg( uint8_t reg_addr, uint8_t* reg_data, uint8_t num_bytes) 
{
    uint8_t read_addr;
    uint8_t buf[257]; 
    int8_t ret;
    
    if(num_bytes > 256)
        return -1;

    read_addr = reg_addr | 0x80; //set MSB to 1 for read
    memset( buf, 0x00, num_bytes + 1);

    ret = spi_write_and_read(SPI_ICM20649_CS_PIN, &read_addr, 1, buf, num_bytes + 1 );
    if (ret < 0)
        return ret;
    
    memcpy(reg_data, &buf[1], num_bytes);

    return ret;
}

void icm20649_read_gyro_accel_data(icm20649_data_t *icm20649_data)
{
    uint8_t rx_buf[12] = {0};

    //REG BANK SEL select userbank 0
    icm20649_write_reg(0x7F, 0x0);

    icm20649_multibyte_read_reg( 0x2D, rx_buf, 12);

    icm20649_data->accel_x = rx_buf[0]<<8 | rx_buf[1];
    icm20649_data->accel_y = rx_buf[2]<<8 | rx_buf[3];
    icm20649_data->accel_z = rx_buf[4]<<8 | rx_buf[5];
    icm20649_data->gyro_x = rx_buf[6]<<8 | rx_buf[7];
    icm20649_data->gyro_y = rx_buf[8]<<8 | rx_buf[9];
    icm20649_data->gyro_z = rx_buf[10]<<8 | rx_buf[11];
    

}
/**************************************************/
static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}


/************************VCNL FUNCTIONS ****************************/
/**
 * @brief Function for setting active mode on VCNL4040 proximity sensor
 */
void vcnl_config(void)
{	
	uint8_t reg1[3] = {VCNL4040_PS_CONF3, ps_conf3_data, ps_ms_data};
    nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg1, sizeof(reg1), false);
    while (m_xfer_done_vc == false);
	
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg2, sizeof(reg2), false);
    while (m_xfer_done_vc == false);
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
void twi_handler_vcnl(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            {
                prox_val = m_sample;
            }
            m_xfer_done_vc = true;
            break;
        default:
            break;
    }
}

/**
 * @brief UART initialization.
 */
void twi_init_vcnl_4040 (void)
{
    ret_code_t err_code;

    const nrf_drv_twi_config_t twi_lm75b_config = {
       .scl                = VCNL4040_SCL,
       .sda                = VCNL4040_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_100K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&vcnl_twi, &twi_lm75b_config, twi_handler_vcnl, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&vcnl_twi);
}

/**
 * @brief Function for reading data from VCNL4040 sensor.
 */
void read_sensor_data()
{
    do
    {
        __WFE();
    }while (m_xfer_done_vc == false);
    
    m_xfer_done_vc = false;

    uint8_t reg3[2] = {VCNL4040_PS_DATA, VCNL4040_ADDR};
    uint8_t reg4[2] = {m_sample_lsb, m_sample_msb};
    uint8_t *p_ret = reg4;
    nrf_drv_twi_xfer_desc_t const vcnl_desc = {NRFX_TWI_XFER_TXRX, VCNL4040_ADDR, sizeof(reg3), sizeof(reg4), reg3, p_ret};
    nrf_drv_twi_xfer(&vcnl_twi, &vcnl_desc, false);
    while(nrf_drv_twi_is_busy(&vcnl_twi) == true);

    m_sample_lsb = *p_ret;
    p_ret++;
    m_sample_msb = *p_ret;

    m_sample = (((m_sample_msb) << 8) | (m_sample_lsb));
    prox_val = m_sample;
    NRF_LOG_INFO("Proximity: %d", m_sample);
}

/************************DS1388 FUNCTIONS ****************************/

/**
 * @brief Function for converting from dec to hex
 */
uint8_t dec2hex(uint8_t val) {
  val = val + 6 * (val / 10);
  return val;
}

/**
 * @brief Function for converting from hex to dec
 */
uint8_t hex2dec(uint8_t val) {
  val = val - 6 * (val >> 4);
  return val;
}

/**
 * @brief Function for setting active mode on DS1388 RTC
 */
void ds_config(void)
{	
	uint8_t reg0[2] = {CONTROL_REG, (EN_OSCILLATOR | DIS_WD_COUNTER)};
    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg0, sizeof(reg0), false);
    while (m_xfer_done_ds == false);
    
    uint8_t reg1[9] = {HUNDRED_SEC_REG,
                        dec2hex(init_time[7]),
                        dec2hex(init_time[6]),
                        dec2hex(init_time[5]),
                        (dec2hex(init_time[4]) | time_format),
                        dec2hex(init_time[3]),
                        dec2hex(init_time[2]),
                        dec2hex(init_time[1]),
                        dec2hex(init_time[0])
    };

    nrf_drv_twi_tx(&ds_twi, DS1388_ADDRESS, reg1, sizeof(reg1), false);
    while (m_xfer_done_ds == false);
}

/**
 * @brief TWI events handler.
 */
void twi_handler_ds(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            m_xfer_done_ds = true;
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //m_xfer_done_ds = true;
            NRF_LOG_INFO("\r\nDATA NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //m_xfer_done_ds = true;
            NRF_LOG_INFO("\r\nADDRESS NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        default:
            //m_xfer_done_ds = true
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
       .scl                = DS1388_SCL,
       .sda                = DS1388_SDA,
       .frequency          = NRF_DRV_TWI_FREQ_400K,
       .interrupt_priority = APP_IRQ_PRIORITY_HIGH,
       .clear_bus_init     = false
    };

    err_code = nrf_drv_twi_init(&ds_twi, &twi_lm75b_config, twi_handler_ds, NULL);
    APP_ERROR_CHECK(err_code);

    nrf_drv_twi_enable(&ds_twi);
}

uint8_t readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done_ds == false);

    m_xfer_done_ds = false;

    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};

    nrf_drv_twi_xfer(&ds_twi, &ds_desc, false);
    
    while(nrf_drv_twi_is_busy(&ds_twi) == true);

    return byte;
}

uint8_t get_time(ds1388_data_t* date)
{
  uint8_t ret;
  
  date->year = readRegister(YEAR_REG);
  date->month = readRegister(MONTH_REG);
  date->date = readRegister(DATE_REG);
  date->day = readRegister(DAY_REG);
  date->hour = readRegister(HOUR_REG);
  date->minute = readRegister(MIN_REG);
  date->second = readRegister(SEC_REG);
  date->hundreth = readRegister(HUNDRED_SEC_REG);
  
  //Time processing 
  date->year = hex2dec(date->year);
  date->month = hex2dec(date->month);
  date->date = hex2dec(date->date);
  date->minute = hex2dec(date->minute);
  date->second = hex2dec(date->second);
  date->hundreth = hex2dec(date->hundreth);

  
  if ((date->hour & 0x40) == HOUR_MODE_24)
  {
    date->hour = hex2dec(date->hour);
    ret = 2;
    return ret;
  }
  else
  { 
    ret = (date->hour & 0x20 )>> 5;
    date->hour = hex2dec(date->hour & 0x1F);
    return ret;
  }
}
