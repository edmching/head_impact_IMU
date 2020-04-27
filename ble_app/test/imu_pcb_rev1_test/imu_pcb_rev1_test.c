//-------------------------------------------
// Title: imu_pcb_rev1_test.c
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This program demostrates the following
// two modes high impact storage mode and continuous sampling mode.
// At the startup it will test all sensors 
// and configure them to their default states
//
// high impact storage mode
// 1. Proximity sensor detection
// 2. high impact detection
// 3. Store a 2 max of number seperate impacts to flash
// 4. Serially outputs the impact data via physical UART connection
// 
// This program can also run in continuous sampling mode by uncomment USE_CONT_SAMPLE_MODE
// Continuous sampling mode
// 1. samples accel, gyro, proximity, and rtc sensors
// 2. Does not use flash
// 
//-------------------------------------------

//general c libraries
#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"
#include "nrf_delay.h"
#include "nrf_gpio.h"
#include "system_nrf52.h"

//sensor drivers
#include "spi_driver.h"
#include "adxl372.h"
#include "icm20649.h"
#include "vcnl4040.h"
#include "ds1388.h"
#include "mt25ql256aba.h"

//app_timer
#include "app_timer.h"
#include "nrf_drv_clock.h"


//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

//Uncomment to use continuous sampling mode and disable impact storage mode
//#define USE_CONT_SAMPLE_MODE

//keep in small since there is limited ram space
#define MAX_SAMPLE_BUF_LENGTH 500 
#define IMPACT_G_THRESHOLD 10000 //in milli-g's
#define IMPACT_DURATION 100 //in milliseconds

typedef struct{
    adxl372_accel_data_t adxl_data;
    icm20649_data_t icm_data;
    ds1388_data_t ds_data;
    uint8_t padding[6]; //to align with 32byte
}impact_sample_t;

//Variable to temporary store the sample set in RAM and to transfer over to flash
impact_sample_t g_sample_set_buf[MAX_SAMPLE_BUF_LENGTH];
impact_sample_t g_flash_output_buf[MAX_SAMPLE_BUF_LENGTH];
uint32_t g_buf_index = 0;

//Variable to know when the sampling is finished
bool g_measurement_done = false;

//variable that enables reading the rtc at the beginning of every high impact
bool g_record_timestamp = false;
APP_TIMER_DEF(m_measurement_timer_id);/**< Handler for measurement timer 
                                         used for the impact duration */ 

//const uint32_t UICR_ADDR_0x20C __attribute__ ((section(".uicrNfcPinsAddress"))) __attribute__((used));
void log_init(void);
static void lfclk_request(void);
static void create_timers(void);
static void spi_ret_check(int8_t ret);
void spi_accel_init(void);
void spi_gyro_init(void);
void spi_flash_init(void);
void spi_accel_uninit(void);
void spi_gyro_uninit(void);
void spi_flash_uninit(void);
void adxl372_init(void);
void sample_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_data);
void flash_store_samples(uint32_t* flash_addr);
void flash_retrieve_samples(uint32_t* flash_addr);
void serial_debug_output_flash_data(void);
void serial_output_flash_data(void);

/**@brief Timeout handler for the measurement timer.
 */
static void measurement_timer_handler(void * p_context)
{
    g_measurement_done = true;
}

//================================================================//
// NOTE: These two functions below are ONLY for PCB REV1
// Due to the limition number of spi/twi peripherals (3)
// accel and flash have separate pin config and will require
// uninitializing one inorder to switch between devices.
// In PCB REV2 we will have the same MOSI, MISO, CLK and
// separate cs pins to switch between devices

// Switch to accel spi instance from flash spi instance 
// MUST BE CALLED before a accel cmd and when flash
// is initialized already! 
void spi_switch_to_accel_from_flash(void)
{
    spi_flash_uninit();
    spi_accel_init();
}

// Switch to flash spi instance from accel spi instance
// MUST BE CALLED before a flash command and when accel
// is initialized already! 
void spi_switch_to_flash_from_accel(void)
{
    spi_accel_uninit();
    spi_flash_init();
}
//================================================================//

int main (void)
{
    // Initialize.
    SystemInit();
    log_init();
    spi_accel_init();
    spi_gyro_init();
    twi_init();

    //for app_timer
    lfclk_request();

    //init accel
    adxl372_test();
#ifndef USE_CONT_SAMPLING_MODE
    adxl372_init();
#

    //init gyro
    icm20649_test();
    icm20649_default_init();

    spi_switch_to_flash_from_accel();
    mt25ql256aba_startup_test();
    mt25ql256aba_bulk_erase();

    spi_switch_to_accel_from_flash();

    vcnl4040_config();
    NRF_LOG_INFO("CONFIG RTC");
    ds1388_config();

    app_timer_init();
    create_timers();

    icm20649_data_t low_g_gyro_data;
    adxl372_accel_data_t high_g_data;
    ds1388_data_t rtc_data;
    uint32_t flash_addr = MT25QL256ABA_LOW_128MBIT_SEGMENT_ADDRESS_START;
    uint32_t num_impacts = 0;

#ifndef USE_CONT_SAMPLE_MODE
    while(1){
        vcnl4040_read_sensor_data();
        adxl372_get_accel_data(&high_g_data);

        //NRF_LOG_INFO("%d, %d, %d", high_g_data.x, high_g_data.y, high_g_data.z);

        g_record_timestamp = false;
        if(high_g_data.x >= IMPACT_G_THRESHOLD|| high_g_data.y >= IMPACT_G_THRESHOLD
                || high_g_data.z >= IMPACT_G_THRESHOLD)
        {
#ifdef DEBUG
            NRF_LOG_INFO("");
            NRF_LOG_INFO("BEGIN MEASUREMENT");
#endif
            app_timer_start(m_measurement_timer_id,
                             APP_TIMER_TICKS(IMPACT_DURATION),
                             measurement_timer_handler);
            while(g_measurement_done == false)
            {
                sample_impact_data(&high_g_data, &low_g_gyro_data, &rtc_data);
            }
            app_timer_stop(m_measurement_timer_id);
            //reset for next impact
            g_measurement_done = false;
            spi_switch_to_flash_from_accel();
            flash_store_samples(&flash_addr);
            flash_retrieve_samples(&flash_addr);
            serial_output_flash_data();
            spi_switch_to_accel_from_flash();
            num_impacts++;
            if(num_impacts == 2)
            {
                while(1)
                {
                    __WFE();
                }
            }  

        }
    }
#endif

#ifdef USE_CONT_SAMPLE_MODE
    adxl372_accel_data_t accel_data;
    icm20649_data_t gyro_data;
    ds1388_data_t ds_data;
    while(1)
    {
        vcnl4040_read_sensor_data();
        icm20649_read_gyro_accel_data(&gyro_data);
        icm20649_convert_data(&gyro_data);
        adxl372_get_accel_data(&accel_data);
        ds1388_get_time(&ds_data);

        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mG",
                        accel_data.x, accel_data.y, accel_data.z);
        NRF_LOG_RAW_INFO("accel x = %d, accel y = %d, accel z = %d, gyro x = %d, gyro y = %d, gyro z = %d \r\n", 
        gyro_data.accel_x, gyro_data.accel_y, gyro_data.accel_z,
        gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z );
        
        NRF_LOG_INFO("      Date: %d, Day:    %d, Hour:   %d, Minute: %d, Second: %d, Hundreth: %d",
                    ds_data.date, ds_data.day,
                    ds_data.hour, ds_data.minute,
                    ds_data.second, ds_data.hundreth);       
        
        nrf_delay_ms(1000);
    }
#endif

    return 0;
}

void sample_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_data)
{
    if(g_record_timestamp == false)
    {
        ds1388_get_time(rtc_data);
        g_sample_set_buf[g_buf_index].ds_data = *rtc_data;
        g_record_timestamp = true;
    }
    adxl372_get_accel_data(high_g_data);
    icm20649_read_gyro_accel_data(low_g_gyro_data);
    icm20649_convert_data(low_g_gyro_data);
    if (g_buf_index < MAX_SAMPLE_BUF_LENGTH)
    {
        g_sample_set_buf[g_buf_index].adxl_data = *high_g_data;
        g_sample_set_buf[g_buf_index].icm_data = *low_g_gyro_data;
        g_buf_index++;
    }
}

void flash_store_samples(uint32_t* flash_addr)
{
    uint8_t flash_addr_buf[3]= {0};
    uint8_t sample_size_bytes = sizeof(impact_sample_t);
    uint8_t *sample_byte_ptr;
    int8_t ret;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("BEGIN STORE SAMPLES...");
    //store one impact sample set to flash
    for (int i = 0; i < g_buf_index; ++i)
    {
#ifdef DEBUG
        NRF_LOG_INFO("");
        NRF_LOG_INFO("WRITE: ID: %d, addr: 0x%03x", i, *flash_addr);
#endif

        mt25ql256aba_check_write_in_progress_flag();
        convert_4byte_address_to_3byte_address(*flash_addr, flash_addr_buf);
        sample_byte_ptr = (uint8_t*) &g_sample_set_buf[i];
        mt25ql256aba_write_enable();
        ret = mt25ql256aba_write_op(MT25QL256ABA_PAGE_PROGRAM, 
                              flash_addr_buf,
                              sizeof(flash_addr_buf), 
                              sample_byte_ptr,
                              sample_size_bytes);
        mt25ql256aba_write_disable();
        *flash_addr = *flash_addr + sample_size_bytes; 
        spi_ret_check(ret);
    }
}

void flash_retrieve_samples(uint32_t* flash_addr)
{
    uint8_t sample_size_bytes = sizeof(impact_sample_t);
    uint32_t addr32 = *flash_addr-sample_size_bytes*g_buf_index;
    uint8_t addr[3] = {0};
    uint8_t *sample_byte_ptr;

#ifdef DEBUG
    NRF_LOG_INFO("");
    NRF_LOG_INFO("BEGIN RETRIEVE SAMPLES");
#endif
    for(int i = 0; i < g_buf_index; ++i) 
    {
        mt25ql256aba_check_write_in_progress_flag();
        convert_4byte_address_to_3byte_address(addr32, addr);
        sample_byte_ptr = (uint8_t*) &g_flash_output_buf[i];
        mt25ql256aba_read_op(MT25QL256ABA_READ,
                              addr,
                              sizeof(addr),
                              sample_byte_ptr,
                              sample_size_bytes);
#ifdef DEBUG
        NRF_LOG_INFO("READ: ID: %d, addr: 0x%03x, OUTPUT: %d (%d)",
                       i, addr32, g_flash_output_buf[i].adxl_data.x,
                        g_sample_set_buf[i].adxl_data.x);
#endif
        addr32 = addr32 + sample_size_bytes;
    }
}

void serial_output_flash_data(void)
{
    NRF_LOG_INFO("\r\n===================IMPACT DATA OUTPUT===================");
    for (int i = 0; i < g_buf_index; ++i)
    {
        NRF_LOG_INFO("");
        NRF_LOG_INFO("ID = %d", i);
        NRF_LOG_INFO("accel x= %d, accel y = %d,  accel z= %d mG's",
                        g_flash_output_buf[i].adxl_data.x, 
                        g_flash_output_buf[i].adxl_data.y,
                        g_flash_output_buf[i].adxl_data.z);
        NRF_LOG_INFO("      accel x = %d, accel y = %d, accel z = %d mG's",
                            g_flash_output_buf[i].icm_data.accel_x,
                            g_flash_output_buf[i].icm_data.accel_y,
                            g_flash_output_buf[i].icm_data.accel_z);
        NRF_LOG_INFO("      gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
                            g_flash_output_buf[i].icm_data.gyro_x,
                            g_flash_output_buf[i].icm_data.gyro_y,
                            g_flash_output_buf[i].icm_data.gyro_z);
        if(i == 0)
        {
            NRF_LOG_INFO("      date: %d day: %d",
                            g_flash_output_buf[i].ds_data.date,
                            g_flash_output_buf[i].ds_data.day);
            NRF_LOG_INFO("      Year: %d Month: %d Hour: %d ",
                                g_flash_output_buf[i].ds_data.year,
                                g_flash_output_buf[i].ds_data.month,
                                g_flash_output_buf[i].ds_data.hour);
            NRF_LOG_INFO("      Minute: %d, Second: %d, Hundreth: %d",
                            g_flash_output_buf[i].ds_data.minute,
                            g_flash_output_buf[i].ds_data.second,
                            g_flash_output_buf[i].ds_data.hundreth); 
        }
    }
    g_buf_index = 0; //reset buf index
    memset(g_flash_output_buf, 0x00, sizeof(g_flash_output_buf));
    NRF_LOG_INFO("\r\n====================DATA OUTPUT FINISH==================");
}

//depreciated
void serial_debug_output_flash_data(void)
{
    NRF_LOG_INFO("\r\n===================IMPACT DATA OUTPUT===================");
    for (int i = 0; i < g_buf_index; ++i)
    {
        NRF_LOG_INFO("");
        NRF_LOG_INFO("ID = %d", i);

        if(g_flash_output_buf[i].adxl_data.x != g_sample_set_buf[i].adxl_data.x ||
            g_flash_output_buf[i].adxl_data.y != g_sample_set_buf[i].adxl_data.y ||
            g_flash_output_buf[i].adxl_data.z != g_sample_set_buf[i].adxl_data.z)
        {
            NRF_LOG_INFO("accel x= %d (%d) accel y = %d (%d),  accel z= %d(%d) mG's",
                            g_flash_output_buf[i].adxl_data.x, g_sample_set_buf[i].adxl_data.x, 
                            g_flash_output_buf[i].adxl_data.y, g_sample_set_buf[i].adxl_data.y,
                            g_flash_output_buf[i].adxl_data.z, g_sample_set_buf[i].adxl_data.z);
        }
        if(g_flash_output_buf[i].icm_data.accel_x != g_sample_set_buf[i].icm_data.accel_x ||
            g_flash_output_buf[i].icm_data.accel_y != g_sample_set_buf[i].icm_data.accel_y||
            g_flash_output_buf[i].icm_data.accel_z != g_sample_set_buf[i].icm_data.accel_z)
        {
            NRF_LOG_INFO("      accel x = %d (%d), accel y = %d (%d), accel z = %d (%d) mG's",
                                g_flash_output_buf[i].icm_data.accel_x, g_sample_set_buf[i].icm_data.accel_x,
                                g_flash_output_buf[i].icm_data.accel_y, g_sample_set_buf[i].icm_data.accel_y,
                                g_flash_output_buf[i].icm_data.accel_z, g_sample_set_buf[i].icm_data.accel_z);
        }
        if(g_flash_output_buf[i].icm_data.gyro_x != g_sample_set_buf[i].icm_data.gyro_x ||
            g_flash_output_buf[i].icm_data.gyro_y != g_sample_set_buf[i].icm_data.gyro_y ||
            g_flash_output_buf[i].icm_data.gyro_z != g_sample_set_buf[i].icm_data.gyro_z)
        {
            NRF_LOG_INFO("      gyro x = %d (%d), gyro y = %d (%d), gyro z = %d (%d) mrad/s", 
                                g_flash_output_buf[i].icm_data.gyro_x, g_sample_set_buf[i].icm_data.gyro_x,
                                g_flash_output_buf[i].icm_data.gyro_y, g_sample_set_buf[i].icm_data.gyro_y,
                                g_flash_output_buf[i].icm_data.gyro_z, g_sample_set_buf[i].icm_data.gyro_z);
        }

        if(g_flash_output_buf[i].ds_data.date!= g_sample_set_buf[i].ds_data.date ||
            g_flash_output_buf[i].ds_data.day != g_sample_set_buf[i].ds_data.day)
        {
            NRF_LOG_INFO("      date: %d (%d) day: %d(%d)",
                            g_flash_output_buf[i].ds_data.date, g_sample_set_buf[i].ds_data.date,
                            g_flash_output_buf[i].ds_data.day,  g_sample_set_buf[i].ds_data.day);
        }
        if(g_flash_output_buf[i].ds_data.year!= g_sample_set_buf[i].ds_data.year ||
            g_flash_output_buf[i].ds_data.month != g_sample_set_buf[i].ds_data.month ||
            g_flash_output_buf[i].ds_data.hour != g_sample_set_buf[i].ds_data.hour)
        {
            NRF_LOG_INFO("      Year: %d (%d) Month: %d(%d) Hour: %d(%d) ",
                            g_flash_output_buf[i].ds_data.year, g_sample_set_buf[i].ds_data.year,
                            g_flash_output_buf[i].ds_data.month, g_sample_set_buf[i].ds_data.month,
                            g_flash_output_buf[i].ds_data.hour, g_sample_set_buf[i].ds_data.hour);
        }
        if(g_flash_output_buf[i].ds_data.minute != g_sample_set_buf[i].ds_data.minute||
            g_flash_output_buf[i].ds_data.second != g_sample_set_buf[i].ds_data.second ||
            g_flash_output_buf[i].ds_data.hundreth != g_sample_set_buf[i].ds_data.hundreth)
        NRF_LOG_INFO("      Minute: %d (%d), Second: %d (%d), Hundreth: %d (%d)",
                        g_flash_output_buf[i].ds_data.minute,g_sample_set_buf[i].ds_data.minute,
                        g_flash_output_buf[i].ds_data.second, g_sample_set_buf[i].ds_data.second,
                        g_flash_output_buf[i].ds_data.hundreth, g_sample_set_buf[i].ds_data.hundreth); 
    }
    //g_buf_index = 0; //reset buf index
    memset(g_flash_output_buf, 0x00, sizeof(g_flash_output_buf));
    NRF_LOG_INFO("\r\n====================DATA OUTPUT FINISH==================");
}


void adxl372_init(void)
{
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



void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

void spi_accel_init(void)
{
    ret_code_t err_code = nrf_drv_spi_init(&accel_spi, &accel_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

}

void spi_gyro_init(void)
{
    ret_code_t err_code = nrf_drv_spi_init(&gyro_spi, &gyro_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
}


void spi_flash_init(void)
{
    ret_code_t err_code = nrf_drv_spi_init(&flash_spi, &flash_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);
}

void spi_accel_uninit(void)
{
    nrf_drv_spi_uninit(&accel_spi);
}

void spi_flash_uninit(void)
{
    nrf_drv_spi_uninit(&flash_spi);
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
static void create_timers(void)
{
    ret_code_t err_code;

    // Create timers
    err_code = app_timer_create(&m_measurement_timer_id,
                                APP_TIMER_MODE_SINGLE_SHOT, 
                                measurement_timer_handler);
    APP_ERROR_CHECK(err_code);
}

static void spi_ret_check(int8_t ret)
{
    if (ret < 0){
        NRF_LOG_INFO("SPI WRITE READ FAIL");
    }
}
