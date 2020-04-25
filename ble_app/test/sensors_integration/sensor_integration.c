//-------------------------------------------
// Title: sensor_integration.c
// Author: UBC Capstone Team 48 - 2019/2020
// Description: This file brings together the
// test code written for each sensor and
// integrates them by performing the
// following steps:
// 1. Sensor/peripheral initialization
// 2. Proximity sensor threshold detection
// 3. High-g data threshold detection
// 4. 
//-------------------------------------------

#include "sensors_integration.h"
#include "nrf_delay.h"

#define NUM_SAMPLES 50
#define USE_DEMO_APP
#define USE_PROX

static void log_init(void);
static void lfclk_request(void);
static void create_timers(void);
void mt25ql256aba_startup_test(void);
static void spi_ret_check(int8_t ret);
void adxl372_startup_test(void);
void mt25ql256aba_erase(void);
void mt25ql256aba_check_ready_flag(void);
void bulk_erase(void);
void full_page_read(void);
void flash_read_bytes(uint16_t num_bytes);
void sample_test_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_data);
void convert_4byte_address_to_3byte_address(uint32_t flash_addr, uint8_t* flash_addr_buf);
void prototype_mt25ql256aba_store_samples(uint32_t* flash_addr);

/**@brief Timeout handler for the measurement timer.
 */
static void measurement_timer_handler(void * p_context)
{
    g_measurement_done = true;
}

void reset_device(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("RESETING DEVICE....");
    mt25ql256aba_check_ready_flag();
    mt25ql256aba_read_op(MT25QL256ABA_RESET_ENABLE, NULL, 0, NULL, 0);
    mt25ql256aba_read_op(MT25QL256ABA_RESET_MEMORY, NULL, 0, NULL, 0);
}

int main (void)
{
    // Initialize.
    log_init();
    spi_init();
    flash_spi_init();
//#ifdef USE_PROX
    //twi_init_vcnl_4040();
//#endif

    //for app_timer
    lfclk_request();

    NRF_LOG_INFO("");
    NRF_LOG_INFO("Sensors test");

    mt25ql256aba_startup_test();
    icm20649_read_test();
    icm20649_write_test();
    adxl372_startup_test();
    //init sensors
    icm20649_init();
    adxl372_init();
    NRF_LOG_INFO("TWI start");
    twi_init();
    NRF_LOG_INFO("DS config start");
    ds_config();
    NRF_LOG_INFO("DS config done");

    mt25ql256aba_erase();
    //bulk_erase();
    //reset_device();
    //nrf_delay_ms(1000);
    NRF_LOG_INFO("Flash Erased");
    //full_page_read();
#ifdef USE_PROX
    NRF_LOG_INFO("VCNL start");
    vcnl_config();
    NRF_LOG_INFO("VCNL configured");
#endif

    app_timer_init();
    create_timers();
    icm20649_data_t low_g_gyro_data;
    adxl372_accel_data_t high_g_data;
    ds1388_data_t rtc_data;
    uint32_t flash_addr = MT25QL256ABA_LOW_128MBIT_SEGMENT_ADDRESS_START;

#ifndef USE_DEMO_APP
    while(1)
    {
        for(int i = 0; i <15; ++i)
            sample_test_impact_data(&high_g_data, &low_g_gyro_data, &rtc_data);

        //prototype_mt25ql256aba_store_samples(&flash_addr);
        mt25ql256aba_store_samples(&flash_addr);
        //mt25ql256aba_retrieve_samples();
        serial_output_flash_data();

        while(1)
        {
            __WFE();
        }
    }
#endif

#ifdef USE_DEMO_APP
    while(1)
    {   
#ifdef USE_PROX
        read_sensor_data();
#endif      
            nrf_delay_ms(500);
            adxl372_get_accel_data(&high_g_data);

            NRF_LOG_INFO("%d, %d, %d", high_g_data.x, high_g_data.y, high_g_data.z);

            if(high_g_data.x >= IMPACT_G_THRESHOLD|| high_g_data.y >= IMPACT_G_THRESHOLD
                    || high_g_data.z >= IMPACT_G_THRESHOLD)
            {
                NRF_LOG_INFO("");
                NRF_LOG_INFO("BEGIN MEASUREMENT");
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
                //prototype_mt25ql256aba_store_samples(&flash_addr);
                mt25ql256aba_store_samples(&flash_addr);
                mt25ql256aba_retrieve_samples();
                serial_output_flash_data();
                while(1)
                {
                    __WFE();
                }

            }
    }
#endif

    return 0;
}

void sample_test_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_data)
{
    static uint32_t i = 0;
    high_g_data->x = i;
    high_g_data->y = i+1;
    high_g_data->z = i+2;
    low_g_gyro_data->accel_x = i+3;
    low_g_gyro_data->accel_y = i+4;
    low_g_gyro_data->accel_z = i+5;
    low_g_gyro_data->gyro_x = i+6;
    low_g_gyro_data->gyro_y = i+7;
    low_g_gyro_data->gyro_z = i+8;
    rtc_data->year = i+9;
    rtc_data->month = i+10;
    rtc_data->date = i +11;
    rtc_data->day = i+12;
    rtc_data->hour = i+13;
    rtc_data->minute = i+14;
    rtc_data->second = i+15;
    rtc_data->hundreth = i+16;
    if (g_buf_index < MAX_SAMPLE_BUF_LENGTH)
    {
        g_sample_set_buf[g_buf_index].adxl_data = *high_g_data;
        g_sample_set_buf[g_buf_index].icm_data = *low_g_gyro_data;
        g_sample_set_buf[g_buf_index].ds_data = *rtc_data;
        g_buf_index++;
        i = i + 17;
    }
}

//TODO debug issue with not being able to output more than 607 bytes
void flash_read_bytes(uint16_t num_bytes)
{
    uint32_t flash_addr = 0x00000000;
    uint8_t addr_buf[3] = {0};
    uint8_t data;
    int8_t ret;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING FLASH READ BYTES...");
    for(int i = 0; i < num_bytes; ++i)
    {
        mt25ql256aba_check_ready_flag();
        convert_4byte_address_to_3byte_address(flash_addr, addr_buf);
        ret = mt25ql256aba_read_op(MT25QL256ABA_READ,
                                    addr_buf,
                                    sizeof(addr_buf), 
                                    &data, 
                                    sizeof(data));
        NRF_LOG_INFO("addr: %d, addr 0x%03x, Data: 0x%02x",flash_addr, flash_addr, data);
        flash_addr++;
        spi_ret_check(ret);
    }

}


void adxl372_startup_test(void)
{
    int8_t ret =0;
    uint8_t device_id, mst_devid, devid;
    device_id = adxl372_get_dev_ID();
    ret |= adxl372_read_reg( ADI_ADXL372_MST_DEVID, &mst_devid);
    ret |= adxl372_read_reg(ADI_ADXL372_DEVID, &devid);

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING ADXL TEST....");
    NRF_LOG_INFO("1: adi device id = 0x%x (0xAD)", device_id);
    if(device_id != ADI_ADXL372_ADI_DEVID_VAL)
    {
        NRF_LOG_INFO("ADXL READ TEST FAIL");
       
    }
    NRF_LOG_INFO("2:mst device id2 = 0x%x (0x1D)", mst_devid);
    if(mst_devid != ADI_ADXL372_MST_DEVID_VAL)
    {
        NRF_LOG_INFO("ADXL READ TEST FAIL");
   
    }
    NRF_LOG_INFO("3:mems id = 0x%x (0xFA)(372 octal)", devid);
    if(devid != ADI_ADXL372_DEVID_VAL)
    {
        NRF_LOG_INFO("ADXL READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
    else{
        NRF_LOG_INFO("ADXL READ TEST PASS");
    }
    // ==========================================
}

void full_page_read(void)
{
    uint8_t addr[3] = {0x00, 0x00, 0x00};
    uint8_t full_page_data[250];
    int8_t ret;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING FULL PAGE READ....")
    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_read_op(MT25QL256ABA_READ, addr, sizeof(addr), full_page_data, sizeof(full_page_data));
    spi_ret_check(ret);
    for(int i = 0; i<sizeof(full_page_data); ++i){
        NRF_LOG_INFO("Data: 0x%x", full_page_data[i]);
    }
}


void mt25ql256aba_startup_test(void)
{
    uint8_t val[3];
    int8_t ret;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING FLASH TEST....");
    ret = mt25ql256aba_read_op(MT25QL256ABA_READ_ID, NULL, 0, val, sizeof(val));
    spi_ret_check(ret);

    NRF_LOG_INFO("1: device id = 0x%x (0x20)", val[0]);
    if(val[0] != 0x20)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }

    nrf_delay_ms(100);
    NRF_LOG_INFO("2:memory type = 0x%x (0xBA)", val[1]);
    if(val[1] != 0xBA)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }

    nrf_delay_ms(100);
    NRF_LOG_INFO("3:memory capacity = 0x%x (0x19)", val[2]);
    if(val[2]!= 0x19)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
}


void sample_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data, ds1388_data_t* rtc_data)
{
    //todo get only the raw values and process them later
    adxl372_get_accel_data(high_g_data);
    icm20649_read_gyro_accel_data(low_g_gyro_data);
    icm20649_convert_data(low_g_gyro_data);
    get_time(rtc_data);
    if (g_buf_index < MAX_SAMPLE_BUF_LENGTH)
    {
        g_sample_set_buf[g_buf_index].adxl_data = *high_g_data;
        g_sample_set_buf[g_buf_index].icm_data = *low_g_gyro_data;
        g_sample_set_buf[g_buf_index].ds_data = *rtc_data;
        g_buf_index++;
    }
}
void bulk_erase(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING BULK ERASE");
    mt25ql256aba_write_enable();
    mt25ql256aba_write_op(MT25QL256ABA_BULK_ERASE, NULL, 0, NULL, 0);
}

void mt25ql256aba_check_ready_flag(void)
{
    uint8_t flash_ready; 

    do{
       mt25ql256aba_read_op(MT25QL256ABA_READ_STATUS_REGISTER, NULL, 0, &flash_ready, sizeof(flash_ready));
       flash_ready = flash_ready & 0x1;
    }while(flash_ready == 1);
}

void mt25ql256aba_store_samples(uint32_t* flash_addr)
{
    uint8_t flash_addr_buf[3]= {0};
    uint8_t buf[32] = {0};
    uint8_t out_buf[32] = {0};
    uint8_t buf_size = sizeof(buf);
    uint8_t sample_buf_size = sizeof(impact_sample_t);
    uint8_t flash_addr_buf_size = sizeof(flash_addr_buf);

    NRF_LOG_INFO("");
    NRF_LOG_INFO("BEGIN STORE SAMPLES...");
    //store one impact sample set to flash
    for (int i = 0; i < g_buf_index ; ++i)
    {
        NRF_LOG_INFO("");
        NRF_LOG_INFO("WRITE: ID: %d, addr: 0x%03x", i, *flash_addr);
        mt25ql256aba_check_ready_flag();
        memcpy(buf, &g_sample_set_buf[i], sample_buf_size);
        convert_4byte_address_to_3byte_address(*flash_addr, flash_addr_buf);
        mt25ql256aba_write_enable();
        mt25ql256aba_write_op(MT25QL256ABA_PAGE_PROGRAM, 
                              flash_addr_buf,
                              flash_addr_buf_size, 
                              buf,
                              buf_size);
        mt25ql256aba_write_disable();
        mt25ql256aba_check_ready_flag();
        mt25ql256aba_read_op(MT25QL256ABA_READ, 
                            flash_addr_buf,
                            flash_addr_buf_size,
                            out_buf,
                            buf_size);
        memcpy(&g_flash_output_buf[i], out_buf, sample_buf_size);
        NRF_LOG_INFO("READ:  ID: %d, addr: 0x%03x, OUTPUT: %d (%d)", i, *flash_addr,
                       g_flash_output_buf[i].adxl_data.x,
                        g_sample_set_buf[i].adxl_data.x);
        
        *flash_addr = *flash_addr + buf_size; 
    }
}

void prototype_mt25ql256aba_store_samples(uint32_t* flash_addr)
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
        NRF_LOG_INFO("");
        NRF_LOG_INFO("WRITE: ID: %d, addr: 0x%03x", i, *flash_addr);

        mt25ql256aba_check_ready_flag();
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

void convert_4byte_address_to_3byte_address(uint32_t flash_addr, uint8_t* flash_addr_buf)
{
    uint8_t* flash_addr_ptr = (uint8_t*)&flash_addr;
    flash_addr_buf[0] = flash_addr_ptr[2]; //high address value
    flash_addr_buf[1] = flash_addr_ptr[1];
    flash_addr_buf[2] = flash_addr_ptr[0]; //low address value
}

void mt25ql256aba_retrieve_samples(void)
{
    uint32_t addr32 = 0x00000000;
    uint8_t addr[3] = {0};
    uint8_t sample_size_bytes = sizeof(impact_sample_t);
    uint8_t *sample_byte_ptr;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("BEGIN RETRIEVE SAMPLES");
    for(int i = 0; i < g_buf_index; ++i) 
    {
        mt25ql256aba_check_ready_flag();
        convert_4byte_address_to_3byte_address(addr32, addr);
        sample_byte_ptr = (uint8_t*) &g_flash_output_buf[i];
        mt25ql256aba_read_op(MT25QL256ABA_READ,
                              addr,
                              sizeof(addr),
                              sample_byte_ptr,
                              sample_size_bytes);
        NRF_LOG_INFO("READ: ID: %d, addr: 0x%03x, OUTPUT: %d (%d)",
                       i, addr32, g_flash_output_buf[i].adxl_data.x,
                        g_sample_set_buf[i].adxl_data.x);
        addr32 = addr32 + sample_size_bytes;
    }
}

void mt25ql256aba_erase(void)
{
    uint8_t addr[3] = {0x00, 0x00, 0x00};

    mt25ql256aba_check_ready_flag();
    mt25ql256aba_write_enable();
    mt25ql256aba_write_op(MT25QL256ABA_ERASE_4KB_SUBSECTOR, addr, sizeof(addr), NULL, 0);
    mt25ql256aba_write_disable();
}


void serial_output_flash_data(void)
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

/*
 * Depreciated
 */ 
void serial_output_impact_data(void)
{
    NRF_LOG_INFO("\r\n===================IMPACT DATA OUTPUT===================");
    for (int i = 0; i < g_buf_index; ++i)
    {
    NRF_LOG_INFO("");
    NRF_LOG_INFO("id=%d, accel x= %d, accel y = %d,  accel z= %d mG's",
                    i, g_sample_set_buf[i].adxl_data.x, g_sample_set_buf[i].adxl_data.y, g_sample_set_buf[i].adxl_data.z);
    NRF_LOG_INFO("      accel x = %d, accel y = %d, accel z = %d mG's, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
                    g_sample_set_buf[i].icm_data.accel_x, g_sample_set_buf[i].icm_data.accel_y, g_sample_set_buf[i].icm_data.accel_z,
                    g_sample_set_buf[i].icm_data.gyro_x, g_sample_set_buf[i].icm_data.gyro_y, g_sample_set_buf[i].icm_data.gyro_z);
    NRF_LOG_INFO("      Date: %d, Day:    %d, Hour:   %d, Minute: %d, Second: %d, Hundreth: %d",
                    g_sample_set_buf[i].ds_data.date, g_sample_set_buf[i].ds_data.day,
                    g_sample_set_buf[i].ds_data.hour, g_sample_set_buf[i].ds_data.minute,
                    g_sample_set_buf[i].ds_data.second, g_sample_set_buf[i].ds_data.hundreth);       
    }
    NRF_LOG_INFO("")
    g_buf_index = 0; //reset buf index
    memset(g_sample_set_buf, 0x00, sizeof(g_sample_set_buf));
    NRF_LOG_INFO("\r\n====================DATA OUTPUT FINISH==================");
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
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING ICM READ TEST....");
    uint8_t who_am_i = 0x0;
    icm20649_read_reg(0x0, &who_am_i);
    NRF_LOG_INFO("1:who_am_i = 0x%x (0xE1)", who_am_i );
    if(who_am_i == 0xE1)
    {
        NRF_LOG_INFO("READ SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR: CHECK WIRING!"); 
        while(1)
        {
            __WFE();
        }
    }

    /********************************************/
}

void icm20649_write_test(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING ICM WRITE TEST....");
    uint8_t write_read;
    //PWR_MGMT 1 select best clk and disable everything else
    icm20649_write_reg(0x06, 0x1);

    icm20649_read_reg(0x06, &write_read);

    NRF_LOG_INFO("2:write_read = 0x%x (0x1)", write_read );
    if(write_read == 0x1)
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR: CHECK WIRING!"); 
        while(1)
        {
            __WFE();
        }
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
    data->gyro_y = ((float) data->gyro_y / 32767.0) * 2000.0 * deg2rad *1000;
    data->gyro_z = ((float) data->gyro_z / 32767.0) * 2000.0 * deg2rad *1000;
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

    NRF_LOG_INFO("Configuring VCNL...");

	uint8_t reg1[3] = {VCNL4040_PS_CONF3, ps_conf3_data, ps_ms_data};
    nrf_drv_twi_tx(&twi, VCNL4040_ADDR, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);
	
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    nrf_drv_twi_tx(&twi, VCNL4040_ADDR, reg2, sizeof(reg2), false);
    while (m_xfer_done == false);
}

/**
 * @brief Function for reading data from VCNL4040 sensor.
 */
void read_sensor_data()
{
    // do
    // {
    //     __WFE();
    // }while (m_xfer_done == false);
    m_xfer_done = false;

    while(prox_val <= PROX_THRESHOLD){
        uint8_t reg3[2] = {VCNL4040_PS_DATA, VCNL4040_ADDR};
        uint8_t reg4[2] = {m_sample_lsb, m_sample_msb};
        uint8_t *p_ret = reg4;
        nrf_drv_twi_xfer_desc_t const vcnl_desc = {NRFX_TWI_XFER_TXRX, VCNL4040_ADDR, sizeof(reg3), sizeof(reg4), reg3, p_ret};
        nrf_drv_twi_xfer(&twi, &vcnl_desc, false);
        while(nrf_drv_twi_is_busy(&twi) == true);

        m_sample_lsb = *p_ret;
        p_ret++;
        m_sample_msb = *p_ret;

        m_sample = (((m_sample_msb) << 8) | (m_sample_lsb));
        prox_val = m_sample;
        NRF_LOG_INFO("Proximity: %d", m_sample);
    }

        uint8_t reg3[2] = {VCNL4040_PS_DATA, VCNL4040_ADDR};
        uint8_t reg4[2] = {m_sample_lsb, m_sample_msb};
        uint8_t *p_ret = reg4;
        nrf_drv_twi_xfer_desc_t const vcnl_desc = {NRFX_TWI_XFER_TXRX, VCNL4040_ADDR, sizeof(reg3), sizeof(reg4), reg3, p_ret};
        nrf_drv_twi_xfer(&twi, &vcnl_desc, false);
        while(nrf_drv_twi_is_busy(&twi) == true);

    return;

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
    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg0, sizeof(reg0), false);
    while (m_xfer_done == false);
  
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

    nrf_drv_twi_tx(&twi, DS1388_ADDRESS, reg1, sizeof(reg1), false);
    while (m_xfer_done == false);

    NRF_LOG_INFO("RTC initialized");
}

/**
 * @brief TWI events handler.
 */
void twi_handler(nrf_drv_twi_evt_t const * p_event, void * p_context)
{
    switch (p_event->type)
    {
        case NRF_DRV_TWI_EVT_DONE:
            // if (p_event->xfer_desc.type == NRF_DRV_TWI_XFER_RX)
            //     {
            //         prox_val = m_sample;
            //     }
            m_xfer_done = true;
            break;
        case NRF_DRV_TWI_EVT_DATA_NACK:
            //m_xfer_done = true;
            NRF_LOG_INFO("\r\nDATA NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        case NRF_DRV_TWI_EVT_ADDRESS_NACK:
            //m_xfer_done = true;
            NRF_LOG_INFO("\r\nADDRESS NACK ERROR");
            NRF_LOG_FLUSH();
            break;
        default:
            //m_xfer_done = true
            break;
    }
}

/**
 * @brief UART initialization.
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

uint8_t readRegister(uint8_t reg_addr)
{
    do
        {
            __WFE();
        }while (m_xfer_done == false);

    m_xfer_done = false;

    nrf_drv_twi_xfer_desc_t const ds_desc = {NRFX_TWI_XFER_TXRX, DS1388_ADDRESS, sizeof(reg_addr), sizeof(byte), &reg_addr, &byte};

    nrf_drv_twi_xfer(&twi, &ds_desc, false);
    
    while(nrf_drv_twi_is_busy(&twi) == true);

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


