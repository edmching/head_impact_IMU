#include "sensors_integration.h"
#include "nrf_delay.h"

static void log_init(void);
static void lfclk_request(void);
static void create_timers(void);
void mt25ql256aba_startup_test(void);
static void spi_ret_check(int8_t ret);
void adxl372_startup_test(void);
void mt25ql256aba_erase(void);
void mt25ql256aba_check_ready_flag(void);
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
    twi_init();
#endif

    //for app_timer
    lfclk_request();

    NRF_LOG_INFO("===============");
    NRF_LOG_INFO("Sensors test");

    mt25ql256aba_startup_test();
    icm20649_read_test();
    icm20649_write_test();
    adxl372_startup_test();
    //init sensors
    icm20649_init();
    adxl372_init();
#ifdef USE_PROX
    vcnl_config();
#endif

    uint8_t addr[3] = {0x00, 0x00, 0x00};
    uint8_t full_page_data[255];
    mt25ql256aba_read_op(MT25QL256ABA_READ, addr, sizeof(addr), full_page_data, sizeof(full_page_data));
    for(int i = 0; i<sizeof(full_page_data); ++i){
        NRF_LOG_INFO("Data: 0x%x", full_page_data[i]);
    }

    app_timer_init();
    create_timers();
    icm20649_data_t low_g_gyro_data;
    adxl372_accel_data_t high_g_data;
    uint32_t flash_addr = MT25QL256ABA_LOW_128MBIT_SEGMENT_ADDRESS_START;
    //uint32_t flash_addr = 0x00000001;

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
                    sample_impact_data(&high_g_data, &low_g_gyro_data);
                }
                app_timer_stop(m_measurement_timer_id);
                //reset for next impact
                g_measurement_done = false;
                mt25ql256aba_store_samples((uint8_t*)&flash_addr);
                mt25ql256aba_retrieve_samples();
                serial_output_flash_data();
            }
#ifdef USE_PROX
        }
#endif
    }

    return 0;
}


void adxl372_startup_test(void)
{
    int8_t ret =0;
    uint8_t device_id, mst_devid, devid;
    device_id = adxl372_get_dev_ID();
    ret |= adxl372_read_reg( ADI_ADXL372_MST_DEVID, &mst_devid);
    ret |= adxl372_read_reg(ADI_ADXL372_DEVID, &devid);

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
            nrf_delay_ms(100);
        }
    }
    else{
        NRF_LOG_INFO("ADXL READ TEST PASS");
    }
    // ==========================================
}

void mt25ql256aba_startup_test(void)
{
    uint8_t val[3];
    int8_t ret;

    ret = mt25ql256aba_read_op(MT25QL256ABA_READ_ID, NULL, 0, val, sizeof(val));
    spi_ret_check(ret);
    
    NRF_LOG_INFO("1: device id = 0x%x (0x20)", val[0]);
    if(val[0] != 0x20)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
    }

    NRF_LOG_INFO("2:memory type = 0x%x (0xBA)", val[1]);
    if(val[1] != 0xBA)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
    }

    NRF_LOG_INFO("3:memory capacity = 0x%x (0x19)", val[2]);
    if(val[2]!= 0x19)
    {
        NRF_LOG_INFO("FLASH READ TEST FAIL");
    }
}


void sample_impact_data (adxl372_accel_data_t* high_g_data, icm20649_data_t* low_g_gyro_data)
{
    //todo get only the raw values and process them later
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

void mt25ql256aba_check_ready_flag(void)
{
    uint8_t flash_ready; 

    mt25ql256aba_read_op(MT25QL256ABA_READ_STATUS_REGISTER, NULL, 0, &flash_ready, sizeof(flash_ready));
    flash_ready = flash_ready & 0x1;
    while(flash_ready == 1);
}

void mt25ql256aba_store_samples(uint8_t* flash_addr_ptr)
{
    uint8_t flash_addr_buf[3]= {0};
    uint32_t num = sizeof(impact_sample_set_t);

    //uint8_t flash_ready = 0x0;
    NRF_LOG_INFO("begin store samples");
    //store one impact sample set to flash
    for (int i = 0; i < g_buf_index; ++i)
    {
        mt25ql256aba_check_ready_flag();
        NRF_LOG_INFO("INDEX: %d", i);
        flash_addr_buf[0] = flash_addr_ptr[2];
        flash_addr_buf[1] = flash_addr_ptr[1];
        flash_addr_buf[2] = flash_addr_ptr[0];
        mt25ql256aba_write_enable();
        mt25ql256aba_write_op(MT25QL256ABA_PAGE_PROGRAM, 
                              flash_addr_buf, sizeof(flash_addr_buf), 
                              (uint8_t*) &g_sample_set_buf[i],
                              sizeof(impact_sample_set_t));
        if(*flash_addr_ptr + num < MT25QL256ABA_LOW_128MBIT_SEGMENT_ADDRESS_END)
        {
            *flash_addr_ptr = *flash_addr_ptr + num;
        }
        else
        {
            *flash_addr_ptr = MT25QL256ABA_LOW_128MBIT_SEGMENT_ADDRESS_START;
        }
    }
}

void mt25ql256aba_retrieve_samples(void)
{
    uint32_t addr32 = 0x00000000;
    uint8_t* addr_ptr = (uint8_t*)&addr32;
    uint8_t addr[3] = {0};
    //uint8_t flash_ready = 0x0;

    NRF_LOG_INFO("BEGIN retrieve samples");
    for(int i = 0; i < g_buf_index; ++i) 
    {
        mt25ql256aba_check_ready_flag();
        addr[0] = addr_ptr[2];
        addr[1] = addr_ptr[1];
        addr[2] = addr_ptr[0];
        mt25ql256aba_read_op(MT25QL256ABA_READ, addr, sizeof(addr), (uint8_t*)&g_flash_output_buf[i], sizeof(impact_sample_set_t));
        NRF_LOG_INFO("INDEX: %d OUTPUT: %d", i, g_flash_output_buf[i].adxl_data.x);
        addr32 = addr32 + sizeof(impact_sample_set_t);
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
    NRF_LOG_INFO("id=%d, accel x= %d, accel y = %d,  accel z= %d mG's",
                    i, g_flash_output_buf[i].adxl_data.x,
                     g_flash_output_buf[i].adxl_data.y,
                     g_flash_output_buf[i].adxl_data.z);
    NRF_LOG_INFO("      accel x = %d, accel y = %d, accel z = %d mG's, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
                        g_flash_output_buf[i].icm_data.accel_x,
                        g_flash_output_buf[i].icm_data.accel_y,
                        g_flash_output_buf[i].icm_data.accel_z,
                        g_flash_output_buf[i].icm_data.gyro_x, 
                        g_flash_output_buf[i].icm_data.gyro_y, 
                        g_flash_output_buf[i].icm_data.gyro_z);
    }
    g_buf_index = 0; //reset buf index
    memset(g_flash_output_buf, 0x00, sizeof(g_flash_output_buf));
    NRF_LOG_INFO("\r\n====================DATA OUTPUT FINISH==================");
}

void serial_output_impact_data(void)
{
    NRF_LOG_INFO("\r\n===================IMPACT DATA OUTPUT===================");
    for (int i = 0; i < g_buf_index; ++i)
    {
    NRF_LOG_INFO("id=%d, accel x= %d, accel y = %d,  accel z= %d mG's",
                    i, g_high_G_buf[i].x, g_high_G_buf[i].y, g_high_G_buf[i].z);
    NRF_LOG_INFO("      accel x = %d, accel y = %d, accel z = %d mG's, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
                    g_low_G_buf[i].accel_x, g_low_G_buf[i].accel_y, g_low_G_buf[i].accel_z,
                    g_low_G_buf[i].gyro_x, g_low_G_buf[i].gyro_y, g_low_G_buf[i].gyro_z);
    }
    g_buf_index = 0; //reset buf index
    memset(g_high_G_buf, 0x00, sizeof(g_high_G_buf));
    memset(g_low_G_buf, 0x00, sizeof(g_low_G_buf));
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
     /*********TEST READ******************/
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

    NRF_LOG_INFO("2:write_read = 0x%x (0x1)", write_read );
    if(write_read == 0x1)
    {
        NRF_LOG_INFO("WRITE SUCCESSFUL");
    }
    else
    {
        NRF_LOG_INFO("VAL ERROR: CHECK WIRING!"); 
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
    while (m_xfer_done == false);
	
    uint8_t reg2[3] = {VCNL4040_PS_CONF1, ps_conf1_data, ps_conf2_data};
    nrf_drv_twi_tx(&vcnl_twi, VCNL4040_ADDR, reg2, sizeof(reg2), false);
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
                prox_val = m_sample;
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
       .frequency          = NRF_DRV_TWI_FREQ_100K,
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
void read_sensor_data()
{
    do
    {
        __WFE();
    }while (m_xfer_done == false);
    
    m_xfer_done = false;

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



