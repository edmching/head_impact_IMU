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

//#include "spi_driver.h"
#include "adxl372.h"
#include "icm20649.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

//const uint32_t UICR_ADDR_0x20C __attribute__ ((section(".uicrNfcPinsAddress"))) __attribute__((used));

void adxl372_test()
{
    int8_t ret = 0;
    uint8_t device_id;
    uint8_t mst_devid;
    uint8_t devid;

    NRF_LOG_INFO("-----------------------------");
    NRF_LOG_INFO("adxl372 test measurement mode");
    nrf_delay_ms(100);

    adxl372_reset();

    //--------------------READ TEST---------------------//
    device_id = adxl372_get_dev_ID();
    ret |= adxl372_read_reg( ADI_ADXL372_MST_DEVID, &mst_devid);
    ret |= adxl372_read_reg(ADI_ADXL372_DEVID, &devid);
    if (ret < 0)
    {
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1)
        {
            __WFE();
        }
    }

    NRF_LOG_INFO("adi device id = 0x%x (0xAD)", device_id);
    if(device_id != ADI_ADXL372_ADI_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
    NRF_LOG_INFO("mst device id2 = 0x%x (0x1D)", mst_devid);
    if(mst_devid != ADI_ADXL372_MST_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
    NRF_LOG_INFO("mems id = 0x%x (0xFA)(372 octal)", devid);
    if(devid != ADI_ADXL372_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
    else{
        NRF_LOG_INFO("ADXL READ TEST PASS");
    }
    // ==========================================

    // Write TEST 
    uint8_t p_reg;
    uint8_t lpf_val;
    uint8_t hpf_val;
    uint8_t op_val;
    adxl372_set_op_mode(STAND_BY);
    adxl372_set_lpf_disable(true);
    adxl372_set_hpf_disable(true);
    ret |= adxl372_read_reg(ADI_ADXL372_POWER_CTL, &p_reg);
    if (ret < 0)
    {
        while(1)
        {
            __WFE();
        }
    }
    lpf_val = (p_reg >> PWRCTRL_LPF_DISABLE_POS) & 0x1;
    hpf_val = (p_reg >> PWRCTRL_HPF_DISABLE_POS) & 0x1;
    op_val = (p_reg >> PWRCTRL_OPMODE_POS) & 0x3;
    NRF_LOG_INFO("lpf val = %d (expected: 1)", lpf_val);
    NRF_LOG_INFO("hpf val = %d (expected: 1)", hpf_val);
    NRF_LOG_INFO("op val = %d (expected: 0)", op_val);
    if(lpf_val != 1 || hpf_val != 1 || op_val !=0)
    {
        NRF_LOG_ERROR("ADXL WRITE TEST FAIL");
        while(1)
        {
            __WFE();
        }
    }
    else{
        NRF_LOG_INFO("ADXL WRITE TEST PASS");
    }
}

void icm20649_test()
{
    NRF_LOG_INFO(" ICM20649 TEST measurement mode");

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
    }
    /********************************************/

    
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
        while(1)
        {
          __WFE();
        }
        
    }
    /********************************************/

}

void spi_init(void)
{
    ret_code_t err_code = nrf_drv_spi_init(&accel_spi, &accel_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

    err_code = nrf_drv_spi_init(&gyro_spi, &gyro_spi_config, spi_event_handler, NULL);
    APP_ERROR_CHECK(err_code);

}

int main (void)
{
    // Initialize.
    SystemInit();
    log_init();
    spi_init();

    adxl372_test();
    adxl372_default_init();
    icm20649_test();
    icm20649_default_init();
    
    adxl372_accel_data_t accel_data;
    icm20649_data_t data;
    float deg2rad = 3.1415/180.0;

    while(1)
    {
        icm20649_read_gyro_accel_data(&data);
        data.accel_x = ((float) data.accel_x/1024.0)*1000;
        data.accel_y = ((float) data.accel_y/1024.0)*1000;
        data.accel_z = ((float) data.accel_z/1024.0)*1000;
        data.gyro_x = ((float) data.gyro_x / 32767.0) * 2000.0 * deg2rad;
        data.gyro_y = ((float) data.gyro_x / 32767.0) * 2000.0 * deg2rad;
        data.gyro_z = ((float) data.gyro_x / 32767.0) * 2000.0 * deg2rad;

        adxl372_get_accel_data(&accel_data);
        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mG",
                        accel_data.x, accel_data.y, accel_data.z);

        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mg, gyro x = %d, gyro y = %d, gyro z = %d mrad/s", 
        data.accel_x, data.accel_y, data.accel_z, data.gyro_x, data.gyro_y, data.gyro_z );
    
        nrf_delay_ms(1000);
    }

    return 0;
}



void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
