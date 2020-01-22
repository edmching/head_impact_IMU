#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"

//adxl372 driver
#include "adxl372.h"
#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

#define NUM_SAMPLES 127

#define READ_DONE 0
#define NOT_READ  -1

#define TEST_FIFO
//#define TEST_REGULAR

static void log_init(void);

int main (void)
{
    // Initialize.
    spi_init();
    log_init();

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
        while(1);
    }

    NRF_LOG_INFO("adi device id = 0x%x (0xAD)", device_id);
    if(device_id != ADI_ADXL372_ADI_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
    }
    NRF_LOG_INFO("mst device id2 = 0x%x (0x1D)", mst_devid);
    if(mst_devid != ADI_ADXL372_MST_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
    }
    NRF_LOG_INFO("mems id = 0x%x (0xFA)(372 octal)", devid);
    if(devid != ADI_ADXL372_DEVID_VAL)
    {
        NRF_LOG_ERROR("ADXL READ TEST FAIL");
        while(1);
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
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
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
        while(1);
    }
    else{
        NRF_LOG_INFO("ADXL WRITE TEST PASS");
    }
    // ============================

    // Measurement TEST
#ifdef TEST_REGULAR
    adxl372_init();
#endif
#ifdef TEST_FIFO
    struct adxl372_device dev;
    adxl372_init_instant_on_mode(&dev, NUM_SAMPLES);
    adxl372_accel_data_t sample_set[NUM_SAMPLES/3];
    int8_t read_fifo_done = NOT_READ;

    while(1)
    {
        read_fifo_done = adxl372_get_fifo_data(&dev, sample_set);

        if(read_fifo_done == READ_DONE)
        {
            for(int i = 0; i < NUM_SAMPLES/3; ++i)
                NRF_LOG_INFO("sample: %d, X accel = %d mG, Y accel = %d mG, Z accel = %d mG", i,
                            sample_set[i].x, sample_set[i].y, sample_set[i].z);
            adxl372_reset();
            while(1);
        }
    
    }
#endif
#ifdef TEST_REGULAR
    adxl372_accel_data_t accel_data;
    while (1)
    {
        adxl372_get_accel_data(&accel_data);
        NRF_LOG_INFO("X accel = %d mG, Y accel = %d mG, Z accel = %d mG",
                        accel_data.x, accel_data.y, accel_data.z);
        nrf_delay_ms(1000);
    }
#endif
    return 0;
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
