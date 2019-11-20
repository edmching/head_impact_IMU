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

    NRF_LOG_INFO("adxl372 test measurement mode");

    // READ TEST
    nrf_delay_ms(100);
    device_id = adxl372_get_dev_ID();
    ret |= adxl372_read_reg(ADI_ADXL372_MST_DEVID, &mst_devid);
    ret |= adxl372_read_reg(ADI_ADXL372_DEVID, &devid);
    if (ret < 0)
    {
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
    }
    NRF_LOG_INFO("adi device id = 0x%x \r", device_id);
    NRF_LOG_INFO("mst device id2 = 0x%x \r", devid)
    NRF_LOG_INFO("mems id = 0x%x \r", mst_devid);
    if(device_id != ADI_ADXL372_ADI_DEVID_VAL ||
        mst_devid != ADI_ADXL372_MST_DEVID_VAL ||
        devid != ADI_ADXL372_DEVID_VAL)
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
    NRF_LOG_INFO("lpf val = %d", lpf_val);
    NRF_LOG_INFO("hpf val = %d", hpf_val);
    NRF_LOG_INFO("op val = %d", op_val);
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
    adxl372_init();
    adxl372_accel_data_t accel_data;

    while(1)
    {
        adxl372_get_accel_data(&accel_data);
    
        NRF_LOG_INFO("X accel = %d mG, Y accel = %d mG, Z accel = %d mG", accel_data.x, accel_data.y, accel_data.z);
    
        nrf_delay_ms(1000);
    }

    return 0;
}


static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
