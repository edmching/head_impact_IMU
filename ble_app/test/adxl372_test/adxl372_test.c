#include <stdbool.h>
#include <stdint.h>

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
    uint8_t device_id;

    spi_init();
    log_init();

    NRF_LOG_INFO("adxl372 test measurement mode");
    device_id = adxl372_get_dev_ID();
    NRF_LOG_INFO("%d", device_id);
    //adxl372_init();
    //adxl372_accel_data_t accel_data;

    while(1)
    {
        /*
        adxl372_get_accel_data(&accel_data);
        accel_data.x = (float) accel_data.x * 100/1000;
        accel_data.y = (float) accel_data.y * 100/1000;
        accel_data.z = (float) accel_data.z * 100/1000;

        NRF_LOG_INFO("X accel = %f", accel_data.x);
        */
    }

    return 0;
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
