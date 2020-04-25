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
#include "vcnl4040.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

//const uint32_t UICR_ADDR_0x20C __attribute__ ((section(".uicrNfcPinsAddress"))) __attribute__((used));
void log_init(void);

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
    twi_init();

    adxl372_test();
    adxl372_default_init();
    icm20649_test();
    icm20649_default_init();

    vcnl4040_config();
    
    adxl372_accel_data_t accel_data;
    icm20649_data_t gyro_data;

    while(1)
    {
        vcnl4040_read_sensor_data();
        icm20649_read_gyro_accel_data(&gyro_data);
        icm20649_convert_data(&gyro_data);
        adxl372_get_accel_data(&accel_data);

        NRF_LOG_INFO("accel x = %d, accel y = %d, accel z = %d mG",
                        accel_data.x, accel_data.y, accel_data.z);
        NRF_LOG_RAW_INFO("accel x = %d, accel y = %d, accel z = %d, gyro x = %d, gyro y = %d, gyro z = %d \r\n", 
        gyro_data.accel_x, gyro_data.accel_y, gyro_data.accel_z,
        gyro_data.gyro_x, gyro_data.gyro_y, gyro_data.gyro_z );
    
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
