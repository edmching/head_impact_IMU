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

static void log_init(void);

int main (void)
{
    // Initialize.
    log_init();

   
    NRF_LOG_INFO("adxl372 test start.");
    adxl372_init();


    while(1)
    {

    }

    return 0;
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}
