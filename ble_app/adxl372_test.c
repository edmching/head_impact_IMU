
#include <stdint.h>
#include <string.h>
#include "nrf.h"
#include "nordic_common.h"
#include "adxl372.h"
#include "spi_driver.h"
#include "nrf_log.h"
#include "nrf_log_ctrl.h"


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