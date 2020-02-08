#include <stdbool.h>
#include <stdint.h>
#include <string.h>

//general nrf
#include "nrf.h"
#include "nordic_common.h"
#include "boards.h"

#include "spi_driver.h"

//for NRF_LOG()
#include "nrf_log.h"
#include "nrf_log_ctrl.h"
#include "nrf_log_default_backends.h"

//for error logging
#include "app_error.h"

#include "nrf_delay.h"

#include "mt25ql256aba.h"

static void log_init(void);

int main (void)
{
    // Initial
    spi_init();
    log_init();

    NRF_LOG_INFO("mt25ql256aba flash test");

    uint8_t val[3];
    int8_t ret;
    uint8_t addr[3] = {0x00, 0x00, 0x00};

    ret = mt25ql256aba_read(MT25QL256ABA_READ_ID, addr, val, 3);
    if (ret < 0)
    {
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
    }

    NRF_LOG_INFO("device id = 0x%x (0x20)", val[0]);
    if(val[0] != 0x20)
    {
        NRF_LOG_ERROR("FLASH READ TEST FAIL");
        while(1);
    }

    nrf_delay_ms(100);

    NRF_LOG_INFO("memory type = 0x%x (0xBA)", val[1]);
    if(val[1] != 0xBA)
    {
        NRF_LOG_ERROR("FLASH READ TEST FAIL");
        while(1);
    }

    NRF_LOG_INFO("memory capacity = 0x%x (0x19)", val[2]);
    if(val[2]!= 0x19)
    {
        NRF_LOG_ERROR("FLASH READ TEST FAIL");
        while(1);
    }

    while(1){
    }
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}