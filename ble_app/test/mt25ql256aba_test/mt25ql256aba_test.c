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
static void spi_ret_check(int8_t ret);

void mt25ql256aba_check_ready_flag(void)
{
    uint8_t flash_ready; 

    do{
       mt25ql256aba_read_op(MT25QL256ABA_READ_STATUS_REGISTER, NULL, 0, &flash_ready, sizeof(flash_ready));
       flash_ready = flash_ready & 0x1;
    }while(flash_ready == 1);
}

void read_flag(void)
{
    uint8_t flag_register;
    mt25ql256aba_read_op(MT25QL256ABA_READ_FLAG_STATUS_REGISTER,NULL, 0, &flag_register, sizeof(flag_register));
    uint8_t program_erase_controller = flag_register >> 7;
    uint8_t program = (flag_register >> 4) & 0x1; 
    uint8_t byte_addressing = flag_register & 0x1;
    uint8_t erase = (flag_register >> 5) & 0x1; 
    uint8_t protection = (flag_register >> 1) & 0x1;
    NRF_LOG_INFO("");
    NRF_LOG_INFO("CHECKING FLAG REG.....");
    NRF_LOG_INFO("program_erase controller %d, erase %d, program %d", 
                    program_erase_controller,erase, program);
    NRF_LOG_INFO("protection %d, byte %d",protection, byte_addressing);
}

void read_test(void)
{
    uint8_t val[3];
    int8_t ret = 0;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING READ TEST");
    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_read_op(MT25QL256ABA_READ_ID, NULL, 0, val, sizeof(val));
    spi_ret_check(ret);
    
    NRF_LOG_INFO("device id = 0x%x (0x20)", val[0]);
    if(val[0] != 0x20)
    {
        NRF_LOG_ERROR("FLASH READ TEST FAIL");
        while(1);
    }

    nrf_delay_ms(100); //to prevent weird display issues

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
    //============================================//
}

void write_test(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING WRITE TEST");
    uint8_t status_reg;
    int8_t ret = 0;

    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_write_enable();
    spi_ret_check(ret);

    ret = mt25ql256aba_read_op(MT25QL256ABA_READ_STATUS_REGISTER, NULL, 0, &status_reg, sizeof(status_reg));
    spi_ret_check(ret);

    uint8_t write_bit = (status_reg >> 1) & 0x1;
    NRF_LOG_INFO("Result = %d (Expected: 1)", write_bit);
    if(write_bit != 0x1)
    {
        NRF_LOG_ERROR("FLASH WRITE ENABLE TEST FAIL");
        while(1);
    }
    else{
        NRF_LOG_INFO("FLASH WRITE ENABLE TEST PASS");
    }
}
void bulk_erase(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING BULK ERASE");
    mt25ql256aba_check_ready_flag();
    mt25ql256aba_write_enable();
    mt25ql256aba_write_op(MT25QL256ABA_BULK_ERASE, NULL, 0, NULL, 0);
}

void page_write(void)
{
    uint8_t addr1[3] = {0x00, 0x00, 0x00};
    uint8_t data1[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    int8_t ret = 0;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("PERFORMING PAGE WRITE...");
    mt25ql256aba_check_ready_flag();
    mt25ql256aba_write_enable();
    ret = mt25ql256aba_write_op(MT25QL256ABA_PAGE_PROGRAM, addr1, sizeof(addr1), data1, sizeof(data1));
    spi_ret_check(ret);
}

void nonvolatile_verify_test(void)
{
    uint8_t addr[3] = {0x00, 0x00, 0x00};
    uint8_t startup_data[8];
    uint8_t data[8] = {0xAB, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    int8_t ret=0;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("NONVOLATILE VERIFY...");
    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_read_op(MT25QL256ABA_READ, addr, sizeof(addr), startup_data, sizeof(startup_data));
    spi_ret_check(ret);
    for(int i = 0; i<8; ++i){
        NRF_LOG_INFO("Data: 0x%x", startup_data[i]);
        if(data[i] != startup_data[i])
        {
            NRF_LOG_INFO("Result = 0x%x (Expect: 0x%x)", startup_data[i], data[i]);
            NRF_LOG_ERROR("FLASH PAGE WRITE TEST FAIL");
            while(1);
        }
    }
}

void erase_subsector(void)
{
    uint8_t addr[3] = {0x00, 0x00, 0x00};
    int8_t ret = 0;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("ERASING SUBSECTOR....");
    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_write_enable();
    spi_ret_check(ret);
    ret = mt25ql256aba_write_op(MT25QL256ABA_ERASE_4KB_SUBSECTOR, addr, sizeof(addr), NULL, 0);
    spi_ret_check(ret);
}

void page_verify(void)
{
    uint8_t addr[3] = {0x00, 0x00, 0x00};
    uint8_t data1[8] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF, 0x00, 0x11};
    uint8_t page_data[8];
    int8_t ret=0;

    NRF_LOG_INFO("");
    NRF_LOG_INFO("VERFIYING PAGE....");
    mt25ql256aba_check_ready_flag();
    ret = mt25ql256aba_read_op(MT25QL256ABA_READ, addr, sizeof(addr), page_data, sizeof(page_data));
    spi_ret_check(ret);
    for(int i = 0; i<8; ++i){
        NRF_LOG_INFO("Result = 0x%x (Expect: 0x%x)", page_data[i], data1[i]);
        if(data1[i] != page_data[i])
        {
            NRF_LOG_ERROR("FLASH PAGE WRITE TEST FAIL");
        }
    }
    //NRF_LOG_INFO("FLASH PAGE WRITE TEST PASS");
    //===========================================//
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

void reset_device(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("RESETING DEVICE....");
    mt25ql256aba_check_ready_flag();
    mt25ql256aba_read_op(MT25QL256ABA_RESET_ENABLE, NULL, 0, NULL, 0);
    mt25ql256aba_read_op(MT25QL256ABA_RESET_MEMORY, NULL, 0, NULL, 0);
}

void exit_4byte_mode(void)
{
    NRF_LOG_INFO("");
    NRF_LOG_INFO("EXIT 4byte....");
    mt25ql256aba_check_ready_flag();
    mt25ql256aba_write_op(0xE9, NULL, 0, NULL, 0);
}

int main (void)
{
    //initialize
    flash_spi_init();
    log_init();

    NRF_LOG_INFO("");
    NRF_LOG_INFO("mt25ql256aba flash test");

    read_test();
    write_test();

    full_page_read();
/*
    page_write();
    nrf_delay_ms(100);
    page_verify();
    */


    NRF_LOG_INFO("FLASH TEST COMPLETE ALL TEST PASS");


    while(1){
    }
}

static void log_init(void)
{
    ret_code_t err_code = NRF_LOG_INIT(NULL);
    APP_ERROR_CHECK(err_code);

    NRF_LOG_DEFAULT_BACKENDS_INIT();
}

static void spi_ret_check(int8_t ret)
{
    if (ret < 0){
        NRF_LOG_ERROR("SPI WRITE READ FAIL");
        while(1);
    }
}