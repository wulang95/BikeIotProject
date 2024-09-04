#include "app_system.h"
#include "hal_drv_flash.h"
#define DBG_TAG         "app_system"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif





void assert_handler(const char *ex_string, const char *func, size_t line)
{
    LOG_E("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    while(1);
}


void flash_partition_erase(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case APP1_ADR:
            hal_drv_flash_erase(APP1_ADDR, APP1_SIZE);
            break;
        case APP2_ADR:
            hal_drv_flash_erase(APP2_ADDR, APP2_SIZE);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_erase(BOOT_CONFIG_ADDR, BOOT_CONFIG_SIZE);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_erase(SYS_CONFIG_ADDR, SYS_CONFIG_SIZE);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_erase(BACK_SYS_CONFIG_ADDR, BACK_SYS_CONFIG_SIZE);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_erase(CAR_SET_ADD, CAR_SET_SIZE);
            break;
        default:
            break;
    }
}

void flash_partition_write(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    switch(flash_part){
        case APP1_ADR:
            hal_drv_flash_write(APP1_ADDR + shift, data, lenth);
            break;
        case APP2_ADR:
            hal_drv_flash_write(APP2_ADDR + shift, data, lenth);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_write(BOOT_CONFIG_ADDR, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_write(SYS_CONFIG_ADDR, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_write(BACK_SYS_CONFIG_ADDR, data, lenth);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_write(CAR_SET_ADD, data, lenth);
            break;
        default:
            break;
    }
}

void flash_partition_read(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    switch(flash_part){
        case APP1_ADR:
            hal_drv_flash_read(APP1_ADDR + shift, data, lenth);
            break;
        case APP2_ADR:
            hal_drv_flash_read(APP2_ADDR + shift, data, lenth);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_read(BOOT_CONFIG_ADDR, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_read(SYS_CONFIG_ADDR, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_read(BACK_SYS_CONFIG_ADDR, data, lenth);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_read(CAR_SET_ADD, data, lenth);
            break;
        default:
            break;
    }
}

void sys_init()
{
    ble_control_init();
}