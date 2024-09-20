#ifndef     __SYS_CORE_H
#define     __SYS_CORE_H
#include    "app_common.h"
#include    "rtos_port_def.h"
#include    "ble_control.h"
#include    "ble_protocol.h"
#include    "car_control.h"
#include    "bike_app_config.h"
#include    "app_rtc_task.h"
#include    "app_virt_uart.h"
#include    "app_led_ind.h"
#include    "mcu_uart.h"
#include    "can_protocol.h"
#include    "net_control.h"
#include    "car_control.h"


void assert_handler(const char *ex_string, const char *func, size_t line);

#define SYS_ALIGN_DOWN(v, n) ((unsigned long)(v) & ~((n)-1))

#define ASSERT(EX) if(!(EX)) {\
    assert_handler(#EX, __func__, __LINE__);\
}   

typedef enum {
    OTA_SUCCESS,
    OTA_FAIL,
    OTA_NEED,
    OTA_NO, 
} OTA_RES;

/*
    step1 app拉取升级文件后，置ota_res为OTA_NEED,写入boot配置。
    step2 在boot中读取boot配置, 为OTA_NEED时,校验app，成功更新ota_res为OTA_SUCCESS，更新origin_app为app_addr。失败ota_res更新为OTA_FAIL。为其它值时直接跳转到origin_app;
    step3 app读取ota_res, 为OTA_SUCCESS或OTA_FAIL，上报后台，并更新ota_res为OTA_NO。

*/
typedef struct {
    uint32_t magic;
    uint32_t origin_app;
    uint32_t app_addr;
    uint32_t app_len;
    uint32_t var;
    uint32_t app_crc32;
    OTA_RES ota_res; 
    uint32_t crc32;
}BOOT_CONFIG_STU;

typedef enum{
    APP1_ADR,
    APP2_ADR,
    BOOT_CONFIG_ADR,
    SYS_CONFIG_ADR,
    BACK_SYS_CONFIG_ADR,
    CAR_SET_ADR,
} FLASH_PARTITION;

struct sys_config_stu{
    uint32_t magic;
    char ip[32];
    uint32_t port; 
    char dev_type[6];  
    char apn[32];
    char DSN[32];
    char sn[16];
    char manufacturer[16];
    char ble_key[32];
    uint32_t crc32;
};
#pragma pack(1)
struct sys_info_stu{
    uint8_t bat_soc;
    uint8_t bat_soh;
    uint8_t fault;
    uint8_t pdp_reg :1;
    uint8_t paltform_connect :1;
    uint8_t gps_state :1;
    uint8_t exits_bat :1;
    uint8_t ble_connect :1;
    uint8_t startup_way;  /*1表示蓝牙开机   2表示app按键开机*/
    uint8_t can_key;
    uint8_t can_protocol_major;
    uint8_t can_protocol_sub;
};
#pragma pack()

#define SOFTVER "1.0"
#define HWVER   "1.0"
#define DEFAULT_MANUFACTURER  "ENGWE" 
#define DEFAULT_SN      "123456789" 
#define DEFAULT_DEV_TYPE    "K10" 
#define DEFAULT_APN     "asia.bics"
#define DEFAULT_IP     "36.137.226.30"
#define DEFAULT_PORT   38026

#define BLE_NAME    "ENGWE"
#define BLE_SUUID   0X1820


extern struct sys_info_stu sys_info;
extern struct sys_config_stu sys_config;
void app_sys_init();
int64_t systm_tick_diff(int64_t time);
void debug_data_printf(char *str_tag, uint8_t *in_data, uint16_t data_len);



#endif