#ifndef     __SYS_CORE_H
#define     __SYS_CORE_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include    "common.h"
#include   "rtos_port_def.h"
#include    "ble_control.h"
#include    "ble_protocol.h"
#include    "car_control.h"
#include    "bike_app_config.h"
#include    "app_rtc_task.h"


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
}BOOT_CONFIG_STR;

typedef enum{
    APP1_ADR,
    APP2_ADR,
    BOOT_CONFIG_ADR,
    SYS_CONFIG_ADR,
    BACK_SYS_CONFIG_ADR,
    CAR_SET_ADR,
} FLASH_PARTITION;

typedef struct {
    uint32_t magic;
    char ip[32];
    uint32_t port;   
    char apn[32];
    char DSN[32];
    char sn[16];
    char manufacturer[16];
    char ble_key[32];
    uint32_t crc32;
}SYS_CONFIG_STR;



#define BLE_NAME    "ENGWE-EG"
#define BLE_SUUID   0X1820
typedef struct {
    uint32_t magic;
    uint8_t lock_sta;             //锁的状态
    uint32_t carInfoUpSw;           //滑板车信息上传开关
    uint32_t carInfoUpInterval;        //滑板车信息上传间隔
    uint32_t carInfoLockUpInterval;     //上锁时上报间隔
    uint8_t carInchSppedDis;      //仪表显示单位
    uint8_t gs_level;             //震动等级
    uint8_t carFixedSpeedMode;    //定速巡航模式   
    uint8_t carLowSpeed;          //低速限速值   单位Km/h
    uint8_t carMidSpeed;          //中速限速值   单位Km/h
    uint8_t carHighSpeed;         //高速限速值   单位Km/h
    uint8_t carStratMode;         //启动方式  0：非零启动 1：零启动
    uint8_t carKeySwLight;        //按键切换大灯
    uint8_t carKeySpeed;          //按键切换速度模式
    
    uint8_t voiceCloseSw    :1;      //音量总开关  1:关闭  0：开启
    uint8_t alarmVoiceSw    :1;      //报警提示音开关 1：关闭  0：开启
    uint8_t unLockVoiceSw   :1;      //解锁提示音    1：关闭 0：开启
    uint8_t lockVoiceSw     :1;      //关锁提示音    1：关闭 0：开启
    uint8_t vioce_volum     :4;       //音量
    uint32_t crc32;
}CAR_SET;

void sys_init();
int64_t systm_tick_diff(int64_t time);
void debug_data_printf(char *str_tag, uint8_t *in_data, uint16_t data_len);



#endif