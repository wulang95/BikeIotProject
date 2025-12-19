#ifndef     __HTTP_UPGRADE_OTA_H
#define     __HTTP_UPGRADE_OTA_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include    "rtos_port_def.h"

//#define AES128_EN
#define AES128_KEY  "1234567890123456"
#define HTTP_OTA_RAW_FILE_NAME "ota_raw.bin"
/*升级状态*/
enum {
    HTTP_OTA_WAIT = 0,
    HTTP_OTA_CHECK,  //检查能否升级
    HTTP_PDP_ACTIVE,
    HTTP_OTA_INIT,  //初始化
    HTTP_OTA_DOWN,
    HTTP_OTA_SUM_CHECK,
    HTTP_OTA_AES128_CHECK,
    HTTP_OTA_DOWN_FAIL,
    HTTP_OTA_UPDATA,
};

/*升级固件类型*/
enum {
    IOT_FIRMWARE_TYPE = 0,
    BLUE_FIRMWARE_TYPE,
    MCU_FIRMWARE_TYPE,
    VOICE_PACK_TYPE1,
    VOICE_PACK_TYPE2,
    VOICE_PACK_TYPE3,
    VOICE_PACK_TYPE4,
    VOICE_PACK_TYPE5,
    ECU_FIRMWARE_TYPE,
    BMS1_FIRMWARE_TYPE,
    BMS2_FIRMWARE_TYPE,
    HMI_FIRMWARE_TYPE,
    LOCK_FIRMWARE_TYPE
};


struct http_upgrade_info_stu {
    uint8_t req_type;
    uint8_t timeout;  //分钟
    char url[255];
    uint32_t crc_sum;   //效验码 OTA下发
    uint8_t ota_stage;   //ota升级状态
    int profile_idx;
    uint8_t stop_flag;
    def_rtos_sem_t http_ota_sem;  //阻塞开始升级
    uint8_t download_fail_cent;
    uint32_t download_start_byte;
};

extern struct http_upgrade_info_stu http_upgrade_info;
void app_http_ota_thread(void *param);
void app_http_ota_init();
int app_iot_ota_jump();
void http_upgrade_start();
void http_upgrade_stop();
#ifdef __cplusplus
}
#endif



#endif