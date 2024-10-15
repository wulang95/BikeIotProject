#ifndef     __HTTP_UPGRADE_OTA_H
#define     __HTTP_UPGRADE_OTA_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include    "rtos_port_def.h"
/*升级状态*/
enum {
    IOT_RECV_OTA_REQ = 100,
    IOT_OTA_REFUSE,
    SERVICE_CLOSE_OTA_PROCESS,
    IOT_BAT_LOW,
    IOT_START_DOWNLOAD = 200,
    IOT_DOWNLOAD_SUCCESS,
    IOT_DOWNLOAD_FAIL,
    DEV_START_UPDATE = 300,
    DEV_UPDATE_SUCCESS,
    DEV_UPDATE_FAIL,
    FAIL_BAT_LOW_RES,
    FAIL_VER_INCOMPATIBLE_RES,
};

/*升级固件类型*/
enum {
    IOT_FIRMWARE_TYPE = 0,
    BLUE_FIRMWARE_TYPE,
    VOICE_PACK_TYPE,
    FENCE_FILE_TYPE,
    GPS_FIRMWARE_TYPE,
    ECU_FIRMWARE_TYPE,
    BMS1_FIRMWARE_TYPE,
    BMS2_FIRMWARE_TYPE,
    HMI_FIRMWARE_TYPE,
    LOCK_FIRMWARE_TYPE
};


struct http_upgrade_info_stu {
    uint8_t req_type;
    uint8_t timeout;  //分钟
    char url[256];
    uint8_t farme_type;
    uint32_t crc_sum;
    uint16_t ota_sta;
    def_rtos_sem_t http_ota_sem;
    uint8_t download_fail_cent;
    uint32_t download_start_byte;
};

extern struct http_upgrade_info_stu http_upgrade_info;
void app_http_ota_thread(void *param);
#ifdef __cplusplus
}
#endif



#endif