#ifndef __NET_ENGWE_PROTOCOL_H
#define __NET_ENGWE_PROTOCOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

enum {
    HEART_UP = 0XFF,  //心跳
    ACK_UP = 0XFE,  //ACK
    NACK_UP = 0XFD, //NACK
    STATUS_PUSH_UP = 0XFC,  //状态推送
    REGULARLY_REPORT_UP = 0XFB,  //定时上报
    CONFIG_FEEDBACK_UP = 0XFA, //配置回传
    SIGN_IN_UP = 0XF9, //签到
    OPERATION_FEEDBACK_UP = 0XF8,  //操作反馈
    REGULARLY_REPORT2_UP = 0XF7,  //定时上报2
    OPERATION_PUSH_UP = 0XF6,  //操作推送
    TRANSPARENT_RETURN_UP = 0XF5,  //透传指令返回
};

enum {
    FOTA_DOWN = 0XB0,  //FOTA
    REAL_OPERATION_DOWN = 0XB1,  //实时操作
    CONFIG_INSTRUCTION_DOWN = 0XB2,  //配置指令
    QUERY_INFORMATION_DOWN = 0XB3,  //查询信息
};

enum {
    GPS_INFO_CMD = 0,   //GPS数据 0x00000001
    NET_INFO_CMD,   //当前网络信息 0x00000002
    BATTRY_INFO_CMD,    //电池信息 0x00000004
    CHARGE_PARAM_CMD, //充电参数 0x00000008
    RIDE_INFO_CMD,  //骑行信息 0x00000010
    CAR_STATE_CMD, //车辆状态 0x00000020
    CAR_CONFIG_CMD, //车辆配置 0x00000040
    LIGHT_STATE_CMD,    //灯光状态 0x00000080
    FIRMWARE_VER_CMD,   //固件版本 0x00000100
    FAULT_CODE_CMD, //故障代码 0x00000200
    IOT_HW_INFO_CMD,    //IoT硬件信息 0x00000400
    IOT_CONFIG_CMD, //IoT配置信息 0x00000800
    REAL_TIME_OPERATE_CMD, //实时操作   0x00001000
    OTA_STATE_INFO_CMD, //OTA状态信息 0x00002000
    OTA_PARAM_CMD,  //OTA 参数 0x00004000
    IOT_POST_SET_CMD, //IoT报文设置 0x00008000
    IOT_REPORT_INV_SET_CMD, //IoT上报间隔设置  0x00010000
    QUERY_PARAM_CMD, //查询参数 0x00020000
    SHEEPFANG_SET_CMD,  //羊圈设置 0x00040000
    FORBIDDEN_ZONE_SET_CMD, //禁区设置 0x00080000
    OPERATE_RES_RETURN, //操作结果返回 0x00100000
    MQTT_SET_CMD,   //MQTT设置 0x00200000
    BMS_HEALTH_CMD,  //电池健康 0x00400000
    RIDE_INV_INFO_CMD,  //骑行区间信息 0x00800000
    CMD_ID_MAX,
};

enum{
    FOTA_IOT_RECV = 0X01,  //IOT收到升级指令
    FOTA_IOT_REFUSE,        //IOT拒绝升级
    FOTA_SERVE_STOP,        //服务器停止升级
    FOTA_BAT_LOW,           //外电池电量太低
    FOTA_NOT_BAT,           //没接外电池
    FOTA_START_DOWN,         //IoT开始下载固件包
    FOTA_DOWN_SUCCESS,      //IoT下载固件包成功
    FOTA_DOWN_FAIL,         //IoT下载固件包失败
    FOTA_SATRT_UPDATE,      //开始更新固件
    FOTA_UPDATE_SUCCESS,      //更新固件成功
    FOTA_UPDATE_FAIL,       //更新固件失败
    FOTA_VER_ERROR,         //版本不兼容更新失败
};

typedef struct {
    uint8_t instruction_param;
    uint8_t sub_param;
    uint16_t seq;
} REAL_OPERATE_STU;


uint16_t net_engwe_cmdId_operate_respos(uint8_t *p, REAL_OPERATE_STU real_operate, uint8_t res, uint8_t fail_reson);
void net_engwe_pack_seq_up(uint8_t cmd_type, uint8_t *cmd_data, uint16_t cmd_len, uint16_t seq_num);
void net_engwe_send_thread(void *param);
void NET_ENGWE_CMD_MARK(uint8_t cmd);
void net_engwe_init();
void net_engwe_data_parse(uint8_t *data, uint16_t len);
void net_engwe_fota_state_push(uint8_t ota_state);
void net_engwe_cmd_push(uint8_t cmd_type, uint32_t info_id);
void net_engwe_signed();
#endif





