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
    GPS_INFO_CMD = 0,   //GPS数据
    NET_INFO_CMD,   //当前网络信息
    BATTRY_INFO_CMD,    //电池信息
    CHARGE_PARAM_CMD, //充电参数
    RIDE_INFO_CMD,  //骑行信息
    CAR_STATE_CMD, //车辆状态
    CAR_CONFIG_CMD, //车辆配置
    LIGHT_STATE_CMD,    //灯光状态
    FIRMWARE_VER_CMD,   //固件版本
    FAULT_CODE_CMD, //故障代码
    IOT_HW_INFO_CMD,    //IoT硬件信息
    IOT_CONFIG_CMD, //IoT配置信息
    REAL_TIME_OPERATE_CMD, //实时操作
    OTA_STATE_INFO_CMD, //OTA状态信息
    OTA_PARAM_CMD,  //OTA 参数
    IOT_POST_SET_CMD, //IoT报文设置
    IOT_REPORT_INV_SET_CMD, //IoT上报间隔设置
    QUERY_PARAM_CMD, //查询参数
    SHEEPFANG_SET_CMD, //羊圈设置
    FORBIDDEN_ZONE_SET_CMD,  //禁区设置
    OPERATE_RES_RETURN, //操作结果返回
    MQTT_SET_CMD,   //MQTT设置
    CAN_TRANS_CMD,  //CAN透传
    RIDE_INV_INFO_CMD,  //骑行区间信息
    CMD_ID_MAX,
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
#endif





