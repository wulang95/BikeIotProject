#ifndef __NET_PROTOCOL_H
#define __NET_PROTOCOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


enum {
    NET_CMD_SIGN_IN_Q0 = 0,
    NET_CMD_GSM_HEART_H0,
    NET_CMD_Q_LOCK_CONTROL_R0,
    NET_CMD_LOCK_OPEN_L0,
    NET_CMD_LOCK_CLOSE_L1,
    NET_CMD_IOT_DEV_CONFIG_S5,
    NET_CMD_Q_CAR_INFO_S6,
    NET_CMD_CAR_CONFIG_S7,
    NET_CMD_ALARM_UP_W0,
    NET_CMD_VOICE_PLAY_V0,
    NET_CMD_Q_LOCATION_D0,
    NET_CMD_LOCATION_TRACK_D1,
    NET_CMD_Q_VER_G0,
    NET_CMD_UP_HMI_FAULT_E0,
    NET_CMD_UP_CONTROL_FAULT_E1,
    NET_CMD_UP_BMS_FAULT_E2,
    NET_CMD_CHECK_STARUP_UPGRADE_U0,
    NET_CMD_Q_UPGRADE_DATA_U1,
    NET_CMD_UPGRADE_SUCCESS_NOTIFI_U2,
    NET_CMD_EVENT_NOTIFI_S1,
    NET_CMD_POWER_ON_OFF_L3,
    NET_CMD_STARUP_HTTP_UPGRADE_U5,
    NET_CMD_HTTP_UPGRADE_STATE_U6,
};

void net_protocol_init();
void NET_CMD_MARK(uint8_t cmd);
void net_recv_data_parse(uint8_t *data, uint16_t len);
void net_protocol_cmd_send(uint8_t cmd);
void net_protocol_send_thread(void *param);






















#endif