#ifndef __NET_CONTROL_H
#define __NET_CONTROL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct gsm_info_stu{
    char imei[20];
    char iccid[24];
    uint8_t online :1;  
    uint8_t csq;
};
extern struct gsm_info_stu gsm_info;
void net_socket_close();
void net_control_init();
void net_update_singal_csq();
void net_socket_thread(void *param);
void pdp_active_thread(void *param);
void net_socket_send(uint8_t *data, uint16_t len);
void iot_mqtt_public(const uint8_t *data, uint16_t len);














#endif