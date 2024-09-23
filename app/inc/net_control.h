#ifndef __NET_CONTROL_H
#define __NET_CONTROL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

struct gsm_info_stu{
    char imei[20];
    char iccid[24];
    uint8_t csq;
    uint8_t online;  
};
extern struct gsm_info_stu gsm_info;

void net_control_init();
void net_socket_thread(void *param);
void pdp_active_thread(void *param);
















#endif