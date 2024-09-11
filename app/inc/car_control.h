#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


struct bat_info_stu{
    uint16_t val;           /*单位0.1V*/
    uint16_t current;       /*单位0.1A*/
    uint8_t temp[3];        /*开尔文*/
    uint8_t charge_sta;     /*0x01:充电 0x00:未充电 0xff:充满*/
    uint8_t soh;  
    uint8_t soc;  
    uint16_t charge_interval;
    uint16_t charge_cycle;
    char softver[6];
    char hwver[6];
};

struct car_info_stu {
    struct bat_info_stu bat_info[2];
    uint8_t headlight_sta;  /*前灯状态*/
    uint8_t gear;   /*挡位*/
    uint8_t speed_limit;   /*限速*/
    uint8_t current_limit;  /*限流*/
    uint8_t wheel;      /*轮径*/
    uint8_t turn_light; /*转向灯*/

    uint8_t sync_time[6]; /*同步时间*/
    uint8_t mile_unit;      /*里程单位*/
    uint8_t bright_lev;     /*亮度等级*/
    uint8_t autoPoweroffTime;   /*自动关机时间*/
    uint8_t jump_password;  /*跳过密码*/
    uint8_t use_password;   /*使用密码*/

    char con_ver[6];    /*控制器版本*/
    char con_protocol_ver[6];   /*控制器协议版本*/
};



void car_init();










#ifdef __cplusplus
}
#endif

#endif