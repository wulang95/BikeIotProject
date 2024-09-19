#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#pragma pack(1)
struct hmi_info_stu{
    uint8_t power_on  :1;
    uint8_t encry_mode_data :7;
    uint8_t key;
    uint8_t startup_mode;
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t fault_code;
    uint8_t display_unit;
    uint8_t encry_lock_sta;
    char hw_ver1[8];
    char hw_ver2[8];
    char soft_ver1[8];
    char soft_ver2[8];
    char sn1[8];
    char sn2[8];
    char sn3[8];
    char sn4[8];
};

struct bat_info_stu{
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t temp_probe_number;
    uint8_t cell_temp;
    uint8_t board_temp;
    uint8_t max_temp;
    uint8_t min_temp;
    uint8_t soh;
    uint8_t soc;
    uint8_t remain_capacity;
    uint8_t full_capactity;
    uint8_t design_capactity;
    uint8_t pack_series_number;
    uint8_t pack_parallel_number;
    uint16_t cell_val[16];
    uint16_t pack_vol;
    uint16_t pack_current;
    uint8_t max_continuous_current; 
    uint8_t peak_current;
    uint8_t max_charge_current;
    uint8_t first_protect[4];
    uint8_t second_protect[4];

    uint8_t charge_det  :1;
    uint8_t charge_sta  :1;
    uint8_t charge_mos  :1;
    uint8_t discharge_mos :1;
    uint8_t chargefull_sta  :1;
    uint8_t double_bms_sta  :2;

    uint8_t discharge_sta;
    uint16_t charge_remain_time;
    uint8_t design_val;
    uint16_t cycle_number;
};

struct car_info_stu {
    struct hmi_info_stu hmi_info;
    struct bat_info_stu bat_info[2];
    uint8_t headlight_sta;  /*前灯状态*/
    uint8_t taillight_sta;
    uint8_t gear;   /*挡位*/
    uint8_t speed_limit;   /*限速*/
    uint8_t current_limit;  /*限流*/
    uint8_t wheel;      /*轮径*/
    uint8_t turn_light; /*转向灯*/
    uint16_t speed;
    uint8_t transfer_data;
    uint16_t avg_speed;
    uint16_t motor_speed;
    uint16_t max_speed;
    uint8_t cycle_time_minute;
    uint8_t cycle_time_hour;
    uint8_t fault_code;
    uint8_t sync_time[6];  /*同步时间*/
    uint8_t mile_unit;      /*里程单位*/
    uint8_t bright_lev;     /*亮度等级*/
    uint8_t autoPoweroffTime;   /*自动关机时间*/
    uint8_t jump_password;  /*跳过密码*/
    uint8_t use_password;   /*使用密码*/
    uint16_t cycle_power;
    uint16_t cycle_avg_power;
    uint16_t motor_avg_power;
    uint16_t motor_consumption;
    uint16_t motor_avg_consumption;
    uint8_t reduce_power_sta;
    uint8_t stop_drive_sta;
    uint16_t total_odo;
    uint16_t single_odo;
    uint8_t remain_odo;
    uint8_t current;
    uint8_t power_sta;
    uint8_t pedal_speed;
    uint8_t pedal_torque;
    uint16_t control_power;
    uint8_t control_torque;
    uint8_t control_temp;
    uint8_t motor_temp;
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t assist_seneor_type;
    uint8_t promote_func;
    uint16_t bus_voltage;
    char con_ver[6];    /*控制器版本*/
    char con_protocol_ver[6];   /*控制器协议版本*/
    char con_hw_ver1[8];
    char con_hw_ver2[8];    /*可忽略*/
    char con_soft_ver1[8];
    char con_soft_ver2[8];  /*可忽略*/    
    char con_sn1[8];
    char con_sn2[8];
    char con_sn3[8];
    char con_sn4[8];
    char con_param_ver1[8];
    char con_param_ver2[8];
    char con_param_ver3[8];
};
#pragma pack()
extern struct car_info_stu car_info;


void car_init();










#ifdef __cplusplus
}
#endif

#endif