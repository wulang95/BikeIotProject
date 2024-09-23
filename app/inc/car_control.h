#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


enum {
    CAR_CMD_LOCK,
    CAR_CMD_UNLOCK,
    CAR_CMD_SET_HEADLIGHT,
    CAR_CMD_SET_TAILLIGHT,
    CAR_CMD_SET_GEAR,
    CAR_CMD_SET_SPEED_LIMIT,
    CAR_CMD_SET_TURN_LIGHT,
    CAR_CMD_SET_MILEAGE_UNIT,
    CAR_CMD_SET_BRIGHTNESS_LEVEL,
    CAR_CMD_SET_POWER_ON_PASSWORD,
};

#pragma pack(1)
struct hmi_info_stu{
    uint8_t power_on  :1;
    uint8_t encry_mode_data :7;
    uint8_t key;
    uint8_t startup_mode;
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t fault_code;
    uint8_t display_unit :1;
    uint8_t encry_lock_sta;
    uint8_t left_turn_light:1;
    uint8_t right_turn_linght:1;
    uint8_t look_car_sta:1;
    char hw_ver[16];
    char soft_ver[16];
    char sn[32];
};

struct bms_info_stu{
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t temp_probe_number;
    uint8_t cell_temp;
    uint8_t board_temp;
    uint8_t mos_temp;
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
    uint16_t max_charge_val;
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
    uint16_t charge_interval_time;
    uint16_t maxcharge_interval_time;
    uint16_t alive_time;
    uint8_t design_val;
    uint16_t cycle_number;
    uint8_t key;
    uint8_t code_len;
    char code[32];
    char soft_ver[32];
    char hw_ver[16];
};

struct car_info_stu {
    struct hmi_info_stu hmi_info;
    struct bms_info_stu bms_info[2];
    uint8_t bms_connect :1;
    uint8_t hmi_connnect :1;
    uint8_t control_connect :1;
    uint8_t lock_connect :1;
    uint8_t lock_sta;
    uint8_t lock_src;
    uint8_t main_power;
    uint8_t headlight_sta;  /*前灯状态*/
    uint8_t taillight_sta;
    uint8_t left_turn_light_sta :2;
    uint8_t right_turn_light_sta :2;
    uint8_t gear;   /*挡位*/
    uint16_t speed_limit;   /*限速 0.1KM/H*/
    uint8_t current_limit;  /*限流 0.1A*/
    uint8_t wheel;      /*轮径*/
    uint16_t speed;
    uint8_t transfer_data;
    uint16_t avg_speed;
    uint16_t motor_speed;
    uint16_t max_speed;
    uint32_t cycle_time_s;
    uint8_t fault_code;
    uint8_t sync_time[6];  /*同步时间*/
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
    uint32_t total_odo;
    uint32_t single_odo;
    uint16_t remain_odo;
    uint8_t current;
    uint8_t power_sta;
    uint8_t pedal_speed;
    uint8_t pedal_torque;
    uint16_t motor_power;
    uint8_t control_torque;
    uint8_t control_temp;
    uint8_t motor_temp;
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t assist_seneor_type;
    uint8_t promote_func;
    uint16_t bus_voltage;
    uint32_t calorie;
    char control_hw_ver[16];
    char control_soft_ver[16];
    char control_sn[32];
    char control_param_ver[24];
};

struct car_state_data_stu {
    uint8_t abnormal_move;
    uint8_t mobile_operation_sta;
    uint8_t slope_data;
    uint8_t attitude;
    uint8_t map_dir;
    uint32_t cur_dir_range;
    uint32_t total_nav_remaintime;
    uint32_t total_nav_range;
};

#pragma pack()

enum {
    CAR_LOCK_STA = 0,
    CAR_UNLOCK_ATA,
};

enum {
    BLE_AUTO_LOCK_SRC,
    BLE_CMD_LOCK_SRC,
    NET_CMD_LOCK_SRC,
    HMI_CMD_LOCK_SRC,
};

struct car_set_save_stu{
    uint32_t magic;
    uint32_t odo;
    uint32_t trip;
    uint8_t  range;
    uint8_t power_on_psaaword[4];
    uint8_t gear;
    uint8_t left_turn_light :2;
    uint8_t right_turn_light :2;
    uint8_t tail_light :1;
    uint8_t head_light :1;
    uint8_t anti_theft_on :1;          //防盗开启
    uint8_t mileage_unit: 1;  //0为mile, 1为Km
    uint8_t lock_sta;             //锁的状态
    uint32_t carInfoUpSw;           //滑板车信息上传开关
    uint32_t carInfoUpInterval;        //滑板车信息上传间隔
    uint32_t carInfoLockUpInterval;     //上锁时上报间隔
    uint8_t carInchSppedDis;      //仪表显示单位
    uint8_t gs_level;             //震动等级
    uint8_t carFixedSpeedMode;    //定速巡航模式   
    uint16_t speed_limit;          // 单位Km/h
    uint8_t bright_lev;
    uint8_t voiceCloseSw    :1;      //音量总开关  1:关闭  0：开启
    uint8_t alarmVoiceSw    :1;      //报警提示音开关 1：关闭  0：开启
    uint8_t unLockVoiceSw   :1;      //解锁提示音    1：关闭 0：开启
    uint8_t lockVoiceSw     :1;      //关锁提示音    1：关闭 0：开启
    uint8_t vioce_volum     :4;       //音量
    uint32_t crc32;
};

extern struct car_info_stu car_info;
extern struct car_set_save_stu car_set_save;
extern struct car_state_data_stu car_state_data;
void car_heart_event();
void car_control_cmd(uint8_t cmd);
void car_init();
void car_lock_control(uint8_t src, uint8_t lock_operate);










#ifdef __cplusplus
}
#endif

#endif