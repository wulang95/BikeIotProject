#ifndef __CAR_CONTROL_H
#define __CAR_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "net_engwe_protocol.h"
#include "ble_protocol.h"

enum{
    IOT_CAR_CMD_SER = 0,
    NET_CAR_CMD_SER,
    BLUE_CAR_CMD_SER,
    HID_CAR_CMD_SER,
};

enum {
    CAR_CMD_LOCK,
    CAR_CMD_UNLOCK,
    CAR_CMD_SET_HEADLIGHT,
    CAR_CMD_SET_TAILLIGHT,
    CAR_CMD_SET_GEAR,
    CAR_CMD_SET_SPEED_LIMIT,
    CAR_CMD_SET_TURN_LIGHT,
    CAR_CMD_SET_MILEAGE_UNIT,
    CAR_CMD_SET_POWER_ON_PASSWORD,
    CAR_CMD_SET_ATSPHLIGHT_MODE,
    CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM,
    CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL,
    CAR_CMD_SET_ATSPHLIGHT_TURN,
    CAR_CMD_SET_ATSPHLIGHT_COLORTYPE,
    CAR_CMD_JUMP_PASSWORD,
    CAR_CMD_EN_POWER_ON_PASSWORD,
    CAR_BMS_CHARGE_SOC_SET,
    CAR_BMS_CHARGE_CURRENT_SET,
    CAR_LOOK_CAR1,
    CAR_LOOK_CAR2,
};

enum {
    CAR_LOCK_SHOCK = 0X01,
    CAR_LOCK_STILL = 0X02,
    CAR_LOCK_TO_SHOCK
};

enum {
    CAR_FALL_STATE = 0X01,
    CAR_NORMAL_STATE = 0X02,
    CAR_FALL_TO_RECOVERY = 0X03,
    CAR_RECOVERY_TO_FALL = 0X04,
};


enum {
    CAR_UNLOCK_MOVE = 0X01, //车运动状态
    CAR_UNLOCK_STILL,  //车静止状态
    CAR_UNLOCK_TO_STILL,   //车运动到静止
    CAR_UNLOCK_TO_MOVE,    //车静止到运动
};

enum {
    CHARGER_NONE_STA =0X00,
    CHARGER_PLUG_IN,
    CHARGER_PUG_OUT,
    CHARGER_FULL,
    CHARGER_QUICK_IN,
    CHARGER_STATE,
};

typedef struct {
    uint8_t cmd;
    uint8_t src;
    union 
    {
        BLE_OPERATE_STU ble_car_control; 
        REAL_OPERATE_STU net_car_control;
    };
}CAR_CMD_Q;

#pragma pack(1)

struct atmosphere_light_task_stu{
    uint8_t task_en;
    int64_t start_timestap;
    int64_t end_timestap;
    uint8_t action;
};

struct atmosphere_light_info_stu{
    struct atmosphere_light_task_stu atmosphere_light_task;
    uint8_t light_mode;
    uint8_t color;
    uint8_t brightness_val;
    uint8_t turn_linght_sta;
    uint8_t ble_sta;
    uint8_t custom_red;
    uint8_t custom_green;
    uint8_t custom_blue;
};

struct hmi_info_stu{
    uint8_t init;
    uint8_t power_on  :1;
    uint8_t passwd_en :1;
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
    uint8_t init;
    uint8_t connect;
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
    uint16_t remain_capacity;
    uint16_t full_capactity;
    uint16_t design_capactity;
    uint8_t pack_series_number;
    uint8_t pack_parallel_number;
    uint16_t cell_val[16];  //单位mv
    uint16_t pack_vol;  //单位0.1V
    uint16_t max_charge_val;
    uint16_t pack_current;
    uint8_t max_continuous_current; 
    uint8_t peak_current;
    uint8_t max_charge_current;
    uint8_t first_protect[4];
    uint8_t second_protect[4];
    uint8_t first_protect_code;
    uint8_t second_protect_code;
    uint8_t fault_code;
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

    struct {
        uint32_t capacity_input_quantity;  /*容量输入量 单位mAh*/
        uint32_t engwe_input_quantity;  /*电能输入量 单位mWh*/
        uint32_t extreme_temperature_use_time;  /*极限温度使用时间 S*/
        uint32_t extreme_temperature_charge_time; /*极端温度充电时间 S*/
        uint16_t deep_discharge_count;  /*深度放电次数*/
        uint8_t battery_self_discharge_rate;  /*电池自耗率 单位0.4%，范围0%~100%*/
        uint8_t engwe_exchange_efficiency;  /*电能换算效率 单位0.4%，范围0%~100%*/
        uint16_t battery_internal_resistance;  /*电池内部阻抗 单位mR*/
        uint8_t dev_type;  /*设备类型 0:三元锂电池  1：磷酸铁锂电池 2-255：预留*/
        uint8_t function_support;  /*功能支持 bit0:是否支持快充, bit1~7:预留*/
        uint32_t capactity_output_quantity;  /*容量输出量 单位mAh*/
        uint32_t engwe_output_quantity;  /*电能输出量 单位mWh*/
    } ece_regulation;

    struct {
        uint8_t year; /* 年， 25表示2025年 */
        uint8_t month; /* 月， 1-12 */
        uint8_t day; /* 日， 1-31 */
    }manufacture_date;
};
struct electronic_lock_stu {
    uint8_t init;
    char soft_ver[8];
    char hw_ver[8];
    char type_str[8];
    char firm_identify[8];
    uint8_t fault_code;
    uint8_t lock_sta;
};

struct car_info_stu {
    struct hmi_info_stu hmi_info;
    struct bms_info_stu bms_info[2];
    struct atmosphere_light_info_stu atmosphere_light_info;
    struct electronic_lock_stu electronic_lock;
    uint8_t con_init;
    uint8_t hmi_connnect :1;
    uint8_t control_connect :1;
    uint8_t lock_connect :1;
    uint8_t lock_sta;
    uint8_t last_lock_sta;
    uint8_t lock_src;
    uint8_t main_power;
    uint8_t last_headlight_sta:1;
    uint8_t headlight_sta :1;  /*前灯状态*/
    uint8_t taillight_sta :1;
    uint8_t left_turn_light_sta :2;
    uint8_t right_turn_light_sta :2;
    uint8_t gear;   /*挡位*/
    uint8_t last_gear;
    uint8_t last_unit;
    uint16_t speed_limit;   /*限速 0.1KM/H*/
    uint16_t last_speed_limit;
    uint8_t current_limit;  /*限流 0.1A*/
    uint8_t wheel;      /*轮径*/
    uint16_t last_speed;
    uint16_t speed;   //单位 0.1KM
    uint8_t transfer_data; //转把数据 0.1V
    uint16_t avg_speed;  //平均车速 0.1km/h
    uint16_t motor_speed; //电机转速
    uint16_t max_speed; //最高车速 0.1km/h
    uint32_t cycle_time_s;  //骑行时间
    uint32_t cycle_total_time; //总骑行时长 Min
    uint8_t fault_code;
    uint8_t sync_time[6];  /*同步时间*/
    uint8_t bright_lev;     /*亮度等级*/
    uint8_t autoPoweroffTime;   /*自动关机时间*/
    uint8_t jump_password;  /*跳过密码*/
    uint8_t use_password;   /*使用密码*/
    uint16_t cycle_power;   //骑行功率
    uint16_t cycle_avg_power;   //骑行平均功率
    uint16_t motor_avg_power;   //电机平均功率
    uint16_t motor_consumption; //电机瞬间功耗 数据范围 00—65535单位：0.01Wh/km
    uint16_t motor_avg_consumption; //电机平均功耗 数据范围 00—65535单位：0.01Wh/km
    uint8_t reduce_power_sta;
    uint8_t stop_drive_sta;
    uint32_t total_odo;   //总里程 单位 0.1KM
    uint32_t single_odo;    //单次里程  单位 0.1KM
    uint16_t remain_odo;    //剩余里程 单位 0.1KM
    uint8_t current;        //母线电流 单位0.1A
    uint8_t power_sta;      
    uint16_t pedal_speed;   //脚踏转速  单位1RPM
    uint16_t m_agv_pedal_speed; //1分钟平均踏频
    uint16_t total_agv_pedal_speed; //总平均踏频
    uint8_t pedal_torque;   //脚踏扭矩   单位1Nm
    uint16_t motor_power;   //电机输出功率 0.1W
    uint8_t control_torque; //电机输出扭矩 1Nm
    uint8_t control_temp;   //控制器温度    -40
    uint8_t motor_temp;     //电机温度 -40
    uint8_t protocol_major_ver;
    uint8_t protocol_sub_ver;
    uint8_t assist_seneor_type;
    uint8_t promote_func;
    uint16_t bus_voltage;  //母线电压 0.1V
    uint32_t trip_calorie;  //消耗卡路里
    uint16_t ebike_calorie;
    uint8_t car_unlock_state;
    uint8_t car_lock_state;
    uint8_t filp_state;
    uint8_t move_alarm;
    uint8_t quick_charger_det :1; //为1时检测到快充接入
    uint8_t quick_charger_flag :1; //快充插入检测标志
    uint8_t charger_det_flag :1;
    uint8_t charger_full_flag :1;
    uint8_t charger_state;
    uint16_t charge_vol;   //单位0.1V
    uint8_t charge_cur;   //单位0.1A
    char control_hw_ver[16];
    char control_soft_ver[16];
    char control_sn[32];
    char control_param_ver[24];
};

struct car_state_data_stu {
    uint8_t abnormal_move;
    uint8_t mobile_operation_sta; //状态标志
    uint8_t slope_data; //坡度数据
    uint8_t attitude;  //姿态
    uint8_t map_dir;   //导航方向
    uint32_t cur_dir_range;  //剩余距离
    uint32_t total_nav_remaintime;  //总导航剩余时间
    uint32_t total_nav_range;   //总导航剩余距离
};

#pragma pack()

enum {
    CAR_LOCK_STA = 0,
    CAR_UNLOCK_STA,
};


struct car_set_save_stu{
    uint32_t magic;
    struct atmosphere_light_info_stu atmosphere_light_set;
    uint32_t odo;
    uint32_t trip;
    uint8_t  range;
    uint8_t power_on_psaaword[4];
    uint8_t gear;
    uint8_t jump_password;
    uint8_t en_power_on_psaaword;
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
    uint16_t speed_limit;          // 单位0.1Km/h
    uint8_t bright_lev;
    uint8_t gps_track_interval;
    uint8_t voiceCloseSw    :1;      //音量总开关  1:关闭  0：开启
    uint8_t alarmVoiceSw    :1;      //报警提示音开关 1：关闭  0：开启
    uint8_t unLockVoiceSw   :1;      //解锁提示音    1：关闭 0：开启
    uint8_t lockVoiceSw     :1;      //关锁提示音    1：关闭 0：开启
    uint8_t vioce_volum     :4;       //音量
    uint8_t look_car_sw;
    uint8_t navigation_quit_time;
    uint32_t crc32;
};

extern struct car_info_stu car_info;
extern struct car_set_save_stu car_set_save;
extern struct car_state_data_stu car_state_data;
void car_heart_event();
int car_control_cmd(uint8_t cmd);
void car_init();
int car_lock_control(uint8_t src, uint8_t lock_operate);
void car_control_thread(void *param);
void CAR_CMD_MARK(CAR_CMD_Q car_cmd_q);






#ifdef __cplusplus
}
#endif

#endif