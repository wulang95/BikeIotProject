#ifndef __NET_EG_PROTOCOL_H
#define __NET_EG_PROTOCOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define   DOWN_HEADRER    0XA5A6
#define   UP_HEARDER      0XA7A8

enum {
    HEART_UP = 0XFF,
    ACK_UP = 0XFE,
    NACK_UP = 0XFD,
    STATUS_PUSH_UP = 0XFC,
    REGULARLY_REPORT_UP = 0XFB,
    CONFIG_FEEDBACK_UP = 0XFA,
    SIGN_IN_UP = 0XF9,
    OPERATION_FEEDBACK_UP = 0XF8,
    REGULARLY_REPORT2_UP = 0XF7,
    OPERATION_PUSH_UP = 0XF6,
    TRANSPARENT_RETURN_UP = 0XF5,
};

enum {
    FOTA_DOWN = 0XB0,
    REAL_OPERATION_DOWN = 0XB1,
    CONFIG_INSTRUCTION_DOWN = 0XB2,
    QUERY_INFORMATION_DOWN = 0XB3,
    CAN_TRANS_DOWN = 0XB2,
};

#pragma pack(1)
typedef struct {
    uint16_t MMC;
    uint16_t MNC;
    uint16_t LAC;
    uint32_t cell_id;
    uint8_t roam_state;
    uint8_t access_net;
    uint8_t fre_band;
    uint8_t csq;
    uint8_t error_rate;
} NET_INFO_STU;

typedef struct {
    uint8_t state;
    uint8_t bms_number;
    uint8_t soc;
    uint8_t soh;
    uint16_t remain_capacity;
    uint16_t full_capacity;
    uint16_t design_capacity;
    uint16_t val;
    uint16_t cycles;
    uint16_t charge_interval;
    uint16_t max_temp;
    uint8_t iot_soc;
} BATTERY_INFO_STU;

typedef struct {
    uint8_t soc;
    uint8_t current;    
} CHARGE_INFO_STU;

typedef struct {
    uint16_t speed;
    uint16_t argv_speed;
    uint16_t high_speed;
    uint32_t current_riding_time;   //3字节
    uint32_t total_riding_time;     //3字节
    uint16_t dst;   //当前行驶里程
    uint16_t remain_mileage;    //剩余行驶里程
    uint16_t odo;   
    uint16_t calori;
    uint16_t step_fre;
    uint16_t avg_step_fre;
    uint16_t total_avg_step_fre;
    uint16_t motor_power;
}CYCLE_INFO_STU;

typedef struct {
    uint8_t lock_sta;
    uint8_t move_sta;
    uint8_t vibration_sta;
    uint8_t flip_sta;
    uint8_t charge_sta;
    uint8_t iot_mode;
    uint8_t battry_plug;
    uint8_t sheep_sta;
    uint8_t penalty_sta;
}CAR_STATE_STU;

typedef struct {
    uint8_t gear;
    uint8_t head_light;
    uint8_t auto_power_off_time;
    uint8_t unit;
    uint8_t speed_limit;
    uint8_t wheel;
    uint8_t turn_light;
    uint8_t hid_sw;
    uint8_t vibration_alarm_sw;
    uint8_t password_sw;
    uint16_t ble_bland;
} CAR_CONFIG_STU;

typedef struct {
    uint8_t brightness;
    uint8_t mode;
    uint8_t colour_type;
    uint8_t rgb_en;
    uint8_t red;
    uint8_t gree;
    uint8_t blue;
    uint8_t task_en;
    uint8_t task_start_h;
    uint8_t task_start_m;
    uint8_t task_end_h;
    uint8_t task_end_m;
    uint8_t exec_action;
}ATMOSPHERE_LIGHT_STU;

typedef struct {
    uint8_t car_fault[8];
    uint8_t iot_fault[4];
} FAULT_INFO_STU;

typedef struct {
    char iccid[20];
    char sn[15];
    char mac[17];
}HW_INFO_STU;

typedef struct {
    uint8_t apn_len;
    char apn[39];
    uint8_t apn_user_len;
    char apn_user[32];
    uint8_t psw_len;
    char psw[32];
    uint8_t ip_len;
    char ip[128];
    uint16_t port;
    uint8_t time_zone;
    uint8_t sensor_set;
    uint8_t sensor_x;
    uint8_t sensor_y;
    uint8_t sensor_z;
}CONFIG_INFO_STU;

typedef struct {
    uint8_t instruction_param;
    uint8_t sub_param;
    uint8_t time_out;
    uint32_t time_stamp;
} REAL_OPERATE_STU;

enum {
    OTA_IOT_REC = 0X01,
    OTA_IOT_REFUSE = 0X02,
    OTA_SERVER_STOP = 0X03,
    OTA_BATTRY_LOW = 0X04,
    OTA_BATTRY_NO = 0X05,
    OTA_START_DOWNLOAD = 0X06,
    OTA_DOWNLOAD_SUCCESS = 0X07,
    OTA_DOWNLOAD_FAIL = 0X08,
    OTA_START_UPDATE = 0X09,
    OTA_UPDATE_SUCCESS = 0X0A,
    OTA_UPDATE_FAIL = 0X0B,
    OTA_VER_ERROR = 0X0C,
};

typedef struct {
    uint8_t ota_sta;
}OTA_STA_STU;


typedef struct {
    uint8_t ota_set;
    uint8_t ota_type1;
    uint8_t ota_type2;
    uint32_t fw_crc;
    uint8_t fw_url_len;
    char *fw_url;
}OTA_PARAM_STU;


typedef struct {
    uint16_t heart_interval;
    uint8_t close_lock_heart_sw;
    uint16_t close_lock_heart_inretval;
    uint8_t open_lock_heart_sw;
    uint16_t open_lock_heart_interval;
    uint8_t inter_battry_heart_sw;
    uint32_t inter_battry_heart_interval;
    uint8_t ble_connect_heart_sw;
    uint16_t ble_connect_heart_interval;
    uint8_t ble_disconnect_heart_sw;
    uint16_t ble_disconnect_heart_interval;
}HEART_INTERVAL_CONFIG_STU;


typedef struct {
    uint8_t qos;
    uint8_t publish_len;
    char publish_str[32];
    uint8_t subscribe_len;
    char subcribe_str[32];
    uint8_t account_len;
    char account_str[32];
    uint8_t psw_len;
    char psw_str[32];
    uint8_t sll_tls_set;
    uint8_t keep_alive_sw;
    uint16_t keep_alive_time;
    uint8_t will_set;
    uint8_t will_len;
    char will_str[32];
} MQTT_SET_STU;

typedef struct 
{
    uint16_t avg_speed;
    uint16_t max_speed;
    uint32_t current_riding_time;
    uint16_t current_cycl_mile;
    uint16_t cycl_burn_calories;
    uint16_t avg_step_fre;
};


#pragma pack()










#endif





