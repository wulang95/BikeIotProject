#ifndef     __SYS_CORE_H
#define     __SYS_CORE_H
#include    "app_common.h"
#include    "rtos_port_def.h"
#include    "ble_control.h"
#include    "ble_protocol.h"
#include    "car_control.h"
#include    "bike_app_config.h"
#include    "app_rtc_task.h"
#include    "app_virt_uart.h"
#include    "app_led_ind.h"
#include    "mcu_uart.h"
#include    "can_protocol.h"
#include    "net_control.h"
#include    "net_protocol.h"
#include    "car_control.h"
#include    "http_upgrade_ota.h"
#include    "app_sensor.h"
#include    "app_audio.h"
#include    "GPS_control.h"
#include "hal_drv_gpio.h"
#include "low_power.h"
#include "hal_drv_net.h"
#include "net_engwe_protocol.h"
#include "app_error.h"

extern def_rtos_task_t app_system_task;
void assert_handler(const char *ex_string, const char *func, size_t line);

#define SYS_ALIGN_DOWN(v, n) ((unsigned long)(v) & ~((n)-1))

#define ASSERT(EX) if(!(EX)) {\
    assert_handler(#EX, __func__, __LINE__);\
}   

typedef enum {
    OTA_SUCCESS,
    OTA_FAIL,
    OTA_NEED,
    OTA_NO, 
} OTA_RES;

#define IOT_MAGIC   0XFAFA5858 

/*
    step1 app拉取升级文件后，置ota_res为OTA_NEED,写入boot配置。
    step2 在boot中读取boot配置, 为OTA_NEED时,校验app，成功更新ota_res为OTA_SUCCESS，更新origin_app为app_addr。失败ota_res更新为OTA_FAIL。为其它值时直接跳转到origin_app;
    step3 app读取ota_res, 为OTA_SUCCESS或OTA_FAIL，上报后台，并更新ota_res为OTA_NO。

*/
typedef struct {
    uint32_t magic;
    uint32_t origin_app;
    uint32_t app_addr;
    uint32_t app_len;
    uint32_t var;
    uint32_t app_crc32;
    OTA_RES ota_res; 
    uint32_t crc32;
}BOOT_CONFIG_STU;

typedef enum{
    DEV_APP_ADR,
    SYS_CONFIG_ADR,
    BACK_SYS_CONFIG_ADR,
    SYS_SET_ADR,
    SENSEOR_CALIBRATION_ADR,
    SHEEP_FOLD_P_ADR,
    FORBIDDEN_ZONE_P_ADR,
} FLASH_PARTITION;
enum {
    LOCK_HEART_SW = 0,
    UNLOCK_HEART_SW,
    INTERNAL_BAT_HEART_SW,
    LOCK_HEART2_SW,
    UNLOCK_HEART2_SW,
    BLE_CONNECT_PUSH_HEART_SW,
    BLE_DISCON_PUSH_HERAT_SW,
};

struct sys_param_set_stu {
    uint32_t magic;
    uint8_t alive_flag;     //激活标志
    uint8_t net_heart_sw;  //bit0:关锁定时上报SW，bit1:开锁定时上报SW, bit2:内电池工作上报SW,bit3:关锁上报2SW， bit4:开锁上报2SW, bit5:ble连接SW bit6:ble无连接SW
    uint16_t unlock_car_heart_interval;   //开锁心跳间隔
    uint16_t net_heart_interval;        
    uint16_t lock_car_heart_interval;   //关锁心跳间隔
    uint32_t internal_battry_work_interval; //内电池工作心跳间隔
    uint16_t unlock_car_heart2_interval;
    uint16_t lock_car_heart2_interval;
    uint16_t ble_connect_operate_push_interval;
    uint16_t ble_disconnect_operate_push_interval;
    uint8_t auto_power_off_time;
    uint32_t ota_cnt;
    uint8_t bms_charge_soc;
    uint8_t bms_charge_current;
    uint32_t net_engwe_state_push_cmdId;
    uint32_t net_engwe_report_time1_cmdId;
    uint32_t net_engwe_report_time2_cmdId;
    uint32_t net_engwe_offline_opearte_push_cmdId;
    uint16_t shock_sensitivity;
    uint32_t crc32;
};

struct sys_config_stu {
    uint32_t magic;
    char ip[128];
    uint32_t port; 
    char dev_type[6];  
    char apn[32];
    char DSN[32];
    char apn_usr[32];
    char apn_passwd[32];
    char mqtt_client_user[32];
    char mqtt_client_pass[32];
    char mqtt_client_id[32];
    char mqtt_pub_topic[64];
    char mqtt_sub_topic[64];
    char mqtt_qos;
    uint16_t mqtt_keep_alive;
    char mqtt_will_msg[64];
    char mqtt_will_topic[64];
    char sn[15];
    char manufacturer[16];
    uint32_t crc32;
};

typedef enum {
    CIRCLE = 1,
    POLYGON,
} SHAPE_E;

typedef struct {
    double lon;     //纬度  负代表南纬
    double lat;     //经度  负代表西经
}Point;

struct circle_p_stu {
    Point center;
    double radius; //km
};

struct polygon_p_stu {
    uint8_t point_num;
    Point p[20];
};

typedef struct {
    uint32_t magic;
    SHAPE_E shape_type;
    union 
    {
        struct circle_p_stu circle;
        struct polygon_p_stu polygon;
    };
    uint32_t crc32;
}SHAPE_SET;

struct sensor_calibration_data_stu {
    uint32_t magic;
    float static_offset_acc[3];
    float static_offset_gyro[3];
    uint32_t crc32;
};

enum {
    IOT_TRANS_MODE = 0,
    IOT_WAIT_ACTIVE_MODE,
    IOT_ACTIVE_MODE,
    IOT_LOW_POWER_MODE,
};

enum {
    SYS_SET_SAVE = 0,
    SYS_CONFIG_SAVE,
    SHEEP_DATA_SAVE,
    FORBIDDEN_DATA_SAVE,
};


#pragma pack(1)
struct sys_info_stu {
    uint8_t car_init;
    uint16_t battry_val;
    uint16_t bat_val;
    uint8_t bat_soc;
    uint8_t bat_soh;
    uint8_t fault;
    uint8_t audio_init : 1;
    uint8_t pdp_reg :1;
    uint8_t paltform_connect :1;
    uint8_t gps_state :1;
    uint8_t power_36v :1;
    uint8_t move_alarm :1;
    uint8_t ble_connect :1;
    uint8_t exits_bat:1;
    uint8_t iot_power_state:1;
    uint8_t algo_timer_run:1;
    uint8_t startup_way;  /*1表示蓝牙开机   2表示app按键开机*/
    uint8_t can_key;
    uint8_t can_protocol_major;
    uint8_t can_protocol_sub;
    uint8_t ota_flag;
    uint8_t static_cali_flag;
    uint16_t mcu_soft_ver;
	uint16_t mcu_hw_ver;
    uint8_t iot_mode;
    uint8_t sheepfang_sta;
    uint8_t fence_sta;
    uint8_t ble_log_sw;
    uint8_t ble_can_trans_sw;
    unsigned long long car_error;
    unsigned iot_error;
};

struct sys_set_var_stu{
    uint8_t sys_poweroff_flag;
    uint8_t sys_updata_falg; //bit0表示sys_param, bit1表示sys_info, bit2表示sheepdata, bit3表示forbiddendata
    uint8_t sys_reboot_flag; 
    uint8_t car_power_en;   //0，无效 1， EN下电  2， EN上电
    uint8_t ble_bind_infoClean; //0， 无效  1，删除
    uint8_t iot_active;   //0,无效 1，取消激活 2，激活
    uint8_t hid_lock_sw;    //0 关 1：开  
    uint8_t hid_lock_sw_type;  //无感解锁开关类型， 0x00, 进入范围内解锁，离开关锁。0x01:iot检测手机接近时解锁  
    uint8_t shock_sw;   //0，关 1：开
};

#pragma pack()

#define SOFTVER "1.1"
#define HWVER   "1.0"
#define DEFAULT_MANUFACTURER  "EG" 
#define DEFAULT_DNS "114.114.114.114"
#define DEFAULT_SN      "123456789" 
#define DEFAULT_DEV_TYPE    "K10" 
#define DEFAULT_APN      "linksnet"
/*"asia.bics"*/
#define DEFAULT_IP     "mqtt://broker.emqx.io"  
/*"iot.engweapp.cn"*/
#define DEFAULT_PORT     1883  
/*9682*/
#define DEFAULT_MQTT_SUB_PRE  "iot/instruction/"
#define DEFAULT_MATT_PUB_PRE  "iot/pub/instruction/"


#define BLE_NAME    "ENGWE"
#define BLE_SUUID   0X1820

#define OTA_FILE    "UFS:ble_simple_peripheral.bin"

extern def_rtos_sem_t NotAlivePdpBlockSem_t;
extern def_rtos_sem_t NotAliveSocketSem_t;
extern def_rtos_sem_t NotAliveSensorSem_t;
extern SHAPE_SET sheepfang_data;
extern SHAPE_SET forbidden_zone_data;
extern struct sys_set_var_stu sys_set_var;
extern struct sys_param_set_stu sys_param_set;
extern struct sys_info_stu sys_info;
extern struct sys_config_stu sys_config;
extern struct sensor_calibration_data_stu sensor_calibration_data;
void app_sys_init();
void system_timer_start();
void system_timer_stop();
void sys_param_save(FLASH_PARTITION flash_part);
int64_t systm_tick_diff(int64_t time);
void debug_data_printf(char *str_tag, const uint8_t *in_data, uint16_t data_len);
void app_system_thread(void *param);
int flash_partition_erase(FLASH_PARTITION flash_part);
int flash_partition_write(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift);
int flash_partition_read(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift);
int flash_partition_size(FLASH_PARTITION flash_part);
void sys_power_off_time_set(uint8_t time);
void sys_log_out(const char *fmt, ...);
void app_system_log_out_thread(void *param);
#endif