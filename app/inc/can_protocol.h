#ifndef   __CAN_PROTOCOL_H
#define   __CAN_PROTOCOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#pragma pack(1)
typedef struct {
    union 
    {
        uint16_t pdu2;
        struct {
            uint8_t da;
            uint8_t pdu1;
        };
    }; 
}PDU_STU;
typedef struct {
    union 
    {
        uint32_t can_id;
        struct {
            uint8_t src;
            PDU_STU pdu;
            uint8_t dp  :1;
            uint8_t r   :1;
            uint8_t p   :3;
            uint8_t res :3;
        };
    };   
} CAN_PDU_STU;


#pragma pack()

typedef struct  {
    stc_can_rxframe_t  can_tx_frame;
    uint8_t cnt;
    uint8_t ask_flag;
    uint8_t cmd_send;
} CAN_SEND_CMD_STU;

typedef enum {
   DFT_DEV_ID = 0X22,
   HMI_ADR =  0X28,
   CONTROL_ADR = 0XEF,
   BMS_ADR = 0XF4,
   SECOND_BMS_ADR = 0XF7,
   IOT_ADR = 0X21,
   CHARGER_ADR = 0X56,
   LOCK_ADR = 0X60,
   TRANS_ADR = 0X63
} DEV_ID;

/*IOT指令*/
enum {
    IOT_MATCH = 0XFB00,
    IOT_CONTROL_DATA1 = 0XFB01,
    IOT_STATE = 0XFB03,
    IOT_SN1 = 0XFB1A,
    IOT_SN2 = 0XFB1B,
    IOT_SOFT_VER1 = 0XFB10,
    IOT_SOFT_VER2 = 0XFB11,
    IOT_HW_VER1 = 0XFB20,
    IOT_HW_VER2 = 0XFB21,
    IOT_SAVE_DATA = 0XFB02,
    IOT_STATE_DATA = 0XFB30,
    IOT_NAVIGATION_DATA = 0XFB31,
    IOT_SNTP = 0XFB32,
};

/*仪表指令*/
enum{
    HMI_MATCH_INFO  =0XFD01,
    HMI_DATA = 0XFD02,
    HMI_STA = 0XFD03,
    HMI_HW_VER1 = 0XFD10,
    HMI_HW_VER2 = 0XFD11,
    HMI_SOFT_VER1 = 0XFD15,
    HMI_SOFT_VER2 = 0XFD16,
    HMI_SN1 = 0XFD1A,
    HMI_SN2 = 0XFD1B,
    HMI_SN3 = 0XFD1C,
    HMI_SN4 = 0XFD1D,
    HMI_ATMOSPHERE_LIGHT_STA = 0XFD20,
};
/*BMS指令*/
enum{
    BMS_MATCH_INFO = 0XFF25,
    BMS_BATTRY_PACK_TEMP = 0XFF01,
    BMS_COMPREHENSIVE_DATA = 0XFF02,
    BMS_CELL_VOL1 = 0XFF03,
    BMS_CELL_VOL2 = 0XFF04,
    BMS_CELL_VOL3 = 0XFF05,
    BMS_CELL_VOL4 = 0XFF06,
    BMS_REALTIME_VOL_CURRENT = 0XFF0A,
    BMS_FIRST_SENCOND_PROTECTION = 0XFF0B,
    BMS_BATTRY_PACK_STA = 0XFF0C,
    BMS_BATTRY_PACK_RECORDDATA = 0XFF10,
    BMS_BATTRY_PACK_CHARGE_PARAM = 0XFF11,
    BMS_PROTECTION_FAULT_INFO = 0XFF0D,
    BMS_BARCODE_A   = 0XFF15,
    BMS_BARCODE_B   = 0XFF16,
    BMS_BARCODE_C   = 0XFF17,
    BMS_BARCODE_D   = 0XFF18,
    BMS_SOFT_VER = 0XFF20,
    BMS_SOFT_VER_EXTEND1 = 0XFF1D,
    BMS_SOFT_VER_EXTEND2 = 0XFF1E,
    BMS_SOFT_VER_EXTEND3 = 0XFF1F,
    BMS_HW_VER_A = 0XFF21,
    BMS_HW_VER_B = 0XFF22,
    BMS_HEALTH_INFO1 = 0XFF34,
    BMS_HEALTH_INFO2 = 0XFF35,
    BMS_HEALTH_INFO3 = 0XFF36,
    BMS_HEALTH_INFO4 = 0XFF37,
    BMS_MANUFACTURE_DATE = 0XFF38,
};

/*控制器指令*/
enum{
    CONTROL_MATCH_INFO = 0XFE01,
    CONTROL_DATA1 = 0XFE02,
    CONTROL_DATA2 = 0XFE03,
    CONTROL_DATA3 = 0XFE04,
    CONTROL_DATA4 = 0XFE05,
    CONTROL_DATA5 = 0XFE07,
    CONTROL_DATA6 = 0XFE09,
    CONTROL_DATA7 = 0XFE0A,
    CONTROL_DATA8 = 0XFE0B,
    CONTROL_DATA9 = 0xFE0C,
    CONTROL_HWVER1 = 0XFE10,
    CONTROL_HWVER2 = 0XFE11,
    CONTROL_SOFTVER1 = 0XFE15,
    CONTROL_SOFTVER2 = 0XFE16,
    CONTROL_SN1 = 0XFE1A,
    CONTROL_SN2 = 0XFE1B,
    CONTROL_SN3 = 0XFE1C,
    CONTROL_SN4 = 0XFE1D,
    CONTROL_PARAMVER1 = 0XFE20,
    CONTROL_PARAMVER2 = 0XFE21,
    CONTROL_PARAMVER3 = 0XFE22,
    CONTROL_CUSTOMERCODE1 = 0XFE25,
    CONTROL_CUSTOMERCODE2 = 0XFE26,
};
/*电子锁指令*/
enum{
    ELECTRONIC_LOCK_MATCH_INFO = 0XFC00,
    ELECTRONIC_LOCK_STATUE = 0XFC01,
    ELECTRONIC_LOCK_HEART = 0XFC10,
    ELECTRONIC_LOCK_HW_VER = 0XFC20,
    ELECTRONIC_LOCK_SOFT_VER = 0XFC21,
    ELECTRONIC_LOCK_TYPE = 0XFC22,
    ELECTRONIC_LOCK_FIRM = 0XFC23,
};
/*充电器指令*/
enum{
    CHARGER_STATE_INFO = 0XFA02,
};

/*变速器指令*/
enum{
    TRANS_GENERAL_STA = 0XF501,
};

enum {
    HMI_CMD_SET_GEAR = 0,
    HMI_CMD_SET_HEADLIGHT,   /*没有回复NG*/
    HMI_CMD_SET_HEADLIGHT_SENMODE,  /*没有回复NG*/
    HMI_CMD_SET_TURNLIGHT,
    HMI_CMD_SET_CYCLE_MODE,
    HMI_CMD_JUMP_PASSWORD,
    HMI_CMD_ENTER_SLEEP,  /*进入休眠功能OK*/
    HMI_CMD_ENTER_WEEK,/*没有回复NG*/
    HMI_CMD_LOOK_CAR,
    CMD_SET_ELECLOCK,
    CMD_SPEED_LIMIT,
    CMD_MILEAGE_UNIT,
    CMD_SET_POWER_ON_PASSWORD,
    CMD_SET_ATMOSPHERE_LIGHT_MODE,
    CMD_SET_ATMOSPHERE_LIGHT_COLOUR,
    CMD_SET_ATMOSPHERE_LIGHT_BRIGHTNESS,
    CMD_SET_ATMOSPHERE_LIGHT_TURN,
    CMD_SET_ATMOSPHERE_LIGHT_R_VAL,
    CMD_SET_ATMOSPHERE_LIGHT_G_VAL,
    CMD_SET_ATMOSPHERE_LIGHT_B_VAL,
    CMD_EN_POWER_ON_PASSWORD,
    CMD_BMS_SET_CHARGE_MODE,
    CMD_BMS_SET_CHARGE_SOC,
    CMD_BMS_DISCHARGE_SW,
    CMD_DOUBLE_BMS_WORK_MODE,
    CMD_CHARGE_POWER,
    CMD_PASS_ON_QUERY,
    CMD_MAX,
};

enum{
    IOT_CONTROL_SET_GEAR = 0,
    IOT_CONTROL_SET_HEADLIGHT,
    IOT_CONTROL_SET_TAILLIHHT,
    IOT_CONTROL_SET_LEFT_TURN_LIGHT,
    IOT_CONTROL_SET_RIGHT_TURN_LIGHT,
    IOT_CONTROL_ANTI_THEFT,
};

enum {
    CAN_NET_TRANS,
    CAN_BLUE_TRANS,
};
struct trans_can_control_stu {
    uint8_t src;
    uint32_t send_can_id;
    uint8_t send_data[8];
    uint8_t send_flag;
    uint32_t rq_can_id;
    uint8_t rq_data[8];
};
struct can_ota_con_stu{
    uint8_t dev_id;
    uint8_t last_ota_step;
    uint8_t ota_step;
    uint16_t crc16;
    uint16_t pack_len;
    uint32_t read_len;
    uint32_t totalen;
    uint32_t total_pack;
    uint32_t pack_cout;
    uint8_t frame_cout;
    uint8_t pack_frame;
    uint8_t buf[4096];
    uint8_t err_cent;
    uint8_t retry;
};

struct can_ota_data_uart_stu 
{
    uint8_t dev_id;
    uint8_t data[512];
    uint16_t data_len;
    uint8_t pack_num;
    uint16_t data_offset;
    def_rtos_sem_t data_sem;
    def_rtos_sem_t data_finish_sem;
};
extern CAN_SEND_CMD_STU can_send_cmd;
extern struct can_ota_data_uart_stu can_ota_data_uart;
extern struct trans_can_control_stu  trans_can_control;
void iot_can_heart_fun();
void can_protocol_rx_thread(void *param);
void can_protocol_init();
void can_protocol_tx_thread(void *param);
void can_png_quest(uint8_t dst, uint16_t png, uint8_t direct);
void iot_can_cmd_control(uint8_t cmd, uint8_t *cmdvar, uint8_t direct);
void iot_can_png_control(uint8_t cmd, uint8_t direct);
void iot_can_trans_func(uint32_t can_id, uint8_t *data, uint8_t direct);
int can_ota_task(DEV_ID dev_id);
void iot_can_state2_fun();
void iot_can_navigation_data();
void iot_quit_navigation();
void iot_en_power_on_passwd();
void mache_dft_adv_task();
void dft_item_test();
void mache_dft_clcy_get_info();
void car_info_debug();
void car_get_config_info(uint8_t cmd, uint8_t direct);
void car_ver_query();



#endif