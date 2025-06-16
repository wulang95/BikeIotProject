/*蓝牙协议*/
#include "app_system.h"
#include <time.h>
#include "hal_drv_rtc.h"
#include "ql_fs.h"
#define DBG_TAG         "ble_protocol"

#ifdef BLE_PROTOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

#define HEAD_H      0X45
#define HEAD_L      0X6E

#define TAIL_H      0X77
#define TAIL_L      0X65

enum {
    IOT_FW = 0X01,
    AGPS_DATA = 0X02,
    UI_TYPE = 0X03,
    FONT_LIB = 0X04,
    OTHER_UPGRADE = 0X05,
    HMI_FW = 0X06,
    CON_FW = 0X07,
    BMS_FW = 0X08,
    LOCK_FW = 0X09,
};

enum {
    OTA_IDEL = 0X00,
    OTA_START = 0X01,
    OTA_DATA,
    OTA_FINISH, 
};


enum {
    BLE_CMD_LARGE_QUERY = 0X2411,
    BLE_CMD_Q_BASIC_SERVICES = 0XFF01,
    BLE_CMD_Q_BAT_INFO = 0XFF02,
    BLE_CMD_Q_CYCLE_CONFIG = 0XFF03,
    BLE_CMD_Q_GENERAL_CONFIG = 0XFF04,
    BLE_CMD_Q_CAR_SERVICE = 0XFF05,
    BLE_CMD_Q_SECOND_BAT_INFO = 0XFF06,
    BLE_CMD_Q_IOT_SERVICE = 0XFF07,
    BLE_CMD_Q_NET_SERVICE = 0XFF08,
    BLE_CMD_Q_IOT_VER = 0XFF09,
    BLE_CMD_Q_CAR_HWVER = 0XFF0A,
    BLE_CMD_S_HEADLIGHT_STA = 0X597,
    BLE_CMD_S_ASSIST_GEAR = 0X1009,
    BLE_CMD_S_SPEED_LIMIT = 0X2A65,
    BLE_CMD_S_CURRENT_LIMIT = 0X2A66,
    BLE_CMD_S_WHEEL_DIAMETER = 0X08C3,
    BLE_CMD_S_TURN_SIGNAL  = 0X0596,
    BLE_CMD_S_TMIE_SYNC = 0X1805,
    BLE_CMD_S_UNITS   = 0X2B46,
    BLE_CMD_S_BRIGHTNESS_LEVEL = 0X07C0,
    BLE_CMD_S_TIME_AUTODOWN = 0X1000,
    BLE_CMD_S_JUMP_PASSWORD   = 0XBEC9,
    BLE_CMD_S_POWERPASSWORD_SW = 0XBECE,
    BLE_CMD_S_POWER_SW   = 0X0107,
    BLE_CMD_S_LOCK_SW  = 0X0108,
    BLE_CMD_LOCK_ASK = 0X0109,
    BLE_CMD_Q_APN  = 0X010A,
    BLE_CMD_S_APN = 0X010B,
    BLE_CMD_Q_HIDKEY_STA = 0X010C,
    BLE_CMD_S_HIDKEY_SW = 0X010D,
    
    BLE_CMD_Q_MOVE_SW = 0X010E,
    BLE_CMD_MOVE_SW = 0X010F,

    BLE_CMD_LOOK_CAR = 0X0110,
    BLE_CMD_Q_CHARGE_POWER = 0X0111,
    BLE_CMD_S_CHARGE_POWER = 0X0112,
    BLE_CMD_CLEAN_BIND_INFO = 0X0119,
    BLE_CMD_Q_BMS_HEALTH_INFO = 0X011C,
    BLE_CMD_U_BMS_HEALTH_INFO = 0X011D,
    BLE_CMD_S_QUIT_NAVIGATION_TIME = 0X0114,
    BLE_CMD_S_RESTORE_FACTORY_CONFIG = 0X8049,
    BLE_CMD_S_CLEAN_ODO_DATA = 0XBEC7,
    BLE_CMD_S_POWER_PASSWORD = 0XBEC8,

    BLE_CMD_U_RIDEDATA_SERVICE = 0X2A63,
    BLE_CMD_NAVIGATION_SERVICE = 0X2A67,

    BLE_CMD_OTA_REQ = 0X0501,
    BLE_CMD_OTA_REQ_ASK = 0X0502,
    BLE_CMD_OTA_SEND_DATA = 0X0503,
    BLE_CMD_OTA_DATA_ASK = 0X0504,
    BLE_CMD_OTA_Q_VER = 0X0505,
    BLE_CMD_OTA_VER_ASK = 0X0506,
    BLE_CMD_CONFIGFILE_SENDREQ = 0X0601,
    BLE_CMD_CONFIGFILE_SENDREQ_ASK = 0X0602,
    BLE_CMD_CONFIGFILE_SENDDATA = 0X0603,
    BLE_CMD_CONFIGFILE_SENDDATA_ASK = 0X0604,

    BLE_CMD_TRANS_REQ = 0X0701,
    BLE_CMD_TRANS_ASK = 0X0702,

    BLE_CMD_IOT_ACTIVE_REQ = 0X0703,
    BLE_CMD_IOT_ACTIVE_REQ_ASK = 0X0704,

    BLE_CMD_Q_ATSPHLIGHT_STA = 0X0801,
    BLE_CMD_ATSPHLIGHT_STA_ASK = 0X0802,
    BLE_CMD_S_ATSPHLIGHT_MODE = 0X0803,
    BLE_CMD_ATSPHLIGHT_MODE_ASK = 0X0804,
    BLE_CMD_ATSPHLIGHT_COLOR_CUSTOM = 0X0805,
    BLE_CMD_ATSPHLIGHT_COLOR_CUSTOM_ASK = 0X0806,
    BLE_CMD_S_ATSPHLIGHT_BRIGHTVAL = 0X0807,
    BLE_CMD_S_ATSPHLIGHT_BRIGHTVAL_Q = 0X0808,
    BLE_CMD_S_ATSPHLIGHT_TURN = 0X0809,
    BLE_CMD_S_ATSPHLIGHT_TURN_ASK = 0X080A,
    BLE_CMD_S_ATSPHLIGHT_COLORTYPE = 0X080B,
    BLE_CMD_S_ATSPHLIGHT_COLORTYPE_ASK = 0X080C,
    BLE_CMD_ATSPHLIGHT_SW = 0X080D,
    BLE_CMD_ATSPHLIGHT_SW_ASK = 0X080E,
    BLE_CMD_Q_ATSPHLIGHT_TIMTASK = 0X080F,
    BLE_CMD_Q_ATSPHLIGHT_TIMTASK_ASK = 0X0810,
    BLE_CMD_S_ATSPHLIGHT_TIMTASK = 0X0811,
    BLE_CMD_S_ATSPHLIGHT_TIMTASK_ASK = 0X0812,

    BLE_CMD_D_Q_AUDIO_PLAY = 0X0901,
    BLE_CMD_U_Q_AUDIO_PLAY = 0X0902,

    BLE_CMD_TRANSPORT_MODE_QES = 0X0A01,
    BLE_CMD_TRANSPORT_MODE_QES_ASK = 0X0A02,
    BLE_CMD_U_LOG = 0X0B01,
    BLE_CMD_LOG_SW = 0X0B02,
    BLE_CMD_LOG_SW_ASK = 0X0B03,

    BLE_CMD_CAN_TRANS_SET = 0X0037,
    BLE_CMD_TRANS_CAN = 0X00FD,
    BLE_CMD_TRANS_CAN_UP = 0X007D,

};

void ble_protocol_data_pack(uint16_t cmd, uint8_t *data, uint16_t data_len, uint8_t *buf, uint16_t *len)
{
    uint8_t *p;
    uint16_t lenth = 0;
    uint16_t check_crc;
    p = malloc(256);
    p[lenth++] = HEAD_H;
    p[lenth++] = HEAD_L;
    p[lenth++] = cmd >> 8;
    p[lenth++] = cmd&0xff;
    p[lenth++] = data_len >> 8;
    p[lenth++] = data_len & 0xff;

    memcpy(&p[lenth], data, data_len);
    lenth += data_len;

    check_crc = drv_modbus_crc16(&p[2], lenth - 2);
    p[lenth++] = check_crc&0xff;
    p[lenth++] = check_crc>>8;
    p[lenth++] = TAIL_H;
    p[lenth++] = TAIL_L;
    ble_cmd_pack(CMD_BLE_TRANS, p, lenth, buf, len);
    free(p);
}



void ble_protocol_send_thread(void *param)
{
    while(1){
        def_rtos_task_sleep_ms(200);
    }
    def_rtos_task_delete(NULL);
}


uint8_t ble_protocol_check(uint8_t *buf, uint16_t len)
{
    uint16_t lenth;
    uint16_t check_crc,rcv_crc;
    if(len < 10) {
        LOG_E("lenth is error");
        return FAIL;
    } 
    lenth = buf[4] << 8 | buf[5];
    if(lenth + 10 != len){
        LOG_E("data lenth is error");
        return FAIL;
    }

    if(buf[0] != HEAD_H || buf[1] != HEAD_L) {
        LOG_E("header is error");
        return FAIL;
    }
    if(buf[len -2] != TAIL_H || buf[len -1] != TAIL_L){
        LOG_E("tail is error");
        return FAIL;
    }

    rcv_crc = buf[lenth + 7] <<8 | buf[lenth + 6];
    check_crc = drv_modbus_crc16(&buf[2], lenth + 4);
    if(rcv_crc != check_crc) {
        LOG_E("crc16 is fail, rcv_crc:%04x, check_crc:%04x", rcv_crc, check_crc);
        return FAIL;
    }
    return OK;
}

static void ble_apn_query_send()
{   
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    memcpy(&data[data_len], sys_config.apn, strlen(sys_config.apn));
    data_len += strlen(sys_config.apn);
    LOG_I("%s", sys_config.apn);
    ble_protocol_data_pack(BLE_CMD_Q_APN, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


static void basic_services_send()
{
    uint8_t data[256], buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    memcpy(&data[data_len], &sys_config.manufacturer[0], 16);
    data_len += 16;
    memcpy(&data[data_len], &sys_config.dev_type[0], 6);
    data_len += 6;
    memcpy(&data[data_len], &ble_info.ver[0], 6);
    data_len += 6;
    memcpy(&data[data_len], HWVER, 6);
    data_len += 6;
    memcpy(&data[data_len], &sys_config.sn[0], 16);
    data_len += 16;
    ble_protocol_data_pack(BLE_CMD_Q_BASIC_SERVICES, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
} 

static void basic_bat_info_send()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = (car_info.bms_info[0].pack_vol >> 8)&0xff;
    data[data_len++] = (car_info.bms_info[0].pack_vol) & 0xff;

    data[data_len++] = (car_info.bms_info[0].pack_current >> 8)&0xff;
    data[data_len++] = car_info.bms_info[1].pack_current&0xff;
    data_len += 3;  /*温度*/
    if(car_info.bms_info[0].charge_sta) {
        data[data_len++] = 0x01;
    } else if(car_info.bms_info[0].chargefull_sta) {
        data[data_len++] = 0xff;
    } else {
        data[data_len++] = 0x00;
    }
    data[data_len++] = car_info.bms_info[0].soh;

    data[data_len++] = ((car_info.bms_info[0].charge_interval_time*10) >> 8)&0xff;
    data[data_len++] = car_info.bms_info[0].charge_interval_time&0xff;

    data[data_len++] = (car_info.bms_info[0].cycle_number >> 8)&0xff;
    data[data_len++] = car_info.bms_info[0].cycle_number&0xff;

    data[data_len++] = strlen(car_info.bms_info[0].soft_ver);
    memcpy(&data[data_len], car_info.bms_info[0].soft_ver, strlen(car_info.bms_info[0].soft_ver));
    data_len += strlen(car_info.bms_info[0].soft_ver);


    data[data_len++] = strlen(car_info.bms_info[0].hw_ver);
    memcpy(&data[data_len], car_info.bms_info[0].hw_ver, strlen(car_info.bms_info[0].hw_ver));
    data_len += strlen(car_info.bms_info[0].hw_ver);

    ble_protocol_data_pack(BLE_CMD_Q_BAT_INFO, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void basic_cycle_config_send()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = car_info.headlight_sta?0xFF:0x00;
    data[data_len++] = car_info.gear;
    data[data_len++] = car_info.speed_limit/10;
    data[data_len++] = car_info.current_limit/10;
    switch (car_info.wheel)
        {
        case 6:
            data[data_len++] = 0;
            break;
        case 8:
            data[data_len++] = 1;
            break;
        case 10:
            data[data_len++] = 2;
            break;
        case 12:
            data[data_len++] = 3;
            break;
        case 14:
            data[data_len++] = 4;
            break;
        case 16:
            data[data_len++] = 5;
            break;
        case 20:
            data[data_len++] = 7;
            break;
        case 22:
            data[data_len++] = 8;
            break;
        case 24:
            data[data_len++] = 9;
            break;
        case 26:
            data[data_len++] = 10;
            break;
        case 28:
            data[data_len++] = 12;
            break;
        case 29:
            data[data_len++] = 13;
            break;
        default:
            break;
        }
    if(car_info.hmi_info.left_turn_light && car_info.hmi_info.right_turn_linght){
        data[data_len++] = 0x03;
    } else if(car_info.hmi_info.left_turn_light && !car_info.hmi_info.right_turn_linght){
        data[data_len++] = 0x01;
    } else if(car_info.hmi_info.left_turn_light && !car_info.hmi_info.right_turn_linght){
        data[data_len++] = 0x02;
    } else {
        data[data_len++] = 0x00;
    }
    ble_protocol_data_pack(BLE_CMD_Q_CYCLE_CONFIG, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void basic_general_config_service_send()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    struct tm *gmt;
    time_t time;
    time = (time_t)hal_drv_rtc_get_timestamp();
    gmt = localtime(&time);

    data[0] = gmt->tm_year - 2000;
    data[1] = gmt->tm_mon;
    data[2] = gmt->tm_mday;
    data[3] = gmt->tm_hour;
    data[4] = gmt->tm_min;
    data[5] = gmt->tm_sec;

    data[6] = car_info.hmi_info.display_unit?0x01:0x02;
    data[7] = car_info.bright_lev;
    data[8] = car_info.autoPoweroffTime;
    data[9] = car_info.jump_password;
    data[10] = car_info.use_password;
    ble_protocol_data_pack(BLE_CMD_Q_GENERAL_CONFIG, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void basic_whole_car_service_send()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = strlen(car_info.control_soft_ver);
    memcpy(&data[data_len], car_info.control_soft_ver, strlen(car_info.control_soft_ver));
    data_len += strlen(car_info.control_soft_ver);

    data[data_len++] = strlen(car_info.control_soft_ver);
    memcpy(&data[data_len], car_info.control_soft_ver, strlen(car_info.control_soft_ver));
    data_len += strlen(car_info.control_soft_ver);

    ble_protocol_data_pack(BLE_CMD_Q_CAR_SERVICE, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);                                          
}

static void car_iot_service_send()
{                          
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = strlen(gsm_info.imei);
    memcpy(&data[data_len], gsm_info.imei, strlen(gsm_info.imei));
    data_len+=strlen(gsm_info.imei);

    data[data_len++] = strlen(gsm_info.iccid);
    memcpy(&data[data_len], gsm_info.iccid, strlen(gsm_info.iccid));
    data_len+=strlen(gsm_info.iccid);
    data[data_len++] = sys_info.bat_soc;
    ble_protocol_data_pack(BLE_CMD_Q_IOT_SERVICE, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void car_net_service_state_send()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = sys_info.paltform_connect ? 0x01:0x02;
    data[data_len++] = gsm_info.csq;
    data[data_len++] = Gps.SateNum;

    ble_protocol_data_pack(BLE_CMD_Q_NET_SERVICE, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_head_light_sta(uint8_t set, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query){
        data[data_len++] = car_info.headlight_sta?0xff:0x00;
    } else{
        if(car_info.lock_sta == CAR_UNLOCK_STA) {
            if(set == 0x00){
                car_set_save.head_light = 0;
            } else {
                car_set_save.head_light = 1;
            }
            car_control_cmd(CAR_CMD_SET_HEADLIGHT);
            data[data_len++] = set;
        } else {
            if(car_info.headlight_sta)
                data[data_len++] = 0xff;
            else data[data_len++] = 0x00;
        }
    }
    ble_protocol_data_pack(BLE_CMD_S_HEADLIGHT_STA, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_assist_gear_service(uint8_t set, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query) {
        data[data_len++] = car_info.gear;
    } else {
        if(car_info.lock_sta == CAR_UNLOCK_STA) {
            if(set <= 5){
                car_set_save.gear = set;
                car_control_cmd(CAR_CMD_SET_GEAR);
                data[data_len++] = set;
            } else {
                data[data_len++] = car_info.gear;
            }
        } else {
            data[data_len++] = car_info.gear;
        }
    }
    ble_protocol_data_pack(BLE_CMD_S_ASSIST_GEAR, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void query_iot_soft_ver()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    memcpy(data, SOFTVER, strlen(SOFTVER));
    data_len += strlen(SOFTVER);
    ble_protocol_data_pack(BLE_CMD_Q_IOT_VER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}
static void query_car_hwver()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    memcpy(data, SOFTVER, strlen(SOFTVER));
    data_len += strlen(SOFTVER);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], __DATE__, strlen(__DATE__));
    data_len += strlen(__DATE__);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], car_info.control_soft_ver, strlen(car_info.control_soft_ver));
    data_len += strlen(car_info.control_soft_ver);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], car_info.hmi_info.soft_ver, strlen(car_info.hmi_info.soft_ver));
    data_len += strlen(car_info.hmi_info.soft_ver);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], car_info.bms_info[0].soft_ver, strlen(car_info.bms_info[0].soft_ver));
    data_len += strlen(car_info.bms_info[0].soft_ver);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], car_info.electronic_lock.soft_ver, strlen(car_info.electronic_lock.soft_ver));
    data_len += strlen(car_info.electronic_lock.soft_ver);
    strcpy((char *)&data[data_len], ",");
    data_len++;
    memcpy(&data[data_len], ble_info.ver, strlen(ble_info.ver));
    data_len += strlen(ble_info.ver);
    ble_protocol_data_pack(BLE_CMD_Q_CAR_HWVER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_speed_limit(uint8_t speed, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query) {
        data[data_len++] = car_info.speed_limit/10;
    } else {
        if(car_info.lock_sta == CAR_UNLOCK_STA) {
            car_set_save.speed_limit = speed * 10;
            car_control_cmd(CAR_CMD_SET_SPEED_LIMIT);
            data[data_len++] = speed;
        } else {
            data[data_len++] = car_info.speed_limit/10;
        }
    }
    ble_protocol_data_pack(BLE_CMD_S_SPEED_LIMIT, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_wheel(uint8_t wheel, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query){
        switch (car_info.wheel)
        {
        case 6:
            data[data_len++] = 0;
            break;
        case 8:
            data[data_len++] = 1;
            break;
        case 10:
            data[data_len++] = 2;
            break;
        case 12:
            data[data_len++] = 3;
            break;
        case 14:
            data[data_len++] = 4;
            break;
        case 16:
            data[data_len++] = 5;
            break;
        case 20:
            data[data_len++] = 7;
            break;
        case 22:
            data[data_len++] = 8;
            break;
        case 24:
            data[data_len++] = 9;
            break;
        case 26:
            data[data_len++] = 10;
            break;
        case 28:
            data[data_len++] = 12;
            break;
        case 29:
            data[data_len++] = 13;
            break;
        default:
            data[data_len++] = 7;
            break;
        }
    LOG_I("%d", data[data_len]);
    } else {
        ;
    }
    ble_protocol_data_pack(BLE_CMD_S_WHEEL_DIAMETER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_time_sync(uint8_t *dat, uint8_t query)
{
    struct tm *gmt = NULL;
    struct tm gm;
    time_t timestamp;
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query){
        timestamp = (time_t)hal_drv_rtc_get_timestamp();
        gmt = localtime(&timestamp);

        data[data_len++] = gmt->tm_year - 100;
        data[data_len++] = gmt->tm_mon + 1;
        data[data_len++] = gmt->tm_mday;
        data[data_len++] = gmt->tm_hour + 8;
        data[data_len++] = gmt->tm_min;
        data[data_len++] = gmt->tm_sec;
        hal_drv_rtc_time_print();
    } else {
        gm.tm_year = dat[0]+100;
        gm.tm_mon = dat[1]-1;
        gm.tm_mday = dat[2];
        gm.tm_hour = dat[3]-8;
        gm.tm_min = dat[4];
        gm.tm_sec = dat[5];
        timestamp = mktime(&gm);
        hal_drv_rtc_set_time(timestamp);
        hal_drv_rtc_time_print();
        memcpy(&data[data_len], &dat[0], 6);
        data_len += 6;
    }
    ble_protocol_data_pack(BLE_CMD_S_TMIE_SYNC, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_cmd_car_turn_light(uint8_t set, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query){
        if(car_info.right_turn_light_sta && car_info.left_turn_light_sta)
            data[data_len++] = 0x03;
        else if(car_info.right_turn_light_sta && !car_info.left_turn_light_sta) 
            data[data_len++] = 0x02;
        else if(!car_info.right_turn_light_sta && car_info.left_turn_light_sta)
            data[data_len++] = 0x01;
        else 
            data[data_len++] = 0x00;
    } else {
        if(car_info.lock_sta == CAR_UNLOCK_STA) {
            switch(set) {
            case 0x00:
                car_set_save.left_turn_light = 0;
                car_set_save.right_turn_light = 0;
            break;
            case 0x01:
                car_set_save.left_turn_light = 1;
                car_set_save.right_turn_light = 0;
            break;
            case 0x02:
                car_set_save.left_turn_light = 0;
                car_set_save.right_turn_light = 1;
            break;
            case 0x03:
                car_set_save.left_turn_light = 1;
                car_set_save.right_turn_light = 1;
            break;
            }
            car_control_cmd(CAR_CMD_SET_TURN_LIGHT);
            data[data_len++] = set;
        } else {
            if(car_info.right_turn_light_sta && car_info.left_turn_light_sta)
                data[data_len++] = 0x03;
            else if(car_info.right_turn_light_sta && !car_info.left_turn_light_sta) 
                data[data_len++] = 0x02;
            else if(!car_info.right_turn_light_sta && car_info.left_turn_light_sta)
                data[data_len++] = 0x01;
            else 
                data[data_len++] = 0x00;
        }    
    }
    ble_protocol_data_pack(BLE_CMD_S_TURN_SIGNAL, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_unit(uint8_t unit, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    if(query){
  //      data[data_len++] = car_info.hmi_info.display_unit ? 0x01: 0x02;
        data[data_len++] = car_set_save.mileage_unit ? 0x01: 0x02;
    } else {
        if(car_info.lock_sta == CAR_UNLOCK_STA){
            if(unit == 0x02) car_set_save.mileage_unit = 0;
            else car_set_save.mileage_unit = 1;
        
            data[data_len++] = unit;
            car_control_cmd(CAR_CMD_SET_MILEAGE_UNIT);
        } else {
           // data[data_len++] = car_info.hmi_info.display_unit ? 0x01: 0x02;
           data[data_len++] = car_set_save.mileage_unit ? 0x01: 0x02;
        }      
    }
    ble_protocol_data_pack(BLE_CMD_S_UNITS, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_brightness_level(uint8_t brightness_level, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    if(query) {
        if(car_info.bright_lev < 20)
            data[data_len++] = 0x01;
        else if(car_info.bright_lev < 40)
            data[data_len++] = 0x02;
        else if(car_info.bright_lev < 60)
            data[data_len++] = 0x03;
        else if(car_info.bright_lev < 80)
            data[data_len++] = 0x04;
        else data[data_len++] = 0x05;
    } else {
        if(car_info.lock_sta == CAR_UNLOCK_STA){
            if(brightness_level == 0x01) 
                car_set_save.bright_lev = 20;
            else if(brightness_level == 0x02)
                car_set_save.bright_lev = 40;
            else if(brightness_level == 0x03)
                car_set_save.bright_lev = 60;
            else if(brightness_level == 0x04)
                car_set_save.bright_lev = 80;
            else 
                car_set_save.bright_lev = 100;
            data[data_len++] = brightness_level;
        } else {
            if(car_info.bright_lev < 20)
                data[data_len++] = 0x01;
            else if(car_info.bright_lev < 40)
                data[data_len++] = 0x02;
            else if(car_info.bright_lev < 60)
                data[data_len++] = 0x03;
            else if(car_info.bright_lev < 80)
                data[data_len++] = 0x04;
            else data[data_len++] = 0x05;
        }
        
    }
    ble_protocol_data_pack(BLE_CMD_S_BRIGHTNESS_LEVEL, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


static void ble_cmd_lock_control(uint8_t dat)
{
    CAR_CMD_Q car_cmd;
    car_cmd.src = BLUE_CAR_CMD_SER;
    car_cmd.ble_car_control.ble_cmd = BLE_CMD_LOCK_ASK;
    if(dat == 0x01) {
        LOG_I("BLE CAR_UNLOCK_STA");
        car_cmd.cmd = CAR_CMD_UNLOCK;
    } else if(dat == 0x02) {
        car_cmd.cmd = CAR_CMD_LOCK;
    }
    CAR_CMD_MARK(car_cmd);
}

void ble_cmd_opearte_res_up(uint16_t cmd)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    switch(cmd){
        case BLE_CMD_LOCK_ASK:
            if(car_info.lock_sta == CAR_LOCK_STA) {
                data[data_len++] = 0x02;
            } else {
                data[data_len++] = 0x01;
            }
        break;
    }
    ble_protocol_data_pack(cmd, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}



 static void ble_cmd_set_apn_info(uint8_t *dat, uint8_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    LOG_I("apn:%s", (char *)dat);
    memset(sys_config.apn, 0, sizeof(sys_config.apn));
    memcpy(sys_config.apn, dat, lenth);
    memcpy(&data[data_len], dat, lenth);
    data_len += lenth;
    ble_protocol_data_pack(BLE_CMD_S_APN, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_set_power_on_password(uint8_t *dat, uint8_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    memset(car_set_save.power_on_psaaword, 0, sizeof(car_set_save.power_on_psaaword));
    memcpy(car_set_save.power_on_psaaword, dat, lenth);
    car_control_cmd(CAR_CMD_SET_POWER_ON_PASSWORD);
    memcpy(&data[data_len], dat, lenth);
    data_len += lenth;
    ble_protocol_data_pack(BLE_CMD_S_POWER_PASSWORD, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_navigation_service(uint8_t *data)
{
    car_state_data.map_dir = data[0];
    car_state_data.cur_dir_range = data[1] << 16 | data[2] << 8 | data[3];
    car_state_data.total_nav_remaintime = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
    car_state_data.total_nav_range = data[8]<<16 | data[9]| data[10];
    LOG_I("cur_dir_range:%d", car_state_data.cur_dir_range);
    LOG_I("total_nav_remaintime:%d", car_state_data.total_nav_remaintime);
    LOG_I("total_nav_range:%d", car_state_data.total_nav_range);
    iot_can_state2_fun();
    iot_can_navigation_data();
    LOG_I("navigation_quit_time:%d", sys_param_set.navigation_quit_time);
    rtc_event_register(CAR_NAVIGATION_QUIT, sys_param_set.navigation_quit_time, 0);
}

static void ble_atsphlight_sta_query()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    struct tm *gmt = NULL;
    
    data[data_len++] = car_info.atmosphere_light_info.light_mode;
    data[data_len++] = car_info.atmosphere_light_info.color;
    data[data_len++] = car_set_save.atmosphere_light_set.atmosphere_light_task.task_en;
    gmt = localtime((time_t *)&car_set_save.atmosphere_light_set.atmosphere_light_task.start_timestap);
    data[data_len++] = gmt->tm_hour;
    data[data_len++] = gmt->tm_min;
    gmt = localtime((time_t *)&car_set_save.atmosphere_light_set.atmosphere_light_task.end_timestap);
    data[data_len++] = gmt->tm_hour;
    data[data_len++] = gmt->tm_min;
    data[data_len++] = car_set_save.atmosphere_light_set.atmosphere_light_task.action;

    data[data_len++] = car_info.atmosphere_light_info.custom_red;
    data[data_len++] = car_info.atmosphere_light_info.custom_green;
    data[data_len++] = car_info.atmosphere_light_info.custom_blue;
    data[data_len++] = car_info.atmosphere_light_info.brightness_val;
    data[data_len++] = car_info.atmosphere_light_info.turn_linght_sta;
    ble_protocol_data_pack(BLE_CMD_ATSPHLIGHT_STA_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_atsphlight_task_query()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    struct tm *gmt = NULL;
    data[data_len++] = car_set_save.atmosphere_light_set.atmosphere_light_task.task_en;
    gmt = localtime((time_t *)&car_set_save.atmosphere_light_set.atmosphere_light_task.start_timestap);
    data[data_len++] = gmt->tm_hour;
    data[data_len++] = gmt->tm_min;
    gmt = localtime((time_t *)&car_set_save.atmosphere_light_set.atmosphere_light_task.end_timestap);
    data[data_len++] = gmt->tm_hour;
    data[data_len++] = gmt->tm_min;
    data[data_len++] = car_set_save.atmosphere_light_set.atmosphere_light_task.action;
    ble_protocol_data_pack(BLE_CMD_Q_ATSPHLIGHT_TIMTASK_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_atsphlight_set_task(uint8_t *dat, uint16_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    struct tm *gmt = NULL;
    time_t timestamp, setstamp;

    car_set_save.atmosphere_light_set.atmosphere_light_task.task_en = data[1];
    timestamp = (time_t)hal_drv_rtc_get_timestamp();
    gmt = localtime(&timestamp);
    gmt->tm_hour = data[2];
    gmt->tm_min = data[3];
    setstamp = mktime(gmt);
    car_set_save.atmosphere_light_set.atmosphere_light_task.start_timestap = setstamp;
    gmt->tm_hour = data[4];
    gmt->tm_min = data[5];
    setstamp = mktime(gmt);
    car_set_save.atmosphere_light_set.atmosphere_light_task.end_timestap = setstamp;
    car_set_save.atmosphere_light_set.atmosphere_light_task.action = data[6];
    memcpy(&data[data_len], dat, lenth);
    data_len += lenth;
    ble_protocol_data_pack(BLE_CMD_S_ATSPHLIGHT_TIMTASK_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}
static void ble_current_limit_set(uint16_t dat, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query) {
        data[data_len++] = car_info.current_limit/10;  
    } else {
        data[data_len++] = car_info.current_limit/10;  
    }
    ble_protocol_data_pack(BLE_CMD_S_CURRENT_LIMIT, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void basic_second_bat_info()
{
    uint8_t data[64] = {0}, buf[96];
    uint16_t data_len = 0;
    uint16_t len;

    memcpy(&data[data_len], &car_info.bms_info[1].pack_vol, 2);
    data_len += 2;
    memcpy(&data[data_len], &car_info.bms_info[1].pack_current, 2);
    data_len += 2;
    data_len += 3; //温度？
    if(car_info.bms_info[1].chargefull_sta) {
        data[data_len++] = 0xff;
    } else if(car_info.bms_info[1].charge_sta) {
        data[data_len++] = 0x01;
    } else {
        data[data_len++] = 0x00;
    }
    data[data_len++] = car_info.bms_info[1].soh;
    memcpy(&data[data_len], &car_info.bms_info[1].charge_interval_time, 2);
    data_len += 2;
    memcpy(&data[data_len], &car_info.bms_info[1].cycle_number, 2);
    data_len += 2;
    data[data_len++] = 8;
    memcpy(&data[data_len], &car_info.bms_info[1].soft_ver[0], 8);
    ble_protocol_data_pack(BLE_CMD_Q_SECOND_BAT_INFO, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_protocol_large_query_service(uint16_t cmd)
{
    LOG_I("cmd:0x%04x", cmd);
    switch (cmd)
    {
        case BLE_CMD_Q_BASIC_SERVICES:
            basic_services_send();
        break;
        case BLE_CMD_Q_BAT_INFO:
            basic_bat_info_send();
        break;
        case BLE_CMD_Q_CYCLE_CONFIG:
            basic_cycle_config_send();
        break;
        case BLE_CMD_Q_GENERAL_CONFIG:
            basic_general_config_service_send();
        break;
        case BLE_CMD_Q_CAR_SERVICE:
            basic_whole_car_service_send();  
        break;
        case BLE_CMD_Q_IOT_SERVICE:
            car_iot_service_send();
        break;
        case BLE_CMD_Q_NET_SERVICE:
            car_net_service_state_send();
        break;
        case BLE_CMD_Q_SECOND_BAT_INFO:
            basic_second_bat_info();
        break;
        case BLE_CMD_S_HEADLIGHT_STA:
            ble_cmd_car_head_light_sta(0, 1);
        break;
        case BLE_CMD_S_ASSIST_GEAR:
            ble_cmd_car_assist_gear_service(0, 1);
        break;
        case BLE_CMD_S_SPEED_LIMIT:
            ble_cmd_car_speed_limit(0, 1);
        break;
        case BLE_CMD_S_CURRENT_LIMIT:
            ble_current_limit_set(0, 1);
        break;
        case BLE_CMD_S_WHEEL_DIAMETER:
            ble_cmd_car_wheel(0, 1);
        break;
        case BLE_CMD_S_TURN_SIGNAL:
            ble_cmd_car_turn_light(0, 1);
        break;
        case BLE_CMD_S_TMIE_SYNC:
            ble_cmd_time_sync(NULL, 1);
        break;
        case BLE_CMD_S_UNITS:
            ble_cmd_car_unit(0, 1);
        break;
        case BLE_CMD_S_BRIGHTNESS_LEVEL:

        break;
        case BLE_CMD_Q_ATSPHLIGHT_STA:
            ble_atsphlight_sta_query();
        break;
        case BLE_CMD_Q_ATSPHLIGHT_TIMTASK:
            ble_atsphlight_task_query();
        break;
        case BLE_CMD_Q_IOT_VER:
            query_iot_soft_ver();
        break;
        case BLE_CMD_Q_CAR_HWVER:
            query_car_hwver();
        break;
        case BLE_CMD_Q_APN:
            ble_apn_query_send();
        break;
        default:
        break;
    }
}
static void ble_set_atsphlight_mode(uint8_t dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

   car_set_save.atmosphere_light_set.light_mode = dat;
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_MODE);

   data[data_len++] = dat;
   ble_protocol_data_pack(BLE_CMD_ATSPHLIGHT_MODE_ASK, &data[0], data_len, &buf[0], &len);
   ble_send_data(buf, len);
}

static void ble_set_atsphlight_color_custom(uint8_t *dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    car_set_save.atmosphere_light_set.custom_red = dat[0];
    car_set_save.atmosphere_light_set.custom_green = dat[1];
    car_set_save.atmosphere_light_set.custom_blue = dat[2];
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM);
    memcpy(&data[data_len], dat, 3);
    data_len += 3;
    ble_protocol_data_pack(BLE_CMD_ATSPHLIGHT_COLOR_CUSTOM_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_atsphlight_brightness_val(uint8_t dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    car_set_save.atmosphere_light_set.brightness_val = dat;
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL);
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_S_ATSPHLIGHT_BRIGHTVAL_Q, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_atsphlight_turn(uint8_t dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    car_set_save.atmosphere_light_set.turn_linght_sta = dat;
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_TURN);
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_S_ATSPHLIGHT_TURN_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_atsphlight_type(uint8_t dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    car_set_save.atmosphere_light_set.color = dat;
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLORTYPE);
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_S_ATSPHLIGHT_COLORTYPE_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_atsphlight_sw(uint8_t dat)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    if(dat == 0x01) {
        car_set_save.atmosphere_light_set.brightness_val = 100;
    } else {
        car_set_save.atmosphere_light_set.brightness_val = 0;
    }
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL);
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_ATSPHLIGHT_SW_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


struct ble_ota_info_stu {
    uint8_t ota_type;
    uint32_t ver;
    uint32_t total_len;
    uint32_t offset;
    uint32_t pack_number;
    char ota_name[64];
};
struct ble_ota_info_stu ble_ota_info;


static void ble_ota_query_handle(uint8_t *dat, uint16_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    ble_ota_info.ota_type = dat[0];
    memcpy(&ble_ota_info.ver, &dat[1], 4);
    ble_ota_info.total_len = dat[5] << 24 | dat[6] << 16 | dat[7] << 8 | dat[8];
    data[data_len++] = ble_ota_info.ota_type;
    LOG_I("ble_ota_info.total_len:%d, ota_type:%d, ver:%0x", ble_ota_info.total_len, ble_ota_info.ota_type, ble_ota_info.ver);
    // if(sys_info.ota_flag  || ble_ota_info.estage != OTA_IDEL) {
    //     data[data_len++] = 0x05;
    // } else if(sys_info.bat_soc < 60 && sys_info.power_36v == 0) {
    //     data[data_len++] = 0x06;
    // } else {
    //     if(ble_ota_file_init(ble_ota_info.ota_type) == 0) {
    //         data[data_len++] = 0x01;
    //         ble_ota_info.data_len = 0;
    //     } else {
    //         data[data_len++] = 0x02;
    //     }
    // } 
    flash_partition_erase(DEV_APP_ADR);
    data[data_len++] = 0x01;
    memset(&data[data_len], 0, 4);
    data_len += 4;
    ble_protocol_data_pack(BLE_CMD_OTA_REQ_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_ota_send_data(uint8_t *dat, uint16_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    uint16_t pack_len;

    if(dat[0] == ble_ota_info.ota_type) {
        ble_ota_info.pack_number = dat[1] << 24 | dat[2] << 16 | dat[3] << 8 | dat[4];
        pack_len = lenth - 5;
        flash_partition_write(DEV_APP_ADR, &dat[5], pack_len, ble_ota_info.offset);
        ble_ota_info.offset += pack_len;
        LOG_I("ble_ota_info.pack_number:%d", ble_ota_info.pack_number);
        LOG_I("ble protocol ota progress %d%%", (ble_ota_info.offset*100)/ble_ota_info.total_len);
        data[data_len++] = dat[0];
        if(ble_ota_info.total_len == ble_ota_info.offset) {
            data[data_len++] = 0x99;
            if(app_iot_ota_jump() != 0){
                LOG_E("iot ota is fail");
            }
        } else {
            data[data_len++] = 0x01;
        }    
    }
    ble_protocol_data_pack(BLE_CMD_OTA_DATA_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


void ble_protocol_audio_play(uint8_t dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;
    switch(dat) {
        case 1:
        case 2:
            voice_play_mark(UNLOCK_VOICE);
        break;
        case 3:
        case 4:
            voice_play_mark(LOCK_VOICE);
        break;
        case 5:
            voice_play_mark(ALARM_VOICE);
        break;
        case 6:
            voice_play_mark(LOOK_CAR_VOICE);
        break;
    }
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_U_Q_AUDIO_PLAY, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


void ble_cmd_jump_password(uint8_t dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;
    if(dat == 0xff){
        car_control_cmd(CAR_CMD_JUMP_PASSWORD);
    }
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_S_JUMP_PASSWORD, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_cmd_en_power_on_password(uint8_t dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;
    if(dat == 1) {
        car_set_save.en_power_on_psaaword = 1;
    } else {
        car_set_save.en_power_on_psaaword = 0;
    }
    car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
    data[data_len++] = dat;
    ble_protocol_data_pack(BLE_CMD_S_POWERPASSWORD_SW, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_cmd_can_trans(uint8_t *data)
{
    stc_can_rxframe_t can_dat = {0};
    can_dat.Cst.Control_f.DLC = data[0] >> 4;
    can_dat.Cst.Control_f.IDE = (data[0] >> 2)&0x01;
    can_dat.Cst.Control_f.RTR = 0;
    can_dat.ExtID = data[1]<< 24 | data[2] << 16 | data[3] << 8 | data[4];
    LOG_I("%02X, %02X, %02X, %02X", data[1], data[2], data[3], data[4]);
    LOG_I("%08X", can_dat.ExtID);
    memcpy(&can_dat.Data[0], &data[5], 8);
    can_data_send(can_dat);
}

void ble_cmd_can_trans_up(stc_can_rxframe_t can_dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len] |= can_dat.Cst.Control_f.DLC<<4;
    data[data_len] &= ~(1<<3);
    data[data_len] |= can_dat.Cst.Control_f.IDE << 2;
    data[data_len] &= ~(0x03);

    data_len++;
    data[data_len++] = (can_dat.ExtID >> 24)&0xff;
    data[data_len++] = (can_dat.ExtID >> 16)&0xff;
    data[data_len++] = (can_dat.ExtID >> 8)&0xff;
    data[data_len++] = can_dat.ExtID&0xff;

    memcpy(&data[data_len], &can_dat.Data[0], 8);
    data_len += 8;
    ble_protocol_data_pack(BLE_CMD_TRANS_CAN_UP, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_query_hid_sw_sta()
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;
    data[data_len++] = (sys_param_set.hid_lock_sw == 1) ? 0x01:0x02;
    ble_protocol_data_pack(BLE_CMD_Q_HIDKEY_STA, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_hid_sw(uint8_t *dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;

    if(dat[0] == 0x01){
        sys_param_set.hid_lock_sw = 1;
    } else {
        sys_param_set.hid_lock_sw = 0;
    }

    if(dat[1] == 0x01){
        sys_set_var.hid_lock_sw_type = 0x00;
    } else {
        sys_set_var.hid_lock_sw_type = 0x01;
    }

    memcpy(&data[data_len], &dat[0], 2);
    ble_protocol_data_pack(BLE_CMD_S_HIDKEY_SW, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_charge_power_query()
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;

    switch(sys_param_set.bms_charge_current){
        case 2:
            data[data_len++] = 0x01;
        break;
        case 4:
            data[data_len++] = 0x02;
        break;
        case 6:
            data[data_len++] = 0x03;
        break;
        case 8:
            data[data_len++] = 0x04;
        break;
        default:
            data[data_len++] = 0x01;
    }
    ble_protocol_data_pack(BLE_CMD_Q_CHARGE_POWER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_charge_power(uint8_t *dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;

    switch(dat[0]){
        case 1:
            sys_param_set.bms_charge_current = 2;
        break;
        case 2:
            sys_param_set.bms_charge_current = 4;
        break;
        case 3:
            sys_param_set.bms_charge_current = 6;
        break;
        case 4:
            sys_param_set.bms_charge_current = 8;
        break;
    }
    data[data_len++] = dat[0];
    car_control_cmd(CAR_BMS_CHARGE_CURRENT_SET);
    ble_protocol_data_pack(BLE_CMD_S_CHARGE_POWER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_set_navigation_quit_time()
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = car_set_save.navigation_quit_time;
    ble_protocol_data_pack(BLE_CMD_S_QUIT_NAVIGATION_TIME, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}


void ble_log_out(uint8_t *data, uint16_t data_len)
{
    uint8_t buf[256];
    uint16_t len;
    ble_protocol_data_pack(BLE_CMD_U_LOG, data, data_len, buf, &len);
    ble_send_data(buf, len);
}

static void ble_cmd_log_sw(uint8_t *dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t len;

    if(dat[0] == 0x01){
        sys_info.ble_log_sw = 1;
    } else {
        sys_info.ble_log_sw = 0;
    }
    data[0] = dat[0];
    ble_protocol_data_pack(BLE_CMD_LOG_SW_ASK, &data[0], 1, &buf[0], &len);
    def_rtos_task_sleep_ms(10);
    ble_send_data(buf, len);
}

static void ble_cmd_trans_set(uint8_t *dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t len;

    if(dat[0] == 0x00) {
        sys_info.ble_can_trans_sw = 0;
    } else {
        sys_info.ble_can_trans_sw = 1;
    }
    data[0] = dat[0];
    ble_protocol_data_pack(BLE_CMD_CAN_TRANS_SET, &data[0], 1, &buf[0], &len);
    ble_send_data(buf, len);
} 
static void ble_cmd_qury_move_sw()
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t len;
    data[0] = sys_param_set.shock_sw?0x01:0x02;
    ble_protocol_data_pack(BLE_CMD_Q_MOVE_SW, &data[0], 1, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_set_move_sw(uint8_t dat)
{
    uint8_t data[16] = {0}, buf[64];
    uint16_t len;
    if(dat == 0x01) {
        sys_param_set.shock_sw = 1;
    } else {
        sys_param_set.shock_sw = 0;
    }
    SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
    data[0] = dat;
    ble_protocol_data_pack(BLE_CMD_MOVE_SW, &data[0], 1, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_look_car()
{
    uint8_t buf[64];
    uint16_t len;
    CAR_CMD_Q car_cmd_q;
    car_cmd_q.src = BLUE_CAR_CMD_SER;
    car_cmd_q.cmd = CAR_LOOK_CAR2;
    car_cmd_q.ble_car_control.ble_cmd = BLE_CMD_LOOK_CAR;
    CAR_CMD_MARK(car_cmd_q);
    ble_protocol_data_pack(BLE_CMD_LOOK_CAR, NULL, 0, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_q_bms_health_info()
{
    uint8_t buf[64], data[64];
    uint16_t len;
    uint16_t lenth = 0;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.capacity_input_quantity >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.capacity_input_quantity >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.capacity_input_quantity >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.capacity_input_quantity&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_input_quantity >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_input_quantity >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_input_quantity >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.engwe_input_quantity&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_use_time >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_use_time >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_use_time >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.extreme_temperature_use_time&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_charge_time >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_charge_time >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.extreme_temperature_charge_time >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.extreme_temperature_charge_time&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.deep_discharge_count >>8 )&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.deep_discharge_count&0xff;

    data[lenth++] = car_info.bms_info[0].ece_regulation.battery_self_discharge_rate;
    data[lenth++] = car_info.bms_info[0].ece_regulation.engwe_exchange_efficiency;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.capactity_output_quantity >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.capactity_output_quantity >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.capactity_output_quantity >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.capactity_output_quantity&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_output_quantity >> 24)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_output_quantity >> 16)&0xff;
    data[lenth++] = (car_info.bms_info[0].ece_regulation.engwe_output_quantity >> 8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.engwe_output_quantity&0xff;

    data[lenth++] = (car_info.bms_info[0].ece_regulation.battery_internal_resistance >>8)&0xff;
    data[lenth++] = car_info.bms_info[0].ece_regulation.battery_internal_resistance&0xff;
    data[lenth++] = car_info.bms_info[0].manufacture_date.year;
    data[lenth++] = car_info.bms_info[0].manufacture_date.month;
    data[lenth++] = car_info.bms_info[0].manufacture_date.day;

    ble_protocol_data_pack(BLE_CMD_U_BMS_HEALTH_INFO, &data[0], lenth, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_cmd_enter_ship_mode(uint8_t dat)
{
    CAR_CMD_Q car_cmd_q;
    car_cmd_q.src = BLUE_CAR_CMD_SER;
    if(dat == 0x01) 
        system_enter_ship_mode(car_cmd_q);


}

void ble_cmd_ship_mode_ask(uint8_t ship_mode)
{
    uint8_t buf[64], data[64];
    uint16_t len;
    uint16_t lenth = 0;
    data[lenth++] = ship_mode;
    ble_protocol_data_pack(BLE_CMD_TRANSPORT_MODE_QES_ASK, &data[0], lenth, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_bat_charge_sw(uint8_t dat)
{
    #if 0
    if(dat == 0x01){
        MCU_CMD_MARK(CMD_MCU_BAT_CHARGE_ON_INDEX);
    } else {
        MCU_CMD_MARK(CMD_MCU_BAT_CHARGE_OFF_INDEX);
    }
    #endif
}

static void ble_debug_voice_sw(uint8_t dat)
{
    #if 0
    if(dat == 0x01){
        voice_play_mark(VOICE_TEST); 
    } else {
        voice_play_off();
    }    
    #endif
}
void ble_protocol_cmd_parse(uint16_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    uint16_t cmd_sub;
    LOG_I("cmd:0x%04x", cmd);
    sys_info.hmi_auto_power_off_sw = 1;
    switch(cmd)
    {
        case BLE_CMD_LARGE_QUERY:
            for(i = 0; i < len; i+=2) {
                cmd_sub = data[i] <<8 | data[i+1];
                ble_protocol_large_query_service(cmd_sub);
            }
        break;
        case BLE_CMD_Q_BASIC_SERVICES:
            basic_services_send();
        break;
        case BLE_CMD_Q_BAT_INFO:
            basic_bat_info_send();
        break;
        case BLE_CMD_Q_CYCLE_CONFIG:
            basic_cycle_config_send();
        break;
        case BLE_CMD_Q_GENERAL_CONFIG:
            basic_general_config_service_send();
        break;
        case BLE_CMD_Q_CAR_SERVICE:
            basic_whole_car_service_send();
        break;
        case BLE_CMD_Q_SECOND_BAT_INFO:
            basic_second_bat_info();
        break;
        case BLE_CMD_Q_IOT_SERVICE:
            car_iot_service_send();    
        break;
        case BLE_CMD_Q_NET_SERVICE:
            car_net_service_state_send();
        break;
        case BLE_CMD_Q_IOT_VER:
            query_iot_soft_ver();
        break;
        case BLE_CMD_Q_CAR_HWVER:
            query_car_hwver();
        break;
        case BLE_CMD_S_HEADLIGHT_STA:
            ble_cmd_car_head_light_sta(data[0], 0);
        break;
        case BLE_CMD_S_ASSIST_GEAR:
            ble_cmd_car_assist_gear_service(data[0], 0);
        break;
        case BLE_CMD_S_SPEED_LIMIT:
            ble_cmd_car_speed_limit(data[0], 0);
        break;

        case BLE_CMD_S_CURRENT_LIMIT:
            ble_current_limit_set(data[0]<<8|data[1], 0);
        break;
        case BLE_CMD_S_WHEEL_DIAMETER:        
        break;

        case BLE_CMD_S_TURN_SIGNAL:
            ble_cmd_car_turn_light(data[0], 0);
        break;
        case BLE_CMD_S_TMIE_SYNC:
            ble_cmd_time_sync(data, 0);
        break;

        case BLE_CMD_S_UNITS:
            ble_cmd_car_unit(data[0], 0);
        break;

        case BLE_CMD_S_BRIGHTNESS_LEVEL:
            ble_cmd_car_brightness_level(data[0], 0);
        break;
        case BLE_CMD_S_TIME_AUTODOWN:

        break;
        case BLE_CMD_S_JUMP_PASSWORD:
            ble_cmd_jump_password(data[0]);
        break;
        case BLE_CMD_S_POWERPASSWORD_SW:
            ble_cmd_en_power_on_password(data[0]);
        break;
        case BLE_CMD_S_POWER_SW:
            ble_cmd_bat_charge_sw(data[0]);
        break;
        case BLE_CMD_S_LOCK_SW:
            ble_cmd_lock_control(data[0]);
        break;
        case BLE_CMD_S_APN:
            ble_cmd_set_apn_info(data, len);
        break;
        case BLE_CMD_Q_HIDKEY_STA:
            ble_query_hid_sw_sta();
        break;
        case BLE_CMD_S_HIDKEY_SW:
            ble_set_hid_sw(data);
        break;
        case BLE_CMD_Q_CHARGE_POWER:
            ble_charge_power_query();
        break;
        case BLE_CMD_S_CHARGE_POWER:
            ble_set_charge_power(data);
        break;
        case BLE_CMD_S_QUIT_NAVIGATION_TIME:
            ble_set_navigation_quit_time();
        break;
        case BLE_CMD_S_POWER_PASSWORD:
            ble_cmd_set_power_on_password(data, len);
        break;
        case BLE_CMD_NAVIGATION_SERVICE:
            ble_navigation_service(data);
        break;
        case BLE_CMD_Q_ATSPHLIGHT_STA:
            
        break;
        case BLE_CMD_S_ATSPHLIGHT_MODE:
            ble_set_atsphlight_mode(data[0]);
        break;
        case BLE_CMD_ATSPHLIGHT_COLOR_CUSTOM:
            ble_set_atsphlight_color_custom(data);
        break;
        case BLE_CMD_S_ATSPHLIGHT_BRIGHTVAL:
            ble_set_atsphlight_brightness_val(data[0]);
        break;
        case BLE_CMD_S_ATSPHLIGHT_TURN:
            ble_set_atsphlight_turn(data[0]);
        break;
        case BLE_CMD_S_ATSPHLIGHT_COLORTYPE:
            ble_set_atsphlight_type(data[0]);
        break;
        case BLE_CMD_ATSPHLIGHT_SW:
            ble_set_atsphlight_sw(data[0]);
        break;
        case BLE_CMD_S_ATSPHLIGHT_TIMTASK:
            ble_atsphlight_set_task(data, len);
        break;
        case BLE_CMD_Q_APN:
            ble_apn_query_send();
        break;
        case BLE_CMD_OTA_REQ:
            ble_ota_query_handle(data, len);
        break;
        case BLE_CMD_OTA_SEND_DATA:
            ble_ota_send_data(data, len);
        break;
        case BLE_CMD_D_Q_AUDIO_PLAY:
            ble_protocol_audio_play(data[0]);
        break;
        case BLE_CMD_CAN_TRANS_SET:
            ble_cmd_trans_set(data);
        break;
        case BLE_CMD_TRANS_CAN:
            ble_cmd_can_trans(data);
        break;
        case BLE_CMD_LOG_SW:
            ble_cmd_log_sw(data);
        break;
        case BLE_CMD_LOOK_CAR:
            ble_cmd_look_car();
        break;
        case BLE_CMD_Q_MOVE_SW:
            ble_cmd_qury_move_sw();
        break;
        case BLE_CMD_MOVE_SW:
            ble_cmd_set_move_sw(data[0]);
        break;
        case BLE_CMD_Q_BMS_HEALTH_INFO:
            ble_cmd_q_bms_health_info();
        break;

        case BLE_CMD_TRANSPORT_MODE_QES:
            ble_cmd_enter_ship_mode(data[0]);
        break;

        case BLE_CMD_IOT_ACTIVE_REQ:
            ble_debug_voice_sw(data[0]);
        break;
        default:
        break;
    }
}

void ble_up_cycle_data_heart_service()
{
  //  uint8_t data[256] = {0}, buf[256];
    uint8_t *data, *buf; 
    uint16_t data_len = 0;
    uint16_t len;

    data = malloc(128);
    buf = malloc(128);
    data[data_len++] = (car_info.speed >> 8) & 0xff;
    data[data_len++] = car_info.speed&0xff;
    data[data_len++] = (car_info.max_speed >> 8)&0xff;
    data[data_len++] = car_info.max_speed&0xff;
    data[data_len++] = (car_info.avg_speed >> 8)&0xff;
    data[data_len++] = car_info.avg_speed&0xff;  //CAN错误码
    data[data_len++] = (car_info.single_odo >> 16)&0xff;
    data[data_len++] = (car_info.single_odo>>8)&0xff;
    data[data_len++] = car_info.single_odo&0xff;
    data[data_len++] = (car_info.cycle_time_s >> 24)&0xff;
    data[data_len++] = (car_info.cycle_time_s>>16)&0xff;
    data[data_len++] = (car_info.cycle_time_s>>8)&0xff;
    data[data_len++] = car_info.cycle_time_s&0xff;
    data[data_len++] = ((car_info.remain_odo/10) >> 8)&0xff;
    data[data_len++] = (car_info.remain_odo/10)&0xff;
    data[data_len++] = (car_info.total_odo >> 16)&0xff;
    data[data_len++] = (car_info.total_odo>>8)&0xff;
    data[data_len++] = car_info.total_odo&0xff;
    data[data_len++] = car_info.gear;
    data[data_len++] = ((car_info.motor_power/10) >> 8)&0xff;
    data[data_len++] = (car_info.motor_power/10)&0xff;
    if(car_info.bms_info[0].connect == 1) {
        data[data_len++] = car_info.bms_info[0].soc;
    } else {
        data[data_len++] = 0;
    }
    data[data_len++] = car_info.pedal_speed >> 8;
    data[data_len++] = car_info.pedal_speed & 0xff;   
    data[data_len++] = ((car_info.ebike_calorie/1000)>>8)&0xff;
    data[data_len++] = (car_info.ebike_calorie/1000)&0xff;
    data[data_len++] = 0;
    data[data_len++] = 0;  //CAN错误码
    data[data_len++] = car_info.headlight_sta?0xff:0x00;
    if(car_info.left_turn_light_sta && car_info.right_turn_light_sta) {
        data[data_len++] = 0x03;
    } else if(car_info.left_turn_light_sta && !car_info.right_turn_light_sta) {
        data[data_len++] = 0x01;
    }else if(!car_info.left_turn_light_sta && car_info.right_turn_light_sta){
        data[data_len++] = 0x02;
    } else{
        data[data_len++] = 0x00;
    }
    data[data_len++] = car_info.hmi_info.fault_code;
    data[data_len++] = 0;
    data[data_len++] = 0;
    data[data_len++] = 0;
    data[data_len++] = 0;
    data[data_len++] = sys_info.paltform_connect ? 0x01:0x02;
    data[data_len++] = gsm_info.csq;
    data[data_len++] = sys_info.bat_soc;
    data[data_len++] = Gps.SateNum;

    data[data_len++] = (car_info.lock_sta == CAR_UNLOCK_STA)?0x01:0x02;
    data[data_len++] = 0x01;  //开关机
    if(car_info.bms_info[0].chargefull_sta) {
        data[data_len++] = 0xFF;
    } else if(car_info.bms_info[0].charge_sta) {
        data[data_len++] = 0x01; 
    }  else {
        data[data_len++] = 0x00;
    }
    data[data_len++] = (car_info.bms_info[0].pack_vol*10) >> 8;
    data[data_len++] = (car_info.bms_info[0].pack_vol*10)&0xff;
    data[data_len++] = car_info.total_agv_pedal_speed>>8;
    data[data_len++] = car_info.total_agv_pedal_speed&0xff;
    data[data_len++] =(car_info.cycle_total_time >> 24)&0xff;
    data[data_len++] =(car_info.cycle_total_time >> 16)&0xff;
    data[data_len++] =(car_info.cycle_total_time >> 8)&0xff;
    data[data_len++] =car_info.cycle_total_time&0xff;
    ble_protocol_data_pack(BLE_CMD_U_RIDEDATA_SERVICE, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
    free(data);
    free(buf);
}



void ble_heart_event()
{
    ble_up_cycle_data_heart_service();
}


void ble_protocol_recv_thread(void *param)
{
    uint8_t step = 0,res;
    uint8_t buf[256], data[256];
    uint16_t len, cmd_w, lenth, rcv_crc, check_crc, i, j;
    int64_t ble_protocol_timeout = 0;
    memset(&ble_ota_info, 0, sizeof(ble_ota_info));
    while(1) {
  //      LOG_I("IS RUN");
        len = ble_trans_data_block_read(buf, 256, RTOS_WAIT_FOREVER);
        LOG_I("ble trans is recv, len:%d", len);
        if(len == 0)continue;
        debug_data_printf("ble_protocol_rec", buf, len);
        for(i = 0; i< len; i++){
            res = buf[i];
            switch(step)
            {
            case 0:
                if(res == HEAD_H){
                    step = 1;
                    j = 0;
                    ble_protocol_timeout = def_rtos_get_system_tick();
                } else {
                    step = 0;
                }
            break;
            case 1:
                if(res == HEAD_L){
                    step = 2;
                } else {
                    step = 0;
                }
            break;
            case 2:
                data[j++] = res;
                cmd_w = res<<8;
                step = 3;
            break;
            case 3:
                data[j++] = res;
                cmd_w |= res;
                step = 4;
            break;
            case 4:
                data[j++] = res;
               lenth = res << 8;
               step = 5;
            break;
            case 5:
               data[j++] = res;
               lenth |= res; 
               if(lenth == 0) step = 7; 
               else step = 6;
            break;
            case 6:
                data[j++] = res;
                if(j == lenth + 4){
                    step = 7;
                }
            break;
            case 7:
                rcv_crc = res;
                step = 8;
            break;
            case 8:
                rcv_crc |= res<< 8;
                check_crc = drv_modbus_crc16(data, j);
                if(rcv_crc == check_crc) {
                    LOG_I("crc is successful");
                    step = 9;
                } else {
                    step = 0;
                    LOG_E("crc is fail, rcv_crc:%02x, check_crc:%02x", rcv_crc, check_crc);
                }
            break;
            case 9:
                if(TAIL_H == res) {
                    step = 10;
                } else {
                    step = 0;
                }
            break;
            case 10:
                if(TAIL_L == res){
                    ble_protocol_cmd_parse(cmd_w, &data[4], lenth);
                } 
                step = 0;
                break;
            }
        } 
        if(def_rtos_get_system_tick() - ble_protocol_timeout > 500) {
            step = 0;
        }
        // if(ble_protocol_check(buf, len) == OK) {
        //     cmd_w = buf[2]<<8 | buf[3];
        //     lenth = buf[4]<<8 | buf[5];
        //     ble_protocol_cmd_parse(cmd_w, &buf[6], lenth);
        // }
    }
    def_rtos_task_delete(NULL);
}