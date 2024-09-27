/*蓝牙协议*/
#include "app_system.h"
#include <time.h>
#include "hal_drv_rtc.h"
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
    BLE_CMD_S_RESTORE_FACTORY_CONFIG = 0X8049,
    BLE_CMD_S_CLEAN_ODO_DATA = 0XBEC7,
    BLE_CMD_S_POWER_PASSWORD = 0XBEC8,

    BLE_CMD_U_RIDEDATA_SERVICE = 0X2A63,
    BLE_CMD_NAVIGATION_SERVICE = 0X2A67,

    BLE_CMD_OTA_REQ = 0X0501,
    BLE_CMD_OTA_REQ_ASK = 0X0502,
    BLE_CMD_OTA_SEND_DATA = 0X0503,
    BLE_CMD_OTA_SEND_ASK = 0X0504,
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
};

void ble_protocol_data_pack(uint16_t cmd, uint8_t *data, uint16_t data_len, uint8_t *buf, uint16_t *len)
{
    uint8_t p[256];
    uint16_t lenth = 0;
    uint16_t check_crc;

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
    data[data_len++] = car_info.wheel;
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
    data[data_len++] = gps_info.starNum;
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
        if(set == 0x00){
            car_set_save.head_light = 0;
        } else {
            car_set_save.head_light = 1;
        }
        car_control_cmd(CAR_CMD_SET_HEADLIGHT);
        data[data_len++] = set;
    }
    ble_protocol_data_pack(BLE_CMD_S_HEADLIGHT_STA, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_assist_gear_service(uint8_t set, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query){
        data[data_len++] = car_info.gear?0xff:0x00;
    } else{
        car_set_save.gear = set;
        car_control_cmd(CAR_CMD_SET_GEAR);
        data[data_len++] = set;
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

    memcpy(data, HWVER, strlen(HWVER));
    data_len += strlen(HWVER);
    ble_protocol_data_pack(BLE_CMD_Q_CAR_HWVER, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_speed_limit(uint8_t speed, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(query) {
        data[data_len++] = car_info.speed & 0xff;
    } else {
        car_set_save.speed_limit = speed * 10;
        car_control_cmd(CAR_CMD_SET_SPEED_LIMIT);
        data[data_len++] = speed;
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
        data[data_len++] = car_info.wheel;
    } else {
        ;
    }
    ble_protocol_data_pack(BLE_CMD_S_CURRENT_LIMIT, &data[0], data_len, &buf[0], &len);
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
    }
    ble_protocol_data_pack(BLE_CMD_S_CURRENT_LIMIT, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_car_unit(uint8_t unit, uint8_t query)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    if(query){
        data[data_len++] = car_info.hmi_info.display_unit ? 0x02: 0x01;
    } else {
        if(unit == 0x02) car_set_save.mileage_unit = 0;
        else car_set_save.mileage_unit = 1;
         data[data_len++] = unit;
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
    }
    ble_protocol_data_pack(BLE_CMD_S_BRIGHTNESS_LEVEL, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_lock_control(uint8_t dat)
{
    if(dat == 0x01) {
        car_lock_control(BLE_CMD_LOCK_SRC, CAR_UNLOCK_ATA);
    } else {
        car_lock_control(BLE_CMD_LOCK_SRC, CAR_LOCK_STA);
    }
}

void ble_cmd_lock_res_ack()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;
    if(car_info.lock_sta == CAR_LOCK_STA) {
        data[data_len++] = 0x02;
    } else {
        data[data_len++] = 0x01;
    }
    ble_protocol_data_pack(BLE_CMD_LOCK_ASK, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_cmd_set_apn_info(uint8_t *dat, uint8_t lenth)
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

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

    memcpy(car_set_save.power_on_psaaword, dat, lenth);

    memcpy(&data[data_len], dat, lenth);
    data_len += lenth;
    ble_protocol_data_pack(BLE_CMD_S_APN, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

static void ble_navigation_service(uint8_t *data)
{
    car_state_data.map_dir = data[0];
    car_state_data.cur_dir_range = data[1] << 16 | data[2] << 8 | data[3];
    car_state_data.total_nav_remaintime = data[4] << 24 | data[5] << 16 | data[6] << 8 | data[7];
    car_state_data.total_nav_remaintime = data[8]<<16 | data[9]| data[10];
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
void ble_protocol_cmd_parse(uint16_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    uint16_t cmd_sub;
    LOG_I("cmd:0x%04x", cmd);
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
        break;
        case BLE_CMD_S_POWERPASSWORD_SW:
        break;
        case BLE_CMD_S_POWER_SW:
        break;
        case BLE_CMD_S_LOCK_SW:
            ble_cmd_lock_control(data[0]);
        break;
        case BLE_CMD_S_APN:
            ble_cmd_set_apn_info(data, len);
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
        default:
        break;
    }
}

void ble_up_cycle_data_heart_service()
{
    uint8_t data[256] = {0}, buf[256];
    uint16_t data_len = 0;
    uint16_t len;

    data[data_len++] = (car_info.speed >> 8) & 0xff;
    data[data_len++] = car_info.speed&0xff;
    data[data_len++] = (car_info.max_speed >> 8)&0xff;
    data[data_len++] = car_info.max_speed&0xff;
    data[data_len++] = (car_info.single_odo >> 16)&0xff;
    data[data_len++] = (car_info.single_odo>>8)&0xff;
    data[data_len++] = car_info.single_odo&0xff;
    data[data_len++] = (car_info.cycle_time_s >> 24)&0xff;
    data[data_len++] = (car_info.cycle_time_s>>16)&0xff;
    data[data_len++] = (car_info.cycle_time_s>>8)&0xff;
    data[data_len++] = car_info.cycle_time_s&0xff;
    data[data_len++] = (car_info.remain_odo >> 8)&0xff;
    data[data_len++] = car_info.remain_odo&0xff;
    data[data_len++] = (car_info.total_odo >> 16)&0xff;
    data[data_len++] = (car_info.total_odo>>8)&0xff;
    data[data_len++] = car_info.total_odo&0xff;
    data[data_len++] = car_info.gear;
    data[data_len++] = ((car_info.motor_power/10) >> 8)&0xff;
    data[data_len++] = (car_info.motor_power/10)&0xff;
    data[data_len++] = car_info.bms_info[0].soc;
    data[data_len++] = car_info.pedal_speed;
    data[data_len++] = ((car_info.calorie/1000)>>8)&0xff;
    data[data_len++] = (car_info.calorie/1000)&0xff;
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
    data[data_len++] = gsm_info.online ? 0x01:0x02;
    data[data_len++] = gsm_info.csq;
    data[data_len++] = sys_info.bat_soc;
    data[data_len++] = gps_info.starNum;
    data[data_len++] = car_info.lock_sta?0x01:0x02;
    data[data_len++] = 0x01;
    ble_protocol_data_pack(BLE_CMD_U_RIDEDATA_SERVICE, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
}

void ble_heart_event()
{
    ble_up_cycle_data_heart_service();
}
void ble_protocol_recv_thread(void *param)
{
    uint8_t buf[256];
    uint16_t len, cmd_w, lenth;

    while(1){
        len = ble_trans_data_block_read(buf, 256, RTOS_WAIT_FOREVER);
        LOG_I("ble trans is recv, len:%d", len);
        if(len == 0)continue;
        debug_data_printf("ble_protol", buf, len);
        if(ble_protocol_check(buf, len) == OK) {
            cmd_w = buf[2]<<8 | buf[3];
            lenth = buf[4]<<8 | buf[5];
            ble_protocol_cmd_parse(cmd_w, &buf[6], lenth);
        }
    }
    def_rtos_task_delete(NULL);
}