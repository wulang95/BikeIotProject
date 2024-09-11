/*蓝牙协议*/
#include "app_system.h"
#define DBG_TAG         "ble_protol"

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
    CMD_LARGE_QUERY = 0X2411,
    CMD_Q_BASIC_SERVICES = 0XFF01,
    CMD_Q_BAT_INFO = 0XFF02,
    CMD_Q_CYCLE_CONFIG = 0XFF03,
    CMD_Q_GENERAL_CONFIG = 0XFF04,
    CMD_Q_CAR_SERVICE = 0XFF05,
    CMD_Q_SECOND_BAT_INFO = 0XFF06,
    CMD_Q_IOT_SERVICE = 0XFF07,
    CMD_Q_NET_SERVICE = 0XFF08,
    CMD_Q_IOT_VER = 0XFF09,
    CMD_Q_CAR_HWVER = 0XFF0A,
    CMD_S_HEADLIGHT_STA = 0X597,
    CMD_S_ASSIST_GEAR = 0X1009,
    CMD_S_SPEED_LIMIT = 0X2A65,
    CMD_S_CURRENT_LIMIT = 0X2A66,
    CMD_S_WHEEL_DIAMETER = 0X08C3,
    CMD_S_TURN_SIGNAL  = 0X0596,
    CMD_S_GENERAL_CONFIG = 0X1805,
    CMD_S_UNITS   = 0X2B46,
    CMD_S_BRIGHTNESS_LEVEL = 0X07C0,
    CMD_S_TIME_AUTODOWN = 0X1000,
    CMD_S_JUMP_PASSWORD   = 0XBEC9,
    CMD_S_POWERPASSWORD_SW = 0XBECE,
    CMD_S_POWER_SW   = 0X0107,
    CMD_S_LOCK_SW  = 0X0108,
    CMD_Q_APN  = 0X010A,
    CMD_S_APN = 0X010B,
    CMD_Q_HIDKEY_STA = 0X010C,
    CMD_S_HIDKEY_SW = 0X010D,
    CMD_S_RESTORE_FACTORY_CONFIG = 0X8049,
    CMD_S_CLEAN_ODO_DATA = 0XBEC7,
    CMD_S_POWER_PASSWORD = 0XBEC8,

    CMD_U_RIDEDATA_SERVICE = 0X2A63,
    CMD_NAVIGATION_SERVICE = 0X2A67,

    CMD_OTA_REQ = 0X0501,
    CMD_OTA_REQ_ASK = 0X0502,
    CMD_OTA_SEND_DATA = 0X0503,
    CMD_OTA_SEND_ASK = 0X0504,
    CMD_OTA_Q_VER = 0X0505,
    CMD_OTA_VER_ASK = 0X0506,
    CMD_CONFIGFILE_SENDREQ = 0X0601,
    CMD_CONFIGFILE_SENDREQ_ASK = 0X0602,
    CMD_CONFIGFILE_SENDDATA = 0X0603,
    CMD_CONFIGFILE_SENDDATA_ASK = 0X0604,

    CMD_TRANS_REQ = 0X0701,
    CMD_TRANS_ASK = 0X0702,

    CMD_IOT_ACTIVE_REQ = 0X0703,
    CMD_IOT_ACTIVE_REQ_ASK = 0X0704,

    CMD_Q_ATSPHLIGHT_STA = 0X0801,
    CMD_ATSPHLIGHT_STA_ASK = 0X0802,
    CMD_S_ATSPHLIGHT_MODE = 0X0803,
    CMD_ATSPHLIGHT_MODE_ASK = 0X0804,
    CMD_ATSPHLIGHT_COLOR_CUSTOM = 0X0805,
    CMD_ATSPHLIGHT_COLOR_CUSTOM_ASK = 0X0806,
    CMD_S_ATSPHLIGHT_BRIGHTVAL = 0X0807,
    CMD_S_ATSPHLIGHT_BRIGHTVAL_Q = 0X0808,
    CMD_S_ATSPHLIGHT_DIR = 0X0809,
    CMD_S_ATSPHLIGHT_DIR_ASK = 0X080A,
    CMD_S_ATSPHLIGHT_COLORTYPE = 0X080B,
    CMD_S_ATSPHLIGHT_COLORTYPE_ASK = 0X080C,
    CMD_ATSPHLIGHT_SW = 0X080D,
    CMD_ATSPHLIGHT_SW_ASK = 0X080E,
    CMD_Q_ATSPHLIGHT_TIMTASK = 0X080F,
    CMD_Q_ATSPHLIGHT_TIMTASK_ASK = 0X0810,
    CMD_S_ATSPHLIGHT_TIMTASK = 0X0811,
    CMD_S_ATSPHLIGHT_TIMTASK_ASK = 0X0812,

    CMD_D_Q_AUDIO_PLAY = 0X0901,
    CMD_U_Q_AUDIO_PLAY = 0X0902,

    CMD_TRANSPORT_MODE_QES = 0X0A01,
    CMD_TRANSPORT_MODE_QES_ASK = 0X0A02,
    CMD_U_LOG = 0X0B01,
    CMD_LOG_SW = 0X0B02,
    CMD_LOG_SW_ASK = 0X0B03,
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
    ble_protocol_data_pack(CMD_Q_BASIC_SERVICES, &data[0], data_len, &buf[0], &len);
    ble_send_data(buf, len);
} 


void ble_protocol_cmd_parse(uint16_t cmd, uint8_t *data, uint16_t len)
{
    uint16_t i = 0;
    uint16_t cmd_sub;
    LOG_I("cmd:%04x", cmd);
    switch(cmd)
    {
        case CMD_LARGE_QUERY:
            for(i = 0; i < len; i+=2) {
                cmd_sub = data[i] <<8 | data[i+1];
                switch (cmd_sub)
                {
                case CMD_Q_BASIC_SERVICES:
                    basic_services_send();
                    break;
                case CMD_Q_BAT_INFO:

                    break;
                case CMD_Q_CYCLE_CONFIG:

                    break;
                case CMD_Q_GENERAL_CONFIG:

                    break;
                case CMD_Q_CAR_SERVICE:
                    
                    break;
                case CMD_Q_SECOND_BAT_INFO:

                    break;
                case CMD_Q_IOT_SERVICE:

                    break;
                case CMD_Q_NET_SERVICE:

                    break;
                default:
                    break;
                }
            }
        break;
        case CMD_Q_IOT_VER:

        break;
        case CMD_Q_CAR_HWVER:

        break;
        default:
        break;
    }
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