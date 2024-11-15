#include "app_system.h"

#define DBG_TAG         "net_protocol"

#ifdef NET_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"


#define HEADER      0XFFFF
#define SERVER_TAG    "*SCOS"
#define IOT_TAG       "*SCOR"
#define PARAM_LEN       256

struct net_send_cmd_con_stu{
    uint8_t cent;
    uint8_t ask_falg;
    uint8_t send_flag;
    uint8_t cmd;
}net_send_cmd_con;

def_rtos_queue_t net_protocol_send_que;

struct net_lock_control_stu {
    uint8_t lock_operate;
    uint16_t key_vaild_time;
    char user_id[20];
    int64_t timestamp;
    uint8_t rand_key;
    uint8_t lock_res;
};

struct net_lock_control_stu net_lock_control;

struct net_cmd_block_stu {
    char cmd_str[8];
    uint8_t reply_send_flag;
    uint8_t reply_recv_flag;
    uint32_t reply_recv_timeout;
    uint8_t send_times;
    void (*net_cmd_handle)(uint8_t send_flag, char (*str)[PARAM_LEN]);
};



uint8_t net_cmd_param_strtok(char *str,  char (*ppr)[PARAM_LEN])
{
    char *p;
    char *pp = str;
    uint8_t i = 0;
    p = strchr(pp, ',');
    if(p == NULL) return 0;
    pp = p+1;
    while((p = strchr(pp, ',')) != NULL) {
        memcpy(ppr[i], pp, p-pp);
        if(p - pp == 0) memcpy(ppr[i], "$$", 2);
        pp = p+1;
        i++;   
    }
    p = strchr(str, '#');
    memcpy(ppr[i], pp, p - pp);
    return i+1;
}

void net_cmd_package_send(char *data, uint16_t len)
{
    char *data_str;
    uint16_t lenth = 0, len_str;
    data_str = malloc(512);
    len_str = sprintf(data_str, "*SCOR,OM,");
    lenth += len_str;
    strncat(data_str, gsm_info.imei, strlen(gsm_info.imei));
    lenth += strlen(gsm_info.imei);
    strcat(data_str, ",");
    lenth += 1;
    strncat(data_str, data, len);
    lenth += len;
    strcat(data_str, "#\n");
    lenth += 2;
    LOG_I("%s", data_str);
    net_socket_send((uint8_t *)data_str, lenth);
    free(data_str);
}

static void net_cmd_sign_in_Q0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len;
    if(send_flag == 1) {
         len = sprintf(data_str,"Q0,%d,%d,%d", sys_info.bat_val, car_info.bms_info[0].soc, gsm_info.csq);
         net_cmd_package_send(data_str, len);  
    }
}

static void net_cmd_gsm_heart_H0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] = {0};
    uint16_t len;
    if(send_flag == 1) {
        len = sprintf(data_str,"H0,%d,%d,%d,%d,%d", car_info.lock_sta, \
        sys_info.bat_val, gsm_info.csq, car_info.bms_info[0].soc, car_info.bms_info[0].charge_sta);
        net_cmd_package_send(data_str, len);  
    }
}

static void net_cmd_q_lock_control_R0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len;
    if(send_flag) {
        net_lock_control.rand_key = (uint8_t)random();
        LOG_I("%d", net_lock_control.rand_key);
        len = sprintf(data_str, "R0,%d,%d,%s,%lld",net_lock_control.lock_operate,\
        net_lock_control.rand_key,net_lock_control.user_id,net_lock_control.timestamp);
        net_cmd_package_send(data_str, len);
    } else {
        memset(&net_lock_control, 0, sizeof(net_lock_control));
        net_lock_control.lock_operate = atoi(ppr[0]);
        net_lock_control.key_vaild_time = atoi(ppr[1]);
        memcpy(net_lock_control.user_id, ppr[2], strlen(ppr[2]));
        net_lock_control.timestamp  = (int64_t)atol(ppr[3]);
    }
}

static void net_cmd_lock_open_L0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len;
    uint8_t key;
    int64_t timestamp;
    if(send_flag) {
        len = sprintf(data_str,"L0,%d,%s,%lld", net_lock_control.lock_res,net_lock_control.user_id,net_lock_control.timestamp);
        net_cmd_package_send(data_str, len);
    } else {
        key = (uint8_t)atoi(ppr[0]);
        timestamp = (uint64_t)atol(ppr[2]);
        if((key == net_lock_control.rand_key) && ((timestamp - net_lock_control.timestamp) < net_lock_control.key_vaild_time)) {
            net_lock_control.timestamp = timestamp;
            car_lock_control(NET_CMD_LOCK_SRC, CAR_UNLOCK_ATA);
            if(car_info.lock_sta == CAR_UNLOCK_ATA) {
                net_lock_control.lock_res = 0;
            } else {
                net_lock_control.lock_res = 1;
            }
        } else {
            net_lock_control.lock_res = 2;
        }  
    }
}

static void net_cmd_lock_close_L1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len;
    uint8_t key;
    int64_t timestamp;
    if(send_flag) {
        len = sprintf(data_str,"L1,%d,%s,%lld", net_lock_control.lock_res,net_lock_control.user_id,net_lock_control.timestamp);
        net_cmd_package_send(data_str, len);
    } else {
        key = (uint8_t)atoi(ppr[0]);
        timestamp = (uint64_t)atol(ppr[2]);
        if((key == net_lock_control.rand_key) && ((timestamp - net_lock_control.timestamp) < net_lock_control.key_vaild_time)) {
            net_lock_control.timestamp = timestamp;
            car_lock_control(NET_CMD_LOCK_SRC, CAR_LOCK_STA);
            if(car_info.lock_sta == CAR_LOCK_STA) {
                net_lock_control.lock_res = 0;
            } else {
                net_lock_control.lock_res = 1;
            }
        } else {
            net_lock_control.lock_res = 2;
        }  
    }    
}

static void net_cmd_iot_dev_config_S5_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len;
    uint8_t car_heart_sw;
    uint16_t car_heart_intrval;
    uint16_t net_heart_interval;
    if(send_flag) {
        car_heart_sw = sys_param_set.unlock_car_heart_sw?2:1;
        net_heart_interval = sys_param_set.net_heart_interval;
        car_heart_intrval = sys_param_set.unlock_car_heart_interval;
        len = sprintf(data_str,"S5,%d,%d,%d",car_heart_sw,net_heart_interval,car_heart_intrval);
        net_cmd_package_send(data_str, len);
    } else {
        car_heart_sw = atoi(ppr[0]);
        net_heart_interval = atoi(ppr[1]);
        car_heart_intrval = atoi(ppr[2]);
        if(car_heart_sw) {
            sys_param_set.unlock_car_heart_sw = car_heart_sw == 1?0:1;
        } 
        if(net_heart_interval) {
            sys_param_set.net_heart_interval = net_heart_interval;
        }
        if(car_heart_intrval) {
            sys_param_set.unlock_car_heart_interval = car_heart_intrval;
        }
        sys_info.sys_updata_falg |= 0x01;
    }
}

static void net_cmd_q_car_info_S6_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "S6,");
        lenth += len;
        len = sprintf(&data_str[len], "%d,%d,%d,%d,", gsm_info.csq,car_info.lock_sta,car_info.bms_info[0].charge_sta,car_info.bms_info[0].soc);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d,%ld,", car_info.bms_info[0].max_temp, car_info.hmi_info.power_on,0,car_info.total_odo/10);
        lenth += len;
        len = sprintf(&data_str[lenth], "%ld,%d,%d,%d,",car_info.single_odo/10, car_info.remain_odo,car_info.speed,car_info.avg_speed);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%ld,%d,%d,",car_info.max_speed,car_info.calorie,car_info.atmosphere_light_info.brightness_val,car_info.atmosphere_light_info.turn_linght_sta);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d,%d,",car_info.atmosphere_light_info.custom_red,car_info.atmosphere_light_info.custom_green,car_info.atmosphere_light_info.custom_blue,\
        sys_info.ble_connect);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d", car_info.bms_info[0].pack_vol*10,car_info.gear,car_info.headlight_sta);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
   }
}

static void net_cmd_car_config_S7_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "S7,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d,%d,", car_set_save.gear,car_set_save.head_light,car_set_save.jump_password,car_set_save.atmosphere_light_set.light_mode);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d,%d,",car_set_save.atmosphere_light_set.color,car_set_save.atmosphere_light_set.brightness_val,\
        car_set_save.atmosphere_light_set.turn_linght_sta,car_set_save.atmosphere_light_set.custom_red);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,%d,%d",car_set_save.atmosphere_light_set.custom_green,car_set_save.atmosphere_light_set.custom_blue,car_set_save.atmosphere_light_set.ble_sta);
        lenth += len;
        net_cmd_package_send(data_str, lenth);  
    } else {
         if(!strcmp(ppr[0], "$$")){
            car_set_save.gear = atoi(ppr[0]);
            car_control_cmd(CAR_CMD_SET_GEAR);
        }
        if(!strcmp(ppr[1], "$$")){
            car_set_save.head_light = atoi(ppr[1]);
            car_control_cmd(CAR_CMD_SET_HEADLIGHT);
        }
        if(!strcmp(ppr[2], "$$")){
            if(atoi(ppr[2])) {
                car_set_save.jump_password = 1;
                car_control_cmd(CAR_CMD_JUMP_PASSWORD);
            }
        }
        if(!strcmp(ppr[3], "$$")){
            if(atoi(ppr[3]) >= 0 && atoi(ppr[3]) <= 5){
                car_set_save.atmosphere_light_set.light_mode = atoi(ppr[3]);
                car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_MODE);
            }
        }
        if(!strcmp(ppr[4], "$$")) {
            car_set_save.atmosphere_light_set.color = atoi(ppr[4]);
            car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLORTYPE);
        }
        if(!strcmp(ppr[5], "$$")){
            car_set_save.atmosphere_light_set.brightness_val = atoi(ppr[5]);
            car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL);
        }
        if(!strcmp(ppr[6], "$$")) {
            car_set_save.atmosphere_light_set.turn_linght_sta = atoi(ppr[6]);
            car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_TURN);  
        } 
        if(!strcmp(ppr[7], "$$")) {
            car_set_save.atmosphere_light_set.custom_red = atoi(ppr[7]);
        }
        if(!strcmp(ppr[8], "$$")) {
            car_set_save.atmosphere_light_set.custom_green = atoi(ppr[8]);
        }
        if(!strcmp(ppr[9], "$$")) {
            car_set_save.atmosphere_light_set.custom_blue = atoi(ppr[9]);
        }
        car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM);  
        if(!strcmp(ppr[10], "$$")) {
           car_set_save.atmosphere_light_set.ble_sta = atoi(ppr[10]);
        }
    }
}

static void net_cmd_alarm_up_W0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "W0,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", car_info.move_alarm);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } 
}

static void net_cmd_voice_play_V0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    static uint8_t voice_set;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "V0,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", voice_set);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } else {
        voice_set = atoi(ppr[0]);
        if(voice_set == 2) {
            car_set_save.look_car_sw = 1;
        } else if(voice_set == 0x80) {
            car_set_save.voiceCloseSw = 1;
        } else if(voice_set == 0x81) {
            car_set_save.voiceCloseSw = 0;
        }
    }
}

void gps_data_net_up()
{
/*    char data[156] = {0};
    uint16_t len = 0, lenth = 0;
    len = sprintf(data, "D0,%d,", GPS_MODE);
    lenth += len;
    if(gps_info.RefreshFlag && gps_info.GPSValidFlag == 'A') {
        len = sprintf(data+ lenth, "%s,", gps_info.Time1);
        lenth += len;
        len = sprintf(data + lenth, "%c,", gps_info.GPSValidFlag);
        lenth += len;
        len = sprintf(data + lenth, "%s,", gps_info.LatLongData);
        lenth += len;
        len = sprintf(data + lenth, "%s,", gps_info.SateNumStr);
        lenth += len;
        len = sprintf(data + lenth, "%s,", gps_info.HDOP);
        lenth += len;
        len = sprintf(data + lenth, "%s,", gps_info.Time2);
        lenth += len;
        len = sprintf(data + lenth, "%s,", gps_info.SeaLevelH);
        lenth += len;
        len = sprintf(data + lenth, "%s", gps_info.Mode);
        lenth += len;
    } else {
        len = sprintf(data + lenth, "%s", ",V,,,,,,,,,,N");
        lenth += len;
    }
    LOG_I("data:%s, len:%d", data, lenth);
    net_cmd_package_send(data, lenth);*/
}

static void net_cmd_q_location_D0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    if(send_flag) {
        gps_data_net_up();
    } else {
      //  GPS_MODE = GPS_SINGAL;
    }
}

static void net_cmd_location_track_D1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "D1,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", car_set_save.gps_track_interval);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } else {
        car_set_save.gps_track_interval = atoi(ppr[0]);
        if(car_set_save.gps_track_interval) {
            rtc_event_register(GPS_TRACK_EVENT, car_set_save.gps_track_interval, 1);
        } else{
            rtc_event_unregister(GPS_TRACK_EVENT);
        }
 //       GPS_MODE = GPS_TRACK;
    }
}

static void net_cmd_q_ver_G0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "G0,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", SOFTVER);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", __DATE__);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", car_info.control_soft_ver);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", car_info.hmi_info.soft_ver);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", car_info.bms_info[0].soft_ver);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", car_info.bms_info[0].soft_ver);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", car_info.electronic_lock.soft_ver);
        lenth += len;
        len = sprintf(&data_str[lenth], "%s,", ble_info.ver);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } 
}

static void net_cmd_up_hmi_fault_E0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "E0,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", car_info.hmi_info.fault_code);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    }
}

static void net_cmd_up_control_fault_E1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "E1,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", car_info.fault_code);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    }
}

static void net_cmd_up_bms_fault_E2_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "E2,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,", car_info.bms_info[0].fault_code);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", car_info.bms_info[1].fault_code);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    }
}

static void net_cmd_event_notifi_S1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    static uint8_t event;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "S1,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d", event);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } else {
        event = atoi(ppr[0]);
        if(event == 1) {
            sys_set_var.sys_poweroff_flag = 1;
        } else if(event == 2) {
            sys_set_var.sys_reboot_flag = 1;
        }
    }
}

static void net_cmd_power_on_off_L3_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    if(send_flag) {
        len = sprintf(&data_str[lenth], "L3,");
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,", sys_set_var.car_power_en);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,", 0);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,", sys_set_var.iot_active);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,", sys_set_var.hid_lock_sw);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    } else {
        sys_set_var.car_power_en = atoi(ppr[0]);
        sys_set_var.ble_bind_infoClean = atoi(ppr[1]);
        sys_set_var.iot_active = atoi(ppr[2]);
        sys_set_var.hid_lock_sw = atoi(ppr[3]);
    }
}



static void net_cmd_trans_can_Z0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    uint64_t can_data;
    if(send_flag){
        len = sprintf(&data_str[lenth], "Z0,");
        lenth +=len;
        len = sprintf(&data_str[lenth], "%ld,", trans_can_control.rq_can_id);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%2d%2d%2d%2d%2d%2d%2d%2d", trans_can_control.rq_data[0], trans_can_control.rq_data[1],trans_can_control.rq_data[2],\
        trans_can_control.rq_data[3], trans_can_control.rq_data[4], trans_can_control.rq_data[5], trans_can_control.rq_data[6], trans_can_control.rq_data[7]);
        lenth +=len;
        net_cmd_package_send(data_str, lenth);
    } else {
        trans_can_control.send_can_id = atoi(ppr[0]);
        trans_can_control.send_flag = 1;
        trans_can_control.src = CAN_NET_TRANS;
        can_data = atoi(ppr[1]);
        trans_can_control.send_data[0] = can_data&&0xff;
        trans_can_control.send_data[1] = (can_data >> 8)&&0xff;
        trans_can_control.send_data[2] = (can_data >> 16)&&0xff;
        trans_can_control.send_data[3] = (can_data >> 24)&&0xff;
        trans_can_control.send_data[4] = (can_data >> 32)&&0xff;
        trans_can_control.send_data[5] = (can_data >> 40)&&0xff;
        trans_can_control.send_data[6] = (can_data >> 48)&&0xff;
        trans_can_control.send_data[7] = (can_data >> 56)&&0xff;
        trans_can_control.rq_can_id = atoi(ppr[2]);
        iot_can_trans_func(trans_can_control.rq_can_id, trans_can_control.send_data, 0);
    }
}


static void net_cmd_startup_http_upgrade_U5_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[256] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 
    static uint8_t updata_type1, updata_type2;
    if(send_flag) {
        len = sprintf(&data_str[lenth], "U5,");
        lenth +=len;
        len = sprintf(&data_str[lenth], "%d,", http_upgrade_info.req_type);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%d,", http_upgrade_info.timeout);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%s,", http_upgrade_info.url);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%d,", updata_type1);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%d,0,", updata_type2);
        lenth +=len;
        len = sprintf(&data_str[lenth], "%lu,", http_upgrade_info.crc_sum);
        lenth +=len;
        net_cmd_package_send(data_str, lenth);
    } else {
        http_upgrade_info.req_type = atoi(ppr[0]);
        http_upgrade_info.timeout = atoi(ppr[1]);
        memset(http_upgrade_info.url, 0x00, sizeof(http_upgrade_info.url));
        memcpy(http_upgrade_info.url, &ppr[2], strlen(ppr[2]));
        updata_type1 = atoi(ppr[3]);
        updata_type2 = atoi(ppr[4]);
        if(atoi(ppr[3]) == 1) {
            switch(atoi(ppr[4])){
                case 1:
                    http_upgrade_info.farme_type = ECU_FIRMWARE_TYPE;
                break;
                case 2:
                    http_upgrade_info.farme_type = BMS1_FIRMWARE_TYPE;
                break;
                case 3:
                    http_upgrade_info.farme_type = BMS2_FIRMWARE_TYPE;
                break;
                case 10:
                    http_upgrade_info.farme_type = HMI_FIRMWARE_TYPE;
                break;
                case 33:
                    http_upgrade_info.farme_type = LOCK_FIRMWARE_TYPE;
                break;
            }  
        } else {
            switch(atoi(ppr[3])) {
                case 0:
                    http_upgrade_info.farme_type = IOT_FIRMWARE_TYPE;
                break;
                case 2:
                    http_upgrade_info.farme_type = BLUE_FIRMWARE_TYPE;
                break;
                case 3:
                    http_upgrade_info.farme_type = VOICE_PACK_TYPE;
                break;
                case 4:
                    http_upgrade_info.farme_type = FENCE_FILE_TYPE;
                break;  
                case 5:
                    http_upgrade_info.farme_type = GPS_FIRMWARE_TYPE;
                break;
            }  
        }
        http_upgrade_info.crc_sum = atoi(ppr[6]);
        def_rtos_smaphore_release(http_upgrade_info.http_ota_sem);
    }
}

static void net_cmd_http_upgrade_state_U6_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{
    char data_str[128] ={0};
    uint16_t len = 0;
    uint16_t lenth = 0; 

    if(send_flag) {
        len = sprintf(&data_str[lenth], "U6,");
        lenth += len;
        switch(http_upgrade_info.farme_type) {
            case IOT_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",0,0);
                lenth += len;
            break;
            case BLUE_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",2,0);
                lenth += len;
            break;
            case VOICE_PACK_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",3,0);
                lenth += len;
            break;
            case FENCE_FILE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",4,0);
                lenth += len;
            break;
            case GPS_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",5,0);
                lenth += len;
            break;
            case ECU_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",1,1);
                lenth += len;
            break;
            case BMS1_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",1,2);
                lenth += len;
            break;
            case BMS2_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",1,3);
                lenth += len;
            break;
            case HMI_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",1,10);
                lenth += len;
            break;
            case LOCK_FIRMWARE_TYPE:
                len = sprintf(&data_str[lenth], "%d,%d,",1,33);
                lenth += len;
            break;
        }
        len = sprintf(&data_str[lenth], "%d,", http_upgrade_info.ota_sta);
        lenth += len;
        len = sprintf(&data_str[lenth], "%d,",http_upgrade_info.download_fail_cent);
        lenth += len;
        len = sprintf(&data_str[lenth], "%ld,",http_upgrade_info.download_start_byte);
        lenth += len;
        net_cmd_package_send(data_str, lenth);
    }
}

struct net_cmd_block_stu net_cmd_block_table[] = {
    {"Q0",      false,      false,      0,      0,      net_cmd_sign_in_Q0_func},
    {"H0",      false,      false,      0,      0,      net_cmd_gsm_heart_H0_func},
    {"R0",      true,       false,      1000,   3,      net_cmd_q_lock_control_R0_func},
    {"L0",      true,       false,      1000,   3,      net_cmd_lock_open_L0_func},
    {"L1",      true,       false,      0,      0,      net_cmd_lock_close_L1_func},
    {"S5",      true,       false,      0,      0,      net_cmd_iot_dev_config_S5_func},
    {"S6",      true,       false,      0,      0,      net_cmd_q_car_info_S6_func},
    {"S7",      true,       false,      0,      0,      net_cmd_car_config_S7_func},
    {"W0",      true,       false,      0,      0,      net_cmd_alarm_up_W0_func},
    {"V0",      true,       false,      0,      0,      net_cmd_voice_play_V0_func},
    {"D0",      true,       false,      0,      0,      net_cmd_q_location_D0_func},
    {"D1",      true,       false,      0,      0,      net_cmd_location_track_D1_func},
    {"G0",      true,       false,      0,      0,      net_cmd_q_ver_G0_func},
    {"E0",      true,       false,      0,      0,      net_cmd_up_hmi_fault_E0_func},
    {"E1",      true,       false,      0,      0,      net_cmd_up_control_fault_E1_func},
    {"E2",      true,       false,      0,      0,      net_cmd_up_bms_fault_E2_func},
    {"S1",      true,       false,      0,      0,      net_cmd_event_notifi_S1_func},
    {"L3",      true,       false,      0,      0,      net_cmd_power_on_off_L3_func},
    {"Z0",      false,      false,      0,      0,      net_cmd_trans_can_Z0_func},
    {"U5",      true,       false,      0,      0,      net_cmd_startup_http_upgrade_U5_func},
    {"U6",      true,       false,      0,      0,      net_cmd_http_upgrade_state_U6_func},
};

void net_protocol_cmd_send(uint8_t cmd)
{
    net_cmd_block_table[cmd].net_cmd_handle(1, NULL);
}


static void net_recv_cmd_handle(char *data, uint16_t len)
{
    char *ptr;
    uint8_t i;
    char par_str[15][PARAM_LEN] = {0};
    LOG_I("[%d]]%s", len, data);
    ptr = strstr(data, "*SCOS");
    if(ptr == NULL) return;
    ptr = strstr(data, gsm_info.imei);
    if(ptr == NULL) return;
    for(i = 0; i < ARRAY_SIZE(net_cmd_block_table); i++){
        ptr = strstr(data, net_cmd_block_table[i].cmd_str);
        if(ptr != NULL) {
            if(net_send_cmd_con.send_flag == 1 && net_send_cmd_con.cmd == i) {
                net_send_cmd_con.ask_falg = 1;
                return;
            }
            LOG_I("%s", ptr);
            net_cmd_param_strtok(ptr, par_str);
            net_cmd_block_table[i].net_cmd_handle(0, par_str);
            if(net_cmd_block_table[i].reply_send_flag) {
                NET_CMD_MARK(i);
            }
            break;
        }
    }
}

void net_recv_data_parse(uint8_t *data, uint16_t len)
{
    char *str;
    uint16_t lenth;
    while(len > 0){
        str = strchr((char *)data, '\n');
        if(str == NULL) {
            LOG_E("recv is error");
            break;
        } else {
            lenth = (str - (char *)data) + 1;
            net_recv_cmd_handle((char *)data, lenth);
            data += lenth;
            len -= lenth;
        }
    }
}

void net_protocol_init()
{
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_queue_create(&net_protocol_send_que, sizeof(uint8_t), 12);
    if(err != RTOS_SUCEESS) {
        LOG_E("net_protocol_send_que is create fail err:%d", err);
    }
    LOG_I("net_protocol_init  is ok");
}

void NET_CMD_MARK(uint8_t cmd)
{
    def_rtos_queue_release(net_protocol_send_que, sizeof(uint8_t), &cmd,  RTOS_WAIT_FOREVER);
}

void net_protocol_send_thread(void *param)
{
    uint8_t net_cmd;
    def_rtosStaus err = RTOS_SUCEESS;
    int64_t time_t;
    while(1){
        net_send_cmd_con.send_flag = 0;
        err = def_rtos_queue_wait(net_protocol_send_que, &net_cmd, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(err != RTOS_SUCEESS) continue; 
        net_send_cmd_con.ask_falg = 0;
        net_send_cmd_con.cent = 0;
        net_send_cmd_con.cmd = net_cmd;
        net_send_cmd_con.send_flag  = 1;
        time_t = def_rtos_get_system_tick();
        net_protocol_cmd_send(net_cmd);
        net_send_cmd_con.cent++;
        for(;;){
            if(net_cmd_block_table[net_cmd].reply_recv_flag) {
                if(net_send_cmd_con.ask_falg) break;
                if(def_rtos_get_system_tick() - time_t >= net_cmd_block_table[net_cmd].reply_recv_timeout) {
                    if(net_send_cmd_con.cent >= net_cmd_block_table[net_cmd].send_times) {
                        break;
                    }
                    net_protocol_cmd_send(net_cmd);
                    net_send_cmd_con.cent++;
                    time_t = def_rtos_get_system_tick();
                }
                def_rtos_task_sleep_ms(5);
            } else {
                break;
            }
        }   
    }
    def_rtos_task_delete(NULL); 
}