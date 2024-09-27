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
#define PARAM_LEN       20

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
    while((p = strchr(pp, ',')) != NULL){
        memcpy(ppr[i], pp, p-pp);
        pp = p+1;
        i++;   
    }
    p = strchr(str, '#');
    memcpy(ppr[i], pp, p - pp);
    return i+1;
}

void net_cmd_package_send(char *data, uint16_t len)
{
    char data_str[512] = {0};
    uint16_t lenth = 0, len_str;
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
    // char data_str[256] ={0};
    // uint16_t len;
    if(send_flag) {
        // len = sprintf(data_str,"S6,%d,%d,%d,%d,%d,%d,%d,%ld,%ld,%d,%d,%d,%d,%ld,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d,%d",
        //  gsm_info.csq,car_info.lock_sta,car_info.bms_info[0].charge_sta,
        //  car_info.bms_info[0].soc,car_info.bms_info[0].max_temp,car_info.hmi_info.power_on, 0, 
        //  car_info.total_odo/10,car_info.single_odo/10,car_info.remain_odo,car_info.speed, car_info.avg_speed,
        //  car_info.max_speed,car_info.calorie,car_info.atmosphere_light_info.light_mode,
        //  car_info.atmosphere_light_info.color,car_info.atmosphere_light_info.brightness_val,
        //  car_info.atmosphere_light_info.turn_linght_sta, car_info.atmosphere_light_info.custom_red,
        //  car_info.atmosphere_light_info.custom_green,car_info.atmosphere_light_info.custom_blue,
        //  sys_info.ble_connect,car_info.bms_info[0].pack_vol*10,car_info.gear,car_info.headlight_sta);
        //  LOG_I("[%d]%s", len,data_str);
        // net_cmd_package_send(data_str, len);
   }
}

static void net_cmd_car_config_S7_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_alarm_up_W0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_voice_play_V0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_q_location_D0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_location_track_D1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_q_ver_G0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_up_hmi_fault_E0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_up_control_fault_E1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_up_bms_fault_E2_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_check_startup_upgrade_U0_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_q_upgrade_data_U1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_upgrade_success_notifi_U2_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_event_notifi_S1_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_power_on_off_L3_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_startup_http_upgrade_U5_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

static void net_cmd_http_upgrade_state_U6_func(uint8_t send_flag, char (*ppr)[PARAM_LEN])
{

}

struct net_cmd_block_stu net_cmd_block_table[] = {
    {"Q0",      false,      false,      0,      0,      net_cmd_sign_in_Q0_func},
    {"H0",      false,      false,      0,      0,      net_cmd_gsm_heart_H0_func},
    {"R0",      true,       false,       1000,   3,      net_cmd_q_lock_control_R0_func},
    {"L0",      true,       false,       1000,   3,      net_cmd_lock_open_L0_func},
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
    {"U0",      true,       false,      0,      0,      net_cmd_check_startup_upgrade_U0_func},
    {"U1",      true,       false,      0,      0,      net_cmd_q_upgrade_data_U1_func},
    {"U2",      true,       false,      0,      0,      net_cmd_upgrade_success_notifi_U2_func},
    {"S1",      true,       false,      0,      0,      net_cmd_event_notifi_S1_func},
    {"L3",      true,       false,      0,      0,      net_cmd_power_on_off_L3_func},
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
    char par_str[10][PARAM_LEN] = {0};
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