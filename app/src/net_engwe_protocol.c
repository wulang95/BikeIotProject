#include "app_system.h"
#include "hal_drv_rtc.h"
#include "hal_drv_net.h"

#define DBG_TAG         "net_engwe_protocol"

#ifdef NET_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

#define   PROTOCOL_VER      0X01
#define   DOWN_HEADRER_H    0XA5
#define   DOWN_HEADRER_L    0XA6
#define   UP_HEARDER_H      0XA7
#define   UP_HEARDER_L      0XA8

static uint8_t g_msg_invalid;
uint32_t CmdIdTable[CMD_ID_MAX] = { 0x00000001, 0x00000002, 0x00000004, 0x00000008,
                                    0x00000010, 0x00000020, 0x00000040, 0x00000080,
                                    0x00000100, 0x00000200, 0x00000400, 0x00000800,
                                    0x00001000, 0x00002000, 0x00004000, 0x00008000,
                                    0x00010000, 0x00020000, 0x00040000, 0x00080000,
                                    0x00100000, 0x00200000, 0x00400000, 0x00800000};

static uint16_t u16_big_to_litel_end(uint8_t *data)
{
    uint16_t d_16 = 0;
    d_16 = data[0]<<8|data[1];
    return d_16;
}

static uint32_t u32_big_to_litel_end(uint8_t *data)
{
    uint32_t d_32 = 0;
    d_32 = data[0] << 24 | data[1] << 16 | data[2] << 8 | data[3];
    return d_32;
}

static void u16_big_to_litel_end_sw(uint8_t *data, uint16_t d_u16)
{
    data[0] = (d_u16 >> 8) &0xff;
    data[1] = (d_u16) &0xff;
}

static void u32_big_to_litel_end_sw(uint8_t *data, uint32_t d_u32)
{
    data[0] = (d_u32 >> 24) &0xff;
    data[1] = (d_u32 >> 16) &0xff;
    data[2] = (d_u32 >> 8) &0xff;
    data[3] = (d_u32) &0xff;
}

uint16_t net_engwe_cmdId_operate_respos(uint8_t *p, REAL_OPERATE_STU real_operate, uint8_t res, uint8_t fail_reson)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[OPERATE_RES_RETURN]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 6;
    p[lenth++] = real_operate.instruction_param;
    p[lenth++] = real_operate.sub_param;
    p[lenth++] = res;
    p[lenth++] = fail_reson;
    return lenth;
}

static uint16_t net_engwwe_iot_hw_info(uint8_t *p)
{
    uint16_t lenth = 0, data_lenth;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[IOT_HW_INFO_CMD]);
    lenth += 4;
    data_lenth = 60;
    u16_big_to_litel_end_sw(&p[lenth], data_lenth);
    lenth += 2;
    memcpy(&p[lenth], gsm_info.iccid, 20);
    lenth += 20;
    memcpy(&p[lenth], sys_config.sn, 15);
    lenth += 15;
    memcpy(&p[lenth], sys_config.mac_str, 17);
    lenth += 17;
    memcpy(&p[lenth], sys_config.dev_type, 6);
    lenth += 6;
    return lenth;
}

static uint16_t net_engwe_cmdId_gps_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[GPS_INFO_CMD]);
    lenth += 4;
    if(Gps.vaild == 0) {
        p[lenth++] = 0;
        p[lenth++] = 2;
        return lenth;
    }
    p[lenth++] = 0;
    p[lenth++] = 18;
    p[lenth++] = Gps.SateNum;
    p[lenth++] = Gps.hdop;
    u16_big_to_litel_end_sw(&p[lenth], Gps.ground_speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], Gps.direction);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], Gps.high);
    lenth += 2;
    u32_big_to_litel_end_sw(&p[lenth], Gps.Long); //纬度
    lenth += 4;
    u32_big_to_litel_end_sw(&p[lenth], Gps.Lat);  //经度
    lenth += 4;
    return lenth;
}

static uint16_t net_engwe_cmdId_car_state(uint8_t *p)
{
    uint16_t lenth = 0;
    uint16_t data_lenth;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[CAR_STATE_CMD]);
    lenth += 4;
    data_lenth = 15;
    u16_big_to_litel_end_sw(&p[lenth], data_lenth);
    lenth += 2;
    p[lenth++] = (car_info.lock_sta == CAR_UNLOCK_STA) ? 0X01 : 0X02;
    if(car_info.lock_sta == CAR_UNLOCK_STA) {
        p[lenth] = car_info.car_unlock_state;
    } else {
        p[lenth] = car_info.car_lock_state;
    }
    lenth++;
    p[lenth++] = car_info.filp_state;
    p[lenth++] = car_info.charger_state;//充电状态
    p[lenth++] = sys_info.iot_mode;
    p[lenth++] = sys_info.power_sta;
    p[lenth++] = sys_info.sheepfang_sta;
    p[lenth++] = sys_info.fence_sta;
    if(car_info.lock_sta == CAR_LOCK_STA) {
        p[lenth++] = 0X01; //挡位
        p[lenth++] = 0X00;  //里程单位
        p[lenth++] = 0X00;  //车辆限速
        p[lenth++] = 0X00;  //转向灯
        p[lenth++] = 0X01;  //大灯
    } else {
        p[lenth++] = car_info.gear + 1;
        p[lenth++] = car_info.hmi_info.display_unit == 0x00?0x02:0x01;
        p[lenth++] = car_info.speed_limit/10;
        p[lenth++] = 0x00;  //目前没转向灯
        p[lenth++] = car_info.headlight_sta ? 0x02:0x01;
    }
    return lenth;
}


static uint16_t net_engwe_cmdId_net_info(uint8_t *p)
{
    uint16_t lenth = 0;
    NET_NW_INFO net_info;
    net_info = hal_drv_get_operator_info();
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[NET_INFO_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 17;
    u16_big_to_litel_end_sw(&p[lenth], net_info.mcc);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], net_info.mnc);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], net_info.lac);
    lenth += 2;
    u32_big_to_litel_end_sw(&p[lenth], net_info.cid);
    lenth += 4;

    
    p[lenth++] = net_info.net_state;
    p[lenth++] = net_info.act;
    p[lenth++] = net_info.fre_band;
    if(net_info.rsrp <= -113) {      /*待确认*/
        p[lenth++] = 99;
    } else {
        p[lenth++] = (net_info.rsrp+113)/2;
    }
    
    p[lenth++] = net_info.bit_error_rate;
    return lenth;
}

static uint16_t net_engwe_cmdId_bms_info(uint8_t bms_num, uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[BATTRY_INFO_CMD]);
    lenth += 4;
    if(car_info.bms_info[bms_num].connect == 1){
        p[lenth++] = 0;
        p[lenth++] = 20;
        p[lenth++] = 0x01;
        p[lenth++] = bms_num + 1;;
        p[lenth++] = car_info.bms_info[bms_num].soc;
        p[lenth++] = car_info.bms_info[bms_num].soh;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].remain_capacity);
        lenth += 2;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].full_capactity);
        lenth += 2;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].design_capactity);
        lenth += 2;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].pack_vol);
        lenth += 2;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].cycle_number);
        lenth += 2;
        car_info.bms_info[bms_num].charge_interval_time *= 10;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].charge_interval_time);
        lenth += 2;
        p[lenth++] = car_info.bms_info[bms_num].max_temp;
        p[lenth++] = sys_info.bat_soc;
    } else {
        p[lenth++] = 0x00;
        p[lenth++] = 6;
        p[lenth++] = 0x00;
        u16_big_to_litel_end_sw(&p[lenth], sys_info.battry_val);
        lenth += 2;
        p[lenth++] = sys_info.bat_soc;
    }
    return lenth; 
}

static uint16_t net_engwe_cmdId_ride_info(uint8_t *p)
{
    int16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[RIDE_INFO_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 34;
    u16_big_to_litel_end_sw(&p[lenth], car_info.speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.avg_speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.max_speed);
    lenth += 2;
    u32_big_to_litel_end_sw(&p[lenth], car_info.cycle_time_s);
    lenth += 4;
    u32_big_to_litel_end_sw(&p[lenth], car_info.cycle_total_time);
    lenth += 4;
 //   u32_big_to_litel_end_sw(&p[lenth], car_info.single_odo);
    u16_big_to_litel_end_sw(&p[lenth], (uint16_t)car_info.single_odo);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.remain_odo);
    lenth += 2;
    u32_big_to_litel_end_sw(&p[lenth], car_info.total_odo);
    lenth += 4;
    u16_big_to_litel_end_sw(&p[lenth], car_info.ebike_calorie * 10);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], (car_info.pedal_speed * 10));
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.m_agv_pedal_speed * 10);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.total_agv_pedal_speed * 10);
    lenth += 2;
    car_info.motor_power /= 10;
    u16_big_to_litel_end_sw(&p[lenth], car_info.motor_power);
    lenth += 2;
    return lenth;
}


void net_engwe_pack_seq_up(uint8_t cmd_type, uint8_t *cmd_data, uint16_t cmd_len, uint16_t seq_num)
{
    uint8_t *puf;
    uint16_t puf_len = 0;
    uint32_t timesp;
    puf = malloc(1024);
    if(puf == NULL){
        return;
    }
    puf[puf_len++] = UP_HEARDER_H;
    puf[puf_len++] = UP_HEARDER_L;
    puf[puf_len++] = PROTOCOL_VER;
    puf[puf_len++] = cmd_type;
    memcpy(&puf[puf_len], gsm_info.imei, 15);
    puf_len += 15;
    puf[puf_len++] = (cmd_len + 27)>>8;
    puf[puf_len++] = (cmd_len + 27)&0xff;
    if(cmd_data) {
        memcpy(&puf[puf_len], cmd_data, cmd_len);
        puf_len += cmd_len;
    }
    timesp = (uint32_t)hal_drv_rtc_get_timestamp();
    puf[puf_len++] = (timesp>>24)&0XFF;
    puf[puf_len++] = (timesp>>16)&0XFF;
    puf[puf_len++] = (timesp>>8)&0XFF;
    puf[puf_len++] = (timesp&0XFF);
    
    puf[puf_len++] = seq_num>>8;
    puf[puf_len++] = seq_num&0xff;
    if(cmd_type != HEART_UP){
        rtc_event_register(NET_HEART_EVENT, sys_param_set.net_heart_interval, 1);
    }
        
    iot_mqtt_public(puf, puf_len);
    free(puf);
}

static void net_engwe_pack_up(uint8_t cmd_type, uint8_t *cmd_data, uint16_t cmd_len)
{
    uint8_t *puf;
    uint16_t puf_len = 0;
    uint32_t timesp;
    static uint16_t seq = 0;
    puf = malloc(1024);
    if(puf == NULL){
        return;
    }
    puf[puf_len++] = UP_HEARDER_H;
    puf[puf_len++] = UP_HEARDER_L;
    puf[puf_len++] = PROTOCOL_VER;
    puf[puf_len++] = cmd_type;
    memcpy(&puf[puf_len], gsm_info.imei, 15);
    puf_len += 15;
    puf[puf_len++] = (cmd_len + 27)>>8;
    puf[puf_len++] = (cmd_len + 27)&0xff;
    if(cmd_data) {
        memcpy(&puf[puf_len], cmd_data, cmd_len);
        puf_len += cmd_len;
    }
    timesp = (uint32_t)hal_drv_rtc_get_timestamp();
    puf[puf_len++] = (timesp>>24)&0XFF;
    puf[puf_len++] = (timesp>>16)&0XFF;
    puf[puf_len++] = (timesp>>8)&0XFF;
    puf[puf_len++] = (timesp&0XFF);
    puf[puf_len++] = (seq >> 8)&0xff;
    puf[puf_len++] = seq&0xff;
    if(cmd_type != HEART_UP){
        rtc_event_register(NET_HEART_EVENT, sys_param_set.net_heart_interval, 1);
    }
    iot_mqtt_public(puf, puf_len);
    free(puf);
    seq++;
    if(seq >= 3000) seq = 0;
}

static void net_engwe_cmdId_real_operate(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t timeout;
    uint32_t time_stamp;
    CAR_CMD_Q car_cmd_q;
    REAL_OPERATE_STU net_car_control;
    uint8_t buf[64];
    uint16_t buf_len;
    if(len != 7){
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    net_car_control.instruction_param = data[0];
    net_car_control.sub_param = data[1];
    net_car_control.seq = seq;
    timeout = data[2];
    time_stamp = u32_big_to_litel_end(&data[3]);
    if(timeout!= 0) {
        if(hal_drv_rtc_get_timestamp() - time_stamp > timeout) {
            buf_len = net_engwe_cmdId_operate_respos(buf, net_car_control, 0x00, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, net_car_control.seq);   
            return;
        }
    }
    car_cmd_q.net_car_control = net_car_control;
    switch(data[0]){
        case 0x01:
            car_cmd_q.cmd = CAR_CMD_UNLOCK;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0x02:
            car_cmd_q.cmd = CAR_CMD_LOCK;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0x03:
            car_set_save.head_light = 1;
            car_cmd_q.cmd = CAR_CMD_SET_HEADLIGHT;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0x04:
            car_set_save.head_light = 0;
            car_cmd_q.cmd = CAR_CMD_SET_HEADLIGHT;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0x05:
        break;
        case 0x06:
            voice_play_mark(data[1]);
            buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
        break;
        case 0x07:
            if(data[1] == 0x01){
                voice_play_mark(LOOK_CAR_VOICE);
                buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
            } else if(data[1] == 0x02) {   //灯闪烁10s
                car_cmd_q.src = NET_CAR_CMD_SER;
                car_cmd_q.cmd = CAR_LOOK_CAR1;
                CAR_CMD_MARK(car_cmd_q); 
            } else if(data[1] == 0x03){
                car_cmd_q.src = NET_CAR_CMD_SER;
                car_cmd_q.cmd = CAR_LOOK_CAR2;
                CAR_CMD_MARK(car_cmd_q); 
            }
        break;
        case 0x08:
            if(data[1] == 0x01){   //运输模式
                system_enter_ship_mode(car_cmd_q);
            } else if(data[1] == 0x02){   //禁用模式
                app_system_deactive_func();
                buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
            }
        break;
        case 0x09:
            voice_play_off();
            buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
        break;
        case 0x0A:
        break;
        case 0x0B:
        break;
        case 0X0C:
            if(data[1] != 0){
                car_set_save.gear = data[1] - 1;
                car_cmd_q.cmd = CAR_CMD_SET_GEAR;
                car_cmd_q.src = NET_CAR_CMD_SER;
                CAR_CMD_MARK(car_cmd_q); 
            }
        break;
        case 0X0D:
            if(data[1] == 0x01){
                car_set_save.mileage_unit = 1;
            } else if(data[1] == 0x02){
                car_set_save.mileage_unit = 0;
            }
            car_cmd_q.cmd = CAR_CMD_SET_MILEAGE_UNIT;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0X0E:
            car_set_save.speed_limit = data[1] * 10;
            car_cmd_q.cmd = CAR_CMD_SET_SPEED_LIMIT;
            car_cmd_q.src = NET_CAR_CMD_SER;
            CAR_CMD_MARK(car_cmd_q); 
        break;
        case 0X0F:
            ble_cmd_mark(BLE_DELETE_BIND_INDEX);
            ble_cmd_mark(BLE_DISCONNECT_INDEX);
            buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
        break;
        case 0X10:  /*转向灯*/

        break;
        case 0x11:
            buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
            sys_reset();
        break;
    }
}


static uint16_t net_engwe_cmdId_carConfig_query(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[CAR_CONFIG_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 0x0c;
    p[lenth++] = sys_param_set.auto_power_off_time;
    p[lenth++] = car_info.wheel;
    p[lenth++] = sys_param_set.hid_lock_sw ? 0x01:0x02;
    p[lenth++] = sys_param_set.shock_sw ? 0x01:0x02;
    if(car_info.lock_sta == CAR_LOCK_STA) {
        p[lenth++] = 0x00;
    } else {
        p[lenth++] = car_info.hmi_info.passwd_en?0x01:0x02;
    }
    memset(&p[lenth], 0xff, 4);
    lenth += 4;
    p[lenth++] = sys_param_set.total_fence_sw ? 0x01:0x02;
    return lenth;
}

static void net_engwe_cmdId_CarSet(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint16_t lenth = 0;
    uint8_t buf[32];
    if(len != 0x0A) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);

    if(data[0] != 0) {
        sys_param_set.auto_power_off_time =  data[2];
        sys_power_off_time_set(sys_param_set.auto_power_off_time);  //自动关锁，细节还有待讨论
    }
    if(data[1] != 0){  //车辆轮径
        car_info.wheel = data[1];
    }
 /*   
    if(data[6] != 0){
        switch(data[6]) {
            case 0x01:
            car_set_save.right_turn_light = 0;
            car_set_save.left_turn_light = 1;
            car_control_cmd(CAR_CMD_SET_TURN_LIGHT);
            break;
            case 0x02:
            car_set_save.left_turn_light = 0;
            car_set_save.right_turn_light = 1;
            car_control_cmd(CAR_CMD_SET_TURN_LIGHT);
            break;
            case 0x03:
            car_set_save.left_turn_light = 1;
            car_set_save.right_turn_light = 1;
            car_control_cmd(CAR_CMD_SET_TURN_LIGHT);
            break;
            case 0x04:
            car_set_save.left_turn_light = 0;
            car_set_save.right_turn_light = 0;
            car_control_cmd(CAR_CMD_SET_TURN_LIGHT);
            break;
        }
    } */

    if(data[2] != 0) {
        if(data[2] == 0x01)
            sys_param_set.hid_lock_sw = 1;
        else if(data[2] == 0x02) 
            sys_param_set.hid_lock_sw = 0;
    }
    if(data[3] != 0) {
        if(data[3] == 0x01) {
            sys_param_set.shock_sw = 1;
        }
        else if(data[3] == 0x02){
            sys_param_set.shock_sw = 0;
        }
    }
    if(data[4] != 0) {
        if(data[4] == 0x01) {   
            car_set_save.en_power_on_psaaword = 1;
            car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
        } else if(data[4] == 0x02) {
            car_set_save.en_power_on_psaaword = 0;
            car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
        }
    }
    if(data[5] != 0xff){
        memcpy(&car_set_save.power_on_psaaword[0], &data[5], 4);
        car_control_cmd(CAR_CMD_SET_POWER_ON_PASSWORD);
    }

    if(data[9] != 0){
        if(data[9] == 0x01)
            sys_param_set.total_fence_sw = 1;
        else if(data[9] == 0x02) {
            sys_param_set.total_fence_sw = 0;
        }
    }
    SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
    /*配置上报*/
    u32_big_to_litel_end_sw(&buf[lenth], CmdIdTable[CAR_CONFIG_CMD]);
    lenth += 4;
    buf[lenth++] = 0x00;
    buf[lenth++] = 0x0C;
    if(car_info.lock_sta == CAR_LOCK_STA) {
        buf[lenth++] = data[0];     //自动关机时间
        buf[lenth++] = data[1];     //轮径
        buf[lenth++] = data[2];     //无感解锁开关
        buf[lenth++] = data[3];     //振动报警开关
        buf[lenth++] = 0x00;        //密码开关
        if(data[5] != 0xff) {
            memset(&buf[lenth], 0x00, 4);
        } else {
            memset(&buf[lenth], 0xff, 4);
        }
        lenth += 4;
        buf[lenth++] = data[9];    //总围栏开关
    } else {
        buf[lenth++] = data[0]; //自动关机时间
        buf[lenth++] = data[1]; //轮径
        buf[lenth++] = data[2];//无感解锁开关
        buf[lenth++] = data[3]; //振动报警开关
        buf[lenth++] = data[4]; //密码开关
        memset(&buf[lenth], 0xff, 4);
        lenth += 4;
        buf[lenth++] = data[9];
    }
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_bms_charge_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[CHARGE_PARAM_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 4;
    p[lenth++] = sys_param_set.bms_charge_soc;
    if(car_info.quick_charger_det) {
        p[lenth++] = sys_param_set.bms_charge_current;
    } else {
        p[lenth++] = 0;
    }
    return lenth;
}

static void net_engwe_cmdId_bms_charge_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t buf[20];
    uint16_t lenth;
    if(len != 2) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    sys_param_set.bms_charge_soc = data[0];
    sys_param_set.bms_charge_current = data[1];
    SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
    if(car_info.quick_charger_det) {
    //    car_control_cmd(CAR_BMS_CHARGE_SOC_SET);
        car_control_cmd(CAR_BMS_CHARGE_CURRENT_SET);
    }
    
    lenth = net_engwe_cmdId_bms_charge_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_atmosphere_light_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[LIGHT_STATE_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 15;
    p[lenth++] = car_set_save.atmosphere_light_set.brightness_val;
    p[lenth++] = car_set_save.atmosphere_light_set.light_mode;
    p[lenth++] = car_set_save.atmosphere_light_set.color;
    p[lenth++] = 0x00;
    p[lenth++] = car_set_save.atmosphere_light_set.custom_blue;
    p[lenth++] = car_set_save.atmosphere_light_set.custom_green;
    p[lenth++] = car_set_save.atmosphere_light_set.custom_red;
    memset(&p[lenth], 0, 6); 
    lenth += 6;
    return lenth;
}

static void net_engwe_cmdId_atmosphere_light_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t buf[20];
    uint16_t lenth;
    if(len != 13) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    car_set_save.atmosphere_light_set.brightness_val = data[0];
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL);
    car_set_save.atmosphere_light_set.light_mode = data[1];
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_MODE);
    car_set_save.atmosphere_light_set.color = data[2];
    car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLORTYPE);
    if(data[3] != 0){
        car_set_save.atmosphere_light_set.custom_blue = data[4];
        car_set_save.atmosphere_light_set.custom_green = data[5];
        car_set_save.atmosphere_light_set.custom_red = data[6];
        car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM);
    }
    lenth = net_engwe_cmdId_atmosphere_light_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_iot_config_query(uint8_t *p)
{
    uint16_t lenth = 0;
    uint16_t data_len;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[IOT_CONFIG_CMD]);
    lenth += 4;
    lenth += 2;
    p[lenth++] = strlen(sys_config.apn);
    memcpy(&p[lenth], sys_config.apn, strlen(sys_config.apn));
    lenth += strlen(sys_config.apn);
    p[lenth++] = strlen(sys_config.apn_usr);
    memcpy(&p[lenth], sys_config.apn_usr, strlen(sys_config.apn_usr));
    lenth += strlen(sys_config.apn_usr);
    p[lenth++] = strlen(sys_config.apn_passwd);
    memcpy(&p[lenth], sys_config.apn_passwd, strlen(sys_config.apn_passwd));
    lenth += strlen(sys_config.apn_passwd);
    p[lenth++] = strlen(sys_config.ip);
    memcpy(&p[lenth], sys_config.ip, strlen(sys_config.ip));
    lenth += strlen(sys_config.ip);
    memcpy(&p[lenth], &sys_config.port, 2);
    lenth += 2;
    memset(&p[lenth], 0, 5);
    lenth += 5;
    data_len = lenth - 4;
    memcpy(&p[4], &data_len, 2);
    return lenth;
}

static void net_engwe_cmdId_iot_config_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t data_len;
    uint16_t port_t;
    uint16_t data_offset = 0;
    uint8_t buf[128];
    uint16_t lenth;
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    data_len = data[data_offset++];
    if(data_len != 0 && data_len <= 39){
        memcpy(sys_config.apn, &data[data_offset], data_len);
        data_offset+=data_len;
    }
    data_len = data[data_offset++];
    if(data_len != 0 && data_len <= 32){
        memcpy(sys_config.apn_usr, &data[data_offset], data_len);
        data_offset += data_len;
    }
    data_len = data[data_offset++];
    if(data_len != 0 && data_len <= 32){
        memcpy(sys_config.apn_passwd, &data[data_offset], data_len);
        data_offset += data_len;
    }
    data_len = data[data_offset++];
    if(data_len != 0 && data_len <= 32){
        memcpy(sys_config.ip, &data[data_offset], data_len);
        data_offset += data_len;
    }
    memcpy(&port_t, &data[data_offset], 2);
    data_offset += 2;
    if(port_t != 0){
        sys_config.port = port_t;
    }
    sys_set_var.sys_updata_falg |= 1<<1;
    lenth = net_engwe_cmdId_iot_config_query(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);

}

static uint16_t net_engwe_cmdId_iot_post_set_query(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[IOT_POST_SET_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 23;
    p[lenth++] = 4;
    p[lenth++] = STATUS_PUSH_UP;
    u32_big_to_litel_end_sw(&p[lenth], sys_param_set.net_engwe_state_push_cmdId);
    lenth += 4;
    p[lenth++] = REGULARLY_REPORT_UP;
    u32_big_to_litel_end_sw(&p[lenth], sys_param_set.net_engwe_report_time1_cmdId);
    lenth += 4;
    p[lenth++] = REGULARLY_REPORT2_UP;
    u32_big_to_litel_end_sw(&p[lenth], sys_param_set.net_engwe_report_time2_cmdId);
    lenth += 4;
    p[lenth++] = OPERATION_PUSH_UP;
    u32_big_to_litel_end_sw(&p[lenth], sys_param_set.net_engwe_offline_opearte_push_cmdId);
    lenth += 4;
    return lenth;
}

static void net_engwe_cmdId_iot_post_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t set_num, i;
    uint8_t post_type;
    uint32_t push_type;
    uint8_t buf[64];
    uint16_t lenth;
    set_num = data[0];
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    for(i = 0; i < set_num; i++){
        post_type = data[1+ i*5];
        push_type =  u32_big_to_litel_end(&data[2+i*5]);
        switch(post_type) {
            case STATUS_PUSH_UP:
            sys_param_set.net_engwe_state_push_cmdId &= push_type;
            sys_param_set.net_engwe_state_push_cmdId |= STATE_PUSH_DEFAULT;
            sys_param_set.net_engwe_state_push_cmdId |= push_type;
            break;
            case REGULARLY_REPORT_UP:
            sys_param_set.net_engwe_report_time1_cmdId = push_type;
            break;
            case REGULARLY_REPORT2_UP:
            sys_param_set.net_engwe_report_time2_cmdId = push_type;
            break;
            case OPERATION_PUSH_UP:
            sys_param_set.net_engwe_offline_opearte_push_cmdId &= push_type;
            sys_param_set.net_engwe_offline_opearte_push_cmdId |= OFFLINE_OPERATE_PUSH_DEFAULT;
            sys_param_set.net_engwe_offline_opearte_push_cmdId |= push_type;
            break;
        }
    }
    sys_set_var.sys_updata_falg |= 1 << 0;
    lenth = net_engwe_cmdId_iot_post_set_query(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_iot_post_set_inv_query(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[IOT_REPORT_INV_SET_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 27;
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.net_heart_interval);
    lenth += 2;
    if(sys_param_set.net_heart_sw & (1 << LOCK_HEART_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.lock_car_heart_interval);
    lenth += 2;
    if(sys_param_set.net_heart_sw &(1 << UNLOCK_HEART_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.unlock_car_heart_interval);
    lenth += 2;
    if(sys_param_set.net_heart_sw & (1<< INTERNAL_BAT_HEART_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u32_big_to_litel_end_sw(&p[lenth], sys_param_set.internal_battry_work_interval);
    lenth += 4;
    if(sys_param_set.net_heart_sw &(1<<LOCK_HEART2_SW)) {
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.lock_car_heart2_interval);
    lenth += 2;
    if(sys_param_set.net_heart_sw & (1<<UNLOCK_HEART2_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.unlock_car_heart2_interval);
    lenth += 2;
    return lenth;
}

static void net_engwe_cmdId_iot_post_inv_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint16_t time_interval;
    uint16_t data_offset = 0;
    uint32_t time_interval32;
    uint8_t buf[64];
    uint16_t lenth;
    if(len != 24) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.net_heart_interval = time_interval;
        rtc_event_register(NET_HEART_EVENT, sys_param_set.net_heart_interval, 1); 
    }
    data_offset += 2;
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<<LOCK_HEART_SW;
    }else if(data[data_offset] == 0x02) {
        sys_param_set.net_heart_sw &= ~(1<< LOCK_HEART_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.lock_car_heart_interval = time_interval;
    }
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<<UNLOCK_HEART_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<<UNLOCK_HEART_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.unlock_car_heart_interval = time_interval;
    }
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<<INTERNAL_BAT_HEART_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<<INTERNAL_BAT_HEART_SW);
    }
    data_offset++;
    time_interval32 = u32_big_to_litel_end(&data[data_offset]);
    data_offset += 4;
    if(time_interval32 != 0){
        sys_param_set.internal_battry_work_interval = time_interval32;
    }
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<<LOCK_HEART2_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<<LOCK_HEART2_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.lock_car_heart2_interval = time_interval;
    }
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<<UNLOCK_HEART2_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<<UNLOCK_HEART2_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.unlock_car_heart2_interval = time_interval;
    }
    regular_heart_update();
    sys_set_var.sys_updata_falg |= 1<<0;
    lenth = net_engwe_cmdId_iot_post_set_inv_query(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_sheepfang_info(uint8_t *p)
{
    uint16_t lenth = 0, data_len, radius;
    uint8_t i;
    int lat, lon;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[SHEEPFANG_SET_CMD]);
    lenth += 4; 
    lenth += 2;
    if(sheepfang_data.shape_type == CIRCLE) {
        p[lenth++] = 0x01;
        lon = (int)(sheepfang_data.circle.center.lon*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lon);
        lenth += 4;
        lat = (int)(sheepfang_data.circle.center.lat*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lat);
        lenth += 4;
        radius = (uint16_t)(sheepfang_data.circle.radius*1000);
        u16_big_to_litel_end_sw(&p[lenth], radius);
        lenth += 2;
    } else if(sheepfang_data.shape_type == POLYGON){
         p[lenth++] = 0x02;
         p[lenth++] = sheepfang_data.polygon.point_num;
        for(i = 0; i < sheepfang_data.polygon.point_num; i++){
            lon = (int)(sheepfang_data.polygon.p[i].lon*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lon);
            lenth += 4;
            lat = (int)(sheepfang_data.polygon.p[i].lat*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lat);
            lenth += 4;  
        }
    } else {
        p[lenth++] = 0x03;
    }
    data_len = lenth - 4;
    u16_big_to_litel_end_sw(&p[4], data_len);
    return lenth;
}

static void net_engwe_cmdId_sheepfang_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint16_t data_len = 0;
    uint8_t i;
    uint8_t buf[256];
    uint16_t lenth,radius;
    int lat, lon;
    if(data[data_len] == 0x01) {
        if(len != 11){
            net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            return;
        }
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        data_len++;
        sheepfang_data.shape_type = CIRCLE;
        lon = u32_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.center.lon = lon;
        sheepfang_data.circle.center.lon /= 1000000;
        data_len += 4;
        lat = u32_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.center.lat = lat;
        sheepfang_data.circle.center.lat /= 1000000;
        data_len += 4;
        radius = u16_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.radius = radius;
        sheepfang_data.circle.radius /= 1000;
        data_len += 2;
        SETBIT(sys_set_var.sys_updata_falg, SHEEP_DATA_SAVE);
    } else if(data[data_len] == 0x02) {
        data_len++;
        sheepfang_data.shape_type = POLYGON;
        sheepfang_data.polygon.point_num = data[data_len++];
        if(sheepfang_data.polygon.point_num > 20 || sheepfang_data.polygon.point_num < 3) {
            net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            return;
        }
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        for(i = 0; i < sheepfang_data.polygon.point_num; i++){
            lon = u32_big_to_litel_end(&data[data_len]);
            sheepfang_data.polygon.p[i].lon = lon;
            sheepfang_data.polygon.p[i].lon /= 1000000;
            data_len += 4;
            lat = u32_big_to_litel_end(&data[data_len]);
            sheepfang_data.polygon.p[i].lat = lat;
            sheepfang_data.polygon.p[i].lat /= 1000000;
            data_len += 4;
        }
        SETBIT(sys_set_var.sys_updata_falg, SHEEP_DATA_SAVE);
    } else {
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        sheepfang_data.shape_type = FENCE_NONE;
        SETBIT(sys_set_var.sys_updata_falg, SHEEP_DATA_SAVE);
    }
    lenth = net_engwe_cmdId_sheepfang_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

static uint16_t net_engwe_cmdId_forbidden_info(uint8_t *p)
{
    uint16_t lenth = 0, data_len;
    uint8_t i;
    int lat, lon;
    uint16_t radius;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[FORBIDDEN_ZONE_SET_CMD]);
    lenth += 4; 
    lenth += 2;
    if(forbidden_zone_data.shape_type == CIRCLE) {
        p[lenth++] = 0x01;
        lon = (int)(forbidden_zone_data.circle.center.lon*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lon);
        lenth += 4;
        lat = (int)(forbidden_zone_data.circle.center.lat * 1000000);
        u32_big_to_litel_end_sw(&p[lenth], lat);
        lenth += 4;
        radius = (uint16_t)(forbidden_zone_data.circle.radius*1000);
        u16_big_to_litel_end_sw(&p[lenth], radius);
        lenth += 2;
    } else if(forbidden_zone_data.shape_type == POLYGON){
        p[lenth++] = 0x02;
        p[lenth++] = forbidden_zone_data.polygon.point_num;
        for(i = 0; i < forbidden_zone_data.polygon.point_num; i++){
            lon = (int) (forbidden_zone_data.polygon.p[i].lon*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lon);
            lenth += 4;
            lat = (int)(forbidden_zone_data.polygon.p[i].lat*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lat);
            lenth += 4;
        }
    } else {
        p[lenth++] = 0x03;
    }
    data_len = lenth - 4;
    u16_big_to_litel_end_sw(&p[4], data_len);
    return lenth;
}

static uint16_t net_engwe_cmdId_fault_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[FAULT_CODE_CMD]);
    lenth += 4;
    p[lenth++] = 0x00;
    p[lenth++] = 0x0e;

    p[lenth++] = (sys_info.car_error >> (8*7))&0xff;
    p[lenth++] = (sys_info.car_error >> (8*6))&0xff;
    p[lenth++] = (sys_info.car_error >> (8*5))&0xff;
    p[lenth++] = (sys_info.car_error >> (8*4))&0xff;
    p[lenth++] = (sys_info.car_error >> (8*3))&0xff;
    p[lenth++] = (sys_info.car_error >> (8*2))&0xff;
    p[lenth++] = (sys_info.car_error >> 8)&0xff;
    p[lenth++] = sys_info.car_error & 0xff;
    u32_big_to_litel_end_sw(&p[lenth], sys_info.iot_error);
    lenth += 4;
    return lenth;
}

static uint16_t net_engwe_ver_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[FIRMWARE_VER_CMD]);
    lenth += 4;
    p[lenth++] = 0x00;
    p[lenth++] = 98;
    memcpy(&p[lenth], sys_config.hw_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], sys_config.soft_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], sys_config.hw_ver, 8);  //蓝牙硬件版本和IOT一致
    lenth += 8;
    memcpy(&p[lenth], ble_info.ver, 8);  //蓝牙硬件版本和IOT一致
    lenth += 8;
    memcpy(&p[lenth], car_info.bms_info[0].hw_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.bms_info[0].soft_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.control_hw_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.control_soft_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.hmi_info.hw_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.hmi_info.soft_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.electronic_lock.hw_ver, 8);
    lenth += 8;
    memcpy(&p[lenth], car_info.electronic_lock.soft_ver, 8);
    lenth += 8;
    return lenth;
}

static uint16_t net_engwe_hw_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[IOT_HW_INFO_CMD]);
    lenth += 4;
    p[lenth++] = 0x00;
    p[lenth++] = 55;
    memcpy(&p[lenth], gsm_info.iccid, 20);
    lenth += 20;
    memcpy(&p[lenth], sys_config.sn, 15);
    lenth += 15;
    memcpy(&p[lenth], ble_info.mac_str, 17);
    lenth += 17;
    memcpy(&p[lenth], sys_config.dev_type, 6);
    lenth += 6;
    return lenth;

}
static uint16_t net_engwe_mqtt_set_info(uint8_t *p)
{
    uint16_t lenth = 0, data_len = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[MQTT_SET_CMD]);
    lenth += 4;
    lenth += 2; //长度
    data_len += 2;

    p[lenth++] = sys_config.mqtt_qos+1;
    data_len++;

    p[lenth++] = strlen(sys_config.mqtt_pub_topic);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_pub_topic, strlen(sys_config.mqtt_pub_topic));
    data_len += strlen(sys_config.mqtt_pub_topic);
    lenth += strlen(sys_config.mqtt_pub_topic);

    p[lenth++] = strlen(sys_config.mqtt_sub_topic);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_sub_topic, strlen(sys_config.mqtt_sub_topic));
    lenth += strlen(sys_config.mqtt_sub_topic);
    data_len += strlen(sys_config.mqtt_sub_topic);

    p[lenth++] = strlen(sys_config.mqtt_client_user);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_client_user, strlen(sys_config.mqtt_client_user));
    lenth += strlen(sys_config.mqtt_client_user);
    data_len += strlen(sys_config.mqtt_client_user);

    p[lenth++] = strlen(sys_config.mqtt_client_pass);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_client_pass, strlen(sys_config.mqtt_client_pass));
    lenth += strlen(sys_config.mqtt_client_pass);
    data_len += strlen(sys_config.mqtt_client_pass);

    p[lenth++] = sys_config.mqtt_will_en? 0x02:0x01;
    data_len++;

    p[lenth++] = strlen(sys_config.mqtt_will_msg);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_will_msg, strlen(sys_config.mqtt_will_msg));
    lenth += strlen(sys_config.mqtt_will_msg);
    data_len += strlen(sys_config.mqtt_will_msg);

    p[lenth++] = strlen(sys_config.mqtt_will_topic);
    data_len++;
    memcpy(&p[lenth], sys_config.mqtt_will_topic, strlen(sys_config.mqtt_will_topic));
    lenth += strlen(sys_config.mqtt_will_topic);
    data_len += strlen(sys_config.mqtt_will_topic);

    u32_big_to_litel_end_sw(&p[4], data_len);
    return lenth;
}

static void net_engwe_cmdId_forbidden_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint16_t data_len = 0;
    uint8_t i;
    uint8_t buf[256];
    uint16_t lenth, radius;
    int lat, lon;

    if(data[data_len] == 0x01) {
        if(len != 11){
            net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            return;
        }
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        data_len++;
        forbidden_zone_data.shape_type = CIRCLE;
        lon = u32_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.center.lon = lon;
        forbidden_zone_data.circle.center.lon /= 1000000;
        data_len += 4;
        lat = u32_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.center.lat = lat;
        forbidden_zone_data.circle.center.lat /= 1000000;
        data_len += 4;
        radius = u16_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.radius = radius;
        forbidden_zone_data.circle.radius /= 1000;
        data_len += 2;
        SETBIT(sys_set_var.sys_updata_falg, FORBIDDEN_DATA_SAVE);
    } else if(data[data_len] == 0x02) {
        data_len++;
        forbidden_zone_data.shape_type = POLYGON;
        forbidden_zone_data.polygon.point_num = data[data_len++];
        if(forbidden_zone_data.polygon.point_num > 20 || forbidden_zone_data.polygon.point_num < 3) {
            net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            return;
        }
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        for(i = 0; i < forbidden_zone_data.polygon.point_num; i++){
            lon = u32_big_to_litel_end(&data[data_len]);
            forbidden_zone_data.polygon.p[i].lon = lon;
            forbidden_zone_data.polygon.p[i].lon /= 1000000;
            data_len += 4;
            lat = u32_big_to_litel_end(&data[data_len]);
            forbidden_zone_data.polygon.p[i].lat = lat;
            forbidden_zone_data.polygon.p[i].lat /= 1000000;
            data_len += 4;
        }
        SETBIT(sys_set_var.sys_updata_falg, FORBIDDEN_DATA_SAVE);
    } else {
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        forbidden_zone_data.shape_type = FENCE_NONE;
        SETBIT(sys_set_var.sys_updata_falg, FORBIDDEN_DATA_SAVE);
    }
    lenth = net_engwe_cmdId_forbidden_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}

void net_engwe_signed()
{
    net_engwe_pack_up(SIGN_IN_UP, NULL, 0);
}

static void net_engwe_cmdId_param_query(uint32_t cmdId, uint16_t seq)
{
    uint8_t i;
    uint8_t *p;
    uint16_t lenth = 0;
    p = malloc(1024);
    if( p == NULL){
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    for(i = 0; i < 32; i++){
        if(cmdId & 1<< i){
            switch(i){
                case GPS_INFO_CMD:
                    lenth += net_engwe_cmdId_gps_info(&p[lenth]);
                break;
                case NET_INFO_CMD:
                    lenth += net_engwe_cmdId_net_info(&p[lenth]);
                break;
                case BATTRY_INFO_CMD:
                    lenth += net_engwe_cmdId_bms_info(0, &p[lenth]);
                    if(car_info.bms_info[1].connect == 1){
                        lenth += net_engwe_cmdId_bms_info(1, &p[lenth]);
                    } 
                break;
                case CHARGE_PARAM_CMD:
                    lenth += net_engwe_cmdId_bms_charge_info(&p[lenth]);
                break;
                case RIDE_INFO_CMD:
                    lenth += net_engwe_cmdId_ride_info(&p[lenth]);
                break;
                case CAR_STATE_CMD:
                    lenth += net_engwe_cmdId_car_state(&p[lenth]);
                break;
                case CAR_CONFIG_CMD:
                    lenth += net_engwe_cmdId_carConfig_query(&p[lenth]);
                break;
                case LIGHT_STATE_CMD:
                    lenth += net_engwe_cmdId_atmosphere_light_info(&p[lenth]);
                break;
                case FIRMWARE_VER_CMD:
                    lenth += net_engwe_ver_info(&p[lenth]);
                break;
                case FAULT_CODE_CMD:
                    lenth += net_engwe_cmdId_fault_info(&p[lenth]);
                break;
                case IOT_HW_INFO_CMD:
                    lenth += net_engwe_hw_info(&p[lenth]);
                break;
                case IOT_CONFIG_CMD:
                    lenth += net_engwe_cmdId_iot_config_query(&p[lenth]);          
                break;
                case OTA_STATE_INFO_CMD:

                break;
                case IOT_POST_SET_CMD:
                    lenth += net_engwe_cmdId_iot_post_set_query(&p[lenth]);
                break;
                case IOT_REPORT_INV_SET_CMD:
                    lenth += net_engwe_cmdId_iot_post_set_inv_query(&p[lenth]);
                break;
                case FORBIDDEN_ZONE_SET_CMD:
                    lenth += net_engwe_cmdId_forbidden_info(&p[lenth]);
                break;
                case SHEEPFANG_SET_CMD:
                    lenth += net_engwe_cmdId_sheepfang_info(&p[lenth]);
                break;
                case MQTT_SET_CMD:
                    lenth += net_engwe_mqtt_set_info(&p[lenth]);
                break;
                case RIDE_INV_INFO_CMD:
                    lenth += net_engwe_cmdId_ride_info(&p[lenth]);
                break;
            }
        }
    }
    LOG_I("lenth:%d", lenth);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, p, lenth, seq);
    free(p);
}

static uint16_t net_engwe_cmdId_bms_health_info( uint8_t bms_num,uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[BMS_HEALTH_CMD]);
    lenth += 4;
    if(car_info.bms_info[bms_num].connect == 0) {
        p[lenth++] = 0x00;
        p[lenth++] = 0x02;
        return lenth;
    }else {
        p[lenth++] = 0x00;
        p[lenth++] = 36;
        p[lenth++] = bms_num+1;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.capacity_input_quantity);
        lenth += 4;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.engwe_input_quantity);
        lenth += 4;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.extreme_temperature_use_time);
        lenth += 4;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.extreme_temperature_charge_time);
        lenth += 4;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.deep_discharge_count);
        lenth += 2;
        p[lenth++] = car_info.bms_info[bms_num].ece_regulation.battery_self_discharge_rate;
        p[lenth++] = car_info.bms_info[bms_num].ece_regulation.engwe_exchange_efficiency;
        u16_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.battery_internal_resistance);
        lenth += 2;
        p[lenth++] = car_info.bms_info[bms_num].ece_regulation.dev_type;
        p[lenth++] = car_info.bms_info[bms_num].ece_regulation.function_support;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.capactity_output_quantity);
        lenth += 4;
        u32_big_to_litel_end_sw(&p[lenth], car_info.bms_info[bms_num].ece_regulation.engwe_output_quantity);
        lenth += 4;
        p[lenth++] = car_info.bms_info[bms_num].manufacture_date.year;
        p[lenth++] = car_info.bms_info[bms_num].manufacture_date.month;
        p[lenth++] = car_info.bms_info[bms_num].manufacture_date.day;
        return lenth;
    }
}

void net_engwe_cmd_push(uint8_t cmd_type, uint32_t info_id)
{
    uint8_t i;
    uint8_t *p;
    uint16_t lenth = 0;
    p = malloc(512);
    if(p == NULL){
        return;
    }
    for(i = 0; i < 32; i++) {
        if(info_id & (1<< i)) {
             switch(i){
                case GPS_INFO_CMD:
                    lenth += net_engwe_cmdId_gps_info(&p[lenth]);
                break;
                case NET_INFO_CMD:
                    lenth += net_engwe_cmdId_net_info(&p[lenth]);
                break;
                case BATTRY_INFO_CMD:
                    lenth += net_engwe_cmdId_bms_info(0, &p[lenth]);
                    if(car_info.bms_info[1].connect == 1){
                        lenth += net_engwe_cmdId_bms_info(1, &p[lenth]);
                    } 
                break;
                case CHARGE_PARAM_CMD:
                    lenth += net_engwe_cmdId_bms_charge_info(&p[lenth]);
                break;
                case RIDE_INFO_CMD:
                    lenth += net_engwe_cmdId_ride_info(&p[lenth]);
                break;
                case CAR_STATE_CMD:
                    lenth += net_engwe_cmdId_car_state(&p[lenth]);
                break;
                case CAR_CONFIG_CMD:
                    lenth += net_engwe_cmdId_carConfig_query(&p[lenth]);
                break;
                case LIGHT_STATE_CMD:
                    lenth += net_engwe_cmdId_atmosphere_light_info(&p[lenth]);
                break;
                case FIRMWARE_VER_CMD:
                    lenth += net_engwe_ver_info(&p[lenth]);
                break;
                case FAULT_CODE_CMD:
                    lenth += net_engwe_cmdId_fault_info(&p[lenth]);
                break;
                case IOT_HW_INFO_CMD:
                    lenth += net_engwwe_iot_hw_info(&p[lenth]);
                break;
                case IOT_CONFIG_CMD:
                    lenth += net_engwe_cmdId_iot_config_query(&p[lenth]);          
                break;
                case OTA_STATE_INFO_CMD:

                break;
                case IOT_POST_SET_CMD:
                    lenth += net_engwe_cmdId_iot_post_set_query(&p[lenth]);
                break;
                case IOT_REPORT_INV_SET_CMD:
                    lenth += net_engwe_cmdId_iot_post_set_inv_query(&p[lenth]);
                break;

                case SHEEPFANG_SET_CMD:
                    lenth += net_engwe_cmdId_sheepfang_info(&p[lenth]);
                break;
                case FORBIDDEN_ZONE_SET_CMD:
                    lenth += net_engwe_cmdId_forbidden_info(&p[lenth]);
                break;
                case MQTT_SET_CMD:
                    lenth += net_engwe_mqtt_set_info(&p[lenth]);
                break;
                case RIDE_INV_INFO_CMD:
                    lenth += net_engwe_cmdId_ride_info(&p[lenth]);
                break;
                case BMS_HEALTH_CMD:
                    lenth += net_engwe_cmdId_bms_health_info(0, &p[lenth]);
                break;
            } 
        }
    }
    net_engwe_pack_up(cmd_type, p, lenth);
    free(p);
}

void net_engwe_fota_state_push(uint8_t ota_state)
{
    uint8_t *buf;
    uint16_t lenth = 0;
    buf = malloc(64);
    if(buf == NULL) {
        return;
    }
    u32_big_to_litel_end_sw(&buf[lenth], CmdIdTable[OTA_STATE_INFO_CMD]);
    lenth += 4;
    buf[lenth++] = 0x00;
    buf[lenth++] = 0x0D;
    switch(sys_param_set.farme_type) {
        case IOT_FIRMWARE_TYPE:
            buf[lenth++] = 0x01;
            buf[lenth++] = 0x00;
        break;
        case BLUE_FIRMWARE_TYPE:
            buf[lenth++] = 0x03;
            buf[lenth++] = 0x00;
        break;
        case MCU_FIRMWARE_TYPE:
            buf[lenth++] = 0x05;
            buf[lenth++] = 0x00;
        break;
        case VOICE_PACK_TYPE1:
            buf[lenth++] = 0x04;
            buf[lenth++] = 0x01;
        break;
        case VOICE_PACK_TYPE2:
            buf[lenth++] = 0x04;
            buf[lenth++] = 0x02;
        break;
        case VOICE_PACK_TYPE3:
            buf[lenth++] = 0x04;
            buf[lenth++] = 0x03;
        break;
        case VOICE_PACK_TYPE4:
            buf[lenth++] = 0x04;
            buf[lenth++] = 0x04;
        break;
        case VOICE_PACK_TYPE5:
            buf[lenth++] = 0x04;
            buf[lenth++] = 0x05;
        break;
        case ECU_FIRMWARE_TYPE:
            buf[lenth++] = 0x02;
            buf[lenth++] = 0x01;
        break;
        case BMS1_FIRMWARE_TYPE:
            buf[lenth++] = 0x02;
            buf[lenth++] = 0x02;
        break;
        case  BMS2_FIRMWARE_TYPE:
            buf[lenth++] = 0x02;
            buf[lenth++] = 0x03;
        break;
        case HMI_FIRMWARE_TYPE:
            buf[lenth++] = 0x02;
            buf[lenth++] = 0x04;
        break;
        case LOCK_FIRMWARE_TYPE:
            buf[lenth++] = 0x02;
            buf[lenth++] = 0x05;
        break;
    }
    memcpy(&buf[lenth], sys_param_set.fw_id, 8);
    lenth += 8;
    buf[lenth++] = ota_state;
    net_engwe_pack_seq_up(STATUS_PUSH_UP, buf, lenth, sys_param_set.ota_seq);
    free(buf);
}


static void net_engwe_fota_deal(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t *buf;
    uint16_t lenth = 0;
    uint8_t res;
    if(data[0] == 0x02) {
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        res = FOTA_SERVE_STOP;
        http_upgrade_stop();
    } else if(data[0] == 0x01 && (len == (16 + data[7])) && (data[7]!= 0)) {
        if(data[1] == 0x02){
            switch(data[2]) {
                case 0x01:
                sys_param_set.farme_type = ECU_FIRMWARE_TYPE;
                break;
                case 0x02:
                sys_param_set.farme_type = BMS1_FIRMWARE_TYPE;
                break;
                case 0x03:
                sys_param_set.farme_type = BMS2_FIRMWARE_TYPE;
                break;
                case 0x04:
                sys_param_set.farme_type = HMI_FIRMWARE_TYPE;
                break;
                case 0x05:
                sys_param_set.farme_type = LOCK_FIRMWARE_TYPE;
                break;
            }
        } else if(data[1] == 0x04){
            switch(data[2]) {
                case 0x01:
                sys_param_set.farme_type = VOICE_PACK_TYPE1;
                break;
                case 0x02:
                sys_param_set.farme_type = VOICE_PACK_TYPE2;
                break;
                case 0x03:
                sys_param_set.farme_type = VOICE_PACK_TYPE3;
                break;
                case 0x04:
                sys_param_set.farme_type = VOICE_PACK_TYPE4;
                break;
                case 0x05:
                sys_param_set.farme_type = VOICE_PACK_TYPE5;
                break;
            }
        } else if(data[1] == 0x01) {
            sys_param_set.farme_type = IOT_FIRMWARE_TYPE;
        } else if(data[1] == 0x03) {
            sys_param_set.farme_type = BLUE_FIRMWARE_TYPE;
        } else if(data[1] == 0x05) {
            sys_param_set.farme_type = MCU_FIRMWARE_TYPE;
        }
        http_upgrade_info.crc_sum = u32_big_to_litel_end(&data[3]);
        if(data[7] > 0) {
            memset(http_upgrade_info.url, 0, 255);
            memcpy(http_upgrade_info.url, &data[8], data[7]);
            LOG_I("URL:%s", http_upgrade_info.url);
        } 
        net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
        res = FOTA_IOT_RECV;
    } else {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    }
    sys_param_set.ota_seq = seq;
    memcpy(&sys_param_set.fw_id[0], &data[len - 8], 8);
    SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
    buf = malloc(64);
    if(buf == NULL){
        return;
    }
    u32_big_to_litel_end_sw(&buf[lenth], CmdIdTable[OTA_STATE_INFO_CMD]);
    lenth +=4;
    buf[lenth++] = 0x00;
    buf[lenth++] = 0x0D;
    buf[lenth++] = data[1];
    buf[lenth++] = data[2];
    memcpy(&buf[lenth], &data[len - 8], 8);
    lenth += 8;
    buf[lenth++] = res;
    net_engwe_pack_seq_up(STATUS_PUSH_UP, buf, lenth, seq);
    http_upgrade_start();
    free(buf);
}


static void net_engwe_mqtt_set(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint8_t *buf;
    uint16_t lenth = 0;
    uint8_t data_lenth;
    buf = malloc(512);
    if(buf == NULL) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    }
    if(data[lenth] != 0) {
        if(data[lenth]>=1 && data[lenth] <= 3) {
            sys_config.mqtt_qos = data[lenth] - 1; 
        } else {
            net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            return;
        }
    }
    lenth++;
    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 64) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0) {
        memset(sys_config.mqtt_pub_topic, 0, sizeof(sys_config.mqtt_pub_topic));
        memcpy(sys_config.mqtt_pub_topic, &data[lenth], data_lenth);
        lenth += data_lenth;
    } 
    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 64) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0) {
        memset(sys_config.mqtt_sub_topic, 0, sizeof(sys_config.mqtt_sub_topic));
        memcpy(sys_config.mqtt_sub_topic, &data[lenth], data_lenth);
        lenth += data_lenth;
    }

    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 32){
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0){
        memset(sys_config.mqtt_client_user, 0, sizeof(sys_config.mqtt_client_user));
        memcpy(sys_config.mqtt_client_user, &data[lenth], data_lenth);
        lenth += data_lenth;
    }
    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 32) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0){
        memset(sys_config.mqtt_client_pass, 0, sizeof(sys_config.mqtt_client_pass));
        memcpy(sys_config.mqtt_client_pass, &data[lenth], data_lenth);
        lenth += data_lenth;
    }
    if(data[lenth] != 0x00){
        sys_config.mqtt_will_en = (data[lenth] == 0x01)?0:1;
    }
    lenth++;
    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 32) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0) {
        memset(sys_config.mqtt_will_msg, 0, sizeof(sys_config.mqtt_will_msg));
        memcpy(sys_config.mqtt_will_msg, &data[lenth], data_lenth);
        lenth += data_lenth;
    }
    data_lenth = data[lenth];
    lenth++;
    if(data_lenth > 64) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
        return;
    } else if(data_lenth > 0) {
        memset(sys_config.mqtt_will_topic, 0, sizeof(sys_config.mqtt_will_topic));
        memcpy(sys_config.mqtt_will_topic, &data[lenth], data_lenth);
        lenth += data_lenth;
    }
    SETBIT(sys_set_var.sys_updata_falg, SYS_CONFIG_SAVE);   
    
    lenth = net_engwe_mqtt_set_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
    free(buf);
}
static void net_engwe_cmdId_handle(uint8_t cmd_type, uint32_t cmdId, uint16_t seq, uint8_t *data, uint16_t len)
{
    LOG_I("cmd_type:%x, cmdId:%0x, len:%d, seq:%x", cmd_type, cmdId, len, seq);
    debug_data_printf("cmd_data", data, len);
    g_msg_invalid = 1;
    switch(cmd_type) {
        case FOTA_DOWN:
            if(cmdId != CmdIdTable[OTA_PARAM_CMD]) {
                net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);  
            } else {
                net_engwe_fota_deal(data, len, seq);
            }
        break;
        case REAL_OPERATION_DOWN:
            LOG_I("CMD:%d, cmdId:%0x", REAL_TIME_OPERATE_CMD, CmdIdTable[REAL_TIME_OPERATE_CMD]);
            if(cmdId != CmdIdTable[REAL_TIME_OPERATE_CMD]) {
                net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            } else {
                net_engwe_cmdId_real_operate(data, len, seq);
            }
        break;
        case CONFIG_INSTRUCTION_DOWN:
             if(cmdId == CmdIdTable[CAR_CONFIG_CMD]) {
                net_engwe_cmdId_CarSet(data, len, seq);
             } else if(cmdId == CmdIdTable[CHARGE_PARAM_CMD]) {
                net_engwe_cmdId_bms_charge_set(data, len, seq);
             }else if(cmdId == CmdIdTable[LIGHT_STATE_CMD]){
                net_engwe_cmdId_atmosphere_light_set(data, len, seq);
             } else if(cmdId == CmdIdTable[IOT_CONFIG_CMD]){
                net_engwe_cmdId_iot_config_set(data, len, seq);
             } else if(cmdId == CmdIdTable[IOT_POST_SET_CMD]) {
                net_engwe_cmdId_iot_post_set(data, len, seq);
             }else if(cmdId == CmdIdTable[IOT_REPORT_INV_SET_CMD]) {
                net_engwe_cmdId_iot_post_inv_set(data, len, seq);
             } else if(cmdId == CmdIdTable[SHEEPFANG_SET_CMD]) {
                net_engwe_cmdId_sheepfang_set(data, len, seq);
             } else if(cmdId == CmdIdTable[FORBIDDEN_ZONE_SET_CMD]) {
                net_engwe_cmdId_forbidden_set(data, len, seq);
             } else if(cmdId == CmdIdTable[MQTT_SET_CMD]) {
                net_engwe_mqtt_set(data, len, seq);
             }
        break;
        case QUERY_INFORMATION_DOWN:  
            if(cmdId != CmdIdTable[QUERY_PARAM_CMD] && len != 4) {
                net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
            } else {
                net_engwe_cmdId_param_query(data[0] << 24 | data[1] << 16 | data[2]<<8| data[3], seq);
            }
        break;
    }
}

static void net_engwe_cmd_handle(uint8_t *data, uint16_t len, uint8_t cmd_type, uint16_t seq)
{
    uint32_t cmd_id;
    uint16_t cmd_len, start = 0;
    uint8_t *p;
START:
    if(len < 6){
        return;
    }
    p = &data[start];
    cmd_id = p[0] << 24 | p[1] << 16 | p[2] << 8| p[3];
    cmd_len = p[4] << 8 | p[5];
    if((cmd_len + 4) > len || cmd_len <= 2) {
        return;
    } 
    net_engwe_cmdId_handle(cmd_type, cmd_id, seq, &p[6], cmd_len - 2);
    start += (cmd_len + 4);
    len -= (cmd_len + 4);
    goto START;
}

void net_engwe_data_parse(uint8_t *data, uint16_t len)
{
 //   uint16_t data_start = 0, cmd_len;
//    uint8_t *p;
    uint8_t cmd;
    uint16_t seq_num, cmd_len;
    debug_data_printf("mqtt_payload_recv", (uint8_t *)data, len);
    g_msg_invalid = 0;
    if(data[0] != DOWN_HEADRER_H || data[1] != DOWN_HEADRER_L) {
        net_engwe_pack_up(NACK_UP, NULL, 0);
        return;
    }
    cmd = data[2];
    cmd_len = data[3] << 8 | data[4];
    if(len != cmd_len){
        net_engwe_pack_up(NACK_UP, NULL, 0);
        return;
    }
    if(cmd_len < 11) return;
    seq_num = data[cmd_len -2] << 8 | data[cmd_len -1];
    net_engwe_cmd_handle(&data[5], cmd_len - 7, cmd, seq_num);
    if(g_msg_invalid != 1) {
        net_engwe_pack_up(NACK_UP, NULL, 0);
    }
    /*
START:   
    if(len < 8) return;
    p = &data[data_start];
    if(p[0] != DOWN_HEADRER_H || p[1] != DOWN_HEADRER_L) {
        len -= 1;
        data_start++;
        goto START;
    }
    cmd = p[2];
    cmd_len = p[3] << 8 | p[4];
    if(len < cmd_len) {
        len -= 2;
        data_start += 2;
        goto START;
    }
    
    seq_num = p[cmd_len -2] << 8 | p[cmd_len -1];
    net_engwe_cmd_handle(&p[5], cmd_len - 7, cmd, seq_num);
    data_start += cmd_len;
    len -= cmd_len;
    goto START;
    if(g_msg_invalid != 1) {
        net_engwe_pack_up(NACK_UP, NULL, 0);
    }*/
}
def_rtos_queue_t net_engwe_cmd_que;
void NET_ENGWE_CMD_MARK(uint8_t cmd)
{
    if(cmd == STATUS_PUSH_UP || cmd == REGULARLY_REPORT_UP || cmd == REGULARLY_REPORT2_UP || cmd == OPERATION_PUSH_UP || cmd == HEART_UP) {
        def_rtos_queue_release(net_engwe_cmd_que, sizeof(uint8_t), &cmd, RTOS_WAIT_FOREVER);
    }
}


void net_engwe_send_thread(void *param)
{
    def_rtosStaus res;
    uint8_t cmd;
    
    while(1) {
        res = def_rtos_queue_wait(net_engwe_cmd_que, (uint8_t *)&cmd, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        switch(cmd) {
            case STATUS_PUSH_UP:
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            break;
            case REGULARLY_REPORT_UP:
                net_engwe_cmd_push(REGULARLY_REPORT_UP, sys_param_set.net_engwe_report_time1_cmdId);
            break;
            case REGULARLY_REPORT2_UP:
                net_engwe_cmd_push(REGULARLY_REPORT_UP, sys_param_set.net_engwe_report_time2_cmdId);
            break;
            case OPERATION_PUSH_UP:
                net_engwe_cmd_push(REGULARLY_REPORT_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
            break;
            case HEART_UP:
                net_engwe_pack_up(HEART_UP, NULL, 0);
            break;
        }
    }
    def_rtos_task_delete(NULL);
}

void net_engwe_init()
{
    def_rtos_queue_create(&net_engwe_cmd_que, sizeof(uint8_t), 5);
}