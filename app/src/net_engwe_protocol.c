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
    data_lenth = 54;
    u16_big_to_litel_end_sw(&p[lenth], data_lenth);
    lenth += 2;
    memcpy(&p[lenth], gsm_info.iccid, 20);
    lenth += 20;
    memcpy(&p[lenth], sys_config.sn, 15);
    lenth += 15;
    memcpy(&p[lenth], ble_info.mac_str, 17);
    lenth += 17;
    return lenth;
}

static uint16_t net_engwe_cmdId_gps_info(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[GPS_INFO_CMD]);
    lenth += 4;
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
    u32_big_to_litel_end_sw(&p[lenth], Gps.Long);
    lenth += 4;
    u32_big_to_litel_end_sw(&p[lenth], Gps.Lat);
    lenth += 4;
    return lenth;
}

static uint16_t net_engwe_cmdId_car_state(uint8_t *p)
{
    uint16_t lenth = 0;
    uint16_t data_lenth;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[CAR_STATE_CMD]);
    lenth += 4;
    data_lenth = 10;
    u16_big_to_litel_end_sw(&p[lenth], data_lenth);
    lenth += 2;
    p[lenth++] = (car_info.lock_sta == CAR_UNLOCK_STA) ? 0X01 : 0X02;
    if(car_info.lock_sta == CAR_UNLOCK_STA) {
        p[lenth] = car_info.car_unlock_state;
    } else {
        p[lenth] =car_info.car_lock_state;
    }
    lenth++;
    p[lenth++] = car_info.filp_state;
    p[lenth++] = 0;//充电状态
    p[lenth++] = sys_info.iot_mode;
    p[lenth++] = car_info.bms_info[0].charge_det ? 0x01:0x02;
    p[lenth++] = sys_info.sheepfang_sta;
    p[lenth++] = sys_info.fence_sta;
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
    p[lenth++] = net_info.act;
    p[lenth++] = net_info.fre_band;
    p[lenth++] = net_info.csq;
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
        p[lenth++] = bms_num;
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
        p[lenth++] = 5;
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
    p[lenth++] = 30;
    u16_big_to_litel_end_sw(&p[lenth], car_info.speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.avg_speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.max_speed);
    lenth += 2;
    u32_big_to_litel_end_sw(&p[lenth], car_info.cycle_time_s);
    lenth += 4;
    u32_big_to_litel_end_sw(&p[lenth], car_info.cycle_total_time_h);
    lenth += 4;
    u32_big_to_litel_end_sw(&p[lenth], car_info.single_odo);
    lenth += 4;
    u16_big_to_litel_end_sw(&p[lenth], car_info.remain_odo);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], (uint16_t)car_info.total_odo);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.ebike_calorie);
    lenth += 2;
    car_info.pedal_speed *= 10;
    u16_big_to_litel_end_sw(&p[lenth], car_info.pedal_speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.m_agv_pedal_speed);
    lenth += 2;
    u16_big_to_litel_end_sw(&p[lenth], car_info.total_agv_pedal_speed);
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
    puf = malloc(512);
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
    iot_mqtt_public(puf, puf_len);
    free(puf);
}

static void net_engwe_pack_up(uint8_t cmd_type, uint8_t *cmd_data, uint16_t cmd_len)
{
    uint8_t *puf;
    uint16_t puf_len = 0;
    uint32_t timesp;
    puf = malloc(512);
    if(puf == NULL){
        return;
    }
    puf[puf_len++] = UP_HEARDER_H;
    puf[puf_len++] = UP_HEARDER_L;
    puf[puf_len++] = PROTOCOL_VER;
    puf[puf_len++] = cmd_type;
    memcpy(&puf[puf_len], gsm_info.imei, 15);
    puf_len += 15;
    puf[puf_len++] = (cmd_len + 25)>>8;
    puf[puf_len++] = (cmd_len + 25)&0xff;
    memcpy(&puf[puf_len], cmd_data, cmd_len);
    puf_len += cmd_len;
    timesp = (uint32_t)hal_drv_rtc_get_timestamp();
    puf[puf_len++] = (timesp>>24)&0XFF;
    puf[puf_len++] = (timesp>>16)&0XFF;
    puf[puf_len++] = (timesp>>8)&0XFF;
    puf[puf_len++] = (timesp&0XFF);
    iot_mqtt_public(puf, puf_len);
    free(puf);
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
        break;
        case 0x08:
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
    }
}


static uint16_t net_engwe_cmdId_carConfig_query(uint8_t *p)
{
    uint16_t lenth = 0;
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[CAR_CONFIG_CMD]);
    lenth += 4;
    p[lenth++] = 0;
    p[lenth++] = 17;
    p[lenth++] = car_set_save.gear;
    p[lenth++] = car_set_save.head_light;
    p[lenth++] = sys_param_set.auto_power_off_time;
    p[lenth++] = car_set_save.mileage_unit == 0 ? 0x02:0x01;
    p[lenth++] = car_set_save.speed_limit;
    p[lenth++] = car_info.wheel;
    p[lenth++] = 0x00; //转向灯
    p[lenth++] = sys_set_var.hid_lock_sw == 0?0x02:0x01;
    p[lenth++] = sys_set_var.shock_sw == 0?0x02:0x01;
    p[lenth++] = car_set_save.en_power_on_psaaword;
    memcpy(&p[lenth], car_set_save.power_on_psaaword, 4);
    lenth += 4;
    p[lenth++] = sys_set_var.ble_bind_infoClean;



    return lenth;
}

static void net_engwe_cmdId_CarSet(uint8_t *data, uint16_t len, uint16_t seq)
{
    uint16_t lenth;
    uint8_t buf[32];
    if(len != 15){
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    if(data[0] != 0) {
        car_set_save.gear = data[0];
        car_control_cmd(CAR_CMD_SET_GEAR);
    }
    if(data[1] != 0) {
        car_set_save.head_light = data[1] == 0x01 ? 0:1;
        car_control_cmd(CAR_CMD_SET_HEADLIGHT);
    }
    if(data[2] != 0) {
        sys_param_set.auto_power_off_time =  data[2];
        sys_power_off_time_set(sys_param_set.auto_power_off_time);
    }
    if(data[3] != 0) {
        car_set_save.mileage_unit = data[3] == 0x01?1:0;
        car_control_cmd(CAR_CMD_SET_MILEAGE_UNIT);
    }
    if(data[4] != 0){
        car_set_save.speed_limit = data[4];
        car_control_cmd(CAR_CMD_SET_SPEED_LIMIT);
    }
    if(data[5] != 0){

    }
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
    }
    if(data[7] != 0) {
        if(data[7] == 0x01)
            sys_set_var.hid_lock_sw = 1;
        else if(data[7] == 0x02) 
            sys_set_var.hid_lock_sw = 0;
    }
    if(data[8] != 0) {
        if(data[8] == 0x01) {
            sys_set_var.shock_sw = 1;
        }
        else if(data[8] == 0x02){
            sys_set_var.shock_sw = 0;
        }
    }
    if(data[9] != 0) {
        if(data[9] == 0x01) {   
            car_set_save.en_power_on_psaaword = 1;
            car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
        } else if(data[9] == 0x02) {
            car_set_save.en_power_on_psaaword = 0;
            car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
        }
    }
    if(data[10] != 0xff){
        memcpy(&car_set_save.power_on_psaaword[0], &data[0], 4);
    }
    if(data[14] == 0x01){
        ble_cmd_mark(CMD_BLE_DELETE_BIND_INFO);
    }
    lenth = net_engwe_cmdId_carConfig_query(buf);
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
    p[lenth++] = sys_param_set.bms_charge_current;
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
    sys_param_set.bms_charge_soc = data[1];
    sys_param_set.bms_charge_current = data[2];
    sys_set_var.sys_updata_falg |= 1;
    car_control_cmd(CAR_BMS_CHARGE_SOC_SET);
    car_control_cmd(CAR_BMS_CHARGE_CURRENT_SET);
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
    memcpy(&p[lenth], &sys_param_set.net_engwe_state_push_cmdId, 4);
    lenth += 4;
    p[lenth++] = REGULARLY_REPORT_UP;
    memcpy(&p[lenth], &sys_param_set.net_engwe_report_time1_cmdId, 4);
    lenth += 4;
    p[lenth++] = REGULARLY_REPORT2_UP;
    memcpy(&p[lenth], &sys_param_set.net_engwe_report_time2_cmdId, 4);
    lenth += 4;
    p[lenth++] = OPERATION_PUSH_UP;
    memcpy(&p[lenth], &sys_param_set.net_engwe_offline_opearte_push_cmdId, 4);
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
        memcpy(&push_type, &data[2+i*5], 4);
        switch(post_type) {
            case STATUS_PUSH_UP:
            sys_param_set.net_engwe_state_push_cmdId = post_type;
            break;
            case REGULARLY_REPORT_UP:
            sys_param_set.net_engwe_report_time1_cmdId = post_type;
            break;
            case REGULARLY_REPORT2_UP:
            sys_param_set.net_engwe_report_time2_cmdId = post_type;
            break;
            case OPERATION_PUSH_UP:
            sys_param_set.net_engwe_offline_opearte_push_cmdId = post_type;
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
    if(sys_param_set.net_heart_sw & (1 << BLE_CONNECT_PUSH_HEART_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.ble_connect_operate_push_interval);
    lenth += 2;
    if(sys_param_set.net_heart_sw &(1<<BLE_DISCON_PUSH_HERAT_SW)){
        p[lenth++] = 0x01;
    } else {
        p[lenth++] = 0x02;
    }
    u16_big_to_litel_end_sw(&p[lenth], sys_param_set.ble_disconnect_operate_push_interval);
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
    if(len != 25) {
        net_engwe_pack_seq_up(NACK_UP, NULL, 0, seq);
    }
    net_engwe_pack_seq_up(ACK_UP, NULL, 0, seq);
    sys_param_set.net_heart_interval = u16_big_to_litel_end(&data[data_offset]);
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
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<< BLE_CONNECT_PUSH_HEART_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<< BLE_CONNECT_PUSH_HEART_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.ble_connect_operate_push_interval = time_interval;
    }
    if(data[data_offset] == 0x01){
        sys_param_set.net_heart_sw |= 1<< BLE_DISCON_PUSH_HERAT_SW;
    } else if(data[data_offset] == 0x02){
        sys_param_set.net_heart_sw &= ~(1<< BLE_DISCON_PUSH_HERAT_SW);
    }
    data_offset++;
    time_interval = u16_big_to_litel_end(&data[data_offset]);
    data_offset += 2;
    if(time_interval != 0){
        sys_param_set.ble_disconnect_operate_push_interval = time_interval;
    }
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
        lat = (int)(sheepfang_data.circle.center.lat*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lat);
        lenth += 4;
        lon = (int)(sheepfang_data.circle.center.lon*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lon);
        lenth += 4;
        radius = (uint16_t)(sheepfang_data.circle.radius*1000);
        u16_big_to_litel_end_sw(&p[lenth], radius);
        lenth += 2;
    } else {
        p[lenth++] = 0x02;
        p[lenth++] = sheepfang_data.polygon.point_num;
        for(i = 0; i < sheepfang_data.polygon.point_num; i++){
            lat = (int)(sheepfang_data.polygon.p[i].lat*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lat);
            lenth += 4;
            lon = (int)(sheepfang_data.polygon.p[i].lon*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lon);
            lenth += 4;
        }
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
        lat = u32_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.center.lat = lat;
        sheepfang_data.circle.center.lat /= 1000000;
        data_len += 4;
        lon = u32_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.center.lon = lon;
        sheepfang_data.circle.center.lon /= 1000000;
        data_len += 4;
        radius = u16_big_to_litel_end(&data[data_len]);
        sheepfang_data.circle.radius = radius;
        sheepfang_data.circle.radius /= 1000;
        data_len += 2;
        sys_set_var.sys_updata_falg |= 1<< SHEEP_DATA_SAVE;
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
            lat = u32_big_to_litel_end(&data[data_len]);
            sheepfang_data.polygon.p[i].lat = lat;
            sheepfang_data.polygon.p[i].lat /= 1000000;
            data_len += 4;
            lon = u32_big_to_litel_end(&data[data_len]);
            sheepfang_data.polygon.p[i].lon = lon;
            sheepfang_data.polygon.p[i].lon /= 1000000;
            data_len += 4;
        }
        sys_set_var.sys_updata_falg |= 1<< SHEEP_DATA_SAVE;
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
    u32_big_to_litel_end_sw(&p[lenth], CmdIdTable[SHEEPFANG_SET_CMD]);
    lenth += 4; 
    lenth += 2;
    if(forbidden_zone_data.shape_type == CIRCLE) {
        p[lenth++] = 0x01;
        lat = (int)(forbidden_zone_data.circle.center.lat * 1000000);
        u32_big_to_litel_end_sw(&p[lenth], lat);
        lenth += 4;
        lon = (int)(forbidden_zone_data.circle.center.lon*1000000);
        u32_big_to_litel_end_sw(&p[lenth], lon);
        lenth += 4;
        radius = (uint16_t)(forbidden_zone_data.circle.radius*1000);
        u16_big_to_litel_end_sw(&p[lenth], radius);
        lenth += 2;
    } else {
        p[lenth++] = 0x02;
        p[lenth++] = forbidden_zone_data.polygon.point_num;
        for(i = 0; i < forbidden_zone_data.polygon.point_num; i++){
            lat = (int)(forbidden_zone_data.polygon.p[i].lat*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lat);
            lenth += 4;
            lon = (int) (forbidden_zone_data.polygon.p[i].lon*1000000);
            u32_big_to_litel_end_sw(&p[lenth], lon);
            lenth += 4;
        }
    }
    data_len = lenth - 4;
    u16_big_to_litel_end_sw(&p[4], data_len);
    return lenth;
}
/*
static uint16_t net_engwe_cmdId_fault_info(uint8_t *p)
{

}
*/

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
        lat = u32_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.center.lat = lat;
        forbidden_zone_data.circle.center.lat /= 1000000;
        data_len += 4;
        lon = u32_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.center.lon = lon;
        forbidden_zone_data.circle.center.lon /= 1000000;
        data_len += 4;
        radius = u16_big_to_litel_end(&data[data_len]);
        forbidden_zone_data.circle.radius = radius;
        forbidden_zone_data.circle.radius /= 1000;
        data_len += 2;
        sys_set_var.sys_updata_falg |= 1<< FORBIDDEN_DATA_SAVE;
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
            lat = u32_big_to_litel_end(&data[data_len]);
            forbidden_zone_data.polygon.p[i].lat = lat;
            forbidden_zone_data.polygon.p[i].lat /= 1000000;
            data_len += 4;
            lon = u32_big_to_litel_end(&data[data_len]);
            forbidden_zone_data.polygon.p[i].lon = lon;
            forbidden_zone_data.polygon.p[i].lon /= 1000000;
            data_len += 4;
        }
        sys_set_var.sys_updata_falg |= 1<< FORBIDDEN_DATA_SAVE;
    }
    lenth = net_engwe_cmdId_forbidden_info(buf);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, buf, lenth, seq);
}


static void net_engwe_cmdId_param_query(uint32_t cmdId, uint16_t seq)
{
    uint8_t i;
    uint8_t *p;
    uint16_t lenth = 0;
    p = malloc(512);
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

                break;
                case FAULT_CODE_CMD:
                    
                break;
                case IOT_HW_INFO_CMD:

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

                break;
                case RIDE_INV_INFO_CMD:

                break;
            }
        }
    }
    LOG_I("lenth:%d", lenth);
    net_engwe_pack_seq_up(CONFIG_FEEDBACK_UP, p, lenth, seq);
    free(p);
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

                break;
                case FAULT_CODE_CMD:

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

                break;
                case RIDE_INV_INFO_CMD:

                break;
            } 
        }
    }
    net_engwe_pack_up(cmd_type, p, lenth);
    free(p);
}

static void net_engwe_cmdId_handle(uint8_t cmd_type, uint32_t cmdId, uint16_t seq, uint8_t *data, uint16_t len)
{
    LOG_I("cmd_type:%x, cmdId:%0x, seq:%x", cmd_type, cmdId, seq);
    debug_data_printf("cmd_data", data, len);
    switch(cmd_type) {
        case FOTA_DOWN:
            
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
             } else if(cmdId == CmdIdTable[CAN_TRANS_CMD]) {

             } else if(cmdId == CmdIdTable[RIDE_INV_INFO_CMD]) {

             } 
        break;
        case QUERY_INFORMATION_DOWN:  
            net_engwe_cmdId_param_query(cmdId, seq);
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
    if((cmd_len + 4) > len) {
        return;
    }
    net_engwe_cmdId_handle(cmd_type, cmd_id, seq, &p[6], cmd_len - 2);
    start += (cmd_len + 4);
    len -= (cmd_len + 4);
    goto START;
}

void net_engwe_data_parse(uint8_t *data, uint16_t len)
{
    uint16_t data_start = 0, cmd_len;
    uint8_t *p;
    uint8_t cmd;
    uint16_t seq_num;
    debug_data_printf("mqtt_payload_recv", (uint8_t *)data, len);
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
}
def_rtos_queue_t net_engwe_cmd_que;
void NET_ENGWE_CMD_MARK(uint8_t cmd)
{
    if(cmd != STATUS_PUSH_UP && cmd != REGULARLY_REPORT_UP && cmd != REGULARLY_REPORT2_UP && cmd != OPERATION_PUSH_UP) {
        return;
    }
    def_rtos_queue_release(net_engwe_cmd_que, sizeof(uint8_t), &cmd, RTOS_WAIT_FOREVER);
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
        }
    }
    def_rtos_task_delete(NULL);
}

void net_engwe_init()
{
    def_rtos_queue_create(&net_engwe_cmd_que, sizeof(uint8_t), 5);
}