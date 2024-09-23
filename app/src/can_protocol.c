#include "app_system.h"
#include "hal_drv_rtc.h"

#define DBG_TAG         "can_protocol"

#ifdef CAN_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

def_rtos_queue_t can_tx_que;
static int64_t check_hmi_timeout, check_control_timeout,check_lock_timeout, check_bms_timeout;
struct can_send_md_stu {
    stc_can_rxframe_t  can_tx_frame;
    uint8_t cnt;
    uint8_t ask_flag;
    uint8_t cmd_send;
} can_send_cmd;

struct can_cmd_order_s {
    uint8_t cmd;
    uint8_t need_ask;
    uint8_t max_cnt;
    uint32_t timeout;
};
struct can_control_cmd_stu {
    uint8_t dts;
    uint8_t cmd_code;
    uint8_t cmd_index;
    uint8_t cmd_len;
};
static struct can_control_cmd_stu can_control_cmd_table[] = {
    {0x28,  0x14,   0x01,   1},
    {0x28,  0x14,   0x02,   1},
    {0x28,  0x14,   0x03,   1},
    {0x28,  0x14,   0x04,   1},
    {0x28,  0x14,   0x05,   1},
    {0x28,  0x14,   0x06,   1},
    {0x28,  0x14,   0x07,   1},
    {0x28,  0x14,   0x08,   1},
    {0x28,  0x14,   0x09,   1},
    {0x60,  0x14,   0x01,   1},
    {0xEF,  0x60,   0x55,   2},
    {0X28,  0X60,   0X05,   1},
    {0x28,  0x60,   0x08,   1}, 
    {0X28,  0X60,   0X02,   4},
};


static struct can_cmd_order_s can_cmd_order[] = {
    {0X14,  true,   3,  1000},  /* 控制指令*/
    {0XEA,  true,   3,  1000}, /*  PNG请求*/
    {0x60,  true,   3,  1000},  /*控制参数*/
};

static void hmi_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240){
        switch(pdu.pdu2) {
            case HMI_MATCH_INFO:
                car_info.hmi_info.power_on = data[0]&0x80?1:0;
                car_info.hmi_info.encry_mode_data = data[0]&0x7f;
                car_info.hmi_info.key = data[1];
                car_info.hmi_info.startup_mode = data[3]&0x07;
                car_info.hmi_info.protocol_major_ver = data[4];
                car_info.hmi_info.protocol_sub_ver = data[5];
         //       debug_data_printf("HMI_MATCH_INFO", data, data_len);
            break;
            case HMI_DATA:
                car_info.hmi_info.left_turn_light = (data[7]>>6)&0x01;
                car_info.hmi_info.right_turn_linght = (data[7]>>5)&0x01;
                car_info.hmi_info.look_car_sta = (data[7]>>4)&0x01;
                car_info.hmi_info.fault_code = data[4];
           //     debug_data_printf("HMI_DATA", data, data_len);  
            break;
            case HMI_STA:
                car_info.hmi_info.display_unit = data[1]&0x80?1:0;
                car_info.hmi_info.encry_lock_sta = data[2];
          //      debug_data_printf("HMI_STA", data, data_len);  
            break;
            case HMI_HW_VER1:
                memcpy(&car_info.hmi_info.hw_ver[0], (char *)data, 8);
                LOG_I("hmi_hw_ver1:%s", car_info.hmi_info.hw_ver);
         //       debug_data_printf("HMI_HW_VER1", data, data_len);  
            break;
            case HMI_HW_VER2:
                memcpy(&car_info.hmi_info.hw_ver[8], (char *)data, 8);
                LOG_I("hmi_hw_ver2:%s", car_info.hmi_info.hw_ver);
          //      debug_data_printf("HMI_HW_VER2", data, data_len); 
            break;
            case HMI_SOFT_VER1:
                memcpy(&car_info.hmi_info.soft_ver[0], (char *)data, 8);
                LOG_I("hmi_soft_ver1:%s", car_info.hmi_info.soft_ver);
           //     debug_data_printf("HMI_SOFT_VER1", data, data_len); 
            break;
            case HMI_SOFT_VER2:
                memcpy(&car_info.hmi_info.soft_ver[8], (char *)data, 8);
                LOG_I("hmi_soft_ver2:%s", car_info.hmi_info.soft_ver);
          //      debug_data_printf("HMI_SOFT_VER2", data, data_len); 
            break;
            case HMI_SN1:
                memcpy(&car_info.hmi_info.sn[0], (char *)data, 8);
                LOG_I("hmi_sn1:%s", car_info.hmi_info.sn);
          //      debug_data_printf("HMI_SN1", data, data_len); 
            break;
            case HMI_SN2:
                memcpy(&car_info.hmi_info.sn[8], (char *)data, 8);
                LOG_I("hmi_sn2:%s", car_info.hmi_info.sn);
         //       debug_data_printf("HMI_SN2", data, data_len); 
            break;
            case HMI_SN3:
                memcpy(&car_info.hmi_info.sn[16], (char *)data, 8);
                LOG_I("hmi_sn3:%s", car_info.hmi_info.sn);
          //      debug_data_printf("HMI_SN3", data, data_len); 
            break;
            case HMI_SN4:
                memcpy(&car_info.hmi_info.sn[24], (char *)data, 8);
                LOG_I("hmi_sn4:%s", car_info.hmi_info.sn);
           //     debug_data_printf("HMI_SN4", data, data_len); 
            break;
        }
    }
}

static void control_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240){
        switch(pdu.pdu2){
            case CONTROL_MATCH_INFO:
          //      debug_data_printf("CONTROL_MATCH_INFO", data, data_len);
                car_info.power_sta = data[0]>>7;
                car_info.protocol_major_ver = data[4];
                car_info.protocol_sub_ver = data[5];
                car_info.assist_seneor_type = (data[6]&0xc0)>>6;
            break;
            case CONTROL_DATA1:
                car_info.gear = data[0]&0xf;
                car_info.promote_func = data[0]>>5&3;
                car_info.taillight_sta = data[1]>>6&0x01;
                car_info.headlight_sta = data[1]>>7;
                car_info.speed = data[3]<<8 | data[4];
                car_info.fault_code = data[5];
           //     debug_data_printf("CONTROL_DATA1", data, data_len);
            break;
            case CONTROL_DATA2:
                car_info.current = data[0];
                car_info.pedal_speed = data[1];
                car_info.pedal_torque = data[2];
                car_info.motor_power = data[3] <<8 | data[4];
                car_info.control_torque = data[5];
                car_info.control_temp = data[6];
                car_info.motor_temp = data[7];
             //   debug_data_printf("CONTROL_DATA2", data, data_len);
            break;
            case CONTROL_DATA3:
                car_info.total_odo = data[0]<<16|data[1]<<8|data[2];
                car_info.single_odo = data[3]<<16|data[4]<<8|data[5];
                car_info.remain_odo = data[6] * 10;
                car_info.wheel = data[7];
             //   debug_data_printf("CONTROL_DATA3", data, data_len);
            break;
            case CONTROL_DATA4:
                car_info.avg_speed = data[0] << 8| data[1];
                car_info.max_speed = data[2] << 8| data[3];
                car_info.current_limit = data[7];
            //    debug_data_printf("CONTROL_DATA4", data, data_len);
            break;
            case CONTROL_DATA5:
                car_info.cycle_time_s = data[0]*60 + data[1]*3600;
                car_info.speed_limit = data[3] << 8 | data[4];
                car_info.transfer_data = data[5];
                car_info.motor_speed = data[6]<<8|data[7];
           //     debug_data_printf("CONTROL_DATA5", data, data_len);
            break;
            case CONTROL_DATA6:
                car_info.cycle_power = data[0] << 8 | data[1];
                car_info.motor_consumption = data[2] << 8 | data[3];
                car_info.motor_avg_consumption = data[4] << 8 | data[5];
                car_info.reduce_power_sta = data[6];
                car_info.stop_drive_sta = data[7];
            //    debug_data_printf("CONTROL_DATA6", data, data_len);
            break;
            case CONTROL_DATA7:
                car_info.cycle_avg_power = data[0] << 8 | data[1];
                car_info.motor_avg_power = data[2] << 8 | data[3];
           //     debug_data_printf("CONTROL_DATA7", data, data_len);
            break;
            case CONTROL_DATA8:
                car_info.calorie = data[0] << 16 | data[1] <<8 | data[2];
                car_info.bus_voltage = data[5] << 8 | data[6];
           //     debug_data_printf("CONTROL_DATA8", data, data_len);
            break;
            case CONTROL_HWVER1:
                memcpy(&car_info.control_hw_ver[0], (char *)&data[0], 8);
                LOG_I("control_hw_ver:%s", car_info.control_hw_ver);
          //      debug_data_printf("CONTROL_HWVER1", data, data_len);
            break;
            case CONTROL_HWVER2:
                memcpy(&car_info.control_hw_ver[8], (char *)&data[0], 8);
           //     debug_data_printf("CONTROL_HWVER2", data, data_len);
            break;
            case CONTROL_SOFTVER1:
                memcpy(&car_info.control_soft_ver[0], (char *)&data[0], 8);
                LOG_I("control_soft_ver:%s", car_info.control_soft_ver);
           //     debug_data_printf("CONTROL_SOFTVER1", data, data_len);
            break;
            case CONTROL_SOFTVER2:
                memcpy(&car_info.control_soft_ver[0], (char *)&data[0], 8);
            //    debug_data_printf("CONTROL_SOFTVER2", data, data_len);
            break;
            case CONTROL_SN1:
                memcpy(&car_info.control_sn[0], (char *)&data[0], 8);
                LOG_I("control_sn1:%s", car_info.control_sn);
             //   debug_data_printf("CONTROL_SN1", data, data_len);
            break;
            case CONTROL_SN2:
                memcpy(&car_info.control_sn[8], (char *)&data[0], 8);
                LOG_I("control_sn:%s", car_info.control_sn);
              //  debug_data_printf("CONTROL_SN2", data, data_len);
            break;
            case CONTROL_SN3:
                memcpy(&car_info.control_sn[16], (char *)&data[0], 8);
                LOG_I("control_sn:%s", car_info.control_sn);
              //  debug_data_printf("CONTROL_SN3", data, data_len);
            break;
            case CONTROL_SN4:
                memcpy(&car_info.control_sn[24], (char *)&data[0], 8);
                LOG_I("control_sn:%s", car_info.control_sn);
             //   debug_data_printf("CONTROL_SN4", data, data_len);
            break;
            case CONTROL_PARAMVER1:
                memcpy(&car_info.control_param_ver[0], (char *)&data[0], 8);
           //     debug_data_printf("CONTROL_PARAMVER1", data, data_len);
            break;
            case CONTROL_PARAMVER2:
                memcpy(&car_info.control_param_ver[8], (char *)&data[0], 8);
            //    debug_data_printf("CONTROL_PARAMVER2", data, data_len);
            break;
            case CONTROL_PARAMVER3:
                memcpy(&car_info.control_param_ver[16], (char *)&data[0], 8);
           //     debug_data_printf("CONTROL_PARAMVER3", data, data_len);
            break;
            case CONTROL_CUSTOMERCODE1:
         //   debug_data_printf("CONTROL_CUSTOMERCODE1", data, data_len);
            break;
            case CONTROL_CUSTOMERCODE2:
         //   debug_data_printf("CONTROL_CUSTOMERCODE2", data, data_len);
            break;
        }
    }
}

static void bms_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240) {
        switch(pdu.pdu2){
            case BMS_MATCH_INFO:
                car_info.bms_info[0].protocol_major_ver = data[4];
                car_info.bms_info[0].protocol_sub_ver = data[5];
                car_info.bms_info[0].key = data[1];
            break;
            case BMS_BATTRY_PACK_TEMP:
                car_info.bms_info[0].temp_probe_number = data[0];
                car_info.bms_info[0].cell_temp = data[1];
                car_info.bms_info[0].mos_temp = data[2];
                car_info.bms_info[0].board_temp = data[3];
                car_info.bms_info[0].max_temp = data[4];
                car_info.bms_info[0].min_temp = data[5];
            break;
            case BMS_COMPREHENSIVE_DATA:
                car_info.bms_info[0].soh = data[0];
                car_info.bms_info[0].soc = data[1];
                car_info.bms_info[0].remain_capacity = data[2]<<8 | data[3];
                car_info.bms_info[0].full_capactity = data[4]<<8 | data[5];
                car_info.bms_info[0].design_capactity = data[6]<<8 | data[7];
            break;
            case BMS_CELL_VOL1:
                car_info.bms_info[0].pack_series_number = data[0];
                car_info.bms_info[0].pack_series_number = data[1];
                car_info.bms_info[0].cell_val[0] = data[2] << 8 | data[3];
                car_info.bms_info[0].cell_val[1] = data[4] << 8 | data[5];
                car_info.bms_info[0].cell_val[2] = data[6] << 8 | data[7];
            break;
            case BMS_CELL_VOL2:
                car_info.bms_info[0].cell_val[3] = data[0]<< 8 | data[1];
                car_info.bms_info[0].cell_val[4] = data[2] << 8 | data[3];
                car_info.bms_info[0].cell_val[5] = data[4] << 8 | data[5];
                car_info.bms_info[0].cell_val[6] = data[6] << 8 | data[7];
            break;
            case BMS_CELL_VOL3:
                car_info.bms_info[0].cell_val[7] = data[0]<< 8 | data[1];
                car_info.bms_info[0].cell_val[8] = data[2] << 8 | data[3];
                car_info.bms_info[0].cell_val[9] = data[4] << 8 | data[5];
                car_info.bms_info[0].cell_val[10] = data[6] << 8 | data[7];
            break;
            case BMS_CELL_VOL4:
                car_info.bms_info[0].cell_val[11] = data[0]<< 8 | data[1];
                car_info.bms_info[0].cell_val[12] = data[2] << 8 | data[3];
                car_info.bms_info[0].cell_val[13] = data[4] << 8 | data[5];
                car_info.bms_info[0].cell_val[14] = data[6] << 8 | data[7];
            break;
            case BMS_REALTIME_VOL_CURRENT:
                car_info.bms_info[0].pack_vol = data[0] << 8 | data[1];
                car_info.bms_info[0].pack_current = data[2] << 8 | data[3];
                car_info.bms_info[0].max_continuous_current = data[4];
                car_info.bms_info[0].max_charge_current = data[5];
                car_info.bms_info[0].max_charge_current = data[6];
            break;
            case BMS_FIRST_SENCOND_PROTECTION:
                memcpy(&car_info.bms_info[0].first_protect[0], &data[0], 4);
                memcpy(&car_info.bms_info[0].second_protect[0], &data[4], 4);
            break;
            case BMS_BATTRY_PACK_STA:
                car_info.bms_info[0].charge_det = data[0]&0x01;
                car_info.bms_info[0].charge_sta = (data[0]>>1)&0x01;
                car_info.bms_info[0].discharge_sta = data[1]&0x01;
                car_info.bms_info[0].charge_mos = data[2]&0x01;
                car_info.bms_info[0].discharge_mos = (data[2]>>1)&0x01;
                car_info.bms_info[0].chargefull_sta = (data[2]>>2)&0x01;
                car_info.bms_info[0].double_bms_sta = data[3]&0x03;
                car_info.bms_info[0].charge_remain_time = data[4] << 8 | data[5];
                car_info.bms_info[0].design_val = data[6];
            break;
            case BMS_BATTRY_PACK_RECORDDATA:
                car_info.bms_info[0].cycle_number = data[0] << 8 | data[1];
                car_info.bms_info[0].charge_interval_time = data[2] << 8 | data[3];
                car_info.bms_info[0].maxcharge_interval_time = data[4] << 8 | data[5];
                car_info.bms_info[0].alive_time = data[6] << 8 | data[7];
            break;
            case BMS_BATTRY_PACK_CHARGE_PARAM:
                car_info.bms_info[0].max_charge_val = data[0] << 8 | data[1];
            break;
            case BMS_PROTECTION_FAULT_INFO:
            break;
            case BMS_BARCODE_A:
                car_info.bms_info[0].code_len = data[0];
                memcpy(&car_info.bms_info[0].code[0], &data[1], 7);
            break;
            case BMS_BARCODE_B:
                memcpy(&car_info.bms_info[0].code[7], &data[0], 8);
            break;
            case BMS_BARCODE_C:
                memcpy(&car_info.bms_info[0].code[15], &data[0], 8);
            break;
            case BMS_BARCODE_D:
                memcpy(&car_info.bms_info[0].code[23], &data[0], 8);
            break;
            case BMS_SOFT_VER:
                memcpy(&car_info.bms_info[0].soft_ver[0], &data[0], 8);
            break;
            case BMS_SOFT_VER_EXTEND1:
                memcpy(&car_info.bms_info[0].soft_ver[8], &data[0], 8);
            break;
            case BMS_SOFT_VER_EXTEND2:
                memcpy(&car_info.bms_info[0].soft_ver[16], &data[0], 8);
            break;
            case BMS_SOFT_VER_EXTEND3:
                memcpy(&car_info.bms_info[0].soft_ver[24], &data[0], 8);
            break;
            case BMS_HW_VER_A:
                memcpy(&car_info.bms_info[0].hw_ver[0], &data[0], 8);
            break;
            case BMS_HW_VER_B:
                memcpy(&car_info.bms_info[0].hw_ver[8], &data[0], 8);
            break;
        }
    }
}

static void iot_response_png(uint16_t pgn)
{
    CAN_PDU_STU can_pdu;
    uint8_t data[8] = {0};
    stc_can_rxframe_t can_dat = {0};
    int64_t stamp;
    can_pdu.src = IOT_ADR;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_pdu.pdu.pdu2 = pgn;
    switch(pgn) {
        case IOT_SN1:
            memcpy(data, &sys_config.sn[0], 8);
        break;
        case IOT_SN2:
            memcpy(data, &sys_config.sn[8], 8);
        break;
        case IOT_SOFT_VER1:
            memcpy(data, SOFTVER, (strlen(SOFTVER) > 8 ? 8:strlen(SOFTVER)));
        break;
        case IOT_SOFT_VER2:
            if(strlen(SOFTVER) > 8) {
                memcpy(data, SOFTVER + 8, ((strlen(SOFTVER) - 8) > 8 ? 8:(strlen(SOFTVER) - 8)));
            }
        break;
        case IOT_HW_VER1:
            memcpy(data, HWVER, (strlen(HWVER) > 8 ? 8:strlen(HWVER)));
        break;
        case IOT_HW_VER2:
            if(strlen(HWVER) > 8) {
                memcpy(data, HWVER + 8, ((strlen(HWVER) - 8) > 8 ? 8:(strlen(HWVER) - 8)));
            }
        break;
        case IOT_SAVE_DATA:
            data[0] = car_set_save.odo >> 16 & 0XFF;
            data[1] = car_set_save.odo >> 8 & 0XFF;
            data[2] = car_set_save.odo & 0XFF;
            data[3] = car_set_save.trip >> 16 & 0XFF;
            data[4] = car_set_save.trip >> 8 & 0XFF;
            data[5] = car_set_save.trip & 0XFF;
            data[6] = car_set_save.range;
        break;
        case IOT_STATE_DATA:
            data[0] |= ((car_state_data.abnormal_move == 0) ? 0 : 1) << 7;
            data[0] |= ((car_state_data.mobile_operation_sta == 0) ? 0 : 1) << 6;
            data[1] = car_state_data.slope_data;
            data[2] = car_state_data.attitude;
            data[3] = car_state_data.map_dir;
            data[4] = car_state_data.cur_dir_range >> 24 & 0xff;
            data[5] = car_state_data.cur_dir_range >> 16 & 0xff;
            data[6] = car_state_data.cur_dir_range >> 8 & 0xff;
            data[7] = car_state_data.cur_dir_range & 0xff;
        break;
        case IOT_NAVIGATION_DATA:
            data[1] = car_state_data.total_nav_remaintime >> 24 & 0xff;
            data[2] = car_state_data.total_nav_remaintime >> 16 & 0xff;
            data[3] = car_state_data.total_nav_remaintime >> 8 & 0xff;
            data[4] = car_state_data.total_nav_remaintime & 0xff;
            data[5] = car_state_data.total_nav_range >> 16 &0xff;
            data[6] = car_state_data.total_nav_range >> 8 &0xff;
            data[7] = car_state_data.cur_dir_range & 0xff;
        break;
        case IOT_SNTP:
            stamp = hal_drv_rtc_get_timestamp();
            memcpy(data, &stamp, 8);
        break;
    }
    memcpy(can_dat.Data, data, 8);
    can_dat.ExtID = can_pdu.can_id;
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    can_data_send(can_dat);
}

static void can_data_recv_parse(stc_can_rxframe_t rx_can_frame)
{
    CAN_PDU_STU can_pdu;
    CAN_PDU_STU send_can_pdu;
    uint16_t pgn;
    
    LOG_I("CAN_ID:%08x, data_len:%d", rx_can_frame.RBUF32_0, rx_can_frame.Cst.Control_f.DLC);
    can_pdu.can_id = rx_can_frame.RBUF32_0;
    if(can_pdu.pdu.da ==  IOT_ADR && can_pdu.pdu.pdu1 == 0XEA) {
        pgn = rx_can_frame.Data[1] << 8 | rx_can_frame.Data[0];
        LOG_I("iot png:%04x", pgn);
        iot_response_png(pgn);
        return;
    }
    switch(can_pdu.src) {
        case HMI_ADR:
            car_info.hmi_connnect = 1;
            check_hmi_timeout = def_rtos_get_system_tick();
            hmi_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;    
        case CONTROL_ADR:
            car_info.control_connect = 1;
            check_control_timeout = def_rtos_get_system_tick();
            control_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case BMS_ADR:
            car_info.bms_connect = 1;
            check_bms_timeout = def_rtos_get_system_tick();
            bms_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case LOCK_ADR:
            car_info.lock_connect = 1;
            check_bms_timeout = def_rtos_get_system_tick();
        break;
    }
    if(can_send_cmd.cmd_send) {
        send_can_pdu.can_id = can_send_cmd.can_tx_frame.ExtID;
        if(send_can_pdu.pdu.da != can_pdu.src) return;
        switch(send_can_pdu.pdu.pdu1){
            case 0xEA:
                if((can_send_cmd.can_tx_frame.Data[1] << 8 | can_send_cmd.can_tx_frame.Data[0]) == can_pdu.pdu.pdu2) {
                    can_send_cmd.ask_flag = 1;
                }
            break;
            case 0x14:
                if(can_pdu.pdu.pdu1 == 0XE7 || can_pdu.pdu.pdu1 == 0XE5){
                    can_send_cmd.ask_flag = 1;
                }
            break;
        }
    }
}

void can_png_quest(uint8_t dst, uint16_t png, uint8_t direct)
{
    CAN_PDU_STU can_pdu;
    uint8_t data[8] = {0};
    stc_can_rxframe_t can_dat = {0};
    def_rtosStaus res;
    can_pdu.src = IOT_ADR;
    can_pdu.pdu.da = dst;
    can_pdu.pdu.pdu1 = 0XEA;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_dat.ExtID = can_pdu.can_id;
    data[0] = png & 0xff;
    data[1] = png >> 8;
    data[2] = 0x00;
    memcpy(&can_dat.Data[0], &data[0], 8);
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    if(direct) {
        can_data_send(can_dat);
    } else {
        res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8 *)&can_dat, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            LOG_E("def_rtos_queue_release is fail");
        }
    }
}

static uint8_t can_check_sum(uint8_t *dat, uint8_t len)
{
    uint8_t check_sum = 0;
    uint8_t i = 0;
    for(i = 0; i < len; i++)
    {
        check_sum +=dat[i];
    }
    return check_sum;

}
void iot_can_cmd_control(uint8_t cmd, uint8_t *cmdvar, uint8_t direct)
{
    CAN_PDU_STU can_pdu = {0};
    uint8_t data[8];
    stc_can_rxframe_t can_dat = {0};
    def_rtosStaus res;

    memset(data, 0, 8);
    can_pdu.src = IOT_ADR;
    can_pdu.pdu.da = can_control_cmd_table[cmd].dts;
    can_pdu.pdu.pdu1 = can_control_cmd_table[cmd].cmd_code;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_dat.ExtID = can_pdu.can_id;
    data[0] = can_control_cmd_table[cmd].cmd_index;
    data[1] = 0;
    data[2] = can_control_cmd_table[cmd].cmd_len;
    memcpy(&data[3], cmdvar, can_control_cmd_table[cmd].cmd_len);
    data[7] = can_check_sum(data, 7);
    memcpy(&can_dat.Data[0], &data[0], 8);
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    if(direct) {
        can_data_send(can_dat);
    } else {
        res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            LOG_E("def_rtos_queue_release is fail");
        }
    }
}

uint8_t can_find_send_order(uint32_t can_id, struct can_cmd_order_s *can_cmd_t)
{
    uint8_t res;
    uint8_t i = 0;
    CAN_PDU_STU can_pdu;
    can_pdu.can_id = can_id;

    for(i = 0; i < ARRAY_SIZE(can_cmd_order); i++){
        if(can_pdu.pdu.pdu1 == can_cmd_order[i].cmd) {
           *can_cmd_t = can_cmd_order[i];
           res = 0;
           break;
        }
    } 
    if(i == ARRAY_SIZE(can_cmd_order)) res = 1;
    return res;  
}

void can_cmd_send_fail(uint32_t can_id)
{
    LOG_E("CAN CMD SEND FAIL, can_id:%0x", can_id);
}

void can_protocol_tx_thread(void *param)
{
    struct can_cmd_order_s can_cmd_t = {0};
    int64_t time_t;
    def_rtosStaus res;
    while(1)
    {
        can_send_cmd.cmd_send = 0;
        res = def_rtos_queue_wait(can_tx_que, (uint8_t *)&can_send_cmd.can_tx_frame, sizeof(stc_can_rxframe_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            LOG_E("def_rtos_queue_wait  is fail");
            continue;
        }
        if (can_find_send_order(can_send_cmd.can_tx_frame.ExtID, &can_cmd_t) != 0) {
            can_data_send(can_send_cmd.can_tx_frame);
            continue;
        } 
        can_send_cmd.ask_flag = 0;
        can_send_cmd.cnt = 0;
        can_send_cmd.cmd_send = 1;
        time_t = def_rtos_get_system_tick();
        can_data_send(can_send_cmd.can_tx_frame);
        can_send_cmd.cnt++;
        for(;;){
            if(can_cmd_t.need_ask) {
                if(can_send_cmd.ask_flag){
                    break;
                } 
                if(def_rtos_get_system_tick() - time_t >= can_cmd_t.timeout){
                    if(can_send_cmd.cnt >= can_cmd_t.max_cnt) {
                        can_cmd_send_fail(can_send_cmd.can_tx_frame.ExtID);
                        break;
                    }
                    can_data_send(can_send_cmd.can_tx_frame);
                    can_send_cmd.cnt++;
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

void can_protocol_rx_thread(void *param)
{
    uint8_t res = RTOS_SUCEESS;
    stc_can_rxframe_t rx_can_frame;
    while(1) {
        res = can_data_recv(&rx_can_frame, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
        can_data_recv_parse(rx_can_frame);
    }
    def_rtos_task_delete(NULL);
}

void iot_can_png_control(uint8_t cmd, uint8_t direct)
{
    CAN_PDU_STU can_pdu;
    uint8_t data[8] = {0};
    stc_can_rxframe_t can_dat = {0};
    def_rtosStaus res;

    can_pdu.src = IOT_ADR;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_pdu.pdu.pdu2 = IOT_CONTROL_DATA1;

    switch(cmd) {
        case IOT_CONTROL_SET_GEAR:
            if(car_set_save.gear != car_info.gear) {
                data[0] |= 1<<7;
                data[0] |= car_set_save.gear&0xf;
            } else {
                return;
            }
        break;
        case IOT_CONTROL_SET_TAILLIHHT:
        case IOT_CONTROL_SET_HEADLIGHT:
        case IOT_CONTROL_SET_LEFT_TURN_LIGHT:
        case IOT_CONTROL_SET_RIGHT_TURN_LIGHT:
            if(car_set_save.head_light != car_info.headlight_sta || car_set_save.tail_light != car_info.taillight_sta ||\
                car_set_save.right_turn_light!= car_info.right_turn_light_sta || car_set_save.left_turn_light!= car_info.left_turn_light_sta){
                data[4] |= 1<<7;
                data[4] |= car_set_save.tail_light << 1;
                data[4] |= car_set_save.head_light;
                data[4] |= (car_set_save.right_turn_light&0x03)<<3;
                data[4] |= (car_set_save.left_turn_light&0x03)<<5;
            }
        break;
        case IOT_CONTROL_ANTI_THEFT:
            if(car_set_save.anti_theft_on) {
                data[1] = 0x56;
            } else {
                data[1] = 0xA9;
            }
        break;
    }
    can_dat.ExtID = can_pdu.can_id;
    memcpy(&can_dat.Data[0], &data[0], 8);
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    if(direct) {
        can_data_send(can_dat);
    } else {
        res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            LOG_E("def_rtos_queue_release is fail");
        }
    }
}

static void iot_can_match_fun()
{
    CAN_PDU_STU can_pdu;
    uint8_t data[8] = {0};
    stc_can_rxframe_t can_dat = {0};
    can_pdu.src = IOT_ADR;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_pdu.pdu.pdu2 = IOT_MATCH;
    data[1] = sys_info.can_key;
    data[3] |= ((sys_info.startup_way == 1) ? 1 : 0) << 1;
    data[3] |= ((sys_info.startup_way == 2) ? 1: 0);
    data[4] |= sys_info.can_protocol_major;
    data[5] |= sys_info.can_protocol_sub;
    can_dat.ExtID = can_pdu.can_id;
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    memcpy(can_dat.Data, data, 8);
    can_data_send(can_dat);
}

static void iot_can_state_fun()
{
    CAN_PDU_STU can_pdu;
    uint8_t data[8] = {0};
    stc_can_rxframe_t can_dat = {0};
    can_pdu.src = IOT_ADR;
    can_pdu.p = 6;
    can_pdu.r = 0;
    can_pdu.dp = 0;
    can_pdu.res = 0;
    can_pdu.pdu.pdu2 = IOT_STATE;

    data[0] = sys_info.bat_soc;
    data[1] = sys_info.bat_soh;
    data[2] = sys_info.fault;
    data[3] |= sys_info.pdp_reg << 7;
    data[3] |= sys_info.paltform_connect << 6;
    data[3] |= sys_info.gps_state << 5;
    data[3] |= sys_info.ble_connect << 4;
    data[3] |= sys_info.exits_bat << 3;
    can_dat.ExtID = can_pdu.can_id;
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    memcpy(can_dat.Data, data, 8);
    can_data_send(can_dat);
}

void iot_can_heart_fun()
{
    static uint32_t time_s = 0;
    if(time_s % 3 == 0) {
        iot_can_match_fun();
    }
    if(time_s % 2 == 0) {
        iot_can_state_fun();
    }
    if(def_rtos_get_system_tick() - check_hmi_timeout > 5000) {
        car_info.hmi_connnect = 0;
    }
    if(def_rtos_get_system_tick() - check_bms_timeout > 5000) {
        car_info.bms_connect = 0;
    }
    if(def_rtos_get_system_tick() - check_control_timeout > 5000) {
        car_info.control_connect = 0;
    }
    if(def_rtos_get_system_tick() - check_lock_timeout > 5000) {
        car_info.lock_connect = 0;
    }
    time_s++;
}
void can_protocol_init()
{
    def_rtosStaus res;
    res = def_rtos_queue_create(&can_tx_que, sizeof(stc_can_rxframe_t), 30);
    if(res != RTOS_SUCEESS) {
        LOG_E("can_tx_que is create fail");
    }
    LOG_I("can_protocol_init is ok");
}