#include "app_system.h"

#define DBG_TAG         "can_protocol"

#ifdef CAN_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

def_rtos_queue_t can_tx_que;
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
    uint8_t cmd_index;
    uint8_t cmd_len;
};
struct can_control_cmd_stu can_control_cmd_table[] = {
    {0x28,  0x01,   1},
    {0x28,  0x02,   1},
    {0x28,  0x03,   1},
    {0x28,  0x04,   1},
    {0x28,  0x05,   1},
    {0x28,  0x06,   1},
    {0x28,  0x07,   1},
    {0x28,  0x08,   1},
    {0x28,  0x09,   1},
    {0x60,  0x01,   1},
};


struct can_cmd_order_s can_cmd_order[] = {
    {0X14,  true,   3,  3000},  /* 控制指令*/
    {0XEA,  true,   3,  3000}, /*  PNG请求*/
};

void hmi_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
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
                car_info.hmi_info.fault_code = data[4];
           //     debug_data_printf("HMI_DATA", data, data_len);  
            break;
            case HMI_STA:
                car_info.hmi_info.display_unit = data[1]&0x80?1:0;
                car_info.hmi_info.encry_lock_sta = data[2];
          //      debug_data_printf("HMI_STA", data, data_len);  
            break;
            case HMI_HW_VER1:
                memcpy(car_info.hmi_info.hw_ver1, (char *)data, 8);
                LOG_I("hmi_hw_ver1:%s", car_info.hmi_info.hw_ver1);
         //       debug_data_printf("HMI_HW_VER1", data, data_len);  
            break;
            case HMI_HW_VER2:
                memcpy(car_info.hmi_info.hw_ver2, (char *)data, 8);
                LOG_I("hmi_hw_ver2:%s", car_info.hmi_info.hw_ver2);
          //      debug_data_printf("HMI_HW_VER2", data, data_len); 
            break;
            case HMI_SOFT_VER1:
                memcpy(car_info.hmi_info.soft_ver1, (char *)data, 8);
                LOG_I("hmi_soft_ver1:%s", car_info.hmi_info.soft_ver1);
           //     debug_data_printf("HMI_SOFT_VER1", data, data_len); 
            break;
            case HMI_SOFT_VER2:
                memcpy(car_info.hmi_info.soft_ver2, (char *)data, 8);
                LOG_I("hmi_soft_ver2:%s", car_info.hmi_info.soft_ver2);
          //      debug_data_printf("HMI_SOFT_VER2", data, data_len); 
            break;
            case HMI_SN1:
                memcpy(car_info.hmi_info.sn1, (char *)data, 8);
                LOG_I("hmi_sn1:%s", car_info.hmi_info.sn1);
          //      debug_data_printf("HMI_SN1", data, data_len); 
            break;
            case HMI_SN2:
                memcpy(car_info.hmi_info.sn2, (char *)data, 8);
                LOG_I("hmi_sn2:%s", car_info.hmi_info.sn2);
         //       debug_data_printf("HMI_SN2", data, data_len); 
            break;
            case HMI_SN3:
                memcpy(car_info.hmi_info.sn3, (char *)data, 8);
                LOG_I("hmi_sn3:%s", car_info.hmi_info.sn3);
          //      debug_data_printf("HMI_SN3", data, data_len); 
            break;
            case HMI_SN4:
                memcpy(car_info.hmi_info.sn4, (char *)data, 8);
                LOG_I("hmi_sn4:%s", car_info.hmi_info.sn4);
           //     debug_data_printf("HMI_SN4", data, data_len); 
            break;
        }
    }
}

void control_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
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
                car_info.control_power = data[3] <<8 | data[4];
                car_info.control_torque = data[5];
                car_info.control_temp = data[6];
                car_info.motor_temp = data[7];
             //   debug_data_printf("CONTROL_DATA2", data, data_len);
            break;
            case CONTROL_DATA3:
                car_info.total_odo = data[0]<<16|data[1]<<8|data[2];
                car_info.single_odo = data[3]<<16|data[4]<<8|data[5];
                car_info.remain_odo = data[6];
                car_info.wheel = data[7];
             //   debug_data_printf("CONTROL_DATA3", data, data_len);
            break;
            case CONTROL_DATA4:
                car_info.avg_speed = data[0] << 8| data[1];
                car_info.max_speed = data[2] << 8| data[3];
            //    debug_data_printf("CONTROL_DATA4", data, data_len);
            break;
            case CONTROL_DATA5:
                car_info.cycle_time_minute = data[0];
                car_info.cycle_time_hour = data[1];
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
                car_info.bus_voltage = data[5] << 8 | data[6];
           //     debug_data_printf("CONTROL_DATA8", data, data_len);
            break;
            case CONTROL_HWVER1:
                memcpy(&car_info.con_hw_ver1[0], (char *)&data[0], 8);
                LOG_I("control_hw_ver:%s", car_info.con_hw_ver1);
          //      debug_data_printf("CONTROL_HWVER1", data, data_len);
            break;
            case CONTROL_HWVER2:
                memcpy(&car_info.con_hw_ver2[0], (char *)&data[0], 8);
           //     debug_data_printf("CONTROL_HWVER2", data, data_len);
            break;
            case CONTROL_SOFTVER1:
                memcpy(&car_info.con_soft_ver1[0], (char *)&data[0], 8);
                LOG_I("control_soft_ver:%s", car_info.con_soft_ver1);
           //     debug_data_printf("CONTROL_SOFTVER1", data, data_len);
            break;
            case CONTROL_SOFTVER2:
                memcpy(&car_info.con_soft_ver2[0], (char *)&data[0], 8);
            //    debug_data_printf("CONTROL_SOFTVER2", data, data_len);
            break;
            case CONTROL_SN1:
                memcpy(&car_info.con_sn1[0], (char *)&data[0], 8);
                LOG_I("control_sn1:%s", car_info.con_sn1);
             //   debug_data_printf("CONTROL_SN1", data, data_len);
            break;
            case CONTROL_SN2:
                memcpy(&car_info.con_sn2[0], (char *)&data[0], 8);
                LOG_I("control_sn2:%s", car_info.con_sn2);
              //  debug_data_printf("CONTROL_SN2", data, data_len);
            break;
            case CONTROL_SN3:
                memcpy(&car_info.con_sn3[0], (char *)&data[0], 8);
                LOG_I("control_sn3:%s", car_info.con_sn3);
              //  debug_data_printf("CONTROL_SN3", data, data_len);
            break;
            case CONTROL_SN4:
                memcpy(&car_info.con_sn4[0], (char *)&data[0], 8);
                LOG_I("control_sn4:%s", car_info.con_sn4);
             //   debug_data_printf("CONTROL_SN4", data, data_len);
            break;
            case CONTROL_PARAMVER1:
                memcpy(&car_info.con_param_ver1[0], (char *)&data[0], 8);
           //     debug_data_printf("CONTROL_PARAMVER1", data, data_len);
            break;
            case CONTROL_PARAMVER2:
                memcpy(&car_info.con_param_ver2[0], (char *)&data[0], 8);
            //    debug_data_printf("CONTROL_PARAMVER2", data, data_len);
            break;
            case CONTROL_PARAMVER3:
                memcpy(&car_info.con_param_ver3[0], (char *)&data[0], 8);
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

void bms_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240) {
        switch(pdu.pdu2){
            case BMS_MATCH_INFO:
            break;
            case BMS_BATTRY_PACK_TEMP:
            break;
            case BMS_COMPREHENSIVE_DATA:
            break;
            case BMS_CELL_VOL1:
            break;
            case BMS_CELL_VOL2:
            break;
            case BMS_CELL_VOL3:
            break;
            case BMS_CELL_VOL4:
            break;
            case BMS_REALTIME_VOL_CURRENT:
            break;
            case BMS_FIRST_SENCOND_PROTECTION:
            break;
            case BMS_BATTRY_PACK_STA:
            break;
            case BMS_BATTRY_PACK_RECORDDATA:
            break;
            case BMS_BATTRY_PACK_CHARGE_PARAM:
            break;
            case BMS_PROTECTION_FAULT_INFO:
            break;
            case BMS_BARCODE_A:
            break;
            case BMS_BARCODE_B:
            break;
            case BMS_BARCODE_C:
            break;
            case BMS_BARCODE_D:
            break;
            case BMS_SOFT_VER:
            break;
            case BMS_SOFT_VER_EXTEND1:
            break;
            case BMS_SOFT_VER_EXTEND2:
            break;
            case BMS_SOFT_VER_EXTEND3:
            break;
            case BMS_HW_VER_A:
            break;
            case BMS_HW_VER_B:
            break;
        }
    }
}
void can_data_recv_parse(stc_can_rxframe_t rx_can_frame)
{
    CAN_PDU_STU can_pdu;
    CAN_PDU_STU send_can_pdu;
    
    can_pdu.can_id = rx_can_frame.RBUF32_0;
    switch(can_pdu.src) {
        case HMI_ADR:
           hmi_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;    
        case CONTROL_ADR:
            control_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case BMS_ADR:
            bms_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
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
    
    LOG_I("CAN_ID:%08x, data_len:%d", rx_can_frame.RBUF32_0, rx_can_frame.Cst.Control_f.DLC);
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
void can_send_control_cmd(uint8_t cmd, uint8_t *cmdvar, uint8_t direct)
{
    CAN_PDU_STU can_pdu = {0};
    uint8_t data[8];
    stc_can_rxframe_t can_dat = {0};
    def_rtosStaus res;

    memset(data, 0, 8);
    can_pdu.src = IOT_ADR;
    can_pdu.pdu.da = can_control_cmd_table[cmd].dts;
    can_pdu.pdu.pdu1 = 0x14,
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

void headlightclose()
{
    uint8_t dat = 0xff;
    dat &= ~(1 << 7); 
    dat &= ~(1 << 6); 
    can_send_control_cmd(CMD_SET_HEADLIGHT, &dat, 0);
}

void headlightclosemode()
{
    uint8_t dat = 0;
    can_send_control_cmd(CMD_SET_HEADLIGHT_SENMODE, &dat, 0);
}

void enter_sleep()
{
    uint8_t dat = 0x56;
    can_send_control_cmd(CMD_ENTER_SLEEP, &dat, 0);
}
void enter_week()
{
    uint8_t dat = 0XA9;
    can_send_control_cmd(CMD_ENTER_SLEEP, &dat, 0);   
}
void can_protocol_init()
{
    def_rtosStaus res;
    res = def_rtos_queue_create(&can_tx_que, sizeof(stc_can_rxframe_t), 30);
    if(res != RTOS_SUCEESS){
        LOG_E("def_rtos_queue_create is fail");
    }
    LOG_I("can_protocol_init is ok");
    headlightclose();
}