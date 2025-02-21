#include "app_system.h"
#include "hal_drv_rtc.h"

#define DBG_TAG         "can_protocol"

#ifdef CAN_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"
enum OTA_STEP{
    OTA_IDEL_STEP = 0,
    OTA_QUEST_STEP,
    OTA_PACK_HEAD_STEP,
    OTA_PACK_DATA_STEP,
    OTA_PACK_DATA_FINISH_STEP,
    OTA_PACK_TAIL_STEP,
    OTA_QUIT_STEP,
};



struct can_ota_con_stu can_ota_con;
struct trans_can_control_stu  trans_can_control;
def_rtos_queue_t can_tx_que;
static int64_t check_hmi_timeout, check_control_timeout,check_lock_timeout, check_bms_timeout;
CAN_SEND_CMD_STU can_send_cmd;

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
    {0x28,  0x14,   0x01,   1}, //HMI_CMD_SET_GEAR
    {0x28,  0x14,   0x02,   1}, //HMI_CMD_SET_HEADLIGHT
    {0x28,  0x14,   0x03,   1}, //HMI_CMD_SET_HEADLIGHT_SENMODE
    {0x28,  0x14,   0x04,   1}, //HMI_CMD_SET_TURNLIGHT
    {0x28,  0x14,   0x05,   1}, //HMI_CMD_SET_CYCLE_MODE
    {0x28,  0x14,   0x06,   1}, //HMI_CMD_JUMP_PASSWORD
    {0x28,  0x14,   0x07,   1}, //HMI_CMD_ENTER_SLEEP
    {0x28,  0x14,   0x08,   1}, //HMI_CMD_ENTER_WEEK
    {0x28,  0x14,   0x09,   1}, //HMI_CMD_LOOK_CAR
    {0x60,  0x14,   0x01,   1}, //CMD_SET_ELECLOCK
    {0xEF,  0x60,   0x55,   2}, //CMD_SPEED_LIMIT
    {0X28,  0X60,   0X05,   1}, //CMD_MILEAGE_UNIT
    {0X28,  0X60,   0X02,   4}, //CMD_SET_POWER_ON_PASSWORD
    {0X28,  0X60,   0X06,   1}, //CMD_SET_ATMOSPHERE_LIGHT_MODE
    {0X28,  0X60,   0X07,   1}, //CMD_SET_ATMOSPHERE_LIGHT_COLOUR
    {0X28,  0X60,   0X08,   1}, //CMD_SET_ATMOSPHERE_LIGHT_BRIGHTNESS
    {0X28,  0X60,   0X09,   1}, //CMD_SET_ATMOSPHERE_LIGHT_TURN
    {0X28,  0X60,   0X0A,   1}, //CMD_SET_ATMOSPHERE_LIGHT_R_VAL
    {0X28,  0X60,   0X0B,   1}, //CMD_SET_ATMOSPHERE_LIGHT_G_VAL
    {0X28,  0X60,   0X0C,   1}, //CMD_SET_ATMOSPHERE_LIGHT_B_VAL
    {0X28,  0X60,   0X01,   1}, //CMD_EN_POWER_ON_PASSWORD
    {0XF4,  0X60,   0X01,   1}, //CMD_BMS_SET_CHARGE_MODE
    {0XF4,  0X60,   0X02,   1}, //CMD_BMS_SET_CHARGE_SOC
    {0XF4,  0X14,   0X01,   1}, //CMD_BMS_DISCHARGE_SW
    {0XF4,  0X14,   0X02,   1}, //CMD_DOUBLE_BMS_WORK_MODE
    {0X59,  0X60,   0X01,   2}, //CMD_CHARGE_POWER
};


static struct can_cmd_order_s can_cmd_order[] = {
    {0X60,  true,   3,   1000},  /*参数设置*/
    {0X14,  true,   3,   1000},  /* 控制指令*/
    {0XEA,  true,   3,   1000}, /*  PNG请求*/
    {0XEB,  true,   5,   2000},  /*进入OTA*/
    {0XEE,  false,  0,   2000}, /*退出OTA*/
    {0XD3,  true,   3,   2000}, /*帧头指令*/
};

static void hmi_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
  //  static uint8_t last_power = 0;   
    if(pdu.pdu1 >= 240){
        switch(pdu.pdu2) {
            case HMI_MATCH_INFO: 

      //          car_info.hmi_info.power_on = data[0]&0x80?1:0;
                car_info.hmi_info.encry_mode_data = data[0]&0x7f;
                car_info.hmi_info.key = data[1];
                car_info.hmi_info.startup_mode = data[3]&0x07;
                car_info.hmi_info.protocol_major_ver = data[4];
                car_info.hmi_info.protocol_sub_ver = data[5];
                if((data[0]&0x80) == 0) {
                    LOG_I("POWER_OFF");
                    car_info.hmi_info.power_on = 0;
                    if(car_info.lock_sta == CAR_UNLOCK_STA) {
                         week_time("lock", 10);
                         voice_play_mark(LOCK_VOICE);   
                         car_info.lock_sta = CAR_LOCK_STA;
                         LOG_I("HMI_CMD_LOCK_SRC CAR_LOCK_STA");
                    }
                } else if(((data[0]&0x80)>>7) == 1 && car_info.hmi_info.power_on == 0) {
                    car_info.hmi_info.power_on = 1;
                    if(app_rtc_event_query_remain_time(CAR_SET_EN_POWER_PASSWD)){
                        car_control_cmd(CAR_CMD_JUMP_PASSWORD);
                        rtc_event_unregister(CAR_SET_EN_POWER_PASSWD);
                    }
                    LOG_I("POWER_ON");
                }

                // if(car_info.hmi_info.power_on == 1 && last_power == 0) {
                //     if(car_info.lock_sta == CAR_LOCK_STA) {
                //          car_lock_control(HMI_CMD_LOCK_SRC, CAR_UNLOCK_ATA);
                //          LOG_I("HMI_CMD_LOCK_SRC CAR_UNLOCK_ATA");
                //     }
                // } 
                // if(car_info.hmi_info.power_on == 0 && last_power == 1) {
                //     if(car_info.lock_sta == CAR_UNLOCK_ATA){
                //          car_lock_control(HMI_CMD_LOCK_SRC, CAR_LOCK_STA); 
                //          LOG_I("HMI_CMD_LOCK_SRC CAR_LOCK_STA");
                //     }
                // }
                // last_power = car_info.hmi_info.power_on;
                debug_data_printf("HMI_MATCH_INFO", data, data_len);
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
                if((7&(data[1]>>4)) == 2 && car_info.hmi_info.power_on == 1) {
                    if(car_info.lock_sta == CAR_LOCK_STA) {
                        car_info.lock_sta = CAR_UNLOCK_STA;
                        week_time("lock", -1); 
                        voice_play_mark(UNLOCK_VOICE);
                        LOG_I("HMI_CMD_LOCK_SRC CAR_UNLOCK_STA");
                    }
                }
                // if(data[2] == 0xA9 || data[2] == 0xA0) {
                //     if(car_info.lock_sta == CAR_UNLOCK_ATA)
                //         car_lock_control(HMI_CMD_LOCK_SRC, CAR_LOCK_STA);
                // } else if(data[2] == 0x56) {
                //     if(car_info.lock_sta == CAR_LOCK_STA) 
                //         car_lock_control(HMI_CMD_LOCK_SRC, CAR_UNLOCK_ATA);
                // }
                debug_data_printf("HMI_STA", data, data_len);  
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
            case HMI_ATMOSPHERE_LIGHT_STA:
                car_info.atmosphere_light_info.light_mode = data[0];
                car_info.atmosphere_light_info.color = data[1];
                car_info.atmosphere_light_info.brightness_val = data[2];
                car_info.atmosphere_light_info.turn_linght_sta = data[3];
                car_info.atmosphere_light_info.ble_sta = data[4];
                car_info.atmosphere_light_info.custom_red = data[5];
                car_info.atmosphere_light_info.custom_green = data[6];
                car_info.atmosphere_light_info.custom_blue = data[7];
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
                car_info.speed = data[4]<<8 | data[3];
                car_info.fault_code = data[5];
           //     debug_data_printf("CONTROL_DATA1", data, data_len);
            break;
            case CONTROL_DATA2:
                car_info.current = data[0];
                car_info.pedal_speed = data[1];
                car_info.pedal_torque = data[2];
                car_info.motor_power = data[4] <<8 | data[3];
                car_info.control_torque = data[5];
                car_info.control_temp = data[6];
                car_info.motor_temp = data[7];
             //   debug_data_printf("CONTROL_DATA2", data, data_len);
            break;
            case CONTROL_DATA3:
                car_info.total_odo = data[2]<<16|data[1]<<8|data[0];
                car_info.single_odo = data[5]<<16|data[4]<<8|data[3];
                car_info.remain_odo = data[6] * 10;
                
                car_info.wheel = data[7];
             //   debug_data_printf("CONTROL_DATA3", data, data_len);
            break;
            case CONTROL_DATA4:
                car_info.avg_speed = data[1] << 8| data[0];
                car_info.max_speed = data[3] << 8| data[2];
                car_info.current_limit = data[7];
            //    debug_data_printf("CONTROL_DATA4", data, data_len);
            break;
            case CONTROL_DATA5:
                car_info.cycle_time_s = data[0]*60 + data[1]*3600;
                car_info.speed_limit = data[4] << 8 | data[3];
                car_info.transfer_data = data[5];
                car_info.motor_speed = data[7]<<8|data[6];
           //     debug_data_printf("CONTROL_DATA5", data, data_len);
            break;
            case CONTROL_DATA6:
                car_info.cycle_power = data[0] << 8 | data[1];
                car_info.motor_consumption = data[3] << 8 | data[2];
                car_info.motor_avg_consumption = data[5] << 8 | data[4];
                car_info.reduce_power_sta = data[6];
                car_info.stop_drive_sta = data[7];
            //    debug_data_printf("CONTROL_DATA6", data, data_len);
            break;
            case CONTROL_DATA7:
                car_info.cycle_avg_power = data[1] << 8 | data[0];
                car_info.motor_avg_power = data[3] << 8 | data[2];
           //     debug_data_printf("CONTROL_DATA7", data, data_len);
            break;
            case CONTROL_DATA8:
                car_info.trip_calorie = data[2]<<16 | data[1]<<8 | data[0];
                car_info.ebike_calorie = data[4] <<8 | data[3];
                car_info.bus_voltage = data[6] << 8 | data[5];
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

static void bms_info_handle(uint8_t bms_num, PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240) {
        switch(pdu.pdu2){
            case BMS_MATCH_INFO:
                car_info.bms_info[bms_num].protocol_major_ver = data[4];
                car_info.bms_info[bms_num].protocol_sub_ver = data[5];
                car_info.bms_info[bms_num].key = data[1];
            break;
            case BMS_BATTRY_PACK_TEMP:
                car_info.bms_info[bms_num].temp_probe_number = data[0];
                car_info.bms_info[bms_num].cell_temp = data[1];
                car_info.bms_info[bms_num].mos_temp = data[2];
                car_info.bms_info[bms_num].board_temp = data[3];
                car_info.bms_info[bms_num].max_temp = data[4];
                car_info.bms_info[bms_num].min_temp = data[5];
            break;
            case BMS_COMPREHENSIVE_DATA:
                car_info.bms_info[bms_num].soh = data[0];
                car_info.bms_info[bms_num].soc = data[1];
                car_info.bms_info[bms_num].remain_capacity = data[3]<<8 | data[2];
                car_info.bms_info[bms_num].full_capactity = data[5]<<8 | data[4];
                car_info.bms_info[bms_num].design_capactity = data[7]<<8 | data[6];
            break;
            case BMS_CELL_VOL1:
                car_info.bms_info[bms_num].pack_series_number = data[0];
                car_info.bms_info[bms_num].pack_series_number = data[1];
                car_info.bms_info[bms_num].cell_val[0] = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].cell_val[1] = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].cell_val[2] = data[7] << 8 | data[6];
            break;
            case BMS_CELL_VOL2:
                car_info.bms_info[bms_num].cell_val[3] = data[1]<< 8 | data[0];
                car_info.bms_info[bms_num].cell_val[4] = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].cell_val[5] = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].cell_val[6] = data[7] << 8 | data[6];
            break;
            case BMS_CELL_VOL3:
                car_info.bms_info[bms_num].cell_val[7] = data[1]<< 8 | data[0];
                car_info.bms_info[bms_num].cell_val[8] = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].cell_val[9] = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].cell_val[10] = data[7] << 8 | data[6];
            break;
            case BMS_CELL_VOL4:
                car_info.bms_info[bms_num].cell_val[11] = data[1]<< 8 | data[0];
                car_info.bms_info[bms_num].cell_val[12] = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].cell_val[13] = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].cell_val[14] = data[7] << 8 | data[6];
            break;
            case BMS_REALTIME_VOL_CURRENT:
                car_info.bms_info[bms_num].pack_vol = data[1] << 8 | data[0];
                car_info.bms_info[bms_num].pack_current = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].max_continuous_current = data[4];
                car_info.bms_info[bms_num].max_charge_current = data[5];
                car_info.bms_info[bms_num].max_charge_current = data[6];
            break;
            case BMS_FIRST_SENCOND_PROTECTION:
                memcpy(&car_info.bms_info[bms_num].first_protect[0], &data[0], 4);
                memcpy(&car_info.bms_info[bms_num].second_protect[0], &data[4], 4);
            break;
            case BMS_BATTRY_PACK_STA:
                car_info.bms_info[bms_num].charge_det = data[0]&0x01;
                car_info.bms_info[bms_num].charge_sta = (data[0]>>1)&0x01;
                car_info.bms_info[bms_num].discharge_sta = data[1]&0x01;
                car_info.bms_info[bms_num].charge_mos = data[2]&0x01;
                car_info.bms_info[bms_num].discharge_mos = (data[2]>>1)&0x01;
                car_info.bms_info[bms_num].chargefull_sta = (data[2]>>2)&0x01;
                car_info.bms_info[bms_num].double_bms_sta = data[3]&0x03;
                car_info.bms_info[bms_num].charge_remain_time = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].design_val = data[6];
            break;
            case BMS_BATTRY_PACK_RECORDDATA:
                car_info.bms_info[bms_num].cycle_number = data[1] << 8 | data[0];
                car_info.bms_info[bms_num].charge_interval_time = data[3] << 8 | data[2];
                car_info.bms_info[bms_num].maxcharge_interval_time = data[5] << 8 | data[4];
                car_info.bms_info[bms_num].alive_time = data[7] << 8 | data[6];
            break;
            case BMS_BATTRY_PACK_CHARGE_PARAM:
                car_info.bms_info[bms_num].max_charge_val = data[1] << 8 | data[0];
            break;
            case BMS_PROTECTION_FAULT_INFO:
                car_info.bms_info[bms_num].first_protect_code = data[1];
                car_info.bms_info[bms_num].second_protect_code = data[2];
                car_info.bms_info[bms_num].fault_code = data[4];
            break;
            case BMS_BARCODE_A:
                car_info.bms_info[bms_num].code_len = data[0];
                memcpy(&car_info.bms_info[bms_num].code[0], &data[1], 7);
            break;
            case BMS_BARCODE_B:
                memcpy(&car_info.bms_info[bms_num].code[7], &data[0], 8);
            break;
            case BMS_BARCODE_C:
                memcpy(&car_info.bms_info[bms_num].code[15], &data[0], 8);
            break;
            case BMS_BARCODE_D:
                memcpy(&car_info.bms_info[bms_num].code[23], &data[0], 8);
            break;
            case BMS_SOFT_VER:
                memcpy(&car_info.bms_info[bms_num].soft_ver[0], &data[0], 8);
                LOG_I("soft_ver:%s", car_info.bms_info[0].soft_ver);
            break;
            case BMS_SOFT_VER_EXTEND1:
                memcpy(&car_info.bms_info[bms_num].soft_ver[8], &data[0], 8);
                LOG_I("soft_ver:%s", car_info.bms_info[0].soft_ver);
            break;
            case BMS_SOFT_VER_EXTEND2:
                memcpy(&car_info.bms_info[bms_num].soft_ver[16], &data[0], 8);
                LOG_I("soft_ver:%s", car_info.bms_info[0].soft_ver);
            break;
            case BMS_SOFT_VER_EXTEND3:
                memcpy(&car_info.bms_info[bms_num].soft_ver[24], &data[0], 8);
                LOG_I("soft_ver:%s", car_info.bms_info[0].soft_ver);
            break;
            case BMS_HW_VER_A:
                memcpy(&car_info.bms_info[bms_num].hw_ver[0], &data[0], 8);
                LOG_I("hw_ver:%s", car_info.bms_info->hw_ver);
            break;
            case BMS_HW_VER_B:
                memcpy(&car_info.bms_info[bms_num].hw_ver[8], &data[0], 8);
                LOG_I("hw_ver:%s", car_info.bms_info->hw_ver);
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

static void lock_info_handle(PDU_STU pdu, uint8_t *data, uint8_t data_len)
{
    if(pdu.pdu1 >= 240) {
        switch(pdu.pdu2) {
            case ELECTRONIC_LOCK_MATCH_INFO:

            break;
            case ELECTRONIC_LOCK_STATUE:
                car_info.electronic_lock.lock_sta  = data[0];
                car_info.electronic_lock.fault_code = data[1];
            break;
            case ELECTRONIC_LOCK_HEART:

            break;
            case ELECTRONIC_LOCK_HW_VER:
                memcpy(&car_info.electronic_lock.hw_ver[0], &data[0], 8);
                LOG_I("car_info.electronic_lock.hw_ver:%s",car_info.electronic_lock.hw_ver);
            break;
            case ELECTRONIC_LOCK_SOFT_VER:
                memcpy(&car_info.electronic_lock.soft_ver[0], &data[0], 8);
                LOG_I("car_info.electronic_lock.soft_ver:%s",car_info.electronic_lock.soft_ver);
            break;
            case ELECTRONIC_LOCK_TYPE:
                memcpy(&car_info.electronic_lock.type_str[0], &data[0], 8);
            break;
            case ELECTRONIC_LOCK_FIRM:
                memcpy(&car_info.electronic_lock.firm_identify[0], &data[0], 8);
            break;
        }
    }
}

static void can_data_recv_parse(stc_can_rxframe_t rx_can_frame)
{
    CAN_PDU_STU can_pdu;
    CAN_PDU_STU send_can_pdu;
    uint16_t pgn;
    
    LOG_I("CAN_ID:%08x", rx_can_frame.RBUF32_0);
    can_pdu.can_id = rx_can_frame.RBUF32_0;
    if(can_pdu.pdu.da ==  IOT_ADR) {
        if(can_pdu.pdu.pdu1 == 0XEA) {
            pgn = rx_can_frame.Data[1] << 8 | rx_can_frame.Data[0];
            LOG_I("iot png:%04x", pgn);
            iot_response_png(pgn);
            return;
        } else if(can_pdu.pdu.pdu1 == 0XD3) {
            if(rx_can_frame.Data[0] == 0x69) {   //异常上报
                LOG_E("FALUT RES:%02x", rx_can_frame.Data[1]);
                can_ota_con.ota_step = OTA_QUIT_STEP;
                // if(rx_can_frame.Data[1] == 0xA5) {  
                //     can_ota_con.ota_step = OTA_IDEL_STEP;//退出异常
                // } else {
                //     can_ota_con.ota_step = OTA_PACK_FAIL_STEP;
                // }
                return;
            }
        }
    }
    if(can_pdu.pdu.pdu2 == 0xef50) {
        LOG_I("STATUE:%d, %d", rx_can_frame.Data[0]>>6, rx_can_frame.Data[0]&0x3f);
    }
    if(trans_can_control.rq_can_id == can_pdu.can_id && trans_can_control.send_flag) {
        trans_can_control.send_flag = 0;
        memcpy(trans_can_control.rq_data, rx_can_frame.Data, 8);
        switch(trans_can_control.src) {
            case CAN_NET_TRANS:
            NET_CMD_MARK(NET_CMD_CAN_TRANS_Z0);
            break;
            default:
            break;
        }
    }
    switch(can_pdu.src) {
        case HMI_ADR:
            car_info.hmi_connnect = 1;
            if(car_info.hmi_info.init == 0 && can_ota_con.ota_step == OTA_IDEL_STEP){
                car_info.hmi_info.init = 1;
                can_png_quest(HMI_ADR, HMI_HW_VER1, 0);
             //   can_png_quest(HMI_ADR, HMI_HW_VER2, 0);
            //    can_png_quest(HMI_ADR, HMI_SOFT_VER1, 0);
           //     can_png_quest(HMI_ADR, HMI_SOFT_VER2, 0);
            //    can_png_quest(HMI_ADR, HMI_SN1, 0);
            //    can_png_quest(HMI_ADR, HMI_SN2, 0);
           //     can_png_quest(HMI_ADR, HMI_SN3, 0);
           //     can_png_quest(HMI_ADR, HMI_SN4, 0);
            }
            check_hmi_timeout = def_rtos_get_system_tick();
            hmi_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;    
        case CONTROL_ADR:
            if(car_info.con_init == 0 && can_ota_con.ota_step == OTA_IDEL_STEP) {
                car_info.con_init = 1;
                can_png_quest(CONTROL_ADR, CONTROL_HWVER1, 0);
                can_png_quest(CONTROL_ADR, CONTROL_SOFTVER1, 0);
                can_png_quest(CONTROL_ADR, CONTROL_SN1, 0);
            //    can_png_quest(CONTROL_ADR, CONTROL_SN2, 0);
            //    can_png_quest(CONTROL_ADR, CONTROL_SN3, 0);
            //    can_png_quest(CONTROL_ADR, CONTROL_SN4, 0);
            }
            car_info.control_connect = 1;
            check_control_timeout = def_rtos_get_system_tick();
            control_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case BMS_ADR:
            if(car_info.bms_info[0].init == 0 && can_ota_con.ota_step == OTA_IDEL_STEP) {
                car_info.bms_info[0].init = 1;
                can_png_quest(BMS_ADR, BMS_SOFT_VER, 0);
                can_png_quest(BMS_ADR, BMS_SOFT_VER_EXTEND1, 0);
                can_png_quest(BMS_ADR, BMS_SOFT_VER_EXTEND2, 0);
            //    can_png_quest(BMS_ADR, BMS_SOFT_VER_EXTEND3, 0);
                can_png_quest(BMS_ADR, BMS_HW_VER_A, 0);
            //    can_png_quest(BMS_ADR, BMS_HW_VER_B, 0);
                can_png_quest(BMS_ADR, BMS_HW_VER_B, 0);
            } 
            car_info.bms_info[0].connect= 1;
            check_bms_timeout = def_rtos_get_system_tick();
            bms_info_handle(0, can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case SECOND_BMS_ADR:
            if(car_info.bms_info[1].init == 0 && can_ota_con.ota_step == OTA_IDEL_STEP) {
                car_info.bms_info[1].init = 1;
                can_png_quest(SECOND_BMS_ADR, BMS_SOFT_VER, 0);
                can_png_quest(SECOND_BMS_ADR, BMS_SOFT_VER_EXTEND1, 0);
                can_png_quest(SECOND_BMS_ADR, BMS_SOFT_VER_EXTEND2, 0);
            //    can_png_quest(BMS_ADR, BMS_SOFT_VER_EXTEND3, 0);
                can_png_quest(SECOND_BMS_ADR, BMS_HW_VER_A, 0);
            //    can_png_quest(BMS_ADR, BMS_HW_VER_B, 0);
                can_png_quest(SECOND_BMS_ADR, BMS_HW_VER_B, 0);
            } 
            car_info.bms_info[1].connect = 1;
            check_bms_timeout = def_rtos_get_system_tick();
            bms_info_handle(1, can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
        case LOCK_ADR:
            if(car_info.electronic_lock.init == 0 && can_ota_con.ota_step == OTA_IDEL_STEP) {
                car_info.electronic_lock.init = 1;
                can_png_quest(LOCK_ADR, ELECTRONIC_LOCK_HW_VER, 0);
                can_png_quest(LOCK_ADR, ELECTRONIC_LOCK_SOFT_VER, 0);
            }
            car_info.lock_connect = 1;
            check_bms_timeout = def_rtos_get_system_tick();
            lock_info_handle(can_pdu.pdu, rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
        break;
    }
    LOG_I("SEND_ID:%08X, CAN_ID:%08x", can_send_cmd.can_tx_frame.ExtID, rx_can_frame.RBUF32_0);
    
    if(can_send_cmd.cmd_send) {
        send_can_pdu.can_id = can_send_cmd.can_tx_frame.ExtID;
        LOG_I("da:%02x, src:%02x", send_can_pdu.pdu.da, can_pdu.src);
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
            case 0xEB:
                LOG_I("0XEB:%08X", rx_can_frame.RBUF32_0);
                if(can_pdu.pdu.pdu1 == 0XEC) {  
                    can_send_cmd.ask_flag = 1; 
                    if(can_ota_con.dev_id == can_pdu.src) {
                        if(rx_can_frame.Data[0] == 0x55) {   //进入OTA
                            LOG_I("OTA_PACK_HEAD_STEP");
                            can_ota_con.ota_step = OTA_PACK_HEAD_STEP;
                        }
                        else if(rx_can_frame.Data[0] == 0xAA) {   //进入静默
                            can_ota_con.ota_step = OTA_QUIT_STEP;
                      //      can_ota_con.ota_step = OTA_IDEL_STEP;  
                        }
                    }    
                }
            break;
            case 0XEE:   //退出OTA
                if(can_pdu.pdu.pdu1 == 0XEF) {
                    can_send_cmd.ask_flag = 1;
            //        can_ota_con.ota_step = OTA_IDEL_STEP; 
                }
            break;
            case 0XD3:
                if (can_send_cmd.can_tx_frame.Data[0] == 0x56)
                {
                    if(rx_can_frame.Data[0] == 0x5A) {
                        can_send_cmd.ask_flag = 1;
                        LOG_I("PACK_HEAD RES:%02X", rx_can_frame.Data[1]);
                        if(rx_can_frame.Data[1] == 0x55) {
                            can_ota_con.err_cent = 0;
                            can_ota_con.ota_step = OTA_PACK_DATA_STEP;
                        } else {
                            can_ota_con.ota_step = OTA_QUIT_STEP;
                        }
                    }
                }
                if(can_send_cmd.can_tx_frame.Data[0] == 0xA9) {
                    if(rx_can_frame.Data[0] == 0xAA) {
                        can_send_cmd.ask_flag = 1;
                        LOG_I("PACK_TAIL RES:%02X", rx_can_frame.Data[1]);
                        if(rx_can_frame.Data[1] == 0x55) {
                            if(can_ota_con.pack_cout == can_ota_con.total_pack) {
                                can_ota_con.ota_step = OTA_QUIT_STEP;
                            } else {
                                can_ota_con.ota_step = OTA_PACK_HEAD_STEP;
                                can_ota_con.retry = 0;
                            }                       
                        } else {
                            // if(can_ota_con.err_cent++ >= 3)
                            //     can_ota_con.ota_step = OTA_PACK_FAIL_STEP;
                            // else {
                            //     can_ota_con.ota_step = OTA_PACK_HEAD_STEP;
                            //     can_ota_con.retry = 1;
                            // }   
                            can_ota_con.ota_step = OTA_QUIT_STEP;  
                        }
                    }
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
void iot_can_trans_func(uint32_t can_id, uint8_t *data, uint8_t direct)
{
    stc_can_rxframe_t can_dat = {0};
    can_dat.ExtID = can_id;
    def_rtosStaus res;

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

    for(i = 0; i < ARRAY_SIZE(can_cmd_order); i++) {
        if(can_pdu.pdu.pdu1 == can_cmd_order[i].cmd) {
           *can_cmd_t = can_cmd_order[i];
           res = 0;
           break;
        }
    } 
    if(i == ARRAY_SIZE(can_cmd_order)) res = 1;
    return res;  
}

void can_cmd_send_fail(stc_can_rxframe_t  can_tx_frame)
{
    LOG_E("CAN CMD SEND FAIL, can_id:%0x", can_tx_frame.ExtID);
    LOG_E("data:%02x, %02x, %02x, %02x, %02x, %02x, %02x, %02x", can_tx_frame.Data[0], can_tx_frame.Data[1], can_tx_frame.Data[2],\
    can_tx_frame.Data[3], can_tx_frame.Data[4], can_tx_frame.Data[5], can_tx_frame.Data[6], can_tx_frame.Data[7]);
    CAN_PDU_STU can_pdu;
    can_pdu.can_id = can_tx_frame.ExtID;
    switch(can_pdu.pdu.pdu1){
        case 0xeb:
        case 0xd3:
            LOG_E("OTA_QUIT_STEP");
            can_ota_con.ota_step = OTA_QUIT_STEP;
        break;
    }

}

void can_protocol_tx_thread(void *param)
{
    struct can_cmd_order_s can_cmd_t = {0};
    int64_t time_t;
    def_rtosStaus res;
    while(1)
    {
 //       LOG_I("IS RUN");
        can_send_cmd.cmd_send = 0;
        res = def_rtos_queue_wait(can_tx_que, (uint8_t *)&can_send_cmd.can_tx_frame, sizeof(stc_can_rxframe_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            LOG_E("def_rtos_queue_wait  is fail");
            continue;
        }
        LOG_I("%0X", can_send_cmd.can_tx_frame.ExtID);
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
        for(;;) {
            if(can_cmd_t.need_ask) {
                if(can_send_cmd.ask_flag) {
                    break;
                } 
                if(def_rtos_get_system_tick() - time_t >= can_cmd_t.timeout) {
                    if(can_send_cmd.cnt >= can_cmd_t.max_cnt) {
                        can_cmd_send_fail(can_send_cmd.can_tx_frame);
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
     //   LOG_I("IS RUN");
        res = can_data_recv(&rx_can_frame, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
        if(sys_info.ble_can_trans_sw) {
            ble_cmd_can_trans_up(rx_can_frame);
        }
        else {
            can_data_recv_parse(rx_can_frame);
        }
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



void iot_can_state2_fun()
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
    can_pdu.pdu.pdu2 = IOT_STATE_DATA;

    data[3] = car_state_data.map_dir;
    data[4] = car_state_data.cur_dir_range & 0xff;
    data[5] = (car_state_data.cur_dir_range >> 8)& 0xff;
    data[6] = (car_state_data.cur_dir_range >> 16)& 0xff;
    data[7] = 0;
    can_dat.ExtID = can_pdu.can_id;
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    memcpy(can_dat.Data, data, 8);
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
}

void iot_can_navigation_data()
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
    can_pdu.pdu.pdu2 = IOT_NAVIGATION_DATA;

    data[4] = 0;
    data[3] = (car_state_data.total_nav_remaintime >> 24)&0xff;
    data[2] = (car_state_data.total_nav_remaintime >> 16)&0xff;
    data[1] = (car_state_data.total_nav_remaintime >> 8)&0xff;
    data[0] = car_state_data.total_nav_remaintime&0xff;

    data[7] = (car_state_data.total_nav_range>>16)&0xff;
    data[6] = (car_state_data.total_nav_range>>8)&0xff;
    data[5] = car_state_data.total_nav_range&0xff;

    can_dat.ExtID = can_pdu.can_id;
    can_dat.Cst.Control_f.DLC = 8;
    can_dat.Cst.Control_f.IDE = 1;
    can_dat.Cst.Control_f.RTR = 0;
    memcpy(can_dat.Data, data, 8);
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
}


void iot_quit_navigation()
{
    car_state_data.map_dir = 0;
    car_state_data.cur_dir_range = 0;
    iot_can_state2_fun();
    car_state_data.total_nav_remaintime = 0;
    car_state_data.total_nav_range = 0;
    iot_can_navigation_data();
}

void iot_en_power_on_passwd()
{
    car_set_save.en_power_on_psaaword = 1;
    car_control_cmd(CAR_CMD_EN_POWER_ON_PASSWORD);
}
static void iot_can_match_fun()
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
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
}

static void iot_can_state_fun()
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
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_dat, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
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
        ;//
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
    can_ota_con.ota_step = OTA_IDEL_STEP;
    res = def_rtos_queue_create(&can_tx_que, sizeof(stc_can_rxframe_t), 30);
    if(res != RTOS_SUCEESS) {
        LOG_E("can_tx_que is create fail");
    }
    
    LOG_I("can_protocol_init is ok");
}

struct can_ota_data_uart_stu can_ota_data_uart;

static void can_ota_enter()
{
    if(can_ota_con.last_ota_step == can_ota_con.ota_step) return;
    stc_can_rxframe_t can_fram = {0};
    CAN_PDU_STU can_id_s;
    def_rtosStaus res;
    can_id_s.src = IOT_ADR;
    can_id_s.pdu.da = can_ota_con.dev_id;
    can_id_s.pdu.pdu1 = 0xEB;
    can_id_s.dp = 0;
    can_id_s.r = 0;
    can_id_s.p = 1;
    can_id_s.res = 0;
    can_fram.ExtID = can_id_s.can_id;
    can_fram.Cst.Control_f.DLC = 8;
    can_fram.Cst.Control_f.IDE = 1;
    can_fram.Cst.Control_f.RTR = 0;
    can_fram.Data[0] = can_ota_con.dev_id;
    can_fram.Data[1] = 0;
    can_fram.Data[2] = 0;
    can_fram.Data[3] = 0;
    can_fram.Data[4] = 0x4E;
    can_fram.Data[5] = 0x4f;
    can_fram.Data[6] = 0x54;
    can_fram.Data[7] = 0x41;
    LOG_I("enter ota");
    MCU_CMD_MARK(CMD_CAN_OTA_START_INDEX);

    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_fram, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
    can_id_s.pdu.da = 0x00;
    can_fram.ExtID = can_id_s.can_id;
    can_data_send(can_fram);
    can_ota_con.last_ota_step = can_ota_con.ota_step;
}

static uint16_t CalcCRC16(uint8_t *data, uint32_t len)
{
    uint16_t wCRCin = 0x0000;
    uint16_t wCPoly = 0x1021;
    uint8_t wChar = 0;
    uint8_t i = 0;
 
    while (len--)
    {
        wChar = *(data++); 
        wCRCin ^= (wChar << 8); 
        for(i = 0;i < 8;i++) 
        { 
            if(wCRCin & 0x8000)
            {
                wCRCin = (wCRCin << 1) ^ wCPoly; 
            } 
            else 
            {
                wCRCin = wCRCin << 1; 
            }
        }
    }
    return (wCRCin);
}

static void can_ota_head(uint8_t retry)
{
    if(can_ota_con.last_ota_step == can_ota_con.ota_step) return;
    stc_can_rxframe_t can_fram = {0};
    CAN_PDU_STU can_id_s;
    def_rtosStaus res;
    can_id_s.src = IOT_ADR;
    can_id_s.pdu.da = can_ota_con.dev_id;
    can_id_s.pdu.pdu1 = 0xD3;
    can_id_s.dp = 0;
    can_id_s.r = 0;
    can_id_s.p = 1;
    can_id_s.res = 0;
    can_fram.ExtID = can_id_s.can_id;
    can_fram.Cst.Control_f.DLC = 6;
    can_fram.Cst.Control_f.IDE = 1;
   can_fram.Cst.Control_f.RTR = 0;
    can_fram.Data[0] = 0x56;
    if(retry == 0) {
        can_ota_con.pack_cout++;
    }
    can_fram.Data[1] = (can_ota_con.pack_cout - 1)&0xff;
    can_fram.Data[2] = ((can_ota_con.pack_cout -1) >> 8) &0xff;
    can_fram.Data[3] = ((can_ota_con.pack_cout - 1) >> 16) &0xff;
    if(can_ota_con.pack_cout == can_ota_con.total_pack) {
        can_ota_con.pack_len = 4096 - (can_ota_con.total_pack*4096 - can_ota_con.totalen);
    } else {
        can_ota_con.pack_len = 4096;
    }
    flash_partition_read(DEV_APP_ADR, can_ota_con.buf, can_ota_con.pack_len, can_ota_con.read_len);
    can_ota_con.crc16 = CalcCRC16(can_ota_con.buf, can_ota_con.pack_len);
    can_ota_con.frame_cout = 0;
    can_ota_con.pack_frame = 0;
    can_ota_con.read_len += can_ota_con.pack_len;
    can_fram.Data[4] = can_ota_con.pack_len &0xff;
    can_fram.Data[5] = (can_ota_con.pack_len >> 8)&0xff;
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_fram, RTOS_WAIT_FOREVER); 
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
    def_rtos_smaphore_release(can_ota_data_uart.data_sem);
    can_ota_con.last_ota_step = can_ota_con.ota_step;  
    can_ota_data_uart.data_offset = 0;
    can_ota_data_uart.dev_id = can_ota_con.dev_id;
    can_ota_data_uart.pack_num = 0;
    LOG_I("can ota progress %%%d", ((can_ota_con.pack_cout*100)/can_ota_con.total_pack));
    LOG_I("OTA HEAD");
}

// static void can_ota_data()
// {
//     stc_can_rxframe_t can_fram = {0};
//     CAN_PDU_STU can_id_s = {0};
//     uint16_t data_offet;
//     can_id_s.src = IOT_ADR;
//     can_id_s.pdu.da = can_ota_con.dev_id;
//     can_id_s.pdu.pdu1 = 0XD0;
//     if(can_ota_con.frame_cout == 32) {
//         can_ota_con.frame_cout = 0;
//         can_ota_con.pack_frame++;
//     }
//     can_id_s.can_id |= can_ota_con.frame_cout << 24;
//     can_fram.ExtID = can_id_s.can_id;
//     data_offet = (can_ota_con.pack_frame*32 + can_ota_con.frame_cout)* 8;
//     if((can_ota_con.pack_len - data_offet) <= 8) {
//         can_fram.Cst.Control_f.DLC = can_ota_con.pack_len - data_offet;
//         can_ota_con.ota_step = OTA_PACK_TAIL_STEP;
//     } else {
//         can_fram.Cst.Control_f.DLC = 8;
//     }
//     can_fram.Cst.Control_f.IDE = 1;
//     can_fram.Cst.Control_f.RTR = 0;
//     memcpy(can_fram.Data, &can_ota_con.buf[data_offet], can_fram.Cst.Control_f.DLC);
//     can_ota_con.frame_cout++;
//     can_data_send(can_fram);
// }


static void can_ota_mcu_data()
{

    if(def_rtos_semaphore_wait(can_ota_data_uart.data_sem, 10000) != RTOS_SUCEESS) {
        LOG_E("OTA MCU DATA IS FAIL");
        can_ota_con.ota_step = OTA_QUIT_STEP;
        return;
    }
    if(can_ota_con.pack_len == can_ota_data_uart.data_offset) {
        MCU_CMD_MARK(CMD_CAN_OTA_DATA_FINISH_INDEX);
        can_ota_con.ota_step = OTA_PACK_DATA_FINISH_STEP;
        return;
    }
    if(can_ota_con.pack_len - can_ota_data_uart.data_offset < 128) {
        can_ota_data_uart.data_len = can_ota_con.pack_len - can_ota_data_uart.data_offset;
    } else {
        can_ota_data_uart.data_len = 128; 
    }
    memcpy(can_ota_data_uart.data, &can_ota_con.buf[can_ota_data_uart.data_offset], can_ota_data_uart.data_len);
    can_ota_data_uart.data_offset += can_ota_data_uart.data_len;
    can_ota_data_uart.pack_num++;
    LOG_I("can_ota_data_uart.pack_num:%d", can_ota_data_uart.pack_num);
    MCU_CMD_MARK(CMD_CAN_OTA_DATA_INDEX);
}

static void can_ota_mcu_data_finish()
{
    LOG_I("OTA_PACK_DATA_FINISH_STEP");
    if(def_rtos_semaphore_wait(can_ota_data_uart.data_finish_sem, 10000) != RTOS_SUCEESS) {
        LOG_E("OTA MCU DATA FINISH IS FAIL");
        can_ota_con.ota_step = OTA_QUIT_STEP;
        return;
    }
    can_ota_con.ota_step = OTA_PACK_TAIL_STEP;
}

static void can_ota_tail()
{
    if(can_ota_con.last_ota_step == can_ota_con.ota_step) return;
    stc_can_rxframe_t can_fram = {0};
    CAN_PDU_STU can_id_s;
    def_rtosStaus res;
    can_id_s.src = IOT_ADR;
    can_id_s.pdu.da = can_ota_con.dev_id;
    can_id_s.pdu.pdu1 = 0xD3;
    can_id_s.dp = 0;
    can_id_s.r = 0;
    can_id_s.p = 1;
    can_id_s.res = 0;
    can_fram.ExtID = can_id_s.can_id;
    can_fram.Cst.Control_f.DLC = 6;
    can_fram.Cst.Control_f.IDE = 1;
    can_fram.Cst.Control_f.RTR = 0;

    can_fram.Data[0] = 0xa9;
    can_fram.Data[1] = (can_ota_con.pack_cout-1)&0xff;
    can_fram.Data[2] = ((can_ota_con.pack_cout-1) >> 8) &0xff;
    can_fram.Data[3] = ((can_ota_con.pack_cout-1) >> 16) &0xff;
    can_fram.Data[4] = can_ota_con.crc16&0xff;
    can_fram.Data[5] = (can_ota_con.crc16>>8)&0xff;
    res = def_rtos_queue_release(can_tx_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_fram, RTOS_WAIT_FOREVER);
    if(res != RTOS_SUCEESS) {
        LOG_E("def_rtos_queue_release is fail");
    }
    can_ota_con.last_ota_step = can_ota_con.ota_step;
}

void can_ota_quit()
{
    stc_can_rxframe_t can_fram = {0};
    CAN_PDU_STU can_id_s;
  //  def_rtosStaus res;
    uint8_t buf[56];
    uint16_t lenth;
    can_id_s.src = IOT_ADR;
    can_id_s.pdu.da = can_ota_con.dev_id;
    can_id_s.pdu.pdu1 = 0XEE;
    can_id_s.dp = 0;
    can_id_s.r = 0;
    can_id_s.p = 1;
    can_id_s.res = 0;

    can_fram.ExtID = can_id_s.can_id;
    can_fram.Cst.Control_f.DLC = 8;
    can_fram.Cst.Control_f.IDE = 1;
    can_fram.Cst.Control_f.RTR = 0;

    can_fram.Data[0] = can_ota_con.dev_id;
    can_fram.Data[1] = 0;
    can_fram.Data[2] = 0;
    can_fram.Data[3] = 0;
    can_fram.Data[4] = 0x45;
    can_fram.Data[5] = 0x58;
    can_fram.Data[6] = 0x49;
    can_fram.Data[7] = 0x54;
    LOG_I("OTA_QUIT_STEP");

    mcu_data_pack(CMD_CAN_OTA_END, NULL, 0, buf, &lenth);
    mcu_uart_send(buf, lenth);

    def_rtos_task_sleep_ms(100);
    mcu_data_pack(CMD_GPS_POWERON, NULL, 0, buf, &lenth);
    mcu_uart_send(buf, lenth);
    can_data_send(can_fram);

    def_rtos_task_sleep_ms(100);
    can_id_s.pdu.da = 0x00;
    can_fram.ExtID = can_id_s.can_id;
    can_data_send(can_fram);
    can_ota_con.last_ota_step = OTA_QUIT_STEP;
}

int can_ota_task(DEV_ID dev_id)
{
    can_ota_con.ota_step = OTA_IDEL_STEP;
    if(can_ota_con.ota_step != OTA_IDEL_STEP) return FAIL;
    can_ota_con.ota_step = OTA_QUEST_STEP;
    MCU_CMD_MARK(CMD_GPS_POWEROFF_INDEX);
    can_ota_con.last_ota_step = OTA_IDEL_STEP;
    can_ota_con.totalen = flash_partition_size(DEV_APP_ADR);
    if(can_ota_con.totalen%4096) {
        can_ota_con.total_pack = can_ota_con.totalen/4096 + 1;
    } else {
        can_ota_con.total_pack = can_ota_con.totalen/4096;
    }
    can_ota_con.read_len = 0;
    can_ota_con.pack_cout = 0;   
    can_ota_con.dev_id = dev_id;
    can_ota_con.retry = 0;
    if(def_rtos_semaphore_create(&can_ota_data_uart.data_sem, 0) != RTOS_SUCEESS){
        LOG_E("can_ota_data_uart.data_sem is create fail");
        return FAIL;
    }
    if(def_rtos_semaphore_create(&can_ota_data_uart.data_finish_sem, 0) != RTOS_SUCEESS){
        LOG_E("can_ota_data_uart.data_finish_sem is create fail");
        return FAIL;
    }
    while (1)
    {
        switch (can_ota_con.ota_step)
        {
        case OTA_QUEST_STEP:
            can_ota_enter();  
            break;
        case OTA_PACK_HEAD_STEP:
            can_ota_head(can_ota_con.retry);
            break;
        case OTA_PACK_DATA_STEP:
            can_ota_mcu_data();
           // can_ota_data();
            break;
        case OTA_PACK_DATA_FINISH_STEP:
            can_ota_mcu_data_finish();
            break;
        case OTA_PACK_TAIL_STEP:
            can_ota_tail();
            break;
        case OTA_QUIT_STEP:
            can_ota_quit();
            def_rtos_semaphore_delete(can_ota_data_uart.data_sem);
            def_rtos_semaphore_delete(can_ota_data_uart.data_finish_sem);
            if(can_ota_con.pack_cout == can_ota_con.total_pack) {
                LOG_I("CAN OTA IS SUCCESS");
                return OK;
            } else {
                return FAIL;
            }
            break;
        default:
            break;
        }
        def_rtos_task_sleep_ms(100);
    }
    return OK;
}
