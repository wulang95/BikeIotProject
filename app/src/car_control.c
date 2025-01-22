#include "app_system.h"
#define DBG_TAG         "car_control"

#ifdef APP_MAIN_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

struct car_info_stu car_info;
struct car_set_save_stu car_set_save;
struct car_state_data_stu car_state_data;
def_rtos_queue_t car_cmd_que;
def_rtos_timer_t  key_timer;

void key_timer_fun()
{
    hal_drv_write_gpio_value(O_KEY_LOW, LOW_L);
    LOG_I("KEY TIMER KYE LOW");
}

void car_heart_event()
{
    iot_can_heart_fun();
}

void car_open_lock()
{
 //   uint8_t val = 0XA9;
    hal_drv_write_gpio_value(O_KEY_LOW, HIGH_L);
    def_rtos_timer_start(key_timer, 4000, 0);
    MCU_CMD_MARK(CMD_CAN_UNLOCK_CAR_INDEX);
    LOG_I("CAR_CMD_UNLOCK");
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_WEEK, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_WEEK, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_WEEK, &val, 1);
}

void car_close_lock()
{
//    uint8_t val = 0X56;
    hal_drv_write_gpio_value(O_KEY_LOW, LOW_L);
    MCU_CMD_MARK(CMD_CAN_LOCK_CAR_INDEX);
    
    LOG_I("CAR_LOCK_STA");
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(CMD_SET_ELECLOCK, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_SLEEP, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_SLEEP, &val, 1);
    // iot_can_cmd_control(HMI_CMD_ENTER_SLEEP, &val, 1);
}


int car_control_cmd(uint8_t cmd)
{
    uint8_t data = 0;
    if(sys_info.power_36v == 0) return FAIL;
    LOG_I("cmd:%d", cmd);
    if(cmd == CAR_CMD_LOCK || cmd == CAR_CMD_UNLOCK) {
        ;
    } else if(car_info.lock_sta == CAR_LOCK_STA) {
        LOG_E("CAR_LOCK_STA CMD IS FAIL");
        return FAIL;
    }
    switch (cmd)
    {
    case CAR_CMD_LOCK:
        car_close_lock();
        break;
    case CAR_CMD_UNLOCK:
        car_open_lock();
        break;
    case CAR_CMD_SET_HEADLIGHT:
        iot_can_png_control(IOT_CONTROL_SET_HEADLIGHT, 0);
        data = car_set_save.head_light << 7 | car_info.taillight_sta << 6;
        iot_can_cmd_control(HMI_CMD_SET_HEADLIGHT, &data, 0);
        break;
    case CAR_CMD_SET_TAILLIGHT:
        iot_can_png_control(IOT_CONTROL_SET_TAILLIHHT, 0);
        data = car_set_save.tail_light << 7 | car_info.headlight_sta << 6;
        iot_can_cmd_control(HMI_CMD_SET_HEADLIGHT, &data, 0);
        break;
    case CAR_CMD_SET_GEAR:
        iot_can_png_control(IOT_CONTROL_SET_GEAR, 0);
        iot_can_cmd_control(HMI_CMD_SET_GEAR, &car_set_save.gear, 0);
        break;
    case CAR_CMD_SET_SPEED_LIMIT:
        iot_can_cmd_control(CMD_SPEED_LIMIT, (uint8_t *)&car_set_save.speed_limit, 1);
        break;    
    case CAR_CMD_SET_TURN_LIGHT:
        iot_can_png_control(IOT_CONTROL_SET_LEFT_TURN_LIGHT, 0);
        iot_can_png_control(IOT_CONTROL_SET_RIGHT_TURN_LIGHT, 0);
        data = car_set_save.left_turn_light << 7 | car_set_save.right_turn_light << 6;
        iot_can_cmd_control(HMI_CMD_SET_TURNLIGHT, &data, 0);
        break;
    case CAR_CMD_SET_MILEAGE_UNIT:
        if(car_set_save.mileage_unit) data |=  1 << 0;  //1<<7
        else data = 0;
        iot_can_cmd_control(CMD_MILEAGE_UNIT, &data, 0);
        break;
    case CAR_CMD_SET_POWER_ON_PASSWORD:
        iot_can_cmd_control(CMD_SET_POWER_ON_PASSWORD, &car_set_save.power_on_psaaword[0], 0);
        break;
    case CAR_CMD_SET_ATSPHLIGHT_MODE:
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_MODE, &car_set_save.atmosphere_light_set.light_mode, 0);
        break;
    case CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM:
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_R_VAL, &car_set_save.atmosphere_light_set.custom_red, 0);
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_G_VAL, &car_set_save.atmosphere_light_set.custom_green, 0);
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_B_VAL, &car_set_save.atmosphere_light_set.custom_blue, 0);
        break;
    case CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL:
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_BRIGHTNESS, &car_set_save.atmosphere_light_set.brightness_val, 0);
        break;
    case CAR_CMD_SET_ATSPHLIGHT_TURN:
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_TURN, &car_set_save.atmosphere_light_set.turn_linght_sta, 0);
        break;
    case CAR_CMD_SET_ATSPHLIGHT_COLORTYPE:
        iot_can_cmd_control(CMD_SET_ATMOSPHERE_LIGHT_COLOUR, &car_set_save.atmosphere_light_set.color, 0);
        break;
    case CAR_CMD_JUMP_PASSWORD:
        data = 0x56;
        iot_can_cmd_control(HMI_CMD_JUMP_PASSWORD, &data, 0);
        break;
    case CAR_CMD_EN_POWER_ON_PASSWORD:
        iot_can_cmd_control(CMD_EN_POWER_ON_PASSWORD, &car_set_save.en_power_on_psaaword, 0);  
        break;
    default:
        break;
    }
    return OK;
}

void CAR_CMD_MARK(CAR_CMD_Q car_cmd_q)
{
    LOG_I("cmd:%d, src:%d, seq:%d", car_cmd_q.cmd, car_cmd_q.src, car_cmd_q.net_car_control.seq);
    def_rtos_queue_release(car_cmd_que, sizeof(CAR_CMD_Q), (uint8_t *)&car_cmd_q, RTOS_WAIT_FOREVER);
}

int car_control_operate_res(CAR_CMD_Q car_cmd_q)
{
    switch(car_cmd_q.cmd){
        case CAR_CMD_LOCK:
            if(car_info.lock_sta == CAR_LOCK_STA) 
                return OK;
        break;
        case CAR_CMD_UNLOCK:
            if(car_info.lock_sta == CAR_UNLOCK_STA) 
                return OK;
        break;
        case CAR_CMD_SET_HEADLIGHT:
            if(car_info.headlight_sta == car_set_save.head_light) 
                return OK;
        break;
        case CAR_CMD_SET_TAILLIGHT:
            if(car_info.taillight_sta == car_set_save.tail_light) 
                return OK;
        break;
        case CAR_CMD_SET_GEAR:
            if(car_info.gear == car_set_save.gear)
                return OK;
        break;
        case CAR_CMD_SET_SPEED_LIMIT:
            if(car_info.speed_limit == car_set_save.speed_limit) 
                return OK;
        break;
        case CAR_CMD_SET_TURN_LIGHT:
            if(car_info.right_turn_light_sta == car_set_save.right_turn_light && \
            car_info.left_turn_light_sta == car_set_save.left_turn_light){
                return OK;
            }
        break;
        case CAR_CMD_SET_MILEAGE_UNIT:
            if(car_info.hmi_info.display_unit == car_set_save.mileage_unit)
                return OK;
        break;
        case CAR_CMD_SET_POWER_ON_PASSWORD:
        case CAR_CMD_SET_ATSPHLIGHT_MODE:
        case CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM:
        case CAR_CMD_SET_ATSPHLIGHT_BRIGHTVAL:
        case CAR_CMD_SET_ATSPHLIGHT_TURN:
        case CAR_CMD_SET_ATSPHLIGHT_COLORTYPE:
        case CAR_CMD_JUMP_PASSWORD:
        case CAR_CMD_EN_POWER_ON_PASSWORD:
        return OK;
        break;
    }
    return FAIL;
}

void car_control_thread(void *param)
{
    def_rtosStaus res;
    CAR_CMD_Q car_cmd_q;
    int64_t cmd_timeout;
    uint8_t data[128];
    uint16_t len;
    while(1) {
        res = def_rtos_queue_wait(car_cmd_que, (uint8_t *)&car_cmd_q, sizeof(CAR_CMD_Q), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        cmd_timeout = def_rtos_get_system_tick();
        if(car_cmd_q.cmd == CAR_CMD_LOCK) {
            if(car_lock_control(car_cmd_q.cmd, CAR_LOCK_STA) != OK){
                len = net_engwe_cmdId_operate_respos(data, car_cmd_q.net_car_control, 0x02, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, data, len, car_cmd_q.net_car_control.seq);
            }
        } else if(car_cmd_q.cmd == CAR_CMD_UNLOCK) {
            if(car_lock_control(car_cmd_q.cmd, CAR_UNLOCK_STA) != OK){
                len = net_engwe_cmdId_operate_respos(data, car_cmd_q.net_car_control, 0x02, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, data, len, car_cmd_q.net_car_control.seq);
            }
        } else if(car_control_cmd(car_cmd_q.cmd) != OK) {
            len = net_engwe_cmdId_operate_respos(data, car_cmd_q.net_car_control, 0x02, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, data, len, car_cmd_q.net_car_control.seq);
        }
        for(;;) {
            if(def_rtos_get_system_tick() - cmd_timeout > 2000) {
                len = net_engwe_cmdId_operate_respos(data, car_cmd_q.net_car_control, 0x02, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, data, len, car_cmd_q.net_car_control.seq);
                break;
            }
            def_rtos_task_sleep_ms(5);
            if(car_control_operate_res(car_cmd_q) == OK) {
                len = net_engwe_cmdId_operate_respos(data, car_cmd_q.net_car_control, 0x01, 0);
                net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, data, len, car_cmd_q.net_car_control.seq);
                break;
            }
        }
    }
    def_rtos_task_delete(NULL);
}

void car_init()
{
    def_rtos_queue_create(&car_cmd_que, sizeof(CAR_CMD_Q), 5);
    memset(&car_info, 0, sizeof(car_info));
    def_rtos_timer_create(&key_timer, NULL, key_timer_fun, NULL); 

 //   car_set_save.mileage_unit= 1;
  //  car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM);
 //   car_control_cmd(CAR_CMD_SET_MILEAGE_UNIT);
    LOG_I("car_init is ok");  
}


int car_lock_control(uint8_t src, uint8_t lock_operate)
{
    static int64_t lock_time_t = 0;
    if(sys_info.power_36v == 0) return FAIL;
    if(lock_operate == car_info.lock_sta) {
        return OK;
    }
     if(def_rtos_get_system_tick() - lock_time_t < 5000 && car_info.lock_sta == CAR_LOCK_STA) {
        return FAIL;
     }
    LOG_I("%d, %d", src, lock_operate);
    if(lock_operate == CAR_LOCK_STA) {
        lock_time_t = def_rtos_get_system_tick();
        car_control_cmd(CAR_CMD_LOCK);
    } else {
        car_control_cmd(CAR_CMD_UNLOCK);
    }
    return OK;
}

