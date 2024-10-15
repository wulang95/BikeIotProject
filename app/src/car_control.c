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

void car_heart_event()
{
    iot_can_heart_fun();
}

void car_control_cmd(uint8_t cmd)
{
    uint8_t data = 0;
    switch (cmd)
    {
    case CAR_CMD_LOCK:
        break;
    case CAR_CMD_UNLOCK:
        break;
    case CAR_CMD_SET_HEADLIGHT:
        iot_can_png_control(IOT_CONTROL_SET_HEADLIGHT, 0);
        break;
    case CAR_CMD_SET_TAILLIGHT:
        iot_can_png_control(IOT_CONTROL_SET_TAILLIHHT, 0);
        break;
    case CAR_CMD_SET_GEAR:
        iot_can_png_control(IOT_CONTROL_SET_GEAR, 0);
        break;
    case CAR_CMD_SET_SPEED_LIMIT:
        iot_can_cmd_control(CMD_SPEED_LIMIT, (uint8_t *)&car_set_save.speed_limit, 0);
        break;    
    case CAR_CMD_SET_TURN_LIGHT:
        iot_can_png_control(IOT_CONTROL_SET_LEFT_TURN_LIGHT, 0);
        iot_can_png_control(IOT_CONTROL_SET_RIGHT_TURN_LIGHT, 0);
        break;
    case CAR_CMD_SET_MILEAGE_UNIT:
        if(car_set_save.mileage_unit) data |= 1<<7;
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
    default:
        break;
    }
}

void car_init()
{
    memset(&car_info, 0, sizeof(car_info));
    can_png_quest(CONTROL_ADR, CONTROL_HWVER1, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SOFTVER1, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SN1, 0);
    // can_png_quest(CONTROL_ADR, CONTROL_SN2, 0);
    // can_png_quest(CONTROL_ADR, CONTROL_SN3, 0);
    // can_png_quest(CONTROL_ADR, CONTROL_SN4, 0);
    // can_png_quest(HMI_ADR, HMI_HW_VER1, 0);
    // can_png_quest(HMI_ADR, HMI_HW_VER2, 0);
    // can_png_quest(HMI_ADR, HMI_SOFT_VER1, 0);
    // can_png_quest(HMI_ADR, HMI_SOFT_VER2, 0);
    // can_png_quest(HMI_ADR, HMI_SN1, 0);
    // can_png_quest(HMI_ADR, HMI_SN2, 0);
    // can_png_quest(HMI_ADR, HMI_SN3, 0);
    // can_png_quest(HMI_ADR, HMI_SN4, 0);
    car_set_save.mileage_unit= 1;
  //  car_control_cmd(CAR_CMD_SET_ATSPHLIGHT_COLOR_CUSTOM);
    car_control_cmd(CAR_CMD_SET_MILEAGE_UNIT);
    LOG_I("car_init is ok");  
}


void car_lock_control(uint8_t src, uint8_t lock_operate)
{
    if(lock_operate == car_info.lock_sta) {
        return;
    }
    car_info.lock_sta = lock_operate;
}

