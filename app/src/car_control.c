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

void car_enter_sleep()
{
    uint8_t dat = 0x56;
    can_send_control_cmd(CMD_JUMP_PASSWORD, &dat, 1); 
    can_send_control_cmd(CMD_JUMP_PASSWORD, &dat, 1); 
    can_send_control_cmd(CMD_ENTER_SLEEP, &dat, 1);
    can_send_control_cmd(CMD_ENTER_SLEEP, &dat, 1);
    can_send_control_cmd(CMD_ENTER_SLEEP, &dat, 1);
}
void car_wake_up()
{
    uint8_t dat;
    dat = 0x56;
    can_send_control_cmd(CMD_JUMP_PASSWORD, &dat, 1); 
    can_send_control_cmd(CMD_JUMP_PASSWORD, &dat, 1); 
    dat = 0XA9;
    can_send_control_cmd(CMD_ENTER_WEEK, &dat, 1); 
    can_send_control_cmd(CMD_ENTER_WEEK, &dat, 1); 
    can_send_control_cmd(CMD_ENTER_WEEK, &dat, 1);   
}

void car_init()
{
    memset(&car_info, 0, sizeof(car_info));
    car_wake_up();
    car_set_save.head_light = 0;
    car_set_save.gear = 3;
    can_png_quest(CONTROL_ADR, CONTROL_HWVER1, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SOFTVER1, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SN1, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SN2, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SN3, 0);
    can_png_quest(CONTROL_ADR, CONTROL_SN4, 0);
    can_png_quest(HMI_ADR, HMI_HW_VER1, 0);
    can_png_quest(HMI_ADR, HMI_HW_VER2, 0);
    can_png_quest(HMI_ADR, HMI_SOFT_VER1, 0);
    can_png_quest(HMI_ADR, HMI_SOFT_VER2, 0);
    can_png_quest(HMI_ADR, HMI_SN1, 0);
    can_png_quest(HMI_ADR, HMI_SN2, 0);
    can_png_quest(HMI_ADR, HMI_SN3, 0);
    can_png_quest(HMI_ADR, HMI_SN4, 0);
    LOG_I("car_init is ok");  
}