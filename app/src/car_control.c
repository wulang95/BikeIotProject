#include "app_system.h"
#define DBG_TAG         "car_control"

#ifdef APP_MAIN_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

struct car_info_stu car_info;

void car_init()
{
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