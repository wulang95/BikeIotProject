#include "app_rtc_task.h"
#include "app_system.h"
#define DBG_TAG         "app_rtc_task"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif


struct rtc_week_str{
    uint32_t reload;
    uint32_t week_time;
    uint8_t vaild;
    uint8_t cycle_en;
};

struct rtc_week_str rtc_week_table[EVENT_MAX];

void app_event_handler(RTC_EVENT event)
{

}

void rtc_event_register(RTC_EVENT event, uint32_t time, uint8_t cycle_en_t)
{
    rtc_week_table[event].vaild = 1;
    rtc_week_table[event].week_time = time;
    rtc_week_table[event].cycle_en = cycle_en_t;
}

void app_rtc_event_thread(void *param)
{
    
}



