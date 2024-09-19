#include "app_rtc_task.h"
#include "app_system.h"
#include "hal_drv_rtc.h"
#define DBG_TAG         "app_rtc_task"

#ifdef APP_RTC_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

def_rtos_sem_t rtc_alarm_sem;

struct rtc_week_str{
    uint32_t reload;
    uint32_t week_time;
    uint8_t vaild;
    uint8_t cycle_en;
};

struct rtc_week_str rtc_week_table[EVENT_MAX];

void rtc_event_handler(RTC_EVENT rtc_e)
{
    switch(rtc_e){
        case CAR_HEART_EVENT:
            LOG_I("CAR_HEART_EVENT");
            break;
        case CAR_SELF:
            LOG_I("CAR_SELF");
            break;
        default:
            break;
    }
}

void rtc_event_register(RTC_EVENT event, uint32_t time, uint8_t cycle_en_t)
{
    rtc_week_table[event].vaild = 1;
    rtc_week_table[event].week_time = time;
    rtc_week_table[event].reload = time;
    rtc_week_table[event].cycle_en = cycle_en_t;
    hal_drv_rtc_set_alarm(time);
}

static void rtc_alarm_call_fun()
{
    def_rtos_smaphore_release(rtc_alarm_sem);
}

void app_rtc_init()
{
    hal_drv_rtc_set_time(1725504899);
    hal_drv_rtc_time_print();
    def_rtos_semaphore_create(&rtc_alarm_sem, 0);
    rtc_event_register(CAR_HEART_EVENT, 10, 1);
    rtc_event_register(CAR_SELF, 5, 1);
    hal_drv_set_alarm_call_fun(rtc_alarm_call_fun);
    LOG_I("app_rtc_init is ok");
}

void app_rtc_event_thread(void *param)
{
    uint8_t i;
    int64_t last_time, cur_time,difsec, min_sec = 0xfffffffffffffff;
    last_time = hal_drv_rtc_get_timestamp();
    while(1)
    {
        def_rtos_semaphore_wait(rtc_alarm_sem, RTOS_WAIT_FOREVER);
        cur_time = hal_drv_rtc_get_timestamp();
        difsec = cur_time - last_time;
        for(i = 0; i< EVENT_MAX; i++){
            if(rtc_week_table[i].vaild){
                if(rtc_week_table[i].week_time > difsec) {
                    rtc_week_table[i].week_time -= difsec;
                } else {
                    rtc_event_handler(i);
                    if(rtc_week_table[i].cycle_en){
                        rtc_week_table[i].week_time = rtc_week_table[i].reload;
                    } else {
                        rtc_week_table[i].vaild = 0;
                    }
                }
            }
        }
        for(i = 0; i < EVENT_MAX; i++) {
            if(rtc_week_table[i].vaild){
                if(rtc_week_table[i].week_time < min_sec) {
                    min_sec = rtc_week_table[i].week_time;
                }
            }
        }
        hal_drv_rtc_set_alarm(min_sec);
        last_time = hal_drv_rtc_get_timestamp();
    }
    def_rtos_task_delete(NULL);
}



