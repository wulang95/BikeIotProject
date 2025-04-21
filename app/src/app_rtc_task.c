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
int64_t last_time;
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
            if(car_info.lock_sta == CAR_UNLOCK_STA){

            }
            LOG_I("CAR_HEART_EVENT");
            break;
        case NET_HEART_EVENT:
            if(sys_info.paltform_connect) {
                NET_ENGWE_CMD_MARK(HEART_UP);
            }
            LOG_I("NET_HEART_EVENT");
            break;
        case NET_REPORT_EVENT:
            if(sys_info.paltform_connect) {
                NET_ENGWE_CMD_MARK(REGULARLY_REPORT_UP);
            }
            LOG_I("NET_REPORT_EVENT");
            break;
        case NET_REPORT2_EVENT:
            if(sys_info.paltform_connect) {
                NET_ENGWE_CMD_MARK(REGULARLY_REPORT2_UP);
            }
            LOG_I("NET_REPORT2_EVENT");
            break;
        case GPS_TRACK_EVENT:
            if(sys_info.paltform_connect) {
                NET_ENGWE_CMD_MARK(REGULARLY_REPORT_UP);
            }
            LOG_I("GPS_TRACK_EVENT");
            break;
        case CAR_NAVIGATION_QUIT:
            iot_quit_navigation();
            LOG_I("CAR_NAVIGATION_QUIT");
            break;
        case CAR_SET_EN_POWER_PASSWD:
            iot_en_power_on_passwd();
            break;
        default:
            break;
    }
}

void rtc_event_register(RTC_EVENT event, uint32_t time, uint8_t cycle_en_t)
{
    LOG_I("event:%d, time:%d", event, time);
    rtc_week_table[event].vaild = 1;
    // if(cycle_en_t == 0) {
    //     rtc_week_table[event].week_time = time + (hal_drv_rtc_get_timestamp() - last_time);
    // } else {
    //     rtc_week_table[event].week_time = time;
    // }
    rtc_week_table[event].week_time = time + (hal_drv_rtc_get_timestamp() - last_time);
    rtc_week_table[event].reload = time;
    rtc_week_table[event].cycle_en = cycle_en_t;
    def_rtos_smaphore_release(rtc_alarm_sem);
}

void rtc_event_unregister(RTC_EVENT event) 
{
    LOG_I("event:%d", event);
    rtc_week_table[event].vaild = 0;
}

static void rtc_alarm_call_fun()
{
    def_rtos_smaphore_release(rtc_alarm_sem);
}

void app_rtc_init()
{
    hal_drv_rtc_set_time(1725504899);
    hal_drv_rtc_time_print();
    hal_rtc_cfg_init();
    def_rtos_semaphore_create(&rtc_alarm_sem, 0);
    hal_drv_set_alarm_call_fun(rtc_alarm_call_fun);
    LOG_I("app_rtc_init is ok");
}

uint32_t app_rtc_event_query_remain_time(RTC_EVENT event)
{
    if(rtc_week_table[event].vaild){
        return rtc_week_table[event].week_time;
    } else {
        return 0;
    }
}

void app_rtc_event_thread(void *param)
{
    uint8_t i;
    int64_t cur_time,difsec, min_sec = 0xfffffffffffffff;
    last_time = hal_drv_rtc_get_timestamp();
    while(1)
    {
        LOG_I("IS RUN");
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
                        min_sec = rtc_week_table[i].week_time;
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
        LOG_I("hal_drv_set min_sec:%d", min_sec);
        hal_drv_rtc_set_alarm(min_sec);
        last_time = hal_drv_rtc_get_timestamp();
    }
    def_rtos_task_delete(NULL);
}



