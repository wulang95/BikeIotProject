#ifndef __APP_RTC_TASK_H
#define __APP_RTC_TASK_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum{
    CAR_HEART_EVENT = 0,
    NET_HEART_EVENT,
    NET_REPORT_EVENT,
    NET_REPORT2_EVENT,
    SENSOR_CHECK_EVENT,
    GPS_TRACK_EVENT,
    CAR_AUTO_POWER_OFF,
    CAR_NAVIGATION_QUIT,
    CAR_SET_EN_POWER_PASSWD,
    BAT_MCU_ADC_GET_EVENT,
    SYSTEM_REBOOT_EVENT,
    BAT_CHARGE_TIMEOUT_EVENT,
    EVENT_MAX,
}RTC_EVENT;

void app_rtc_init();
void rtc_event_register(RTC_EVENT event, uint32_t time, uint8_t cycle_en_t);
void app_rtc_event_thread(void *param);
void rtc_event_unregister(RTC_EVENT event);
uint32_t app_rtc_event_query_remain_time(RTC_EVENT event);











#endif