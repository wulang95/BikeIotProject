#ifndef __APP_RTC_TASK_H
#define __APP_RTC_TASK_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum{
    CAR_HEART_EVENT = 0,
    BLE_HEART_EVENT,
    EVENT_MAX,
}RTC_EVENT;

void app_rtc_init();
void rtc_event_register(RTC_EVENT event, uint32_t time, uint8_t cycle_en_t);
void app_rtc_event_thread(void *param);
void rtc_event_unregister(RTC_EVENT event);












#endif