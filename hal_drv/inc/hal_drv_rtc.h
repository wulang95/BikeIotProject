#ifndef __HAL_DRV_RTC_H
#define __HAL_DRV_RTC_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>




void hal_drv_rtc_init();
void hal_drv_rtc_set_time(int64_t timestamp);
int64_t hal_drv_rtc_get_timestamp();
void hal_drv_set_alarm_call_fun(void(*rtc_alarm_call)());
void hal_drv_rtc_set_alarm(int64_t sec);
void hal_drv_rtc_time_print();
void hal_drv_rtc_deinit();
void hal_rtc_cfg_init();

#endif