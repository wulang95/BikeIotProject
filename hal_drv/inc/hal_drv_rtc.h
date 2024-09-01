#ifndef __HAL_DRV_RTC_H
#define __HAL_DRV_RTC_H




typedef void(*RTC_ALARM_CALL_FUN)();


void hal_drv_rtc_init();
void hal_drv_rtc_set_time(uint64_t timestamp);
uint64_t hal_drv_rtc_get_timestamp();
uint64_t hal_drv_rtc_set_alarm(uint64_t sec, RTC_ALARM_CALL_FUN rtc_alarm_call);
void hal_drv_rtc_time_print();
void hal_drv_rtc_deinit();


#endif