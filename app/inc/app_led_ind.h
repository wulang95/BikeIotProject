#ifndef     __APP_LED_IND_H
#define     __APP_LED_IND_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



typedef enum {
    LED_SYS_FAULT = 0,
    LED_SYS_OTA,
    LED_ALL_OFF,
    LED_WAIT_ALIVE,
    LED_ARARM,
    LED_TEST,
    LED_IND_MAX,
}LED_IND;

void app_set_led_ind(LED_IND led_ind_sta);
void app_led_init();















#ifdef __cplusplus
}
#endif

#endif