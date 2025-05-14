#ifndef _APP_ADC_CALAC_H_
#define _APP_ADC_CALAC_H_


#include <stdio.h>
#include <string.h>
#include <stdlib.h>




uint8_t app_get_bat_soc(uint16_t bat_vol);
uint16_t app_get_bat_val();
uint16_t app_get_sys_power_val();
void app_bat_charge_check();
void app_get_bat_temp_info();








#endif