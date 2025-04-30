#include "app_system.h"

#define DBG_TAG         "app_adc_calac"

#ifdef APP_LED_IND_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"

float bat_vol_soc_table[] = {3000, 3680, 3740, 3770, 3790, 3820, 3870, 3920, 3980, 4060, 4200};/*mv*/

uint8_t app_get_bat_soc(uint16_t bat_vol)
{
    uint8_t i, soc;
    uint8_t middel_size = sizeof(bat_vol_soc_table)/sizeof(bat_vol_soc_table[0])/2;
    if(bat_vol < bat_vol_soc_table[middel_size]){
        for(i = 0; i < middel_size; i++) {
            if(bat_vol > bat_vol_soc_table[i]) {
                continue;
            } else {
                break;
            }
        }
        if(i == 0) {
            soc = 0;
        } else {
            soc = (i - 1)*10 + (bat_vol - bat_vol_soc_table[i-1]) *10/(bat_vol_soc_table[i] - bat_vol_soc_table[i-1]);
        }
    } else if(bat_vol > bat_vol_soc_table[middel_size]) {
        for(i = middel_size; i < sizeof(bat_vol_soc_table)/sizeof(bat_vol_soc_table[0]); i++){
            if(bat_vol > bat_vol_soc_table[i]){
                continue;
            } else {
                break;
            }
        }
        if(i == sizeof(bat_vol_soc_table)/sizeof(bat_vol_soc_table[0])){
            soc = 100;
        } else {
            soc = (i - 1)*10 +  (bat_vol - bat_vol_soc_table[i-1])*10/(bat_vol_soc_table[i] - bat_vol_soc_table[i-1]);
        }
    } else {
        soc = 50;
    }
    return soc;
}

uint16_t app_get_bat_val()
{
    float bat_f;
    bat_f = (float)sys_info.power_adc.bat_val_adc*2.5/4095;
    return (uint16_t)(bat_f*sys_info.power_adc.bat_val_rate*1000);
}

uint16_t app_get_sys_power_val()
{
    float bat_f;
    bat_f = (float)sys_info.power_adc.sys_power_adc*2.5/4095;
    return (uint16_t)(bat_f*sys_info.power_adc.sys_power_rate*10);
}

void app_bat_charge_check()
{
    if(sys_info.bat_charge_state == BAT_CHARGE_ON && (sys_info.power_36v == 0 || sys_info.bat_temp > 65 || sys_info.bat_val >= 4200)) {
        MCU_CMD_MARK(CMD_MCU_BAT_CHARGE_OFF_INDEX);
        rtc_event_register(BAT_MCU_ADC_GET_EVENT, sys_info.adc_discharge_get_interval, 1);
        return;
    }

    if(sys_info.bat_charge_state == BAT_CHARGE_OFF && sys_info.power_36v == 1 && sys_info.bat_temp < 60 && sys_info.bat_val < 4100){
        MCU_CMD_MARK(CMD_MCU_BAT_CHARGE_ON_INDEX);
        rtc_event_register(BAT_MCU_ADC_GET_EVENT, sys_info.adc_charge_get_interval, 1);
        return;
    }
    
}