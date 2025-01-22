#include   "app_system.h"
#define DBG_TAG         "app_error"

#ifdef APP_ERROR_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"




void iot_error_print(uint8_t err_type, uint8_t error)
{
    char iot_error_str[IOT_MAX_ERROR][30]={"SENSOR_ERROR", "GPS_ERROR", "BLE_ERROR","MCU_CONN_ERROR", "AUDIO_ERROR"};
    LOG_E("%s", iot_error_str[error]);
}


void iot_error_set(uint8_t err_type, uint8_t error)
{
    switch(err_type){
        case CAR_ERROR_TYPE:
            sys_info.car_error |= 1 << error;
        break;
        case IOT_ERROR_TYPE:
            sys_info.iot_error |= 1<< error;
            iot_error_print(IOT_ERROR_TYPE, error);
        break;
    }
}


void iot_error_clean(uint8_t err_type, uint8_t error)
{
    switch(err_type){
        case CAR_ERROR_TYPE:
            sys_info.car_error &= ~(1 << error);
        break;
        case IOT_ERROR_TYPE:
            sys_info.iot_error &= ~(1<< error);
        break;
    }
}

uint8_t iot_error_check(uint8_t err_type, uint8_t error)
{
    switch(err_type) {
        case CAR_ERROR_TYPE:
            if(sys_info.car_error & (1<< error)) return 1;
        break;
        case IOT_ERROR_TYPE:
            if(sys_info.iot_error & (1 << error)) return 1;
        break;
    }
    return 0;
}