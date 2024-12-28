#include "app_system.h"
#include "hal_drv_flash.h"
#include "hal_drv_uart.h"
#include "hal_virt_at.h"
#include "ql_fs.h"
#define DBG_TAG         "app_system"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"
struct sys_info_stu sys_info;
struct sys_config_stu sys_config;
struct sys_config_stu sys_config_back;
struct sys_param_set_stu sys_param_set;
struct sys_set_var_stu sys_set_var;
struct sensor_calibration_data_stu sensor_calibration_data;
NET_NW_INFO nw_info;
int ota_fd;

void assert_handler(const char *ex_string, const char *func, size_t line)
{
    LOG_E("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    while(1);
}




int flash_partition_erase(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd > 0) ql_fclose(ota_fd);
        //    hal_drv_flash_erase(DEV_APP_ADDR, DEV_APP_SIZE);
            ota_fd = ql_fopen(OTA_FILE, "wb+");
            if(ota_fd < 0) return FAIL;
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_erase(SYS_CONFIG_ADDR, SYS_CONFIG_SIZE);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_erase(BACK_SYS_CONFIG_ADDR, BACK_SYS_CONFIG_SIZE);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_erase(SYS_SET_ADDR, SYS_SET_SIZE);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_erase(SENSOR_CALIBRATION_DATA_ADDR, SENSOR_CALIBRATION_DATA_SEIZE);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_write(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    int res;
    switch(flash_part){
        case DEV_APP_ADR:
      //      hal_drv_flash_write(DEV_APP_ADDR + shift, data, lenth);
            if(ota_fd < 0) {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
            } 
            res = ql_fseek(ota_fd, shift, 0);
            if(res < 0) {
                LOG_E("ql_fseek is fail");
                return FAIL;
            }
            res = ql_fwrite(data, lenth, 1, ota_fd);
            if(res < 0) {
                LOG_E("ql_fwrite is fail!");
                return FAIL;
            }
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_write(SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_write(BACK_SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_write(SYS_SET_ADDR + shift, data, lenth);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_write(SENSOR_CALIBRATION_DATA_ADDR + shift, data, lenth);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_read(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    int res;
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd  < 0) {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
            } 
            res = ql_fseek(ota_fd, shift, 0);
            if(res < 0) {
                LOG_E("ql_fseek is fail");
                return FAIL;
            }
            res = ql_fread(data, lenth, 1, ota_fd);
            if(res < 0) {
                LOG_E("ql_fread is fail");
                return FAIL;
            }
//            hal_drv_flash_read(DEV_APP_ADDR + shift, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_read(SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_read(BACK_SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_read(SYS_SET_ADDR + shift, data, lenth);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_read(SENSOR_CALIBRATION_DATA_ADDR + shift, data, lenth);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_size(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd > 0) {
                return ql_fsize(ota_fd);
            } else {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
                return ql_fsize(ota_fd);
            }  
        break; 
        default:
        break;
    }
    return OK;
}

void sys_param_save(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case SYS_CONFIG_ADR:
            sys_config.crc32 = GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4);
            flash_partition_erase(SYS_CONFIG_ADR);
            flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
        break;
        case BACK_SYS_CONFIG_ADR:
            sys_config_back.crc32 = GetCrc32((uint8_t *)&sys_config_back, sizeof(sys_config_back) - 4);
            flash_partition_erase(BACK_SYS_CONFIG_ADR);
            flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        break;
        case SYS_SET_ADR:
            sys_param_set.crc32 = GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4);
            flash_partition_erase(SYS_SET_ADR);
            flash_partition_write(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
        break;
        case SENSEOR_CALIBRATION_ADR:
            sensor_calibration_data.crc32 = GetCrc32((uint8_t *)&sensor_calibration_data, sizeof(sensor_calibration_data) - 4);	
            flash_partition_erase(SENSEOR_CALIBRATION_ADR);
            flash_partition_write(SENSEOR_CALIBRATION_ADR, (void *)&sensor_calibration_data, sizeof(sensor_calibration_data) , 0);
        break;
        default:
            break;
    }
}

void debug_data_printf(char *str_tag, uint8_t *in_data, uint16_t data_len)
{
    uint16_t i, len;
    char data_str[4];
    char str[512];
    sprintf(str, "%s[%d]:", str_tag, data_len);
    len = data_len > 512?512:data_len;
    for(i = 0; i < len; i++){
        sprintf(data_str, "%02x ", in_data[i]);
        strncat(str, data_str, strlen(data_str));
    }
    LOG_I("%s", str);
}


int64_t systm_tick_diff(int64_t time)
{
    int64_t sec;
    const int64_t cur_t = def_rtos_get_system_tick();
    sec = cur_t - time;
    return sec;
}

void sensor_input_handler()
{
    LOG_I("algo is week");
}

void ble_connect_handler()
{
    LOG_I("ble connect");
    week_time("ble", -1);
    sys_info.ble_connect = 1;
}

static void hal_drv_init()
{
    hal_drv_set_gpio_irq(I_SENSOR_IN, RISING_EDGE, DOWN_MODE, sensor_input_handler);
    hal_drv_gpio_init(O_RED_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_WHITE_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_BAT_CHARGE_CON, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_MCU_CONEC, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);

    hal_drv_set_gpio_irq(I_BLE_CON_SIG, RISING_EDGE, DOWN_MODE, ble_connect_handler);
 //   hal_drv_gpio_init(I_BLE_CON_SIG, IO_INPUT, DOWN_MODE, LOW_L);

    hal_drv_gpio_init(O_KEY_LOW, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_KEY_HIGH, IO_OUTPUT, PULL_NONE_MODE, LOW_L);

    hal_drv_gpio_init(O_BLE_WEEK_SIG, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(I_MCU_WEEK, IO_INPUT, DOWN_MODE, L_NONE);
    hal_drv_gpio_init(I_36VPOWER_DET, IO_INPUT, UP_MODE, L_NONE);
    hal_drv_gpio_init(O_BLE_POWER, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);

    hal_drv_uart_init(BLE_UART, BLE_BAUD, BLE_PARITY);
    hal_drv_uart_init(MCU_UART, MCU_BAUD, MCU_PARITY);
    hal_drv_write_gpio_value(O_MCU_CONEC, HIGH_L);
    hal_virt_at_init();
    hal_drv_write_gpio_value(O_BAT_CHARGE_CON, HIGH_L);
    hal_drv_write_gpio_value(O_BLE_WEEK_SIG, LOW_L);
    LOG_I("hal_drv_init is ok");
}

void sys_param_set_default_init()
{
    memset(&sys_param_set, 0, sizeof(sys_param_set));
    sys_param_set.magic = IOT_MAGIC;
    sys_param_set.ota_cnt = 0;
    sys_param_set.unlock_car_heart_sw = 0;
    sys_param_set.unlock_car_heart_interval = 10;
    sys_param_set.net_heart_interval = 240;
    sys_param_set.crc32 = GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4);
    flash_partition_erase(SYS_SET_ADR);
    flash_partition_write(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
}

void sys_param_set_init()
{
    flash_partition_read(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
    if(sys_param_set.magic != IOT_MAGIC || sys_param_set.crc32 != GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4)) {
        sys_param_set_default_init();
        LOG_E("sys_set_param save is fail!");
    }
    LOG_I("ota_cnt:%d", sys_param_set.ota_cnt);
}

static void sys_config_default_init()
{
    memset(&sys_config, 0, sizeof(sys_config));
    memcpy(&sys_config.manufacturer[0], DEFAULT_MANUFACTURER, strlen(DEFAULT_MANUFACTURER));
    memcpy(&sys_config.sn[0], DEFAULT_SN, strlen(DEFAULT_SN));
    memcpy(&sys_config.dev_type, DEFAULT_DEV_TYPE, strlen(DEFAULT_DEV_TYPE));
    memcpy(&sys_config.apn, DEFAULT_APN, strlen(DEFAULT_APN));
    memcpy(&sys_config.ip, DEFAULT_IP, strlen(DEFAULT_IP));
    memcpy(&sys_config.DSN, DEFAULT_SN, strlen(DEFAULT_SN));
    sys_config.port = DEFAULT_PORT;
    sys_config.magic = IOT_MAGIC;
    sys_config.alive_sta = 0;
    sys_config.crc32 = GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4);
    flash_partition_erase(SYS_CONFIG_ADR);
    flash_partition_erase(BACK_SYS_CONFIG_ADR);
    flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
    flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
}

void sys_config_init()
{
    flash_partition_read(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
    if(sys_config.magic != IOT_MAGIC || sys_config.crc32 != GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4)) {
        flash_partition_read(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        if(sys_config_back.magic != IOT_MAGIC || sys_config_back.crc32 != GetCrc32((uint8_t *)&sys_config_back, sizeof(sys_config_back) - 4)) {
            LOG_E("sys_config save is fail!");
            sys_config_default_init();
        } else {
            sys_config = sys_config_back;
            flash_partition_erase(SYS_CONFIG_ADR);
            flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
        }
    } else {
        flash_partition_read(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        if(sys_config_back.magic != IOT_MAGIC || sys_config_back.crc32 != GetCrc32((uint8_t *)&sys_config_back, sizeof(sys_config_back) - 4)) {
            sys_config_back = sys_config;
            flash_partition_erase(BACK_SYS_CONFIG_ADR);
            flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        }
    }
    LOG_I("APN:%s, DSN:%s, IP:%s, PORT:%d", sys_config.apn, sys_config.DSN, sys_config.ip, sys_config.port);
}

def_rtos_task_t app_system_task = NULL;
def_rtos_timer_t system_timer;
def_rtos_sem_t system_task_sem;

void app_system_thread(void *param)
{
    def_rtosStaus res;
    int64_t csq_time_t = 0;
    int64_t ble_info_up_time_t = 0;
    uint16_t bat_val;
    uint8_t TEMP = 0;
    while (1)
    {
  //      LOG_I("IS RUN");
        res = def_rtos_semaphore_wait(system_task_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }

        if(sys_info.sys_updata_falg != 0) {
            if(sys_info.sys_updata_falg & 0x01) {
                sys_info.sys_updata_falg &= ~0x01;
                flash_partition_erase(SYS_SET_ADR);
                flash_partition_write(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
            } 
            if(sys_info.sys_updata_falg & 0x02) {
                sys_info.sys_updata_falg &= ~0x02;
                flash_partition_erase(SYS_CONFIG_ADR);
                flash_partition_erase(BACK_SYS_CONFIG_ADR);
                flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
                flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
            }
        }
        if(def_rtos_get_system_tick() - gps_resh_time_t > 10*1000) {
        //   gps_info.RefreshFlag = 0;
        }   
        if(def_rtos_get_system_tick() - csq_time_t > 3*1000) {
            net_update_singal_csq();
            nw_info = hal_drv_get_operator_info();
            LOG_I("MCC:%d, mnc:%d, lac:%d, cid:%d", nw_info.mcc, nw_info.mnc, nw_info.lac, nw_info.cid);
            LOG_I("net_state:%d, act:%d, csq:%d, bit_error_rate:%d", nw_info.net_state, nw_info.act, nw_info.csq, nw_info.bit_error_rate);
            csq_time_t = def_rtos_get_system_tick();
          //  LOG_I("CSQ:%d", gsm_info.csq);
        //    LOG_I("ptich:%.2f,roll:%.2f,yaw:%.2f",euler_angle[0],euler_angle[1],euler_angle[2]);
            LOG_I("car_info.speed_limit:%d, car_info.pedal_speed:%d", car_info.speed_limit, car_info.pedal_speed);
            LOG_I("car_info.total_odo:%d, car_info.single_odo:%d", car_info.total_odo, car_info.single_odo);
            LOG_I("car_info.remain_odo:%d, car_info.wheel:%d", car_info.remain_odo, car_info.wheel);
            LOG_I("car_info.current:%d, car_info.pedal_torque:%d", car_info.current, car_info.pedal_torque);
            LOG_I("car_info.bus_voltage:%d, car_info.calorie:%d", car_info.bus_voltage, car_info.calorie);
            LOG_I("car_info.hmi_info.display_unit:%d, car_info.gear:%d", car_info.hmi_info.display_unit, car_info.gear);
            LOG_I("car_info.speed:%d, car_info.current_limit:%d", car_info.speed, car_info.current_limit);
            // if(TEMP == 1) {
            //     imu_algo_timer_start();
            //     TEMP = 0;
            // }
            if(TEMP == 0) {
                GPS_Start(GPS_MODE_CONT);
            //    voice_play_mark(ALARM_VOICE);
                // LOG_I("imu_algo_timer_stop");
                // imu_algo_timer_stop();
                TEMP = 1;
            } 
        }
        if(sys_set_var.ble_bind_infoClean) {
            ble_cmd_mark(BLE_DELETE_BIND_INDEX);
            sys_set_var.ble_bind_infoClean = 0;
        }
        if(sys_set_var.hid_lock_sw) {
            if(sys_set_var.hid_lock_sw == 1) {

            } else if(sys_set_var.hid_lock_sw ==2) {

            }
            sys_set_var.hid_lock_sw = 0;
        }
        if(sys_set_var.car_power_en) {
            if(sys_set_var.car_power_en == 1) {

            } else if(sys_set_var.car_power_en == 2) {
                
            }
            sys_set_var.car_power_en = 0;
        }
        if(sys_set_var.iot_active) {
            if(sys_set_var.iot_active == 1) {
                    
            } else if(sys_set_var.iot_active == 2) {

            }
            sys_set_var.iot_active = 0;
        }
        if(sys_info.car_init == 0) {
            if(car_info.lock_sta == CAR_UNLOCK_ATA) {
                sys_info.car_init = 1;
            }
        }
       
//        if(car_info.lock_sta == CAR_LOCK_STA) {
      //      car_heart_event();
 //       }
        if(hal_drv_read_gpio_value(I_BLE_CON_SIG)) {   //蓝牙连接状态检测
            if(car_info.lock_sta == CAR_UNLOCK_ATA) {
                ble_heart_event();
            } else {
                if(def_rtos_get_system_tick() - ble_info_up_time_t > 5000) {
                    ble_heart_event();
                    ble_info_up_time_t = def_rtos_get_system_tick();
                }
            }   
        } else if(sys_info.ble_connect == 1){  
            week_time("ble", 10);       
            sys_info.ble_connect = 0;
            LOG_I("BLE DISCONNECT!");
        }

        if(hal_drv_read_gpio_value(I_36VPOWER_DET) == 0) {    //36V电源检测
            sys_info.power_36v = 1;
        } else {
            sys_info.power_36v = 0;
        }

        hal_adc_value_get(BAT_ADC_VAL, (int *)&bat_val);
        LOG_I("bat_val:%d", bat_val);
    }
    def_rtos_task_delete(NULL);
}

void system_timer_fun()
{
    def_rtos_smaphore_release(system_task_sem);
}

void system_timer_stop()
{
    def_rtos_timer_stop(system_timer);
}

void system_timer_start()
{
    def_rtos_timer_start(system_timer, 1000, 1);
}


void sys_param_init()
{
    ota_fd = -1;
    sys_config_init();
    sys_param_set_init();
    LOG_I("OTA_CNT:%d", sys_param_set.ota_cnt);
    memset(&sys_info, 0, sizeof(sys_info));
}


void ota_test()
{
    char ota_str[] = "123456789";
    char ota_s[4] = {0};
    if(flash_partition_erase(DEV_APP_ADR) < 0) {
        LOG_E("ota erase is error");
    }

    if(flash_partition_write(DEV_APP_ADR, ota_str, sizeof(ota_str), 0) < 0){
        LOG_E("ota write is error");
    }
    if(flash_partition_read(DEV_APP_ADR, ota_s, 3, 6) < 0) {
        LOG_E("ota read is error");
    } else {
        LOG_I("%s", ota_s);
    }
}
extern void electron_fence_test();
void app_sys_init()
{
    hal_drv_init();
    app_led_init();
    app_rtc_init();
    qmi8658_sensor_init();
    ble_control_init();
    net_control_init();
    sys_param_init();
    can_protocol_init();
    net_protocol_init();
    app_http_ota_init();
    low_power_init();
    register_module("sys");
    register_module("ble");
    register_module("lock");
 //   flash_partition_erase(DEV_APP_ADR);
     week_time("sys", -1); 
     electron_fence_test();
 //   rtc_event_register(NET_HEART_EVENT,  6, 1);
 //   rtc_event_register(NET_HEART_EVENT,  sys_param_set.net_heart_interval, 1);
    if(sys_param_set.unlock_car_heart_sw) {
        rtc_event_register(CAR_HEART_EVENT, sys_param_set.unlock_car_heart_interval, 1);
    }
    def_rtos_semaphore_create(&system_task_sem, 0);
    def_rtos_timer_create(&system_timer, app_system_task, system_timer_fun, NULL);
    def_rtos_timer_start(system_timer, 1000, 1);
    LOG_I("app_sys_init is ok");
}