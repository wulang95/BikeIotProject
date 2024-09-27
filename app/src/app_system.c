#include "app_system.h"
#include "hal_drv_flash.h"
#include "hal_drv_gpio.h"
#include "hal_drv_uart.h"
#include "hal_virt_at.h"

#define DBG_TAG         "app_system"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"
struct sys_info_stu sys_info;
struct sys_config_stu sys_config;
struct sys_param_set_stu sys_param_set;
void assert_handler(const char *ex_string, const char *func, size_t line)
{
    LOG_E("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    while(1);
}


void flash_partition_erase(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case APP1_ADR:
            hal_drv_flash_erase(APP1_ADDR, APP1_SIZE);
            break;
        case APP2_ADR:
            hal_drv_flash_erase(APP2_ADDR, APP2_SIZE);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_erase(BOOT_CONFIG_ADDR, BOOT_CONFIG_SIZE);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_erase(SYS_CONFIG_ADDR, SYS_CONFIG_SIZE);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_erase(BACK_SYS_CONFIG_ADDR, BACK_SYS_CONFIG_SIZE);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_erase(CAR_SET_ADD, CAR_SET_SIZE);
            break;
        default:
            break;
    }
}

void flash_partition_write(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    switch(flash_part){
        case APP1_ADR:
            hal_drv_flash_write(APP1_ADDR + shift, data, lenth);
            break;
        case APP2_ADR:
            hal_drv_flash_write(APP2_ADDR + shift, data, lenth);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_write(BOOT_CONFIG_ADDR, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_write(SYS_CONFIG_ADDR, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_write(BACK_SYS_CONFIG_ADDR, data, lenth);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_write(CAR_SET_ADD, data, lenth);
            break;
        default:
            break;
    }
}

void flash_partition_read(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    switch(flash_part) {
        case APP1_ADR:
            hal_drv_flash_read(APP1_ADDR + shift, data, lenth);
            break;
        case APP2_ADR:
            hal_drv_flash_read(APP2_ADDR + shift, data, lenth);
            break;
        case BOOT_CONFIG_ADR:
            hal_drv_flash_read(BOOT_CONFIG_ADDR, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_read(SYS_CONFIG_ADDR, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_read(BACK_SYS_CONFIG_ADDR, data, lenth);
            break;
        case CAR_SET_ADR:
            hal_drv_flash_read(CAR_SET_ADD, data, lenth);
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

}

static void hal_drv_init()
{
    hal_drv_set_gpio_irq(I_SENSOR_IN, RISING_EDGE, DOWN_MODE, sensor_input_handler);
    hal_drv_gpio_init(O_RED_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_WHITE_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_BAT_CHARGE_CON, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(I_MCU_CONEC, IO_INPUT, DOWN_MODE, L_NONE);
    hal_drv_gpio_init(I_BLE_CON_SIG, IO_INPUT, DOWN_MODE, L_NONE);
    hal_drv_gpio_init(O_KEY_HIGH, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_BLE_WEEK_SIG, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_MCU_WEEK, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(I_36VPOWER_DET, IO_INPUT, DOWN_MODE, L_NONE);
    hal_drv_gpio_init(O_BLE_POWER, IO_OUTPUT, PULL_NONE_MODE, LOW_L);

    hal_drv_uart_init(BLE_UART, BLE_BAUD, BLE_PARITY);
    hal_drv_uart_init(MCU_UART, MCU_BAUD, MCU_PARITY);
    hal_virt_at_init();
    LOG_I("hal_drv_init is ok");
}

void sys_param_set_init()
{
    sys_param_set.unlock_car_heart_sw = 0;
    sys_param_set.unlock_car_heart_interval = 10;
    sys_param_set.net_heart_interval = 240;
}

void sys_config_init()
{
    memset(&sys_config, 0, sizeof(sys_config));
    memcpy(&sys_config.manufacturer[0], DEFAULT_MANUFACTURER, strlen(DEFAULT_MANUFACTURER));
    memcpy(&sys_config.sn[0], DEFAULT_SN, strlen(DEFAULT_SN));
    memcpy(&sys_config.dev_type, DEFAULT_DEV_TYPE, strlen(DEFAULT_DEV_TYPE));
    memcpy(&sys_config.apn, DEFAULT_APN, strlen(DEFAULT_APN));
    memcpy(&sys_config.ip, DEFAULT_IP, strlen(DEFAULT_IP));
    memcpy(&sys_config.DSN, DEFAULT_SN, strlen(DEFAULT_SN));
    sys_config.port = DEFAULT_PORT;
    memset(&sys_info, 0, sizeof(sys_info));
    LOG_I("sys_config_init is ok");
}

def_rtos_task_t app_system_task = NULL;
def_rtos_timer_t system_timer;
def_rtos_sem_t system_task_sem;


void app_system_thread(void *param)
{
    def_rtosStaus res;
    int64_t csq_time_t = 0;
    while (1)
    {
        res = def_rtos_semaphore_wait(system_task_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
        if(sys_info.sys_updata_falg & 0x01) {
            rtc_event_register(NET_HEART_EVENT,  sys_param_set.net_heart_interval, 1);
            if(sys_param_set.unlock_car_heart_sw) {
                rtc_event_register(CAR_HEART_EVENT, sys_param_set.unlock_car_heart_interval, 1);
            } else {
                rtc_event_unregister(CAR_HEART_EVENT);
            }
            sys_info.sys_updata_falg &= ~0x01;
        }
        if(def_rtos_get_system_tick() - csq_time_t > 10) {
            net_update_singal_csq();
            csq_time_t = def_rtos_get_system_tick();
        }
//        if(car_info.lock_sta == CAR_LOCK_STA) {
      //      car_heart_event();
 //       }
        if(hal_drv_read_gpio_value(I_BLE_CON_SIG)){
      //      ble_heart_event();
        }
    }
    def_rtos_task_delete(NULL);
}

void system_timer_fun()
{
    def_rtos_smaphore_release(system_task_sem);
}


void app_sys_init()
{
    hal_drv_init();
    app_led_init();
    app_rtc_init();
    ble_control_init();
    mcu_uart_init();
    net_control_init();
    sys_config_init();
    sys_param_set_init();
    can_protocol_init();
    net_protocol_init();
    rtc_event_register(NET_HEART_EVENT,  sys_param_set.net_heart_interval, 1);
    if(sys_param_set.unlock_car_heart_sw){
        rtc_event_register(CAR_HEART_EVENT, sys_param_set.unlock_car_heart_interval, 1);
    }
    def_rtos_semaphore_create(&system_task_sem, 0);
    def_rtos_timer_create(&system_timer, app_system_task, system_timer_fun, NULL);
    def_rtos_timer_start(system_timer, 1000, 1);
    LOG_I("app_sys_init is ok");
}