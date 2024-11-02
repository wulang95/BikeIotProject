#include "app_led_ind.h"
#include "hal_drv_gpio.h"
#include "rtos_port_def.h"
#include "bike_app_config.h"


#define DBG_TAG         "app_led_ind"

#ifdef APP_LED_IND_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"

def_rtos_timer_t led_timer;

enum {
    LED_STOP = 0,
    LED_LOOP = 1,
};

typedef struct {
    uint8_t led;
    uint8_t value;
    uint16_t active;
}LED_CONTROL_STU;

LED_CONTROL_STU sys_fault_ind[] = {
        {O_RED_IND, 1,  100},
        {O_RED_IND, 0,  4900},
        {O_RED_IND, 0,  LED_LOOP},
    };

LED_CONTROL_STU sys_ota_ind[] = {
    {O_WHITE_IND, 1, 100},
    {O_WHITE_IND, 0, 2000},
    {O_WHITE_IND, 0, LED_STOP},
};

LED_CONTROL_STU *led_control_que[] = {
        sys_fault_ind,
        sys_ota_ind
};



enum {
    LED_W = 0,
    LED_R,
    LED_MAX,
};

static LED_CONTROL_STU *led_cur_ind;
static uint8_t led_step;
void led_set_value(uint8_t led, uint8_t value)
{
    hal_drv_write_gpio_value(led, value);
}

void app_set_led_ind(LED_IND led_ind_sta)
{
    led_step = 0;
    led_set_value(O_WHITE_IND, 0);
    led_set_value(O_RED_IND, 0);
    led_cur_ind = led_control_que[led_ind_sta];
    def_rtos_timer_start(led_timer, 10, 0);
}



void app_led_timer_func(void *param)
{
    switch (led_cur_ind[led_step].active){
        case LED_LOOP:
            led_step = 0;
            def_rtos_timer_start(led_timer, 10, 0);
        break;
        case LED_STOP:
            led_set_value(led_cur_ind[led_step].led, led_cur_ind[led_step].value);
        break;
        default:
            led_set_value(led_cur_ind[led_step].led, led_cur_ind[led_step].value);
            def_rtos_timer_start(led_timer, led_cur_ind[led_step].active, 0);
            led_step++;
        break;
    }
    
}

void app_led_init()
{
    def_rtos_timer_create(&led_timer, NULL, app_led_timer_func, NULL);
    LOG_I("app_led_init is ok");
}








