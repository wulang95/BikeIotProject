#include "hal_drv_gpio.h"
#include "hal_resources_config.h"


struct hal_gpio_config_table_s {
    uint8_t pin;
    uint8_t gpio_func;
};

struct hal_gpio_config_table_s hal_gpio_config_table[GPIO_MAX] = {
    {PIN_NONE,       0},        //GPIO_0
    {GPIO1_PIN,      0},        //GPIO_1
    {GPIO2_PIN,      0},        //GPIO_2
    {GPIO3_PIN,      0},        //GPIO_3
    {GPIO4_PIN,      0},        //GPIO_4
    {GPIO5_PIN,      0},        //GPIO_5
    {PIN_NONE,       0},        //GPIO_6
    {PIN_NONE,       0},        //GPIO_7
    {GPIO8_PIN,      0},        //GPIO_8
    {PIN_NONE,       0},        //GPIO_9
    {PIN_NONE,       0},        //GPIO_10
    {PIN_NONE,       0},        //GPIO_11
    {PIN_NONE,       0},        //GPIO_12
    {GPIO13_PIN,     0},        //GPIO_13
    {GPIO14_PIN,     0},        //GPIO_14
    {GPIO15_PIN,     0},        //GPIO_15
    {PIN_NONE,       0},        //GPIO_16
    {PIN_NONE,       0},        //GPIO_17
    {GPIO18_PIN,     0},        //GPIO_18
    {GPIO19_PIN,     0},        //GPIO_19
    {GPIO20_PIN,     4},        //GPIO_20
    {GPIO21_PIN,     4},        //GPIO_21
    {GPIO22_PIN,     0},        //GPIO_22
    {PIN_NONE,       0},        //GPIO_23
    {GPIO24_PIN,     1},        //GPIO_24
    {GPIO25_PIN,     1},        //GPIO_25
    {PIN_NONE,       0},        //GPIO_26
    {PIN_NONE,       0},        //GPIO_27
    {PIN_NONE,       0},        //GPIO_28
    {PIN_NONE,       0},        //GPIO_29
    {PIN_NONE,       0},        //GPIO_30
    {PIN_NONE,       0},        //GPIO_31  
};

void hal_drv_gpio_init(uint8_t gpio_num, IO_DIR_E dir, IO_MODE_E mode, IO_VAL_E value)
{
    if(gpio_num < GPIO_0 || gpio_num > GPIO_31) return;
    if(hal_gpio_config_table[gpio_num].pin == PIN_NONE) return;
    ql_pin_set_func(hal_gpio_config_table[gpio_num].pin, hal_gpio_config_table[gpio_num].gpio_func);

    ql_gpio_init(gpio_num, dir, mode, value);
}

void hal_drv_write_gpio_value(uint8_t gpio_num, IO_VAL_E value)
{
    ql_gpio_set_level(gpio_num, value);
}

uint8_t hal_drv_read_gpio_value(uint8_t gpio_num)
{
    ql_LvlMode  gpio_lvl;
    ql_gpio_get_level(gpio_num, &gpio_lvl);
    return gpio_lvl;
}

void hal_drv_set_gpio_irq(uint8_t gpio_num, IO_EDGE_E trigger, IO_MODE_E e_mode, void(*gpio_irq_call_fun)(void *))
{
    ql_pin_set_func(hal_gpio_config_table[gpio_num].pin, hal_gpio_config_table[gpio_num].gpio_func);
    ql_int_register(gpio_num, EDGE_TRIGGER, DEBOUNCE_EN, trigger, e_mode, gpio_irq_call_fun, NULL);
    ql_int_enable(gpio_num);
}

