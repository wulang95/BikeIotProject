#include "hal_drv_gpio.h"
#include "hal_resources_config.h"


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

