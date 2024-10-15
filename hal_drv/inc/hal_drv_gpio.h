#ifndef   __HAL_DRV_GPIO_H
#define   __HAL_DRV_GPIO_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
typedef enum{
    IO_INPUT,
    IO_OUTPUT
}IO_DIR_E;

typedef enum {
    PULL_NONE_MODE,
    DOWN_MODE,
    UP_MODE
}IO_MODE_E;

typedef enum{
    LOW_L,
    HIGH_L,
    L_NONE
}IO_VAL_E;

typedef enum {
    RISING_EDGE,
    FALL_EDGE,
    BOTH_EDGE
}IO_EDGE_E;

void hal_drv_gpio_init(uint8_t gpio_num, IO_DIR_E dir, IO_MODE_E mode, IO_VAL_E value);
void hal_drv_write_gpio_value(uint8_t gpio_num, IO_VAL_E value);
uint8_t hal_drv_read_gpio_value(uint8_t gpio_num);
void hal_drv_set_gpio_irq(uint8_t gpio_num, IO_EDGE_E trigger, IO_MODE_E e_mode, void(*gpio_irq_call_fun)(void *));
void hal_adc_value_get(int channel, int *adc);









#endif