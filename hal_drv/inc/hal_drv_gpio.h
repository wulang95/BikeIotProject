#ifndef   __HAL_DRV_GPIO_H
#define   __HAL_DRV_GPIO_H


enum {
    IO_INPUT,
    IO_OUTPUT
};

enum {
    DOWN_MODE,
    UP_MODE
};

enum {
    RISING_EDGE,
    FALL_EDGE
};


typedef void(*GPIO_IRQ_FUN)();
void hal_drv_set_gpio_mode(uint8_t gpio_num, uint8_t mode);
void hal_drv_write_gpio_value(uint8_t gpio_num, uint8_t value);
uint8_t hal_drv_read_gpio_value(uint8_t gpio_num);
void hal_drv_set_gpio_irq(uint8_t gpio_num, uint8_t trigger, GPIO_IRQ_FUN gpio_irq_call_fun);










#endif