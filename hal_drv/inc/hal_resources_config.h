#ifndef __HAL_CONFIG_H
#define __HAL_CONFIG_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ql_gpio.h"
#include "quec_pin_index.h"


/*配置GPIO的引脚号*/
#define GPIO1_PIN       5 
#define GPIO2_PIN       6
#define GPIO3_PIN       7
#define GPIO4_PIN       1
#define GPIO5_PIN       20
#define GPIO8_PIN       104
#define GPIO13_PIN      21
#define GPIO14_PIN      40
#define GPIO15_PIN      41
#define GPIO18_PIN      37
#define GPIO19_PIN      36
#define GPIO20_PIN      95
#define GPIO21_PIN      97
#define GPIO22_PIN      83
#define GPIO24_PIN      38
#define GPIO25_PIN      30

#define UART2_TX_PIN                     QUEC_PIN_UART2_TXD
#define UART2_RX_PIN                     QUEC_PIN_UART2_RXD
#define UART2_TX_FUNC                    0x01
#define UART2_RX_FUNC                    0x01

#define PIN_NONE        -1

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
















#endif
