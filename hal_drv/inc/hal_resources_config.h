#ifndef __HAL_CONFIG_H
#define __HAL_CONFIG_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "ql_gpio.h"
#include "quec_pin_index.h"


/*配置GPIO的引脚号*/
#define GPIO0_PIN       4
#define GPIO1_PIN       5 
#define GPIO2_PIN       6
#define GPIO3_PIN       7
#define GPIO4_PIN       108
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


















#endif
