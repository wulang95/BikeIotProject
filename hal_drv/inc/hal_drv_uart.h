#ifndef __HAL_DRV_UART_H
#define __HAL_DRV_UART_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


enum {
    PARITY_NONE,
    PARITY_ODD,
    PARITY_EVENT
};

enum{
    UART1,
    UART2
};


void hal_drv_uart_init(uint8_t uart_num, uint32_t buad_rate, uint8_t parity);
void hal_drv_uart_deint(uint8_t uart_num);
int hal_drv_uart_send(uint8_t uart_num, uint8_t *buf, uint16_t len);
uint16_t hal_drv_uart_read(uint8_t uart_num,uint8_t *data, uint16_t len, uint32_t time);   /*阻塞接收*/






#endif