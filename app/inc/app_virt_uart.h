#ifndef __APP_VIRT_UART_H
#define __APP_VIRT_UART_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define     VIRT_BUF_LEN    256

void app_virt_uart_write(uint8_t cmd_src, char *cmd_str);

enum {
    AT_VIRT_BLE = 0,
    AT_VIRT_GSM
};

void app_virt_uart_thread(void *param);













#ifdef __cplusplus
}
#endif

#endif