#ifndef __APP_VIRT_UART_H
#define __APP_VIRT_UART_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

#define     VIRT_BUF_LEN    256

struct virt_uart_at_stu {
    uint8_t cmd_source;
    char virt_txbuf[VIRT_BUF_LEN];
    uint16_t len;
};

struct virt_uart_at_stu virt_uart_at;

enum {
    AT_VIRT_BLE = 0,
    AT_VIRT_GSM
};

void app_virt_uart_thread(void *param);














#ifdef __cplusplus
}
#endif

#endif