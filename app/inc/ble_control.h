#ifndef     __BLE_CONTROL_H
#define     __BLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


#include "sys_core.h"
#include "hal_drv_uart.h"
#define BLE_UART    UART1


#define GAP_ADVTYPE_MANUFACTURER_SPECIFIC       0xFF
#define GAP_ADVTYPE_LOCAL_NAME_COMPLETE         0x09
#define GAP_ADVTYPE_16BIT_MORE                  0x02

enum {
    CMD_BLE_ADV_START = 0,      /*0x04*/
    CMD_BLE_GET_MAC,            /*0x01*/
    CMD_BLE_SET_ADV_DATA,       /*0x0c*/
    CMD_BLE_SET_SCANRSP_DATA,   /*0x02*/
    CMD_BLE_GET_VER,            /*0x03*/
    CMD_BLE_ADV_STOP,           /*0x0b*/
    CMD_BLE_DISCONNECT,         /*0x05*/
    CMD_BLE_SET_ADV_INTERVAL,   /*0x09*/
    CMD_BLE_SET_CON_PARAM,      /*0x0a*/
    CMD_BLE_HID_UNLOCK,         /*0x07*/
    CMD_BLE_HID_LOCK,           /*0x08*/ 
    CMD_BLE_TRANS,              /*0x06*/
    CMD_BLE_MAX,
};


void ble_send_data(uint8_t *data, uint16_t len);
void ble_control_init();
void ble_control_recv_thread(void *param);
void ble_control_send_thread(void *param);














#ifdef __cplusplus
}
#endif

#endif