#ifndef     __BLE_CONTROL_H
#define     __BLE_CONTROL_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


     

#define GAP_ADVTYPE_MANUFACTURER_SPECIFIC       0xFF
#define GAP_ADVTYPE_LOCAL_NAME_COMPLETE         0x09
#define GAP_ADVTYPE_16BIT_MORE                  0x02

enum {
    BLE_ADV_START_INDEX = 0,        /*0x04*/
    BLE_GET_MAC_INDEX,              /*0x01*/
    BLE_SET_ADV_DATA_INDEX,         /*0x0c*/
    BLE_SET_SCANRSP_DATA_INDEX,     /*0x02*/
    BLE_GET_VER_INDEX,              /*0x03*/
    BLE_ADV_STOP_INDEX,             /*0x0b*/
    BLE_DISCONNECT_INDEX,           /*0x05*/
    BLE_SET_ADV_INTERVAL_INDEX,     /*0x09*/
    BLE_SET_CON_PARAM_INDEX,        /*0x0a*/
    BLE_HID_UNLOCK_INDEX,           /*0x07*/
    BLE_HID_LOCK_INDEX,             /*0x08*/ 
    BLE_TRANS_INDEX,                /*0x06*/
    BLE_VIRT_INDEX,                 /*0x0d*/
    BLE_INDEX_MAX
};

enum {
    CMD_BLE_ADV_START = 0x04,           /*0x04*/
    CMD_BLE_GET_MAC = 0x01,            /*0x01*/
    CMD_BLE_SET_ADV_DATA = 0x0c,       /*0x0c*/
    CMD_BLE_SET_SCANRSP_DATA = 0x02,   /*0x02*/
    CMD_BLE_GET_VER = 0x03,            /*0x03*/
    CMD_BLE_ADV_STOP = 0x0b,           /*0x0b*/
    CMD_BLE_DISCONNECT = 0x05,         /*0x05*/
    CMD_BLE_SET_ADV_INTERVAL = 0x09,   /*0x09*/
    CMD_BLE_SET_CON_PARAM = 0x0a,      /*0x0a*/
    CMD_BLE_HID_UNLOCK = 0x07,         /*0x07*/
    CMD_BLE_HID_LOCK = 0x08,           /*0x08*/ 
    CMD_BLE_TRANS = 0x06,              /*0x06*/
    CMD_BLE_VIRT_AT = 0x0d,             /*0x0d*/ 
};

extern struct ble_info_s ble_info;
void ble_send_data(uint8_t *data, uint16_t len);
void ble_control_init();
void ble_control_recv_thread(void *param);
void ble_control_send_thread(void *param);
uint16_t ble_trans_data_block_read(uint8_t *buf, uint16_t len, uint32_t time_out);
void ble_set_adv_data(uint8_t *data, uint8_t len);
void ble_set_scanrsp_data(uint8_t *data, uint8_t len);
void ble_cmd_pack(uint8_t cmd, uint8_t *data, uint16_t len, uint8_t *buff, uint16_t *buf_len);













#ifdef __cplusplus
}
#endif

#endif