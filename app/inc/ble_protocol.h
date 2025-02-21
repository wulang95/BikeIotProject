#ifndef __BLE_PROTOCOL_H
#define __BLE_PROTOCOL_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "mcu_uart.h"


void ble_protocol_recv_thread(void *param);
void ble_protocol_send_thread(void *param);
void ble_heart_event();
void ble_log_out(uint8_t *data, uint16_t data_len);
void ble_cmd_can_trans_up(stc_can_rxframe_t can_dat);




#endif