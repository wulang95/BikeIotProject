/*蓝牙协议*/
#include "ble_protocol.h"
#include "app_system.h"
#define DBG_TAG         "ble_protol"

#ifdef BLE_PROTOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"


void ble_protocol_send_thread(void *param)
{
    while(1){
        def_rtos_task_sleep_ms(200);
    }
    def_rtos_task_delete(NULL);
}

void ble_protocol_recv_thread(void *param)
{
    uint8_t buf[256], tx_buf[256];
    uint16_t len,tx_lenth;
    LOG_I("ble_protocol_recv_thread is run");
    while(1){
        len = ble_trans_data_block_read(buf, 256, RTOS_WAIT_FOREVER);
        LOG_I("ble trans is recv, len:%d", len);
        if(len == 0)continue;
        debug_data_printf("ble_protol", buf, len);
        ble_cmd_pack(CMD_BLE_TRANS, buf, len, tx_buf, &tx_lenth);
        ble_send_data(tx_buf, tx_lenth);
    }
    def_rtos_task_delete(NULL);
}