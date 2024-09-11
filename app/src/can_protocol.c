#include "app_system.h"

#define DBG_TAG         "can_protocol"

#ifdef CAN_PROTOCOL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

void can_data_recv_parse(stc_can_rxframe_t rx_can_frame)
{
    LOG_I("CAN_ID:%08x, data_len:%d", rx_can_frame.RBUF32_0, rx_can_frame.Cst.Control_f.DLC);
    debug_data_printf("can_revc_data", rx_can_frame.Data, rx_can_frame.Cst.Control_f.DLC);
    can_data_send(rx_can_frame);
}

void can_protocol_rx_thread(void *param)
{
    uint8_t res = RTOS_SUCEESS;
    stc_can_rxframe_t rx_can_frame;
    while(1) {
        res = can_data_recv(&rx_can_frame, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
        can_data_recv_parse(rx_can_frame);
    }
    def_rtos_task_delete(NULL);
}