#include "app_virt_uart.h"
#include "hal_virt_at.h"
#include "app_system.h"


#define DBG_TAG         "app_virt_uart"

#ifdef APP_VIRT_UART_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"


struct virt_uart_at_stu  virt_uart_at;

void app_virt_uart_thread(void *param)
{
    uint16_t len;
    char at_buf[256];
    while(1) {
        memset(at_buf, 0, sizeof(at_buf));
        len = hal_virt_at_read(at_buf, 256, RTOS_WAIT_FOREVER);
        LOG_I("[%d]%s", len, at_buf);
        if(len == 0) continue;
        if(virt_uart_at.cmd_source == AT_VIRT_BLE) {
            ble_cmd_pack(CMD_BLE_VIRT_AT, (uint8_t *)at_buf, len, (uint8_t *)virt_uart_at.virt_txbuf, &virt_uart_at.len);
            ble_send_data((uint8_t *)virt_uart_at.virt_txbuf, virt_uart_at.len);
        }
    }
    def_rtos_task_delete(NULL);
}




