/*app_main入口*/
#include "app_system.h"
#define DBG_TAG         "app_main"

#ifdef APP_MAIN_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"


def_rtos_task_t app_task_start = NULL;
def_rtos_task_t ble_control_task_recv = NULL;
def_rtos_task_t ble_control_task_send = NULL;
def_rtos_task_t ble_protocol_task_recv = NULL;
def_rtos_task_t ble_protocol_task_send = NULL;
def_rtos_task_t app_rtc_event_task = NULL;
def_rtos_task_t app_virt_uart_task = NULL;
def_rtos_task_t can_protocol_recv_task = NULL;
def_rtos_task_t mcu_uart_recv_task = NULL;
def_rtos_task_t net_socket_task = NULL;
def_rtos_task_t pdp_active_task = NULL;
def_rtos_task_t can_protocol_send_task = NULL;
extern def_rtos_task_t app_system_task;
def_rtos_task_t net_protocol_send_task = NULL;
def_rtos_task_t mcu_uart_send_task =NULL;
def_rtos_task_t http_ota_task = NULL;
void app_start_thread(void *param)
{
    LOG_I("app start thread is run");
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_task_create(&ble_control_task_recv, 2048, TASK_PRIORITY_NORMAL, ble_control_recv_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_control_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_control_task_send, 2048, TASK_PRIORITY_NORMAL, ble_control_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_control_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_protocol_task_recv, 2048, TASK_PRIORITY_NORMAL, ble_protocol_recv_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_protocol_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_protocol_task_send, 2048, TASK_PRIORITY_NORMAL, ble_protocol_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_protocol_send_thread is create fail!");
    }
    err = def_rtos_task_create(&app_rtc_event_task, 1024, TASK_PRIORITY_NORMAL, app_rtc_event_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("app_rtc_event_thread is create fail!");
    }

    err = def_rtos_task_create(&app_virt_uart_task, 2048, TASK_PRIORITY_NORMAL, app_virt_uart_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("app_virt_uart_thread is create fail!");
    }
    err = def_rtos_task_create(&can_protocol_recv_task, 2048, TASK_PRIORITY_NORMAL, can_protocol_rx_thread); 
    if(err != RTOS_SUCEESS){
        LOG_E("can_protocol_rx_thread is create fail!");
    }
    err = def_rtos_task_create(&mcu_uart_recv_task, 2048, TASK_PRIORITY_NORMAL, mcu_uart_recv_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("mcu_uart_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&pdp_active_task, 2048, TASK_PRIORITY_NORMAL, pdp_active_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("pdp_active_thread is create fail!");
    }
    err = def_rtos_task_create(&net_socket_task, 1024*4, TASK_PRIORITY_NORMAL, net_socket_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("net_socket_thread is create fail!");
    }
    err = def_rtos_task_create(&can_protocol_send_task, 1024*4, TASK_PRIORITY_NORMAL, can_protocol_tx_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("can_protocol_tx_thread is create fail!");
    }
    err = def_rtos_task_create(&app_system_task, 2048, TASK_PRIORITY_NORMAL, app_system_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("system_timer_thread is create fail!");
    }
    err = def_rtos_task_create(&net_protocol_send_task, 2048, TASK_PRIORITY_NORMAL, net_protocol_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("net_protocol_send_thread is create fail!");
    }

    err = def_rtos_task_create(&mcu_uart_send_task, 2048, TASK_PRIORITY_NORMAL, mcu_uart_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("mcu_uart_send_thread is create fail!");
    }

    err = def_rtos_task_create(&http_ota_task, 1024*4, TASK_PRIORITY_NORMAL, app_http_ota_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("app_http_ota_thread is create fail!");
    }
    
    def_rtos_task_delete(NULL);
}

void app_main()
{
    def_rtosStaus err = RTOS_SUCEESS;
    app_sys_init();         /*外设驱动初始化   系统参数初始化 */
    car_init();             /*  对整车初始化  */
    LOG_I("VERSION softver:%s, hwsoft:%s", SOFTVER, HWVER);
    LOG_I("DATA TIME:%s_%s", __DATE__, __TIME__);
    err =def_rtos_task_create(&app_task_start, 1024, TASK_PRIORITY_NORMAL, app_start_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("app_start_thread is create fail!");
    }
}