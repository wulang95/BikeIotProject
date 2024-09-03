/*app_main入口*/
#include "sys_core.h"
#define DBG_TAG         "app_main"

#ifdef APP_MAIN_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif



def_rtos_task_t app_task_start = NULL;
def_rtos_task_t ble_control_task_recv = NULL;
def_rtos_task_t ble_control_task_send = NULL;
def_rtos_task_t ble_protocol_task_recv = NULL;
def_rtos_task_t ble_protocol_task_send = NULL;


void app_start_thread(void *param)
{
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_task_create(&ble_control_task_recv, 1024, TASK_PRIORITY_NORMAL, ble_control_recv_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_control_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_control_task_send, 1024, TASK_PRIORITY_NORMAL, ble_control_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_control_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_protocol_task_recv, 1024, TASK_PRIORITY_NORMAL, ble_protocol_recv_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_protocol_recv_thread is create fail!");
    }
    err = def_rtos_task_create(&ble_protocol_task_send, 1024, TASK_PRIORITY_NORMAL, ble_protocol_send_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("ble_protocol_send_thread is create fail!");
    }
}

void app_main()
{
    def_rtosStaus err = RTOS_SUCEESS;
    sys_init();         /*外设驱动初始化   系统参数初始化 */
    car_init();         /*  对整车初始化  */
    LOG_I("app_main is printf1");
    LOG_I("app_main is printf2");
    LOG_I("app_main is printf3");
    err =def_rtos_task_create(&app_task_start, 1024, TASK_PRIORITY_NORMAL, app_start_thread);
    if(err != RTOS_SUCEESS){
        LOG_E("app_start_thread is create fail!");
    }
    def_rtos_task_delete(app_task_start);
}