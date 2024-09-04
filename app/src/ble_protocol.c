/*蓝牙协议*/
#include "ble_protocol.h"
#include "app_system.h"


void ble_protocol_send_thread(void *param)
{
    while(1){
        def_rtos_task_sleep_ms(200);
    }
    def_rtos_task_delete(NULL);
}

void ble_protocol_recv_thread(void *param)
{
    while(1){
        def_rtos_task_sleep_ms(200);
    }
    def_rtos_task_delete(NULL);
}