#include "app_system.h"

#define DBG_TAG         "http_upgrade"

#ifdef HTTP_UPGRADE_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"



struct http_upgrade_info_stu http_upgrade_info;

void app_http_ota_thread(void *param)
{
 //   int64_t http_start_time_t;
    def_rtosStaus res;
    while(1){
        res = def_rtos_semaphore_wait(http_upgrade_info.http_ota_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        } 
   //     http_start_time_t = def_rtos_get_system_tick();
        if(sys_info.power_36v == 0 &&  sys_info.bat_soc < 80) {
            http_upgrade_info.ota_sta = IOT_BAT_LOW;
            http_upgrade_info.download_fail_cent = 0;
            http_upgrade_info.download_start_byte = 0;
            NET_CMD_MARK(NET_CMD_HTTP_UPGRADE_STATE_U6);
            continue;
        }
    }
    def_rtos_task_delete(NULL);
}