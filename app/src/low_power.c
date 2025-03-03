#include   "low_power.h"
#include   "app_system.h"
#include   "ql_power.h"
#define DBG_TAG         "low_power"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"


typedef struct {
    int week_time;   //模块多久后进入低功耗 单位：S， -1时不进入休眠。
    char *module_str;  //模块名称
} SYS_MODULE;

static SYS_MODULE sys_module_table[10];
def_rtos_task_t low_power_task = NULL;
static def_rtos_timer_t low_power_timer;
static def_rtos_sem_t enter_power_sem; 
static def_rtos_sem_t exit_power_sem;
static uint8_t low_power_flag;  //进入低功耗标志， 为1标志进入低功耗
int register_module(char *str)
{
    uint8_t i;
    if(str == NULL || sizeof(str) > 20) {
        return FAIL;
    }
    for(i = 0; i < 10; i++) {
        if(sys_module_table[i].module_str != NULL && (strcmp(sys_module_table[i].module_str, str) == 0)) {
            return OK;
        }
        else if(sys_module_table[i].module_str == NULL) {
            sys_module_table[i].module_str = malloc(sizeof(str));
            strcpy(sys_module_table[i].module_str, str);
            break;
        }
    }
    if(i == 10) return FAIL;
    LOG_I("register %s", sys_module_table[i].module_str);
    return OK;
}


int week_time(char *module_str, int time)
{
    uint8_t i;
    for(i = 0; i < 10; i++) {
        if(strcmp(sys_module_table[i].module_str, module_str) == 0) {
            sys_module_table[i].week_time = time;
            break;
        }
    }
    if(i == 10) return FAIL;
    if(low_power_flag == 1){  //进入了休眠
        def_rtos_smaphore_release(exit_power_sem);  //释放退出休眠信号量
    }
    if(def_rtos_timer_is_running(low_power_timer) != 1) {
        def_rtos_timer_start(low_power_timer, 1000, 1);
        LOG_I("low_power_timer is start");
    }      
    LOG_I("%s:%d", sys_module_table[i].module_str, sys_module_table[i].week_time);
    return OK;
}


static void enter_low_power()
{
    LOG_I("IS ENTER LOW POWER");
    if(ql_autosleep_enable(QL_ALLOW_SLEEP) != 0)
    {
        LOG_E("failed to set auto sleep");
    }
    sys_info.iot_mode = IOT_LOW_POWER_MODE;
    LOG_I("IOT_LOW_POWER_MODE");
    net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
    ble_cmd_mark(BLE_ENTER_SLEEP_INDEX);
    hal_drv_write_gpio_value(O_BLE_WEEK_SIG, LOW_L);
    system_timer_stop();
    app_set_led_ind(LED_ALL_OFF);
    voice_play_off();
    imu_algo_timer_stop();
}

static void exit_lower_power()
{
    LOG_I("IS EXIT LOWER POWER");
    if(ql_autosleep_enable(QL_NOT_ALLOW_SLEEP) != 0 )
    {
        LOG_E("failed to set auto sleep");
    }
    if(sys_param_set.alive_flag == 1) {
        sys_info.iot_mode = IOT_ACTIVE_MODE;
    } else {
        sys_info.iot_mode = IOT_WAIT_ACTIVE_MODE;
    }
    hal_drv_write_gpio_value(O_BLE_WEEK_SIG, HIGH_L);
    system_timer_start();
    
}

void low_power_thread(void *param)
{
    while(1) {
//        LOG_I("IS RUN");
        def_rtos_semaphore_wait(enter_power_sem, RTOS_WAIT_FOREVER);
        enter_low_power();
        low_power_flag = 1;
        def_rtos_semaphore_wait(exit_power_sem, RTOS_WAIT_FOREVER);
        exit_lower_power();
        low_power_flag = 0;
    }
    def_rtos_task_delete(NULL);
}


static void low_power_timer_fun()
{
    uint8_t i;
    uint8_t temp = 0;
    for(i = 0; i < 10; i++) {
        if(sys_module_table[i].week_time == -1) {
            def_rtos_timer_stop(low_power_timer);  //定时器停止
            return;
        } else if(sys_module_table[i].week_time > 0) {
            sys_module_table[i].week_time--;
            temp = 1;
        } 
    }
    if(temp == 0) {
        def_rtos_timer_stop(low_power_timer);
        if(low_power_flag == 0) {
            def_rtos_smaphore_release(enter_power_sem);
        }
    }
}

void low_power_init()
{
    memset(sys_module_table, 0, sizeof(sys_module_table));
    low_power_flag = 0;
    def_rtos_timer_create(&low_power_timer, low_power_task, low_power_timer_fun, NULL);
    if(def_rtos_semaphore_create(&enter_power_sem, 0) != RTOS_SUCEESS){
        LOG_E("enter_power_sem is create fail");
        return;
    }
    if(def_rtos_semaphore_create(&exit_power_sem, 0) != RTOS_SUCEESS){
        LOG_E("exit_power_sem is create fail");
        return;
    }
}


