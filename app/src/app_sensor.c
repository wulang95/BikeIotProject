#include   "app_system.h"
#include    "qmi8658.h"
#include    "imualgo_axis9.h"
#include  "math.h"
#define DBG_TAG         "sensor"

#ifdef APP_SENSOR_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"
QST_Filter gyro_filter;
QST_Filter accel_filter;
QST_Filter_Buffer gyro_buf[3];
QST_Filter_Buffer accel_buf[3];

static float accl[3],gyro[3];
static float accel_correct[3] = {0, 0, 0};
static float gyro_correct[3] = {0, 0, 0};
static float dt = 0.01f;
float euler_angle[3] = {0, 0, 0};
static float quater[4] = {0, 0, 0, 0};
static float line_acc[3] = {0, 0, 0};

def_rtos_timer_t  algo_timer;
def_rtos_task_t imu_algo_task = NULL;
def_rtos_sem_t imu_algo_sem;

void qst_algo_inti(void)
{
	set_cutoff_frequency(100, 1, &gyro_filter);      //第一个参数为频率100hz，与算法库调用周期有关，如例程中算法库调用周期为10ms 
	set_cutoff_frequency(100, 2, &accel_filter);
}

void imu_algo_thread(void *param)
{
	def_rtosStaus res;
	def_rtos_timer_start(algo_timer, 10, 1);
	while(1)
	{
		res = def_rtos_semaphore_wait(imu_algo_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
		qmi8658_read_xyz(accl, gyro);		
	//	LOG_I("accl[0]:%0.2f, accl[1]:%0.2f, accl[2]:%0.2f", accl[0], accl[1], accl[2]);
	//	LOG_I("gyro[0]:%0.2f, gyro[1]:%0.2f, gyro[2]:%0.2f", gyro[0], gyro[1], gyro[2]);	
		accel_correct[0] = Filter_Apply(accl[0],&accel_buf[0],&accel_filter);
		accel_correct[1] = Filter_Apply(accl[1],&accel_buf[1],&accel_filter);
		accel_correct[2] = Filter_Apply(accl[2],&accel_buf[2],&accel_filter);
		gyro_correct[0] = Filter_Apply(gyro[0],&gyro_buf[0],&gyro_filter);
		gyro_correct[1] = Filter_Apply(gyro[1],&gyro_buf[1],&gyro_filter);
		gyro_correct[2] = Filter_Apply(gyro[2],&gyro_buf[2],&gyro_filter);
		qst_fusion_update(accel_correct, gyro_correct, &dt, euler_angle, quater, line_acc);
	}
	def_rtos_task_delete(NULL);
}


void imu_algo_timer()     // 10ms调用1次
{
    def_rtos_smaphore_release(imu_algo_sem);
}

void imu_algo_timer_stop()
{
	def_rtos_timer_stop(algo_timer);
}


void qmi8658_sensor_init()
{
	def_rtosStaus res;
	
    qst_algo_inti();
    def_rtos_task_sleep_ms(50);
    qmi8658_init();
    init_state_recognition(&qmi8658_read_reg);
	res = def_rtos_semaphore_create(&imu_algo_sem, 0);
	if(res != RTOS_SUCEESS) {
		LOG_E("imu_algo_sem create is fail");
	}
    res = def_rtos_timer_create(&algo_timer, imu_algo_task, imu_algo_timer, NULL); 
	if(res != RTOS_SUCEESS) {
		LOG_E("algo_timer create is fail!");
	}
	LOG_I("qmi8658_sensor_init");
}


