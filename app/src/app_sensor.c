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
static float dt = 0.05f;
float euler_angle[3] = {0, 0, 0};
static float quater[4] = {0, 0, 0, 0};
static float line_acc[3] = {0, 0, 0};

def_rtos_timer_t  algo_timer;
def_rtos_task_t imu_algo_task = NULL;
def_rtos_sem_t imu_algo_sem;

void qst_algo_inti(void)
{
	set_cutoff_frequency(20, 1, &gyro_filter);      //第一个参数为频率100hz，与算法库调用周期有关，如例程中算法库调用周期为10ms 
	set_cutoff_frequency(20, 2, &accel_filter);
}

void imu_algo_timer()     // 10ms调用1次
{
    def_rtos_smaphore_release(imu_algo_sem);
}

void qmi8658_sensor_init()
{
    int64_t sensor_cali_time_t;
    qst_algo_inti();
    def_rtos_task_sleep_ms(50);
    if(qmi8658_init() != 1) {
		iot_error_set(IOT_ERROR_TYPE, SENSOR_ERROR);
		LOG_I("qmi8658_init is fail");
		return; 
	}
	iot_error_clean(IOT_ERROR_TYPE, SENSOR_ERROR);
    init_state_recognition(&qmi8658_read_reg);
    sensor_cali_time_t = def_rtos_get_system_tick();
    while(1){
        qmi8658_read_xyz(accl, gyro);
        def_rtos_task_sleep_ms(50);
        if(sys_info.static_cali_flag == 1){
            LOG_I("calic is succeeful");
            break;
        }
        if(def_rtos_get_system_tick() - sensor_cali_time_t > 10000){
            LOG_E("calic is fail");
            break;
        }
    }
	LOG_I("qmi8658_sensor_init");
}

/*   摔倒检测    */
#define FILTER_ALPHA        0.2f   // 低通滤波系数
#define FALL_ACC_THRESH     3.0f   // 跌倒加速度阈值(3g)
#define FALL_ANGLE_THRESH   60.0f  // 跌倒角度阈值(60度)
#define RECOVERY_ANGLE      30.0f  // 扶起角度阈值(30度)
#define POST_FALL_TIME      3000   // 跌倒后检测扶起时间窗(3秒)
#define STABLE_DURATION     2000   // 稳定状态维持时间(2秒)

typedef enum {
    STATE_NORMAL,        // 正常状态
    STATE_FALL_DETECTED, // 跌倒检测
    STATE_POST_FALL,     // 跌倒后状态
    STATE_RECOVERY       // 扶起检测
} FallRecoveryState;

typedef struct {
    FallRecoveryState state;
    uint32_t timestamp;
    float filtered_acc[3];
    float filtered_gyro[3];
    float last_angle;
    bool alert_active;
} DetectionContext;



// 低通滤波器
void low_pass_filter(float *new_data, float *filtered_data) {
    for(int i=0; i<3; i++) {
        filtered_data[i] = FILTER_ALPHA*new_data[i] + (1-FILTER_ALPHA)*filtered_data[i];
    }
}

float calculate_tilt_angle(float accel_x, float accel_y, float accel_z) {
    float vertical_accel = accel_z; // 假设Z轴垂直地面
    return acosf(fabsf(vertical_accel) / (sqrtf(accel_x*accel_x + accel_y*accel_y + accel_z*accel_z))) * (180.0f / M_PI);
}

float calculate_body_angle(float acc_x, float acc_y, float acc_z) {
    // 使用反正切计算相对于垂直轴的角度
    float vertical = acc_z;
    float horizontal = sqrtf(acc_x*acc_x + acc_y*acc_y);
    return atan2f(horizontal, fabsf(vertical)) * (180.0f / M_PI);
}

float dynamic_threshold_adjust(float baseline, float current) {
    // 简单的动态阈值调整示例
    return baseline * (1.0f + 0.2f * fabsf(current - baseline)/baseline);
}

// 报警控制函数
void trigger_alert() {
    car_info.filp_state = CAR_RECOVERY_TO_FALL;
    LOG_I("CAR_RECOVERY_TO_FALL");
    net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
}

void cancel_alert() {
    car_info.filp_state = CAR_FALL_TO_RECOVERY;
    LOG_I("CAR_FALL_TO_RECOVERY");
    net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
    LOG_I("CAR IS REV");
}



// 状态处理函数
void process_state(DetectionContext *ctx) {
   /* float current_angle = calculate_body_angle(
        ctx->filtered_acc[0], 
        ctx->filtered_acc[1], 
        ctx->filtered_acc[2]
    );*/
    float current_angle = calculate_tilt_angle(ctx->filtered_acc[0], 
        ctx->filtered_acc[1], 
        ctx->filtered_acc[2]);
    float svm = sqrtf(
        ctx->filtered_acc[0]*ctx->filtered_acc[0] +
        ctx->filtered_acc[1]*ctx->filtered_acc[1] +
        ctx->filtered_acc[2]*ctx->filtered_acc[2]
    );

    uint32_t current_time = def_rtos_get_system_tick();
    
    switch(ctx->state) {
        case STATE_NORMAL: {
            // 动态调整阈值
            float dyn_fall_thresh = dynamic_threshold_adjust(FALL_ACC_THRESH, svm);
            LOG_I("dyn_fall_thresh:%f, svm:%f, current_angle:%f", dyn_fall_thresh, svm, current_angle);
            if(svm > dyn_fall_thresh && current_angle > FALL_ANGLE_THRESH) {
                ctx->state = STATE_FALL_DETECTED;
                ctx->timestamp = current_time;
            } else {
                car_info.filp_state = CAR_NORMAL_STATE;
            }
            break;
        }
        
        case STATE_FALL_DETECTED: {
            if((current_time - ctx->timestamp) > 500) { // 持续500ms确认
                if(ctx->alert_active == false) {
                    trigger_alert();
                }
                car_info.filp_state = CAR_FALL_STATE;
				LOG_I("CAR IS FALL");
                ctx->alert_active = true;
                ctx->state = STATE_POST_FALL;
                ctx->timestamp = current_time;
            }
            break;
        }
        
        case STATE_POST_FALL: {
            // 在3秒时间窗口内检测扶起
            if(current_angle < RECOVERY_ANGLE) {
                ctx->state = STATE_RECOVERY;
                ctx->timestamp = current_time;
            }
            
            if((current_time - ctx->timestamp) > POST_FALL_TIME) {
                ctx->state = STATE_NORMAL;
            }
            break;
        }
        
        case STATE_RECOVERY: {
            // 需要持续稳定状态2秒
            if(fabsf(current_angle - ctx->last_angle) < 5.0f) { // 角度变化小于5度
                if((current_time - ctx->timestamp) > STABLE_DURATION) {
                    cancel_alert();
                    ctx->alert_active = false;
                    ctx->state = STATE_NORMAL;
                    car_info.filp_state = CAR_NORMAL_STATE;
                }
            } else {
                ctx->timestamp = current_time; // 重置计时器
            }
            ctx->last_angle = current_angle;
            break;
        }
    }
}


void imu_algo_thread(void *param)
{
	float last_accl[3];
	DetectionContext ctx = {
		.state = STATE_NORMAL,
		.alert_active = false,
		.filtered_acc = {0},
		.filtered_gyro = {0},
	};
	uint16_t cent = 0;
	def_rtosStaus res;
	qmi8658_sensor_init();
    def_rtos_timer_start(algo_timer, 50, 1);
    sys_info.algo_timer_run = 1;
    imu_algo_timer_stop();
	while(1)
	{
		res = def_rtos_semaphore_wait(imu_algo_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
		qmi8658_read_xyz(accl, gyro);	
		if(memcmp(accl, last_accl, 3) == 0) {
			if(++cent == 10) {
				iot_error_set(IOT_ERROR_TYPE, SENSOR_ERROR);
				cent = 0;
			}
		} else {
			cent = 0;
		}
		if(car_info.lock_sta == CAR_LOCK_STA) {
			low_pass_filter(accl, ctx.filtered_acc);
			low_pass_filter(gyro, ctx.filtered_gyro);
			process_state(&ctx);
		} else {
            car_info.filp_state = CAR_NORMAL_STATE;
        }
		LOG_I("accl[0]:%0.2f, accl[1]:%0.2f, accl[2]:%0.2f", accl[0], accl[1], accl[2]);
		LOG_I("gyro[0]:%0.2f, gyro[1]:%0.2f, gyro[2]:%0.2f", gyro[0], gyro[1], gyro[2]);	
		accel_correct[0] = Filter_Apply(accl[0],&accel_buf[0],&accel_filter);
		accel_correct[1] = Filter_Apply(accl[1],&accel_buf[1],&accel_filter);
		accel_correct[2] = Filter_Apply(accl[2],&accel_buf[2],&accel_filter);
		gyro_correct[0] = Filter_Apply(gyro[0],&gyro_buf[0],&gyro_filter);
		gyro_correct[1] = Filter_Apply(gyro[1],&gyro_buf[1],&gyro_filter);
		gyro_correct[2] = Filter_Apply(gyro[2],&gyro_buf[2],&gyro_filter);
		qst_fusion_update(accel_correct, gyro_correct, &dt, euler_angle, quater, line_acc);
		LOG_I("ptich:%.2f,roll:%.2f,yaw:%.2f",euler_angle[0],euler_angle[1],euler_angle[2]); 
		memcpy(last_accl, accl, 3);
	}
	def_rtos_task_delete(NULL);
}




void imu_algo_timer_start()
{
    if(sys_info.algo_timer_run == 1) return;
	sys_info.algo_timer_run = 1;
    LOG_I("imu_algo_timer_start");
	if(RTOS_SUCEESS != def_rtos_timer_start(algo_timer, 50, 1)){
        LOG_E("algo_timer is start fail");
    }
	QMI8658_Wakeup_Process();
//	qmi8658_enable_amd(0, 	qmi8658_Int1, 0); 
//	qmi8658_restart();
//	qmi8658_enableSensors(QMI8658_ACCGYR_ENABLE);
}

void imu_algo_timer_stop()
{
    if(sys_info.algo_timer_run == 0) return;
	sys_info.algo_timer_run = 0;
	if(RTOS_SUCEESS != def_rtos_timer_stop(algo_timer)){
        LOG_E("algo_timer is stop fail");
    }
	qmi8658_enable_amd(1, 	qmi8658_Int1, 1);  //关闭同步
	LOG_I("imu_algo_timer_stop");
}

void app_sensor_init()
{
	def_rtosStaus res;
	res = def_rtos_semaphore_create(&imu_algo_sem, 0);
	if(res != RTOS_SUCEESS) {
		LOG_E("imu_algo_sem create is fail");
	}
    res = def_rtos_timer_create(&algo_timer, imu_algo_task, imu_algo_timer, NULL); 
	if(res != RTOS_SUCEESS) {
		LOG_E("algo_timer create is fail!");
	}
}


