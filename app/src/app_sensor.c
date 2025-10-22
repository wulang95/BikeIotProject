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

int qmi8658_sensor_init()
{
 //   int64_t sensor_cali_time_t;
    qst_algo_inti();
    def_rtos_task_sleep_ms(50);
    if(qmi8658_init() != 1) {
		iot_error_set(IOT_ERROR_TYPE, SENSOR_ERROR);
		LOG_I("qmi8658_init is fail");
		return -1; 
	}
    LOG_I("qmi8658_init is succeeful");
	iot_error_clean(IOT_ERROR_TYPE, SENSOR_ERROR);
    init_state_recognition(&qmi8658_read_reg);
    // sensor_cali_time_t = def_rtos_get_system_tick();
    // while(1){
    //     qmi8658_read_xyz(accl, gyro);
    //     def_rtos_task_sleep_ms(50);
    //     if(sys_info.static_cali_flag == 1){
    //         LOG_I("calic is succeeful");
    //         break;
    //     }
    //     if(def_rtos_get_system_tick() - sensor_cali_time_t > 10000){
    //         LOG_E("calic is fail");
    //         break;
    //     }
    // }
	LOG_I("qmi8658_sensor_init");
    return 0;
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



// 震动检测状态结构体
typedef struct {
    // 配置参数
    float acc_threshold;   // 加速度阈值 (m/s²)
    float gyro_threshold;  // 陀螺仪阈值 (rad/s)
    float mag_threshold;   // 合加速度阈值 (m/s²)
    int window_size;       // 滑动窗口大小
    
    // 内部状态
    float *acc_buffer;     // 加速度历史缓冲区
    float *gyro_buffer;    // 陀螺仪历史缓冲区
    int buffer_index;      // 缓冲区索引
    float prev_magnitude;  // 前一个合加速度值
} VibrationDetector;

// 初始化震动检测器
VibrationDetector* init_vibration_detector(float acc_thresh, float gyro_thresh, 
                                          float mag_thresh, int window_size) {
    VibrationDetector* detector = malloc(sizeof(VibrationDetector));
    detector->acc_threshold = acc_thresh;
    detector->gyro_threshold = gyro_thresh;
    detector->mag_threshold = mag_thresh;
    detector->window_size = window_size;
    
    // 分配缓冲区
    detector->acc_buffer = malloc(window_size * sizeof(float));
    detector->gyro_buffer = malloc(window_size * sizeof(float));
    detector->buffer_index = 0;
    detector->prev_magnitude = 0.0f;
    
    // 初始化缓冲区
    for (int i = 0; i < window_size; i++) {
        detector->acc_buffer[i] = 0.0f;
        detector->gyro_buffer[i] = 0.0f;
    }
    
    return detector;
}

// 释放震动检测器
void free_vibration_detector(VibrationDetector* detector) {
    free(detector->acc_buffer);
    free(detector->gyro_buffer);
    free(detector);
}


// 计算加速度计合量
float acceleration_magnitude(float ax, float ay, float az) {
    return sqrtf(ax*ax + ay*ay + az*az);
}

// 计算陀螺仪合量
float gyro_magnitude(float gx, float gy, float gz) {
    return sqrtf(gx*gx + gy*gy + gz*gz);
}

// 高通滤波器 (去除重力影响)
void high_pass_filter(float *ax, float *ay, float *az, float alpha) {
    static float prev_ax = 0, prev_ay = 0, prev_az = 0;
    
    *ax = alpha * (*ax + prev_ax);
    *ay = alpha * (*ay + prev_ay);
    *az = alpha * (*az + prev_az);
    
    prev_ax = *ax;
    prev_ay = *ay;
    prev_az = *az;
}

// 检测震动事件
bool detect_vibration(VibrationDetector* detector, float *acc, float *gyro) {
    // 1. 应用高通滤波器去除重力影响
    high_pass_filter(&acc[0], &acc[1], &acc[2], 0.8f);
    
    // 2. 计算当前合加速度
    float current_magnitude = acceleration_magnitude(acc[0], acc[1], acc[2]);
    
    // 3. 计算加速度变化率
    float delta_acc = fabsf(current_magnitude - detector->prev_magnitude);
    detector->prev_magnitude = current_magnitude;
    
    // 4. 更新缓冲区
    detector->acc_buffer[detector->buffer_index] = current_magnitude;
    detector->gyro_buffer[detector->buffer_index] = gyro_magnitude(gyro[0], gyro[1], gyro[2]);
    detector->buffer_index = (detector->buffer_index + 1) % detector->window_size;
    
    // 5. 计算滑动窗口平均值
    float avg_acc = 0.0f;
    float avg_gyro = 0.0f;
    for (int i = 0; i < detector->window_size; i++) {
        avg_acc += detector->acc_buffer[i];
        avg_gyro += detector->gyro_buffer[i];
    }
    avg_acc /= detector->window_size;
    avg_gyro /= detector->window_size;
    
    // 6. 检测震动条件
    bool condition1 = (current_magnitude > detector->mag_threshold);        // 合加速度超过阈值
    bool condition2 = (delta_acc > detector->acc_threshold);                // 加速度变化率超过阈值
    bool condition3 = (avg_gyro > detector->gyro_threshold);               // 平均角速度超过阈值
    bool condition4 = (fabsf(avg_acc - current_magnitude) > 0.5f);         // 当前值与平均值差异大
    
    // 震动事件需要满足多个条件
    return (condition1 && condition2) || (condition1 && condition3) || (condition2 && condition4);
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
	res = qmi8658_sensor_init();
    if(res != 0) {
        iot_error_set(IOT_ERROR_TYPE, SENSOR_ERROR);
        sys_info.sensor_init = 0;
    } else {
        def_rtos_timer_start(algo_timer, 50, 1);
        sys_info.algo_timer_run = 1;
        sys_info.sensor_init = 1;
        sys_param_set.sensor_con_err_cnt = 0;
        SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
    }
    VibrationDetector* detector = init_vibration_detector(
        0.8f,   // 加速度阈值 (m/s²)
        0.2f,   // 陀螺仪阈值 (rad/s)
        0.8f,   // 合加速度阈值 (m/s²)
        5       // 滑动窗口大小
    );
    imu_algo_timer_stop();
	while(1)
	{
		res = def_rtos_semaphore_wait(imu_algo_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
		qmi8658_read_xyz(accl, gyro);	
		if(memcmp(accl, last_accl, 3) == 0) {
			if(++cent == 15 && (iot_error_check(IOT_ERROR_TYPE, SENSOR_ERROR) == 0)) {
				iot_error_set(IOT_ERROR_TYPE, SENSOR_ERROR);
				cent = 0;
			}
            LOG_I("cent:%d", cent);
		} else {
            if(iot_error_check(IOT_ERROR_TYPE, SENSOR_ERROR) == 1) {
                iot_error_clean(IOT_ERROR_TYPE, SENSOR_ERROR);
            }
			cent = 0;
		}
		if(car_info.lock_sta == CAR_LOCK_STA) {
			low_pass_filter(accl, ctx.filtered_acc);
			low_pass_filter(gyro, ctx.filtered_gyro);
			process_state(&ctx);
		} else {
            car_info.filp_state = CAR_NORMAL_STATE;
        }
        car_info.move_alarm = detect_vibration(detector, accl, gyro);
        LOG_I("car_info.move_alarm:%d", car_info.move_alarm);
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
        car_state_data.slope_data = (int8_t)euler_angle[0];
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
    if(sys_info.algo_timer_run == 0 || sys_info.static_cali_flag == 0) return;   //静态校准未完成
	if(RTOS_SUCEESS != def_rtos_timer_stop(algo_timer)){
        LOG_E("algo_timer is stop fail");
    }
    sys_info.algo_timer_run = 0;
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


int app_sensor_reinit()
{
    if(sys_info.sensor_error_flag == 1) return 1;
    sys_param_set.sensor_con_err_cnt++;
    sys_param_save(SYS_SET_ADR);
    cat1_reset_reson_save(NET_RESET_SENSOR_ABNORMAL);
    MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);  //cat1断电重启
    return 0;
}