#ifndef    __APP_SENSOR_H
#define    __APP_SENSOR_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

extern def_rtos_task_t imu_algo_task;
extern float euler_angle[3];
void qmi8658_sensor_init();
void imu_algo_thread(void *param);
void imu_algo_timer_stop();
void imu_algo_timer_start();











#ifdef __cplusplus
}
#endif


#endif