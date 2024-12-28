#ifndef __LOW_POWER_H
#define __LOW_POWER_H

#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "rtos_port_def.h"


extern def_rtos_task_t low_power_task;


int register_module(char *str);
int week_time(char *module_str, int time);
void low_power_thread(void *param);
void low_power_init();























#ifdef __cplusplus
}
#endif

#endif
