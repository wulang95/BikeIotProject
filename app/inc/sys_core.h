#ifndef     __SYS_CORE_H
#define     __SYS_CORE_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include    "common.h"
#include   "rtos_port_def.h"
#include    "ble_control.h"
#include    "ble_protocol.h"
#include    "car_control.h"
#include    "log_port.h"
#include    "bike_app_config.h"


void assert_handler(const char *ex_string, const char *func, size_t line);

#define SYS_ALIGN_DOWN(v, n) ((unsigned long)(v) & ~((n)-1))

#define ASSERT(EX) if(!(EX)) {\
    assert_handler(#EX, __func__, __LINE__);\
}   

void sys_init();





#endif