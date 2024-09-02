#include "sys_core.h"

#define DBG_TAG         "sys_core"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif





void assert_handler(const char *ex_string, const char *func, size_t line)
{
    LOG_E("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    while(1);
}



void sys_init()
{
    
}