#include "app_system.h"

#define DBG_TAG         "http_upgrade"

#ifdef HTTP_UPGRADE_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"



struct http_upgrade_info_stu http_upgrade_info;