#include "app_system.h"

#define DBG_TAG         "lock"

#ifdef LOCK_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"