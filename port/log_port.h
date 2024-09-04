#ifndef     __LOG_PORT_H
#define     __LOG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif
#include "ql_log.h"
#include "bike_app_config.h"
#define DBG_ERROR       0
#define DBG_INFO        1
#define DBG_WARNING     2
#define DBG_LOG         3


#ifdef DBG_TAG
#ifndef DBG_SECTION_NAME
#define DBG_SECTION_NAME    DBG_TAG
#endif
#else
/* compatible with old version */
#ifndef DBG_SECTION_NAME
#define DBG_SECTION_NAME    "DBG"
#endif
#endif /* DBG_TAG */


#ifdef  DBG_LVL
#ifndef DBG_LEVEL
#define DBG_LEVEL   DBG_LVL
#endif

#else
#ifndef DBG_LEVEL
#define DBG_LEVEL   DBG_WARNING
#endif
#endif

#if (DBG_LEVEL >= DBG_LOG && DBG_ENABLE)
#define LOG_D(fmt, ...)      QL_LOG(QL_LOG_LEVEL_DEBUG, DBG_SECTION_NAME, fmt, ##__VA_ARGS__)
#else
#define LOG_D(...)
#endif

#if (DBG_LEVEL >= DBG_INFO && DBG_ENABLE)
#define LOG_I(fmt, ...)      QL_LOG(QL_LOG_LEVEL_INFO, DBG_SECTION_NAME, fmt, ##__VA_ARGS__)
#else
#define LOG_I(...)
#endif

#if (DBG_LEVEL >= DBG_WARNING && DBG_ENABLE)
#define LOG_W(fmt, ...)      QL_LOG(QL_LOG_LEVEL_WARN, DBG_SECTION_NAME, fmt, ##__VA_ARGS__)
#else
#define LOG_W(...)
#endif


#if (DBG_LEVEL >= DBG_ERROR && DBG_ENABLE)
#define LOG_E(fmt, ...)       QL_LOG(QL_LOG_LEVEL_ERROR, DBG_SECTION_NAME, fmt, ##__VA_ARGS__)
#else
#define LOG_E(...)
#endif


#ifdef __cplusplus
}
#endif

#endif