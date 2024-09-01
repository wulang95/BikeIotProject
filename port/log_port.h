#ifndef     __LOG_PORT_H
#define     __LOG_PORT_H

#ifdef __cplusplus
extern "C" {
#endif

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
#ifdef  DBG_ENABLE
#ifdef  DBG_LVL
#ifndef DBG_LEVEL
#define DBG_LEVEL   DBG_LVL
#endif
#define dbg_log_line(LOG_LEVEL, fmt, ...)   QL_LOG(LOG_LEVEL, DBG_SECTION_NAME, fmt, ##__VA_ARGS__)

#else
#ifndef DBG_LEVEL
#define DBG_LEVEL   DBG_WARNING
#endif
#endif
#else
#define dbg_log_line(LOG_LEVEL, fmt, ...) 
#endif

#if (DBG_LEVEL >= DBG_LOG)
#define LOG_D(fmt, ...)      dbg_log_line(QL_LOG_LEVEL_DEBUG, fmt, ...)
#else
#define LOG_D(...)
#endif

#if (DBG_LEVEL >= DBG_INFO)
#define LOG_I(fmt, ...)      dbg_log_line(QL_LOG_LEVEL_INFO,  fmt, ...)
#else
#define LOG_I(...)
#endif

#if (DBG_LEVEL >= DBG_WARNING)
#define LOG_W(fmt, ...)      dbg_log_line(QL_LOG_LEVEL_WARN, fmt, ...)
#else
#define LOG_W(...)
#endif


#if (DBG_LEVEL >= DBG_ERROR)
#define LOG_E(fmt, ...)       dbg_log_line(QL_LOG_LEVEL_ERROR, fmt, ...)
#else
#define LOG_E(...)
#endif


#ifdef __cplusplus
}
#endif

#endif