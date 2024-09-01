#ifndef __RTOS_DEF_H
#define __RTOS_DEF_H


enum {
    TASK_PRIORITY_IDLE =1,
    TASK_PRIORITY_LOW = 4,
    TASK_PRIORITY_BELOW_NORMAL = 8,
    TASK_PRIORITY_NORMAL = 12,
    TASK_PRIORITY_ABOVE_NORMAL = 16,
    TASK_PRIORITY_HIGH = 25,
    TASK_PRIORITY_REALTIME = 30  
};

/*对RTOS的API进行封装*/
enum{
    RTOS_WAIT_FOREVER = 0XFFFFFFFFUL,
    RTOS_NO_WAIT = 0
};

#define RTOS_SUCEESS        0

typedef int def_rtosStaus;

#define def_rtos_task_t     ql_task_t
#define def_rtos_sem_t      ql_sem_t
#define def_rtos_queue_t    ql_queue_t
#define def_rtos_timer_t    ql_timer_t
#define def_rtos_mutex_t    ql_mutex_t


#define def_rtos_task_create(taskRef, tacksize, priority, taskStart)  ql_rtos_task_create((taskRef), (tacksize), (priority), #taskStart, (taskStart), NULL, 0);
#define def_rtos_task_delete(taskRef)   ql_rtos_task_delete(taskRef)

#define def_rtos_task_sleep_ms(ms)  ql_rtos_task_sleep_ms(ms)

#define def_rtos_enter_critical   ql_rtos_exit_critical
#define def_rtos_exit_critical(critical)  ql_rtos_exit_critical(critical)

#define def_rtos_semaphore_create(semaRef, initcalCount) ql_rtos_semaphore_create((semaRef), (initcalCount))  
#define def_rtos_semaphore_wait(semaRef, timeout)   ql_rtos_semaphore_wait((semaRef), (timeout))
#define def_rtos_smaphore_release(semaRef)  ql_rtos_semaphore_release(semaRef)
#define def_rtos_semaphore_delete(semaRef)   ql_rtos_semaphore_delete(semaRef)
#define def_rtos_semaphore_get_cnt(semaref, cnt_ptr)   ql_rtos_semaphore_get_cnt((semaref), (cnt_ptr)) 

#define def_rtos_mutex_create(mutexRef)     ql_rtos_mutex_create(mutexRef)
#define def_rtos_mutex_lock(mutexRef, timeout)  ql_rtos_mutex_lock((mutexRef),(timeout))
#define def_rtos_mutex_unlock(mutexRef)     ql_rtos_mutex_unlock(mutexRef)
#define def_rtos_mutex_delete(mutexRef)     ql_rtos_mutex_delete(mutexRef)

#define def_rtos_queue_create(msgQRef, maxSize, maxNumber)  ql_rtos_queue_create((msgQRef), (maxSize), (maxNumber))
#define def_rtos_queue_wait(msgQRef, recvMsg, size, timeout)  ql_rtos_queue_wait((msgQRef), (recvMsg), (size), (timeout))
#define def_rtos_queue_release(msgQRef, size, recvMsg, timeout) ql_rtos_queue_release((msgQRef), (size), (recvMsg), (timeout))
#define def_rtos_queue_get_cnt(msgQRef, cnt_ptr)    ql_rtos_queue_get_cnt((msgQRef), (cnt_ptr))
#define def_rtos_queue_delete(msgQRef)  ql_rtos_queue_delete(msgQRef)



#define def_rtos_timer_create(timerRef, taskRef, callBackoutine, timerArgc)     ql_rtos_timer_create((timerRef), (taskRef), (callBackoutine), (timerArgc))
#define def_rtos_timer_start(timerRef, set_Time, cycicalEn)     ql_rtos_timer_start(timerRef, set_Time, cycicalEn)
#define def_rtos_timer_stop(timerRef)       ql_rtos_timer_stop(timerRef)
#define def_rtos_timer_delete(timerRef)     ql_rtos_timer_delete(timerRef)

#define def_rtos_get_system_tick   ql_rtos_get_system_tick() 
















#endif