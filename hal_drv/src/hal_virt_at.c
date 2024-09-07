#include "hal_virt_at.h"
#include "ql_api_virt_at.h"
#include "rtos_port_def.h"
#include "ringbuffer.h"
#include "app_common.h"

#define DBG_TAG         "hal_virt_at"

#ifdef HAL_VIRT_AT_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"

struct rt_ringbuffer *virtAt_Ringbuf;
def_rtos_sem_t virtAt_sem;
#define VIRT_RINGBUG_LEN      256

uint16_t hal_virt_at_read(char *buf, uint16_t len, uint32_t timeout)
{
    uint16_t buf_len, read_len;
    if(RTOS_SUCEESS == def_rtos_semaphore_wait(virtAt_sem, timeout)) {
        buf_len = rt_ringbuffer_data_len(virtAt_Ringbuf);
        read_len = MIN(buf_len, len);
        rt_ringbuffer_get(virtAt_Ringbuf, (uint8_t *)buf, read_len);
        return read_len;
    }
    return 0;
}

static void hal_virt_at_noticy_cb(unsigned int ind_type, unsigned int size)
{
    unsigned int real_size = 0;
    uint8_t flag = 0;
    uint8_t buf[64];
    if(QUEC_VIRT_AT_RX_RECV_DATA_IND == ind_type) {
        while(size) {
            real_size= MIN(size, 64);
            ql_virt_at_read(QL_VIRT_AT_PORT_0, buf, real_size);
            rt_ringbuffer_put(virtAt_Ringbuf, buf, real_size);
            LOG_D("%s]", (char *)buf);
            flag = 1;
            size -= real_size;
        }  
    }
    if(flag) def_rtos_smaphore_release(virtAt_sem);
}


void hal_virt_at_init()
{
    def_rtosStaus res = RTOS_SUCEESS;
    def_rtos_semaphore_create(&virtAt_sem, 0);
    virtAt_Ringbuf = rt_ringbuffer_create(VIRT_RINGBUG_LEN);
    res = ql_virt_at_open(QL_VIRT_AT_PORT_0,  hal_virt_at_noticy_cb);
    if(res != RTOS_SUCEESS) {
        LOG_E("ql_virt_at_open is fail, res :%d", res);
        return;
    }
    LOG_I("ql_virt_at_open is success");
}

void hal_virt_at_write(char *buf)
{
    def_rtosStaus res = RTOS_SUCEESS;
    LOG_I("[%d]:%s", strlen(buf), buf);
    res = ql_virt_at_write(QL_VIRT_AT_PORT_0, (unsigned char*)buf, strlen(buf));    
    if(res != RTOS_SUCEESS) {
        LOG_E("ql_virt_at_write is fail, res:%d", res);
        return;
    }
    LOG_D("ql_virt_at_write is success");
}


