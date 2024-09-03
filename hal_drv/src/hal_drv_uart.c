#include "hal_drv_uart.h"
#include "rtos_port_def.h"
#include "log_port.h"
#include "ql_uart.h"
#include "ql_gpio.h"
#include "hal_resources_config.h"
#include "ringbuffer.h"
#define DBG_TAG         "hal_drv_uart"

#ifdef HAL_UART_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#define MIN(a,b) ((a) < (b) ? (a) : (b))

struct rt_ringbuffer *UART2_Ringbuf;
def_rtos_sem_t uart2_rec_sem;
#define UART2_BUF_MAXLEN    1024

struct rt_ringbuffer *UART1_Ringbuf;
def_rtos_sem_t uart1_rec_sem;
#define UART1_BUF_MAXLEN    1024

uint16_t hal_drv_uart_read(uint8_t uart_num,uint8_t *data, uint16_t len, uint32_t time)
{
    uint16_t buf_len, read_len;
    if(uart_num == UART2){
        if(RTOS_SUCEESS == def_rtos_semaphore_wait(uart2_rec_sem, time)){
            buf_len = rt_ringbuffer_data_len(UART2_Ringbuf);
            read_len = MIN(buf_len, len);
            rt_ringbuffer_get(UART2_Ringbuf, data, read_len);
            return read_len;
        } else {
            return -1;
        }
    } else if(uart_num == UART1){
        if(RTOS_SUCEESS == def_rtos_semaphore_wait(uart1_rec_sem, time)){
            buf_len = rt_ringbuffer_data_len(UART1_Ringbuf);
            read_len = MIN(buf_len, len);
            rt_ringbuffer_get(UART1_Ringbuf, data, read_len);
            return read_len;
        } else {
            return -1;
        }
    } 
    return -1;
}

static void ql_uart2_notify_cb(uint32 ind_type, ql_uart_port_number_e port, uint32 size)
{
    uint8_t buf[64];
    unsigned int real_size = 0;
    int read_len = 0;
    switch(ind_type)
    {
        case QUEC_UART_RX_OVERFLOW_IND:  //rx buffer overflow
        case QUEC_UART_RX_RECV_DATA_IND:
            while(size){
                real_size = MIN(size, 64);
                read_len = ql_uart_read(port, buf, real_size);
                if(read_len < 2 || buf[0] == 0xFF) {
                    return;
                }
                if((read_len > 0) && (size >= read_len)){
                    size -= read_len;
                    rt_ringbuffer_put(UART2_Ringbuf, buf, read_len);
                } else {
                    break;
                }
            }
        break;
        case QUEC_UART_TX_FIFO_COMPLETE_IND: 
        break;
    }
    def_rtos_smaphore_release(uart2_rec_sem);
}


static void ql_uart1_notify_cb(uint32 ind_type, ql_uart_port_number_e port, uint32 size)
{
    uint8_t buf[64];
    unsigned int real_size = 0;
    int read_len = 0;
    switch(ind_type)
    {
        case QUEC_UART_RX_OVERFLOW_IND:  //rx buffer overflow
        case QUEC_UART_RX_RECV_DATA_IND:
            while(size){
                real_size = MIN(size, 64);
                read_len = ql_uart_read(port, buf, real_size);
                if(read_len < 2 || buf[0] == 0xFF) {
                    return;
                }
                if((read_len > 0) && (size >= read_len)) {
                    size -= read_len;
                    rt_ringbuffer_put(UART1_Ringbuf, buf, read_len);
                } else {
                    break;
                }
            }
        break;
        case QUEC_UART_TX_FIFO_COMPLETE_IND: 
        break;
    }
    def_rtos_smaphore_release(uart1_rec_sem);
}

void hal_drv_uart_send(uint8_t uart_num, uint8_t *buf, uint16_t len)
{
    if(uart_num == UART2)
        ql_uart_write(QL_UART_PORT_2, buf, len);

    if(uart_num == UART1)
        ql_uart_write(QL_UART_PORT_1, buf, len);

}

void hal_drv_uart_init(uint8_t uart_num, uint32_t buad_rate, uint8_t parity)
{
    ql_uart_config_s uart_cfg = {0};

    uart_cfg.baudrate = buad_rate;
    uart_cfg.flow_ctrl = QL_FC_NONE;
    uart_cfg.data_bit = QL_UART_DATABIT_8;
    uart_cfg.stop_bit = QL_UART_STOP_1;
    uart_cfg.parity_bit = parity;

    if(uart_num == UART2) {
        def_rtos_semaphore_create(&uart2_rec_sem, 0);
        UART2_Ringbuf = rt_ringbuffer_create(UART2_BUF_MAXLEN);
        ql_uart_set_dcbconfig(QL_UART_PORT_2, &uart_cfg);
        ql_pin_set_func(UART2_TX_PIN, UART2_TX_FUNC);
        ql_pin_set_func(UART2_RX_PIN, UART2_RX_FUNC);
        ql_uart_open(QL_UART_PORT_2);
        ql_uart_register_cb(QL_UART_PORT_2, ql_uart2_notify_cb);
    }  else if(uart_num == UART1){
        def_rtos_semaphore_create(&uart1_rec_sem, 0);
        UART1_Ringbuf = rt_ringbuffer_create(UART1_BUF_MAXLEN);
        ql_uart_set_dcbconfig(QL_UART_PORT_1, &uart_cfg);
        ql_uart_open(QL_UART_PORT_1);
        ql_uart_register_cb(QL_UART_PORT_1, ql_uart1_notify_cb);
    }
}