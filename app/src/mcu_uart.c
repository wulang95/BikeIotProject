#include "app_system.h"
#include "hal_drv_uart.h"

#define DBG_TAG         "mcu_uart"

#ifdef MCU_UART_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

#define HEADH  0XAA
#define HEADL  0X55

def_rtos_queue_t can_rcv_que;
def_rtos_queue_t mcu_cmd_que;

uint8_t mcu_cmd_table[] = {0X0C, 0X0E, 0X0D, 0x0f, 0x0b, 0x0a, 0x09, 0X08};
struct mcu_cmd_order_stu{
    uint8_t need_ask;
    uint16_t rely_timeout;
    uint8_t max_send_times;
};

struct mcu_cmd_order_stu mcu_cmd_order_table[] = {
    {false,     0,          0},     //CMD_CAN_TRANS
    {true,      1000,       3},     //CMD_GPS_POWERON
    {true,      1000,       3},     //CMD_GPS_POWEROFF
    {false,     0,          0},     //CMD_GPS_DATA
    {false,     0,          0},     //CMD_GPS_TRANS
    {true,      1000,       3},     //CMD_GPS_DEEPSLEEP
    {true,      1000,       3},     //CMD_GPS_HOST_START
    {true,      1000,       3},
};

struct mcu_cmd_send_crtl_stu
{
    uint8_t cmd;
    uint8_t send_cent;
    uint8_t ask_flag;
    uint8_t send_flag;
};
struct mcu_cmd_send_crtl_stu mcu_cmd_send_crtl;

uint8_t can_data_recv(stc_can_rxframe_t *can_rxframe, uint32_t time_out)
{
    uint8_t res = RTOS_SUCEESS;
    res = def_rtos_queue_wait(can_rcv_que, (uint8_t *)can_rxframe, sizeof(stc_can_rxframe_t), time_out);
    return res;
}


void mcu_data_pack(uint8_t cmd, uint8_t *data, uint16_t data_len, uint8_t *buf, uint16_t *lenth)
{
    uint8_t *p = buf;
    uint16_t len = 0;
    uint16_t crc_val;
    p[len++] = HEADH;
    p[len++] = HEADL;
    p[len++] = cmd;
    p[len++] = data_len>>8;
    p[len++] = data_len&0xff;
    if(data != NULL) {
        memcpy(&p[len], &data[0], data_len);
        len += data_len;
    }
    crc_val = Package_CheckSum(&p[2], len - 2);
    p[len++] = crc_val&0xff;
    p[len++] = crc_val>>8;
    *lenth = len;
}


void mcu_uart_send(uint8_t *data, uint16_t len)
{
    #ifdef MCU_WEEK
    uint8_t c = 0xff;
    hal_drv_uart_send(MCU_UART, &c, 1);
    #endif
    hal_drv_uart_send(MCU_UART, data, len);
    debug_data_printf("mcu_send", data, len);
}

void can_data_send(stc_can_rxframe_t can_txframe)
{
    uint8_t buf[56];
    uint16_t len;
    mcu_data_pack(CMD_CAN_TRANS, (uint8_t *)&can_txframe, sizeof(stc_can_rxframe_t), buf, &len);
    mcu_uart_send(buf, len);
}




void mcu_recv_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t data_len)
{
    stc_can_rxframe_t can_frame;
    LOG_I("cmd:%02x", cmd);
    switch (cmd)
    {
    case CMD_CAN_TRANS:
        memcpy(&can_frame, data, data_len);
        def_rtos_queue_release(can_rcv_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_frame, RTOS_WAIT_FOREVER);
        break;
    case CMD_GPS_POWERON:
    break;
    case CMD_GPS_POWEROFF:
    break;
    case CMD_GPS_DATA:
        LOG_I("GPS_data:%s", (char *)data);
        gps_data_trans(data, data_len);
    break;
    case CMD_GPS_DEEPSLEEP:
    break;
    case CMD_GPS_HOST_START:
    break;
    case CMD_CAT_REPOWERON:
    break;
    default:
        break;
    }
    if(mcu_cmd_send_crtl.send_flag && mcu_cmd_send_crtl.cmd == cmd) {
        mcu_cmd_send_crtl.ask_flag = 1;
    }
}

void MCU_CMD_MARK(uint8_t cmd)
{
    LOG_I("CMD:%d", cmd);
    if(cmd == CMD_CAN_TRANS_INDEX || cmd == CMD_GPS_DATA || cmd == CMD_GPS_TRANS_INDEX) return;
    def_rtos_queue_release(mcu_cmd_que, sizeof(uint8_t), &cmd, RTOS_WAIT_FOREVER);
}

void mcu_send_fail()
{
    LOG_E("mcu_send_fail");
}

void mcu_uart_send_thread(void *param)
{
    uint8_t cmd_index;
    def_rtosStaus res;
    int64_t time_t;
    uint8_t buf[36];
    uint16_t lenth;
    while(1) {
        res = def_rtos_queue_wait(mcu_cmd_que, &cmd_index, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        mcu_cmd_send_crtl.cmd = mcu_cmd_table[cmd_index];
        mcu_cmd_send_crtl.send_flag = 1;
        mcu_cmd_send_crtl.send_cent = 0;
        mcu_cmd_send_crtl.ask_flag = 0;
        mcu_data_pack(mcu_cmd_send_crtl.cmd, NULL, 0, buf, &lenth);
        mcu_uart_send(buf, lenth);
        mcu_cmd_send_crtl.send_cent++;
        time_t = def_rtos_get_system_tick();
        for(;;) {
            if(mcu_cmd_order_table[cmd_index].need_ask) {
                if(mcu_cmd_send_crtl.ask_flag == 1){
                    break;
                }
                if(def_rtos_get_system_tick() - time_t >= mcu_cmd_order_table[cmd_index].rely_timeout) {
                    if(mcu_cmd_send_crtl.send_cent >= mcu_cmd_order_table[cmd_index].max_send_times) {
                        mcu_send_fail();
                        break;
                    }
                    mcu_uart_send(buf, lenth);
                    mcu_cmd_send_crtl.send_cent++;
                    time_t = def_rtos_get_system_tick();
                }
                def_rtos_task_sleep_ms(5);
            } else break;
        }
        mcu_cmd_send_crtl.send_flag = 0;
    }
    def_rtos_task_delete(NULL);
}

void mcu_uart_recv_thread(void *param)
{
    uint8_t rcv[256],  data[256], rec_len = 0, c;
    uint8_t cmd, step = 0;
    uint16_t len, i, j;
    uint16_t check_sum = 0, rcv_check = 0;
    int64_t start_t = 0;
    mcu_uart_init();
    while(1){
        len = hal_drv_uart_read(MCU_UART, rcv, 256, RTOS_WAIT_FOREVER);
        if(len == 0) continue;
        debug_data_printf("mcurcv",rcv, len);
        for(i = 0; i < len; i++){
            c = rcv[i];
            switch(step) {
                case 0:
                    if(c == HEADH) {
                        start_t = def_rtos_get_system_tick();
                        step = 1;
                        check_sum = 0;
                        rcv_check = 0;
                        rec_len = 0;
                        j = 0;
                        memset(&data, 0, sizeof(data));
                    }
                break;
                case 1:
                    if(c == HEADL) {
                        step = 2;
                    } else {
                        step = 0;
                    }
                break;
                case 2:
                    cmd = c;
                    step = 3;
                    check_sum += c;
                break;
                case 3:
                    check_sum += c;
                    rec_len = c<<8;
                    step =4;
                break;
                case 4:
                    check_sum += c;
                    rec_len |= c;
                    if(rec_len)
                        step = 5;
                    else step = 6; 
                break;
                case 5:
                    data[j++] = c;
                    check_sum += c;
                    if(j == rec_len) step = 6;
                break;
                case 6:
                    rcv_check = c;
                    step = 7;
                break;
                case 7:
                    rcv_check |= c << 8;
                    check_sum = check_sum^0xffff;
                    if(check_sum == rcv_check)
                        mcu_recv_cmd_handler(cmd, data, rec_len);
                    else LOG_E("CHECK is fail, check_sum:%04x, rcv_check:%04x", check_sum, rcv_check);
                    step = 0;
                break;    
            }
        }
        if(systm_tick_diff(start_t) > 500) {
            step = 0;
        }
    }
    def_rtos_task_delete(NULL);
}

void mcu_uart_init()
{

    def_rtos_queue_create(&mcu_cmd_que, sizeof(uint8_t), 12);
    def_rtos_queue_create(&can_rcv_que, sizeof(stc_can_rxframe_t), 12);
    hal_drv_uart_init(MCU_UART, MCU_BAUD, MCU_PARITY);
    GPS_Init();
    LOG_I("mcu_uart_init is ok");
}