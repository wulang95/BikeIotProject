/*蓝牙控制*/
#include "ble_control.h"
#include "app_system.h"
#include "hal_drv_uart.h"
#include "ringbuffer.h"
#include "hal_drv_gpio.h"
#include "app_virt_uart.h"
#define DBG_TAG         "ble_control"

#ifdef BLE_CONTROL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define BLE_TRANSBUF_LEN    1024
struct rt_ringbuffer *ble_transbuf;

uint8_t ble_cmd_table[BLE_INDEX_MAX] = {0x04, 0x01, 0x0c, 0x02, 0x03, 0x0b, 0x05, 0x09, 0x0a, 0x07, 0x08, 0x06, 0x0d};

struct ble_cmd_rely_order_s{
    uint8_t need_ask;           //是否需要应答
    uint16_t rely_timeout;       //应答最大超时
    uint8_t max_send_times;     //最大发送次数
};

struct ble_cmd_rely_order_s ble_cmd_rely_order[] = {
//  是否需要应答        超时时间           超时次数      
    {true,              1000,                3        },         /*CMD_BLE_ADV_START*/
    {true,              1000,                3        },         /*CMD_BLE_GET_MAC*/
    {true,              1000,                3        },         /*CMD_BLE_SET_ADV_DATA*/
    {true,              1000,                3        },         /*CMD_BLE_SET_SCANRSP_DATA*/
    {true,              1000,                3        },         /*CMD_BLE_GET_VER*/
    {true,              1000,                3        },         /*CMD_BLE_ADV_STOP*/
    {true,              1000,                3        },         /*CMD_BLE_DISCONNECT*/
    {true,              1000,                3        },         /*CMD_BLE_SET_ADV_INTERVAL*/
    {true,              1000,                3        },         /*CMD_BLE_SET_CON_PARAM*/
    {false,              0,                  0        },         /*CMD_BLE_HID_UNLOCK*/
    {false,              0,                  0        },         /*CMD_BLE_HID_LOCK*/
    {false,              0,                  0        },         /*CMD_BLE_TRANS*/
    {false,              0,                  0        },         /*CMD_BLE_VIRT_AT*/
    {true,              1000,                3        },         /*CMD_BLE_DELETE_BIND_INFO*/
};

#define SENDDATALEN         256

struct ble_cmd_send_var_s{
    uint8_t cmd_index;                //当前发送命令
    uint8_t ask_flag;            //发送完成
    uint8_t send_cnt;               //发送次数
    uint8_t send_buf[SENDDATALEN];  //发送buffer
    uint16_t sendlen;                //发送长度
} ble_cmd_send_var;

def_rtos_queue_t ble_send_cmd_que;
def_rtos_sem_t ble_trans_recv_sem;





struct ble_info_s ble_info;
void ble_cmd_pack(uint8_t cmd, uint8_t *data, uint16_t len, uint8_t *buff, uint16_t *buf_len)
{
    uint16_t lenth = 0;
    uint16_t check;
    uint8_t *buf;
    buf = buff;
    buf[lenth++] = 0x55;
    buf[lenth++] = 0xaa;
    buf[lenth++] = cmd;

    switch(cmd){
        case CMD_BLE_ADV_START:
        case CMD_BLE_GET_MAC:
        case CMD_BLE_GET_VER:
        case CMD_BLE_ADV_STOP:
        case CMD_BLE_DISCONNECT:
        case CMD_BLE_HID_UNLOCK:
        case CMD_BLE_HID_LOCK:
        case CMD_BLE_DELETE_BIND_INFO:
            buf[lenth++] = 0x00;
            buf[lenth++] = 0x00;
        break;
        case CMD_BLE_SET_ADV_DATA:
            buf[lenth++] = 0x00;
            buf[lenth++] = ble_info.ble_adv_data.len;
            memcpy(&buf[lenth], &ble_info.ble_adv_data.data[0], ble_info.ble_adv_data.len);
            lenth += ble_info.ble_adv_data.len;
        break;
        case CMD_BLE_SET_SCANRSP_DATA:
            buf[lenth++] = 0x00;
            buf[lenth++] = ble_info.ble_scanrsp_data.len;
            memcpy(&buf[lenth], &ble_info.ble_scanrsp_data.data[0], ble_info.ble_scanrsp_data.len);
            lenth += ble_info.ble_scanrsp_data.len;
        break;
        case CMD_BLE_SET_ADV_INTERVAL:
            buf[lenth++] = 0x00;
            buf[lenth++] = 4;
            buf[lenth++] = ble_info.ble_adv_param.adv_inv_min<<8;
            buf[lenth++] = ble_info.ble_adv_param.adv_inv_min&0xff;
            buf[lenth++] = ble_info.ble_adv_param.adv_inv_max<<8;
            buf[lenth++] = ble_info.ble_adv_param.adv_inv_max&0xff;
        break;
        case CMD_BLE_SET_CON_PARAM:
            buf[lenth++] = 0x00;
            buf[lenth++] = 0x08;
            buf[lenth++] = ble_info.ble_con_param.con_inv_min<<8;
            buf[lenth++] = ble_info.ble_con_param.con_inv_min&0xff;
            buf[lenth++] = ble_info.ble_con_param.con_inv_max<<8;
            buf[lenth++] = ble_info.ble_con_param.con_inv_max&0xff;
            buf[lenth++] = ble_info.ble_con_param.con_latency<<8;
            buf[lenth++] = ble_info.ble_con_param.con_latency&0xff;
            buf[lenth++] = ble_info.ble_con_param.con_timeout<<8;
            buf[lenth++] = ble_info.ble_con_param.con_timeout&0xff;
        break;
        case CMD_BLE_VIRT_AT:
        case CMD_BLE_TRANS:
        buf[lenth++] = len >> 8;
        buf[lenth++] = len&0xff;
        memcpy(&buf[lenth], data, len);
        lenth += len;
        break;
    default:
        break;
    }
    check = Package_CheckSum(&buf[2], lenth - 2);  
    buf[lenth++] = check&0xff;
    buf[lenth++] = check>>8;
    *buf_len = lenth;
}

void ble_cmd_mark(uint8_t cmd)
{
    def_rtos_queue_release(ble_send_cmd_que, sizeof(uint8_t), &cmd,  RTOS_WAIT_FOREVER);
}

void ble_send_data(uint8_t *data, uint16_t len)
{
    hal_drv_uart_send(BLE_UART, data, len);
    debug_data_printf("ble_send", data, len);
}



void ble_cmd_send_fail(uint8_t cmd)
{

}
  

void ble_control_send_thread(void *param)
{
    def_rtosStaus res;
    int64_t time_t;
    uint8_t cmd_index;
    while (1)
    {
        res = def_rtos_queue_wait(ble_send_cmd_que, &ble_cmd_send_var.cmd_index, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        cmd_index = ble_cmd_send_var.cmd_index;
        ble_cmd_send_var.ask_flag = 0;
        ble_cmd_send_var.send_cnt = 0;
        ble_cmd_pack(ble_cmd_table[cmd_index], NULL, 0, ble_cmd_send_var.send_buf, &ble_cmd_send_var.sendlen);
        ble_send_data(ble_cmd_send_var.send_buf, ble_cmd_send_var.sendlen);
        ble_cmd_send_var.send_cnt++;
        time_t = def_rtos_get_system_tick();
        for(;;){
            if(ble_cmd_rely_order[cmd_index].need_ask) {
                if(ble_cmd_send_var.ask_flag == 1){
                    break;
                } 
                if(def_rtos_get_system_tick() - time_t >= ble_cmd_rely_order[cmd_index].rely_timeout){
                    if(ble_cmd_send_var.send_cnt >= ble_cmd_rely_order[cmd_index].max_send_times){
                        ble_cmd_send_fail(ble_cmd_send_var.cmd_index);
                        break;
                    }
                    ble_send_data(ble_cmd_send_var.send_buf, ble_cmd_send_var.sendlen);
                    ble_cmd_send_var.send_cnt++;
                    time_t = def_rtos_get_system_tick();
                }
                def_rtos_task_sleep_ms(5);
            } else break;
        }
    }
    def_rtos_task_delete(NULL); 
}

uint16_t ble_trans_data_block_read(uint8_t *buf, uint16_t len, uint32_t time_out)
{
    uint16_t buf_len, read_len;
    uint8_t res = RTOS_SUCEESS;
    res = def_rtos_semaphore_wait(ble_trans_recv_sem, time_out);
    if(res == RTOS_SUCEESS) {
        buf_len = rt_ringbuffer_data_len(ble_transbuf);
        read_len = MIN(buf_len, len);
        rt_ringbuffer_get(ble_transbuf, buf, read_len);
        return read_len;
    }
    return 0;
}

void ble_recv_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t len)
{
    char *p = NULL;
    LOG_I("cmd:%d", cmd);
    switch(cmd)
    {
        case CMD_BLE_ADV_START:
        case CMD_BLE_SET_ADV_DATA:
        case CMD_BLE_SET_SCANRSP_DATA:
        case CMD_BLE_DISCONNECT:
        case CMD_BLE_SET_ADV_INTERVAL:
        case CMD_BLE_SET_CON_PARAM:
        case CMD_BLE_ADV_STOP:
        case CMD_BLE_DELETE_BIND_INFO:
            break;
        case CMD_BLE_HID_UNLOCK:
            break;
        case CMD_BLE_HID_LOCK:
            break;
        case CMD_BLE_TRANS:
            rt_ringbuffer_put(ble_transbuf, data, len);
            def_rtos_smaphore_release(ble_trans_recv_sem);
            break;
        case CMD_BLE_GET_VER:
            LOG_I("BLE_VER:%s", (char *)data);
            memcpy(&ble_info.ver[0], &data[0], strlen((char *)data));
            break;
        case CMD_BLE_GET_MAC:
            memcpy(ble_info.mac, data, 6);
            break;
        case CMD_BLE_VIRT_AT:
            p = (char *)malloc(len+3);
            sprintf(p, "%s\r\n", (char *)data);
            LOG_I("[%d]%s", strlen(p), p);
            app_virt_uart_write(AT_VIRT_BLE, p);
            free(p);
            break;
    }
    if(ble_cmd_table[ble_cmd_send_var.cmd_index] == cmd) {
        ble_cmd_send_var.ask_flag = 1;
    }
}

void ble_control_recv_thread(void *param)
{
    uint8_t rcv[256],  data[256], rec_len = 0, c;
    uint8_t cmd, step = 0;
    uint16_t len, i, j;
    uint16_t check_sum = 0, rcv_check = 0;
    int64_t start_t = 0;
    while(1){
        len = hal_drv_uart_read(BLE_UART, rcv, 256, RTOS_WAIT_FOREVER);
        if(len == 0) continue;
        debug_data_printf("blerecv",rcv, len);
        for(i = 0; i < len; i++){
            c = rcv[i];
            switch(step) {
                case 0:
                    if(c == 0x55) {
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
                    if(c == 0xaa) {
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
                        ble_recv_cmd_handler(cmd, data, rec_len);
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

static void ble_power_init()
{
    char buf[24], *p;
    uint16_t len;
    uint8_t i;
    hal_drv_write_gpio_value(O_BLE_POWER, LOW_L);
    for(i = 0; i < 3; i++){
        def_rtos_task_sleep_ms(10);
        len = hal_drv_uart_read(BLE_UART, (uint8_t *)buf, 24, 300);
        if(len) {
            LOG_I("%s", buf);
            p = strstr(buf, "mac:");
            if(p) {
                HexSrt_To_Value(ble_info.mac, p+4, 12);
                ble_info.init = 1;
                break;
            }
        } else {
            hal_drv_write_gpio_value(O_BLE_POWER, HIGH_L);
            LOG_E("ble init recv fail count:%d", i);
            def_rtos_task_sleep_ms(8000);
        }
        hal_drv_write_gpio_value(O_BLE_POWER, LOW_L);
        def_rtos_task_sleep_ms(10);
    }
    if(ble_info.init != 1) {
        LOG_E("ble init fail");
        return;
    } 
} 

void ble_param_init()
{
    ble_info.ble_adv_param.adv_inv_max = 600;
    ble_info.ble_adv_param.adv_inv_min = 600;
    ble_info.ble_con_param.con_inv_max = 12;
    ble_info.ble_con_param.con_inv_min = 12;
    ble_info.ble_con_param.con_latency = 0;
    ble_info.ble_con_param.con_timeout = 800;
}

void ble_set_adv_data(uint8_t *data, uint8_t len)
{
    memcpy(ble_info.ble_adv_data.data, data, len);
    ble_info.ble_adv_data.len = len;
    ble_cmd_mark(BLE_SET_ADV_DATA_INDEX);
}

void ble_set_scanrsp_data(uint8_t *data, uint8_t len)
{
    memcpy(ble_info.ble_scanrsp_data.data, data, len);
    ble_info.ble_scanrsp_data.len = len;
    ble_cmd_mark(BLE_SET_SCANRSP_DATA_INDEX);
}

void sys_set_ble_adv_start()
{
    uint8_t buf[32];
    char str[30];
    uint8_t len = 0;
    sprintf(str, "%s-%02X", BLE_NAME, ble_info.mac[5]);
    LOG_I("%s", str);
    buf[1+len] = GAP_ADVTYPE_LOCAL_NAME_COMPLETE;
    len++;
    memcpy(&buf[1+len], str, strlen(str));
    len += strlen(str);
    buf[0] = len;
    len += 1;
    buf[len++] = 3;
    buf[len++] = GAP_ADVTYPE_16BIT_MORE;
    buf[len++] = BLE_SUUID >> 8;
    buf[len++] = BLE_SUUID&0XFF;
    ble_set_adv_data(buf, len);
    debug_data_printf("ble_adv_data", buf, len);

    len = 0;
    buf[1 +len] = GAP_ADVTYPE_MANUFACTURER_SPECIFIC;
    len++;
    memcpy(&buf[1+len], &ble_info.mac[0], 6);
    len += 6;
    buf[0] = len;
    len++;
    ble_set_scanrsp_data(buf, len);
    debug_data_printf("ble_scanrsp_data", buf, len);
    ble_cmd_mark(BLE_ADV_START_INDEX);
}

void ble_control_init()
{
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_queue_create(&ble_send_cmd_que, sizeof(uint8_t), 12);
    if(err != RTOS_SUCEESS) {
        LOG_E("ble_send_cmd_que is create fail err:%d", err);
    }
    def_rtos_semaphore_create(&ble_trans_recv_sem, 0);
    ble_transbuf = rt_ringbuffer_create(BLE_TRANSBUF_LEN);
    memset(&ble_info, 0, sizeof(ble_info));
    ble_power_init();
    ble_param_init();
    ble_cmd_mark(BLE_GET_VER_INDEX);
    sys_set_ble_adv_start();
    LOG_I("ble_control_init is ok");
}

