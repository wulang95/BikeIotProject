/*蓝牙控制*/
#include "ble_control.h"

#define DBG_TAG         "ble_control"

#ifdef APP_MAIN_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

uint8_t ble_cmd_table[] = {0x04, 0x01, 0x0c, 0x02, 0x03, 0x0b, 0x05, 0x09, 0x0a, 0x07, 0x08, 0x06};

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
    {true,              1000,                3        },          /*CMD_BLE_DISCONNECT*/
    {true,              1000,                3        },         /*CMD_BLE_SET_ADV_INTERVAL*/
    {true,              1000,                3         },         /*CMD_BLE_SET_CON_PARAM*/
    {false,              0,                  0        },         /*CMD_BLE_HID_UNLOCK*/
    {false,              0,                  0         },         /*CMD_BLE_HID_LOCK*/
    {false,              0,                  0         },         /*CMD_BLE_TRANS*/
};

#define SENDDATALEN         256

struct ble_cmd_send_var_s{
    uint8_t cur_cmd;                //当前发送命令
    uint8_t send_finish;            //发送完成
    uint8_t send_cnt;               //发送次数
    uint8_t send_buf[SENDDATALEN];  //发送buffer
    uint8_t sendlen;                //发送长度
} ble_cmd_send_var;

def_rtos_queue_t ble_send_cmd_que;
def_rtos_sem_t ble_send_sem;
def_rtos_sem_t ble_trans_recv_sem;

struct ble_adv_info_s {
    uint8_t data[32];
    uint8_t len;
};

struct ble_adv_param_s{
    uint16_t adv_inv_min;
    uint16_t adv_inv_max;
};

struct ble_con_param_s{
    uint16_t con_inv_min;
    uint16_t con_inv_max;
    uint16_t con_latency;
    uint16_t con_timeout;
};

struct ble_info_s{
    uint16_t  ver;
    uint8_t mac[6];
    struct ble_adv_param_s ble_adv_param;
    struct ble_con_param_s ble_con_param;
    struct ble_adv_info_s ble_adv_data;
    struct ble_adv_info_s ble_scanrsp_data;
};

struct ble_info_s ble_info;
void ble_cmd_pack(uint8_t cmd, uint8_t *data, uint16_t len, uint8_t *buff, uint16_t *buf_len)
{
    uint16_t lenth = 0;
    uint16_t check;
    uint8_t *buf;
    buf = buff;
    buf[lenth++] = 0x55;
    buf[lenth++] = 0xaa;
    buf[lenth++] = ble_cmd_table[cmd];

    switch(cmd){
        case CMD_BLE_ADV_START:
        case CMD_BLE_GET_MAC:
        case CMD_BLE_GET_VER:
        case CMD_BLE_ADV_STOP:
        case CMD_BLE_DISCONNECT:
        case CMD_BLE_HID_UNLOCK:
        case CMD_BLE_HID_LOCK:
            buf[lenth++] = 0x00;
            buf[lenth++] = 0x00;
        break;
        case CMD_BLE_SET_ADV_DATA:
            buf[lenth++] = 0x00;
            buf[lenth++] = ble_info.ble_adv_data.len;
            memcpy(&buf[lenth], &ble_info.ble_adv_data.data[0], ble_info.ble_adv_data.len);
            lenth += ble_info.ble_adv_data.len;
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
        case CMD_BLE_TRANS:
        buf[lenth++] = len >> 8;
        buf[lenth++] = len&0xff;
        memcpy(&buf[lenth], data, len);
        lenth += len;
        break;
    default:
        break;
    }
    check = Package_CheckSum(&buf[0], lenth);  
    buf[lenth++] = check&0xff;
    buf[lenth++] = check>>8;
    buf_len = lenth;
}

void ble_cmd_mark(uint8_t cmd)
{
    def_rtos_queue_release(ble_send_cmd_que, sizeof(uint8_t), &cmd,  RTOS_WAIT_FOREVER);
}




void ble_send_data(uint8_t *data, uint16_t len)
{
    def_rtos_semaphore_wait(ble_send_sem, RTOS_WAIT_FOREVER);
    hal_drv_uart_send(BLE_UART, data, len);
    def_rtos_smaphore_release(ble_send_sem);
}

void ble_control_send_thread(void *param)
{
    uint8_t cmd;
    while (1)
    {
        def_rtos_queue_wait(ble_send_cmd_que, &ble_cmd_send_var.cur_cmd, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        ble_cmd_send_var.send_finish = 0;
        ble_cmd_pack(ble_cmd_send_var.cur_cmd, NULL, 0, ble_cmd_send_var.send_buf, &ble_cmd_send_var.sendlen);
        ble_send_data(ble_cmd_send_var.send_buf, ble_cmd_send_var.sendlen);
        


    }
    def_rtos_task_delete(NULL);
    
}

void ble_control_recv_thread(void *param)
{

}


void ble_control_init()
{
    def_rtos_queue_create(&ble_send_cmd_que, sizeof(uint8_t), 12);
    def_rtos_semaphore_create(&ble_send_sem, 1);
    def_rtos_semaphore_create(&ble_trans_recv_sem, 0);

}

