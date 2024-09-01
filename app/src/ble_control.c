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
    uint8_t rely_timeout;       //应答最大超时
    uint8_t max_send_times;     //最大发送次数
};

struct ble_cmd_rely_order_s ble_cmd_rely_order[] = {
//  是否需要应答        超时时间           超时次数      
    {true,              1000,                3        },         /*CMD_BLE_ADV_START*/
    {true,              1000,                3        },         /*CMD_BLE_GET_MAC*/
    {true,              1000,                3         },         /*CMD_BLE_SET_ADV_DATA*/
    {true,              1000,                3        },         /*CMD_BLE_SET_SCANRSP_DATA*/
    {true,              1000,                3         },         /*CMD_BLE_GET_VER*/
    {true,              1000,                3        },         /*CMD_BLE_ADV_STOP*/
    {true,              1000,                3        },          /*CMD_BLE_DISCONNECT*/
    {true,              1000,                3        },         /*CMD_BLE_SET_ADV_INTERVAL*/
    {true,              1000,                3         },         /*CMD_BLE_SET_CON_PARAM*/
    {false,              0,                  0        },         /*CMD_BLE_HID_UNLOCK*/
    {false,              0,                  0         },         /*CMD_BLE_HID_LOCK*/
    {false,              0,                  0         },         /*CMD_BLE_TRANS*/
};

#define SENDDATALEN         36

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
void ble_cmd_mark(uint8_t cmd)
{
    def_rtos_queue_release(ble_send_cmd_que, sizeof(uint8_t), &cmd,  RTOS_WAIT_FOREVER);
}




void ble_send_data(uint8_t *data, uint16_t len)
{

}





void ble_control_init()
{
    def_rtos_queue_create(&ble_send_cmd_que, sizeof(uint8_t), 12);
    def_rtos_semaphore_create(&ble_send_sem, 1);
    def_rtos_semaphore_create(&ble_send_sem, 0);

}

