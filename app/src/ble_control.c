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

uint8_t ble_cmd_table[BLE_INDEX_MAX] = {0x04, 0x01, 0x0c, 0x02, 0x03, 0x0b, 0x05, 0x09, 0x0a, 0x07, 0x08, 0x06, 0x0d, 0x0f, 0x0e, 0X10, 0X11,0X12};

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
    {true,              1000,                3        },         /*CMD_BLE_ENTER_SLEEP*/
    {true,              2000,                3        },         /*CMD_BLE_OTA_START*/
    {true,              2000,                3        },         /*CMD_BLE_OTA_DATA*/
    {true,              5000,                3        },         /*CMD_BLE_OTA_END*/
    {false,              0,                  0        },         /*CMD_BLE_SIGN*/
    {false,              0,                  0        },        /*CMD_BLE_HEART*/
};

#define SENDDATALEN         256

struct ble_cmd_send_var_s{
    uint8_t cmd_index;                //当前发送命令
    uint8_t ask_flag;            //发送完成
    uint8_t send_flag;              //发送标志
    uint8_t send_cnt;               //发送次数
    uint8_t send_buf[SENDDATALEN];  //发送buffer
    uint16_t sendlen;                //发送长度
} ble_cmd_send_var;

def_rtos_queue_t ble_send_cmd_que;
def_rtos_sem_t ble_trans_recv_sem;
struct ble_info_s ble_info;

struct ble_ota_config_stu{
    uint32_t firm_ver;
    uint32_t ota_crc;
    uint32_t total;
};

struct ble_ota_ctrl_stu{
    uint8_t ota_sta;
    uint32_t offset;
    uint32_t total_len;
    uint8_t *data;
    uint8_t data_len;
    uint8_t ota_res;
    uint16_t pack_num;
    def_rtos_sem_t con_sem_t;
};

enum {
    BLE_OTA_IDEL_STEP = 0,
    BLE_OTA_START_STEP,
    BLE_OTA_DATA_STEP,
    BLE_OTA_END_STEP,
    BLE_OTA_ERROR_STEP,
};
int64_t ble_heart_time_t;
static struct ble_ota_config_stu ble_ota_config;
static struct ble_ota_ctrl_stu ble_ota_ctrl;
static void ble_ota_start()
{
    unsigned int CRC32 = 0xFFFFFFFF;
    uint32_t read_len, offset = 0, total_len;
    ble_ota_config.firm_ver = 0x01010101;
    ble_ota_config.total = flash_partition_size(DEV_APP_ADR);
    ble_ota_ctrl.total_len = ble_ota_config.total;
    total_len =  ble_ota_config.total;
    offset = 256;
    total_len -= offset;
    while(total_len > 0) {
        read_len = MIN(total_len, 128);
        flash_partition_read(DEV_APP_ADR, ble_ota_ctrl.data, read_len, offset);
        CRC32 = GetCrc32_cum(ble_ota_ctrl.data, read_len, CRC32);
        offset += read_len;
        total_len -= read_len; 
    }
    ble_ota_config.ota_crc = CRC32;
    LOG_I("firm_ver:%0x,total_len:%d, CRC:%0x", ble_ota_config.firm_ver, ble_ota_config.total, ble_ota_config.ota_crc);
    ble_cmd_mark(BLE_OTA_START_INDEX);
    if(def_rtos_semaphore_wait(ble_ota_ctrl.con_sem_t, 10000) != RTOS_SUCEESS) {
        LOG_E("BLE OTA START IS FAIL");
        ble_ota_ctrl.ota_sta = BLE_OTA_ERROR_STEP;
        return;
    }
    ble_ota_ctrl.ota_sta = BLE_OTA_DATA_STEP;
}

static void ble_ota_data()
{
    ble_ota_ctrl.data_len = MIN((ble_ota_ctrl.total_len - ble_ota_ctrl.offset), 128);
    flash_partition_read(DEV_APP_ADR, &ble_ota_ctrl.data[0], ble_ota_ctrl.data_len , ble_ota_ctrl.offset);
    ble_ota_ctrl.offset += ble_ota_ctrl.data_len;
    LOG_I("pack_num:%d, offset:%d", ble_ota_ctrl.pack_num, ble_ota_ctrl.offset);
    ble_cmd_mark(BLE_OTA_DATA_INDEX);
    if(def_rtos_semaphore_wait(ble_ota_ctrl.con_sem_t, 10000) != RTOS_SUCEESS) {
        LOG_E("BLE OTA DATA IS FAIL");
        ble_ota_ctrl.ota_sta = BLE_OTA_ERROR_STEP;
        return;
    }
    ble_ota_ctrl.pack_num++;
    if(ble_ota_ctrl.offset%256 == 0){
        LOG_I("ble ota progress %%%d", ((ble_ota_ctrl.offset*100)/ble_ota_ctrl.total_len));
    }
    if(ble_ota_ctrl.total_len == ble_ota_ctrl.offset) {
        ble_ota_ctrl.ota_sta = BLE_OTA_END_STEP;
    }
}

static void ble_ota_end()
{
    ble_cmd_mark(BLE_OTA_END_INDEX);
    LOG_E("BLE_OTA_END_STEP");
    if(def_rtos_semaphore_wait(ble_ota_ctrl.con_sem_t, 16000) != RTOS_SUCEESS) {
        LOG_E("BLE OTA END IS FAIL");
        ble_ota_ctrl.ota_sta = BLE_OTA_ERROR_STEP;
        return;
    }
}

int ble_ota_task()
{
    int64_t ble_ota_start_time_t;
    ble_ota_ctrl.ota_sta = BLE_OTA_IDEL_STEP;
    if(ble_ota_ctrl.ota_sta != BLE_OTA_IDEL_STEP) return FAIL;
    ble_ota_ctrl.data = malloc(128);
    if(ble_ota_ctrl.data == NULL) return FAIL;
    if(def_rtos_semaphore_create(&ble_ota_ctrl.con_sem_t, 0) != RTOS_SUCEESS){
        LOG_E("ble_ota_ctrl.con_sem_t is create fail");
        return FAIL;
    }
    LOG_I("ble_ota_task is run");
    ble_cmd_mark(BLE_ADV_STOP_INDEX);
    ble_ota_ctrl.ota_sta = BLE_OTA_START_STEP;
    ble_ota_ctrl.offset = 0;
    ble_ota_ctrl.ota_res = 0xff;
    ble_ota_ctrl.data_len = 0;
    ble_ota_ctrl.pack_num = 0;
    ble_ota_start_time_t = def_rtos_get_system_tick();
    while(1){
        switch(ble_ota_ctrl.ota_sta){
            case BLE_OTA_START_STEP:
                ble_ota_start();
            break;
            case BLE_OTA_DATA_STEP:
                ble_ota_data();
            break;
            case BLE_OTA_END_STEP:
                ble_ota_end();
                free(ble_ota_ctrl.data);
                def_rtos_semaphore_delete(ble_ota_ctrl.con_sem_t);
                if(ble_ota_ctrl.ota_res == 0) {
                    LOG_I("BLE OTA IS SUCCESS");
                    ble_cmd_mark(BLE_GET_VER_INDEX);
                    return OK;
                }
                else {
                    ble_cmd_mark(BLE_ADV_START_INDEX);
                    LOG_I("BLE OTA IS FAIL");
                    return FAIL;
                }
            break;
            case BLE_OTA_ERROR_STEP:
                LOG_I("BLE_OTA_ERROR_STEP");
                ble_cmd_mark(BLE_ADV_START_INDEX);
                free(ble_ota_ctrl.data);
                def_rtos_semaphore_delete(ble_ota_ctrl.con_sem_t);
                return FAIL;
            break;
        }
        if(def_rtos_get_system_tick() - ble_ota_start_time_t > 10*60*1000){
            return FAIL;
        }
    }
}

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
        case CMD_BLE_ENTER_SLEEP:
        case CMD_BLE_OTA_END:
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
        case CMD_BLE_OTA_START:
            buf[lenth++] = sizeof(struct ble_ota_config_stu) >> 8;
            buf[lenth++] = sizeof(struct ble_ota_config_stu) & 0xff;
            memcpy(&buf[lenth], &ble_ota_config, sizeof(struct ble_ota_config_stu));
            lenth += sizeof(struct ble_ota_config_stu);
        break;
        case CMD_BLE_OTA_DATA:
            buf[lenth++] = (ble_ota_ctrl.data_len + 2) >> 8;
            buf[lenth++] = (ble_ota_ctrl.data_len + 2)&0xff;
            memcpy(&buf[lenth], &ble_ota_ctrl.pack_num, 2);
            lenth += 2;
            memcpy(&buf[lenth], &ble_ota_ctrl.data[0], ble_ota_ctrl.data_len);
            lenth += ble_ota_ctrl.data_len;
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
    LOG_I("CMD:%d", cmd);
    def_rtos_queue_release(ble_send_cmd_que, sizeof(uint8_t), &cmd,  RTOS_WAIT_FOREVER);
}

void ble_send_data(uint8_t *data, uint16_t len)
{
    uint16_t send_len, offset = 0;
    while(len > 0){
        send_len = MIN(32, len);
        hal_drv_uart_send(BLE_UART, &data[offset], send_len);
        debug_data_printf("ble_send", &data[offset], send_len);
        len -= send_len;
        offset += send_len;
        def_rtos_task_sleep_ms(5);
    }
//  hal_drv_uart_send(BLE_UART, data, len);    
}

void ble_cmd_send_fail(uint8_t cmd)
{
    iot_error_set(IOT_ERROR_TYPE, BLE_ERROR);
    LOG_E("ble send cmd:%02x is fail", cmd);
}
  
void ble_control_send_thread(void *param)
{
    def_rtosStaus res;
    int64_t time_t;
    uint8_t cmd_index;
    while (1)
    {
 //       LOG_I("IS RUN");
        res = def_rtos_queue_wait(ble_send_cmd_que, &ble_cmd_send_var.cmd_index, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        cmd_index = ble_cmd_send_var.cmd_index;
        ble_cmd_send_var.ask_flag = 0;
        ble_cmd_send_var.send_cnt = 0;
        ble_cmd_send_var.send_flag = 1;
        LOG_I("CMD_INDEX:%d, cmd:%02x", cmd_index, ble_cmd_table[cmd_index]);
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
                        ble_cmd_send_fail(ble_cmd_table[cmd_index]);
                        break;
                    }
                    ble_send_data(ble_cmd_send_var.send_buf, ble_cmd_send_var.sendlen);
                    ble_cmd_send_var.send_cnt++;
                    time_t = def_rtos_get_system_tick();
                }
                def_rtos_task_sleep_ms(5);
            } else break;
        }
        ble_cmd_send_var.send_flag = 0;
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
    memcpy(&buf[1+len], DEFAULT_MANUFACTURER, 2);
    len += 2;
    memcpy(&buf[1+len], DEFAULT_DEV_TYPE, 2);
    len += 2;
    memcpy(&buf[1+len], &ble_info.ver[0], strlen(ble_info.ver));
    len += strlen(ble_info.ver);
    buf[0] = len;
    len++;
    ble_set_scanrsp_data(buf, len);
    debug_data_printf("ble_scanrsp_data", buf, len);
    ble_cmd_mark(BLE_ADV_START_INDEX);
}

void ble_recv_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t len)
{
    LOG_I("cmd:%0x", cmd);
    switch(cmd)
    {
        case CMD_BLE_ADV_START:
            ble_info.adv_sta = 1;
            LOG_I("ble start adv");
        break;
        case CMD_BLE_ADV_STOP:
            ble_info.adv_sta = 0;
            LOG_I("ble stop adv");
        break;
        case CMD_BLE_SET_ADV_DATA:
        case CMD_BLE_SET_SCANRSP_DATA:
        case CMD_BLE_DISCONNECT:
        case CMD_BLE_SET_ADV_INTERVAL:
        case CMD_BLE_SET_CON_PARAM:
        case CMD_BLE_DELETE_BIND_INFO:
        case CMD_BLE_ENTER_SLEEP:
            break;
        case CMD_BLE_HEART:
            
        break;
        case CMD_BLE_SIGN:
            LOG_I("ble sign");
            if(ble_info.init == 1 && ble_info.adv_sta == 1) {   //防止系统异常重启
                LOG_I("ble reset");
                sys_set_ble_adv_start();
            }
        break;
        case CMD_BLE_HID_UNLOCK:   //持续检测到蓝牙信号，无关远离和靠近
            if(sys_param_set.hid_lock_sw) {
                // if(sys_set_var.hid_lock_sw_type == 0) {
                //     car_lock_control(HID_CAR_CMD_SER, CAR_UNLOCK_STA);
                //     ble_cmd_mark(CMD_BLE_HID_UNLOCK);
                // } else {
                if(car_info.hmi_info.power_on && car_info.lock_sta == CAR_LOCK_STA){
                    car_control_cmd(CAR_CMD_JUMP_PASSWORD);
                    rtc_event_unregister(CAR_SET_EN_POWER_PASSWD);
                }
                // }
            }
            ble_cmd_mark(BLE_HID_UNLOCK_INDEX);
            break;
        case CMD_BLE_HID_LOCK:  //持续检测到蓝牙信号，无关远离和靠近
            if(sys_param_set.hid_lock_sw) {
                if(car_info.hmi_info.power_on && car_info.lock_sta == CAR_LOCK_STA){
                    car_control_cmd(CAR_CMD_JUMP_PASSWORD);
                    rtc_event_unregister(CAR_SET_EN_POWER_PASSWD);
                }
            }
            ble_cmd_mark(BLE_HID_LOCK_INDEX);
            // if(sys_set_var.hid_lock_sw) {
            //     if(sys_set_var.hid_lock_sw_type == 0) {
            //         car_lock_control(HID_CAR_CMD_SER, CAR_LOCK_STA);
            //         ble_cmd_mark(CMD_BLE_HID_LOCK);
            //     }
            // }
            break;
        case CMD_BLE_TRANS:
            rt_ringbuffer_put(ble_transbuf, data, len);
            def_rtos_smaphore_release(ble_trans_recv_sem);
            break;
        case CMD_BLE_GET_VER:
            LOG_I("BLE_VER:%s", (char *)data);
            memcpy(&ble_info.ver[0], &data[0], strlen((char *)data));
            if(ble_info.adv_sta == 0) {
                sys_set_ble_adv_start();
            }
            break;
        case CMD_BLE_GET_MAC:
            memcpy(ble_info.mac, data, 6);
            break;
        case CMD_BLE_VIRT_AT:
            // p = (char *)malloc(len+3);
            // sprintf(p, "%s\r\n", (char *)data);
            // LOG_I("[%d]%s", strlen(p), p);
            // app_virt_uart_write(AT_VIRT_BLE, p);
            // free(p);
            break;
        case CMD_BLE_OTA_START:
            LOG_I("BLE_OTA_START");
            def_rtos_smaphore_release(ble_ota_ctrl.con_sem_t);
            break;
        case CMD_BLE_OTA_DATA:
            LOG_I("BLE_OTA_DATA");
            def_rtos_smaphore_release(ble_ota_ctrl.con_sem_t);
            break;
        case CMD_BLE_OTA_END:
            LOG_I("BLE_OTA_END");
            ble_ota_ctrl.ota_res = data[0];
            def_rtos_smaphore_release(ble_ota_ctrl.con_sem_t);
            break;
    }
    if(ble_cmd_table[ble_cmd_send_var.cmd_index] == cmd && ble_cmd_send_var.send_flag == 1) {
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
    def_rtosStaus err = RTOS_SUCEESS;
    err = def_rtos_queue_create(&ble_send_cmd_que, sizeof(uint8_t), 12);
    if(err != RTOS_SUCEESS) {
        LOG_E("ble_send_cmd_que is create fail err:%d", err);
    }
    def_rtos_semaphore_create(&ble_trans_recv_sem, 0);
    ble_transbuf = rt_ringbuffer_create(BLE_TRANSBUF_LEN);
    ble_control_init();
    while(1){
 //       LOG_I("IS RUN");
        len = hal_drv_uart_read(BLE_UART, rcv, 256, RTOS_WAIT_FOREVER);
        if(len == 0) continue;
        if(iot_error_check(IOT_ERROR_TYPE, BLE_ERROR) == 1) {
            iot_error_clean(IOT_ERROR_TYPE, BLE_ERROR);
        }
        debug_data_printf("blerecv",rcv, len);
        ble_heart_time_t = def_rtos_get_system_tick();
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
        if(systm_tick_diff(start_t) > 3000) {
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
    ble_info.init = 0;
    hal_drv_write_gpio_value(O_BLE_POWER, HIGH_L);
    for(i = 0; i < 3; i++) {
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
            hal_drv_write_gpio_value(O_BLE_POWER, LOW_L);
            LOG_E("ble init recv fail count:%d", i);
            def_rtos_task_sleep_ms(8000);
        }
        hal_drv_write_gpio_value(O_BLE_POWER, HIGH_L);
        def_rtos_task_sleep_ms(80);
    }
    if(ble_info.init != 1) {
        iot_error_set(IOT_ERROR_TYPE, BLE_ERROR);
        LOG_E("ble init fail");
        return;
    } 
    sprintf(ble_info.mac_str, "%02X:%02X:%02X:%02X:%02X:%02X", ble_info.mac[0], ble_info.mac[1], ble_info.mac[2], ble_info.mac[3], ble_info.mac[4], ble_info.mac[5]);
} 

int ble_reinit()
{
    static uint8_t step = 0;
    static int64_t ble_time_t = 0;
    if(ble_info.init == 0) step = 0;
    switch(step) {
        case 0:
            ble_info.init = 2;
            LOG_I("ble power off");
            hal_drv_write_gpio_value(O_BLE_POWER, LOW_L);
            ble_time_t = def_rtos_get_system_tick();
            step = 1;
        break;
        case 1:
            if(def_rtos_get_system_tick() - ble_time_t > 5000) {
                LOG_I("ble power on");
                hal_drv_write_gpio_value(O_BLE_POWER, HIGH_L);
                ble_time_t = def_rtos_get_system_tick();
                step = 2;
            } 
        break;
        case 2:
            if(def_rtos_get_system_tick() - ble_time_t > 2000){
                LOG_I("ble is error");
                ble_info.adv_sta = 0;
                sys_set_ble_adv_start();
                step = 0;
                ble_info.init = 1;
                return 0;
            }
        break;
    }
    return 1;
}



void ble_control_init()
{
    if(iot_error_check(IOT_ERROR_TYPE, BLE_ERROR) == 1) {
        iot_error_clean(IOT_ERROR_TYPE, BLE_ERROR);
    }
    memset(&ble_info, 0, sizeof(ble_info));
    ble_power_init();
    ble_param_init();
    ble_cmd_mark(BLE_GET_VER_INDEX);
    LOG_I("ble_control_init is ok");
}

