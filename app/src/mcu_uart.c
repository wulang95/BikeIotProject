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

uint8_t mcu_cmd_table[CMD_INDEX_MAX] = {0X0C, 0X0E, 0X0D, 0x0f, 0x0b, 0x0a, 0x09, 0X08, 0X07, 0X06, 0X05, 0X04, 0x03, 0x10, 0x11, 0x12, 0X13,\
0X14, 0X15, 0X16, 0x17};
struct mcu_cmd_order_stu {
    uint8_t need_ask;
    uint16_t rely_timeout;
    uint8_t max_send_times;
};

struct mcu_cmd_order_stu mcu_cmd_order_table[CMD_INDEX_MAX] = {
    {false,     0,          0},     //CMD_CAN_TRANS
    {true,      1000,       3},     //CMD_GPS_POWERON
    {true,      1000,       3},     //CMD_GPS_POWEROFF
    {false,     0,          0},     //CMD_GPS_DATA
    {false,     0,          0},     //CMD_GPS_TRANS
    {true,      1000,       3},     //CMD_GPS_DEEPSLEEP
    {true,      1000,       3},     //CMD_GPS_HOST_START
    {true,      1000,       3},     //CMD_CAT_REPOWERON
    {false,      0,         0},     //CMD_CRC_ERROR
    {true,      3000,       3},    //CMD_CAN_OTA_DATA
    {true,      1000,       3},     //CMD_CAN_OTA_START
    {true,      1000,       3},     //CMD_CAN_OTA_END
    {true,      1000,       3},    //CMD_CAN_OTA_DATA_FINISH
    {true,      1000,       3},     //CMD_CAN_LOCK_CAR
    {true,      1000,       3},     //CMD_CAN_UNLOCK_CAR
    {false,     0,          0},     //CMD_CAN_CAR_CONTROL
    {true,      1000,       3},     //CMD_SHIP_MODE
    {true,      2000,       3},    //CMD_MCU_OTA_START
    {true,      2000,       3},    //CMD_MCU_OTA_DATA
    {true,      5000,       3},     //CMD_MCU_OTA_END
    {true,      1000,       3},     //CDM_MCU_VER
};


struct mcu_cmd_send_crtl_stu
{
    uint8_t cmd;
    uint8_t send_cent;
    uint8_t ask_flag;
    uint8_t send_flag;
};
struct mcu_cmd_send_crtl_stu mcu_cmd_send_crtl;

enum {
    MCU_OTA_IDEL_STEP = 0,
    MCU_OTA_START_STEP,
    MCU_OTA_DATA_STEP,
    MCU_OTA_END_STEP,
    MCU_OTA_ERROR_STEP,
};

struct mcu_ota_ctrl_stu{
    uint8_t ota_sta;
    uint32_t offset;
    uint32_t total_len;
    uint8_t *data;
    uint8_t data_len;
    uint8_t ota_res;
    uint16_t pack_num;
    def_rtos_sem_t con_sem_t;
};

struct ota_config_stu {
	uint16_t soft_ver;
	uint16_t hw_ver;
	uint32_t total_len;
	uint32_t file_crc32;
};

struct mcu_ota_ctrl_stu mcu_ota_ctrl;
struct ota_config_stu ota_config;
void mcu_ota_start()
{
    unsigned int CRC32 = 0xFFFFFFFF;
    uint32_t read_len, offset = 0, total_len;
    ota_config.hw_ver = 0x10;
    ota_config.soft_ver = 0x10;
    ota_config.total_len = flash_partition_size(DEV_APP_ADR);
    mcu_ota_ctrl.total_len = ota_config.total_len;
    total_len = ota_config.total_len;
    while(total_len > 0) {
        read_len = MIN(total_len, 128);
        flash_partition_read(DEV_APP_ADR, mcu_ota_ctrl.data, read_len, offset);
        CRC32 = GetCrc32_cum(mcu_ota_ctrl.data, read_len, CRC32);
        offset += read_len;
        total_len -= read_len;    
    }
    ota_config.file_crc32 = CRC32;
    LOG_I("hw_ver:%02x,soft_ver:%02x,total_len:%d, CRC:%0x", ota_config.hw_ver, ota_config.soft_ver, ota_config.total_len, ota_config.file_crc32);
    MCU_CMD_MARK(CMD_MCU_OTA_START_INDEX);
    if(def_rtos_semaphore_wait(mcu_ota_ctrl.con_sem_t, 10000) != RTOS_SUCEESS) {
        LOG_E("OTA MCU START IS FAIL");
        mcu_ota_ctrl.ota_sta = MCU_OTA_ERROR_STEP;
        return;
    }
    mcu_ota_ctrl.ota_sta = MCU_OTA_DATA_STEP;
}

void mcu_ota_data()
{
    mcu_ota_ctrl.data_len  = MIN((mcu_ota_ctrl.total_len - mcu_ota_ctrl.offset), 128);
    flash_partition_read(DEV_APP_ADR, &mcu_ota_ctrl.data[0], mcu_ota_ctrl.data_len , mcu_ota_ctrl.offset);
    mcu_ota_ctrl.offset += mcu_ota_ctrl.data_len;
    LOG_I("pack_num:%d, offset:%d", mcu_ota_ctrl.pack_num, mcu_ota_ctrl.offset);
    MCU_CMD_MARK(CMD_MCU_OTA_DATA_INDEX);
    if(def_rtos_semaphore_wait(mcu_ota_ctrl.con_sem_t, 10000) != RTOS_SUCEESS) {
        LOG_E("OTA MCU DATA IS FAIL");
        mcu_ota_ctrl.ota_sta = MCU_OTA_ERROR_STEP;
        return;
    }
    mcu_ota_ctrl.pack_num++;
    if(mcu_ota_ctrl.offset%256 == 0) {
        LOG_I("mcu ota progress %%%d", ((mcu_ota_ctrl.offset*100)/mcu_ota_ctrl.total_len));
    } 
    if(mcu_ota_ctrl.total_len  == mcu_ota_ctrl.offset) {
        mcu_ota_ctrl.ota_sta = MCU_OTA_END_STEP;
    }
}

void mcu_ota_end()
{
    MCU_CMD_MARK(CMD_MCU_OTA_END_INDEX);
    if(def_rtos_semaphore_wait(mcu_ota_ctrl.con_sem_t, 20000) != RTOS_SUCEESS) {
        LOG_E("OTA MCU END IS FAIL");
        mcu_ota_ctrl.ota_sta = MCU_OTA_ERROR_STEP;
        return;
    }
    free(mcu_ota_ctrl.data);
    def_rtos_semaphore_delete(mcu_ota_ctrl.con_sem_t);
}
int mcu_ota_task()
{
    if(mcu_ota_ctrl.ota_sta != MCU_OTA_IDEL_STEP) return FAIL;
    mcu_ota_ctrl.data = malloc(128);
    if(mcu_ota_ctrl.data == NULL) return FAIL;
    if(def_rtos_semaphore_create(&mcu_ota_ctrl.con_sem_t, 0) != RTOS_SUCEESS){
        LOG_E("mcu_ota_ctrl.con_sem_t is create fail");
        return FAIL;
    }
    MCU_CMD_MARK(CMD_GPS_POWEROFF_INDEX);
    car_control_cmd(CAR_CMD_LOCK);
    mcu_ota_ctrl.ota_sta = MCU_OTA_START_STEP;
    mcu_ota_ctrl.offset = 0;
    mcu_ota_ctrl.ota_res = 0xff;
    mcu_ota_ctrl.data_len = 0;
    mcu_ota_ctrl.pack_num = 0;
    while(1){
        switch(mcu_ota_ctrl.ota_sta) {
        case MCU_OTA_START_STEP:
            mcu_ota_start();
            break;
        case MCU_OTA_DATA_STEP:
            mcu_ota_data();
        break;
        case MCU_OTA_END_STEP:
            mcu_ota_end();
            if(mcu_ota_ctrl.ota_res == 0) return OK;
            else return FAIL;
        break;
        case MCU_OTA_ERROR_STEP:
            free(mcu_ota_ctrl.data);
            def_rtos_semaphore_delete(mcu_ota_ctrl.con_sem_t);
            return FAIL;
        break;
        }
    }
    return OK;
} 

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
    if(cmd == CMD_MCU_OTA_DATA){
        p[len++] = (mcu_ota_ctrl.data_len + 2) >> 8;
        p[len++] = (mcu_ota_ctrl.data_len + 2)&0xff;
        memcpy(&p[len], &mcu_ota_ctrl.pack_num, 2);
        len += 2;
        memcpy(&p[len], &mcu_ota_ctrl.data[0], mcu_ota_ctrl.data_len);
        len += mcu_ota_ctrl.data_len;
    }
    else if(cmd == CMD_MCU_OTA_START) {
        p[len++] = sizeof(struct ota_config_stu) >> 8;
        p[len++] = sizeof(struct ota_config_stu)&0xff;
        memcpy(&p[len], &ota_config, sizeof(struct ota_config_stu));
        len += sizeof(struct ota_config_stu);
    } else if(cmd == CMD_CAN_OTA_DATA) {
        p[len++] = (can_ota_data_uart.data_len + 2)>>8;
        p[len++] = (can_ota_data_uart.data_len + 2)&0xff;
        p[len++] = can_ota_data_uart.dev_id;
        p[len++] = can_ota_data_uart.pack_num;
        memcpy(&p[len], &can_ota_data_uart.data[0], can_ota_data_uart.data_len);
        len += can_ota_data_uart.data_len;
    } else {
        p[len++] = data_len>>8;
        p[len++] = data_len&0xff;
        if(data != NULL) {
            memcpy(&p[len], &data[0], data_len);
            len += data_len;
        }
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
    CAN_PDU_STU can_pdu;
    can_pdu.can_id = can_txframe.ExtID;
    if(can_pdu.pdu.pdu1 == 0x14 || can_pdu.pdu.pdu1 == 0x60) {
        mcu_data_pack(CMD_CAN_CAR_CONTROL, (uint8_t *)&can_txframe, sizeof(stc_can_rxframe_t), buf, &len);
    } else {
        mcu_data_pack(CMD_CAN_TRANS, (uint8_t *)&can_txframe, sizeof(stc_can_rxframe_t), buf, &len);
    }
    
    mcu_uart_send(buf, len);
}


void mcu_recv_cmd_handler(uint8_t cmd, uint8_t *data, uint16_t data_len)
{
    stc_can_rxframe_t can_frame;
    LOG_I("mcu_cmd_send_crtl.cmd:%02x,cmd:%02x", mcu_cmd_send_crtl.cmd,cmd);
    switch (cmd)
    {
    case CMD_CAN_TRANS:
        memcpy(&can_frame, data, data_len);
        LOG_I("can_id:%08x,", can_frame.ExtID);
        def_rtos_queue_release(can_rcv_que, sizeof(stc_can_rxframe_t), (uint8_t *)&can_frame, RTOS_WAIT_FOREVER);
        break;
    case CMD_GPS_POWERON:
        Gps.GpsPower = GPS_POWER_ON;
    break;
    case CMD_GPS_POWEROFF:
        Gps.GpsPower = GPS_POWER_OFF;
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
    case CMD_CRC_ERROR:
        LOG_E("CMD_CRC_ERROR");
    break;
    case CMD_CAN_OTA_DATA:
        LOG_I("CMD_CAN_OTA_DATA");
        def_rtos_smaphore_release(can_ota_data_uart.data_sem);
    break;
    case CMD_CAN_OTA_DATA_FINISH:
        if(data[0] == 1)  def_rtos_smaphore_release(can_ota_data_uart.data_finish_sem);
    break;
    case CMD_CAN_OTA_START:
    break;
    case CMD_CAN_OTA_END:
    break;
    case CMD_CAN_LOCK_CAR:
    break;
    case CMD_CAN_UNLOCK_CAR:
    break;
    case CMD_CAN_CAR_CONTROL:
        can_send_cmd.ask_flag = 1;
        LOG_I("CMD_CAN_CAR_CONTROL");
    break;
    case CMD_SHIP_MODE:
    break;
    case CMD_MCU_OTA_START:
        LOG_I("CMD_MCU_OTA_START");
        def_rtos_smaphore_release(mcu_ota_ctrl.con_sem_t);
    break;
    case CMD_MCU_OTA_DATA:
        LOG_I("CMD_MCU_OTA_DATA");
        def_rtos_smaphore_release(mcu_ota_ctrl.con_sem_t);
    break;
    case CMD_MCU_OTA_END:
        LOG_I("CMD_MCU_OTA_END");
        mcu_ota_ctrl.ota_res = data[0];
        def_rtos_smaphore_release(mcu_ota_ctrl.con_sem_t);
    break;
    case CDM_MCU_VER:
        sys_info.mcu_soft_ver = data[0] << 8 | data[1];
        sys_info.mcu_hw_ver = data[2] << 8 | data[3];
        LOG_I("mcu_soft_ver:%0x, mcu_hw_ver:%0x", sys_info.mcu_soft_ver, sys_info.mcu_hw_ver);
    break;
    default:
        break;
    }
    if(mcu_cmd_send_crtl.send_flag && mcu_cmd_send_crtl.cmd == cmd) {
        mcu_cmd_send_crtl.ask_flag = 1;
        LOG_I("RECV ASK OK");
    }
}

void MCU_CMD_MARK(uint8_t cmd)
{
    LOG_I("CMD:%d", cmd);
    if(cmd == CMD_CAN_TRANS_INDEX || cmd == CMD_GPS_DATA_INDEX || cmd == CMD_GPS_TRANS_INDEX || cmd == CMD_CAN_CAR_CONTROL_INDEX) return;
    def_rtos_queue_release(mcu_cmd_que, sizeof(uint8_t), &cmd, RTOS_WAIT_FOREVER);
}

void mcu_send_fail(uint8_t cmd)
{
    LOG_E("mcu_send_fail, cmd:%02x", cmd);
}

void mcu_uart_send_thread(void *param)
{
    uint8_t cmd_index;
    def_rtosStaus res;
    int64_t time_t;
    uint8_t buf[256];
    uint16_t lenth;
    while(1) {
   //     LOG_I("IS RUN");
        res = def_rtos_queue_wait(mcu_cmd_que, &cmd_index, sizeof(uint8_t), RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) continue;
        mcu_cmd_send_crtl.cmd = mcu_cmd_table[cmd_index];
        LOG_I("CMD_INDEX:%d, cmd:%02x", cmd_index, mcu_cmd_send_crtl.cmd);
        mcu_cmd_send_crtl.send_flag = 1;
        mcu_cmd_send_crtl.send_cent = 0;
        mcu_cmd_send_crtl.ask_flag = 0;
        mcu_data_pack(mcu_cmd_send_crtl.cmd, NULL, 0, buf, &lenth);
        mcu_uart_send(buf, lenth);
        mcu_cmd_send_crtl.send_cent++;
        time_t = def_rtos_get_system_tick();
        for(;;) {
            if(mcu_cmd_order_table[cmd_index].need_ask) {
                if(mcu_cmd_send_crtl.ask_flag == 1) {
                    break;
                }
                if(def_rtos_get_system_tick() - time_t >= mcu_cmd_order_table[cmd_index].rely_timeout) {
                    if(mcu_cmd_send_crtl.send_cent >= mcu_cmd_order_table[cmd_index].max_send_times) {
                        mcu_send_fail(mcu_cmd_send_crtl.cmd);
                        break;
                    }
                    mcu_uart_send(buf, lenth);
                    mcu_cmd_send_crtl.send_cent++;
                    time_t = def_rtos_get_system_tick();
                }
                def_rtos_task_sleep_ms(5);
            } else break;
        }
        LOG_I("cmd_index:%d, ask_flag:%d, send_cent:%d, need_ask:%d, max_send_times:%d", cmd_index, mcu_cmd_send_crtl.ask_flag, mcu_cmd_send_crtl.send_cent, mcu_cmd_order_table[cmd_index].need_ask, mcu_cmd_order_table[cmd_index].max_send_times);
        mcu_cmd_send_crtl.send_flag = 0;
        mcu_cmd_send_crtl.cmd = 0;
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
    //    LOG_I("IS RUN");
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
    MCU_CMD_MARK(CMD_CAN_OTA_END_INDEX);
    MCU_CMD_MARK(CDM_MCU_VER_INDEX);
    GPS_Init();
    LOG_I("mcu_uart_init is ok");
}