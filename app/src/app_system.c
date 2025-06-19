#include "app_system.h"
#include "ringbuffer.h"
#include "hal_drv_flash.h"
#include "hal_drv_uart.h"
#include "hal_virt_at.h"
#include "ql_fs.h"
#include "hal_drv_net.h"
#include "hal_drv_iic.h"
#include <stdarg.h>
#define DBG_TAG         "app_system"

#ifdef APP_SYS_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif
#include    "log_port.h"

struct sys_info_stu sys_info;
struct sys_config_stu sys_config;
struct sys_config_stu sys_config_back;
struct sys_param_set_stu sys_param_set;
struct sys_set_var_stu sys_set_var;
struct sensor_calibration_data_stu sensor_calibration_data;
SHAPE_SET sheepfang_data;
SHAPE_SET forbidden_zone_data;
NET_NW_INFO nw_info;
int ota_fd;
uint16_t shock_cent;
uint64_t shock_time_out;

void assert_handler(const char *ex_string, const char *func, size_t line)
{
    LOG_E("(%s) assertion failed at function:%s, line number:%d \n", ex_string, func, line);
    while(1);
}


void sys_power_off_time_set(uint8_t time)
{
    switch(time) {
            case 0x01:
                rtc_event_register(CAR_AUTO_POWER_OFF, 5*60, 0); 
            break;
            case 0x02:
                rtc_event_register(CAR_AUTO_POWER_OFF, 10*60, 0); 
            break;
            case 0x03:
                rtc_event_register(CAR_AUTO_POWER_OFF, 15*60, 0); 
            break;
            case 0x04:
                rtc_event_register(CAR_AUTO_POWER_OFF, 20*60, 0); 
            break;
            case 0x05:
                rtc_event_register(CAR_AUTO_POWER_OFF, 30*60, 0); 
            break;
            case 0x06:
                rtc_event_register(CAR_AUTO_POWER_OFF, 60*60, 0); 
            break;
            case 0x07:
                rtc_event_register(CAR_AUTO_POWER_OFF, 90*60, 0); 
            break;
            case 0xFF:
                rtc_event_unregister(CAR_AUTO_POWER_OFF);
            break;
    }
}

int flash_partition_erase(FLASH_PARTITION flash_part)
{
    int err;
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd > 0) ql_fclose(ota_fd);
            ql_remove(OTA_FILE);
        //    hal_drv_flash_erase(DEV_APP_ADDR, DEV_APP_SIZE);
            ota_fd = ql_fopen(OTA_FILE, "wb+");
            if(ota_fd < 0) return FAIL;
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_erase(SYS_CONFIG_ADDR, SYS_CONFIG_SIZE);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_erase(BACK_SYS_CONFIG_ADDR, BACK_SYS_CONFIG_SIZE);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_erase(SYS_SET_ADDR, SYS_SET_SIZE);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_erase(SENSOR_CALIBRATION_DATA_ADDR, SENSOR_CALIBRATION_DATA_SIZE);
            break;
        case SHEEP_FOLD_P_ADR:
            err = hal_drv_flash_erase(SHEEP_FOLD_POINT_ADDR, SHEEP_FOLD_POINT_SIZE);
            LOG_I("err:%d", err);
            break;
        case FORBIDDEN_ZONE_P_ADR:
            err = hal_drv_flash_erase(FORBIDDEN_ZONE_POINT_ADDR, FORBIDDEN_ZONE_SIZE);
            LOG_I("err:%d", err);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_write(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    int res;
    switch(flash_part) {
        case DEV_APP_ADR:
      //      hal_drv_flash_write(DEV_APP_ADDR + shift, data, lenth);
            if(ota_fd < 0) {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
            } 
            res = ql_fseek(ota_fd, shift, 0);
            if(res < 0) {
                LOG_E("ql_fseek is fail");
                return FAIL;
            }
            res = ql_fwrite(data, lenth, 1, ota_fd);
            if(res < 0) {
                LOG_E("ql_fwrite is fail!");
                return FAIL;
            }
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_write(SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_write(BACK_SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_write(SYS_SET_ADDR + shift, data, lenth);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_write(SENSOR_CALIBRATION_DATA_ADDR + shift, data, lenth);
            break;
        case SHEEP_FOLD_P_ADR:  
            res = hal_drv_flash_write(SHEEP_FOLD_POINT_ADDR + shift, data, lenth);
            LOG_I("res:%d", res);
            break;
        case FORBIDDEN_ZONE_P_ADR:
            res = hal_drv_flash_write(FORBIDDEN_ZONE_POINT_ADDR + shift, data, lenth);
            LOG_I("res:%d", res);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_read(FLASH_PARTITION flash_part, void *data, size_t lenth, int32_t shift)
{
    int res;
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd  < 0) {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
            } 
            res = ql_fseek(ota_fd, shift, 0);
            if(res < 0) {
                LOG_E("ql_fseek is fail");
                return FAIL;
            }
            res = ql_fread(data, lenth, 1, ota_fd);
            if(res < 0) {
                LOG_E("ql_fread is fail");
                return FAIL;
            }
//            hal_drv_flash_read(DEV_APP_ADDR + shift, data, lenth);
            break;
        case SYS_CONFIG_ADR:
            hal_drv_flash_read(SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case BACK_SYS_CONFIG_ADR:
            hal_drv_flash_read(BACK_SYS_CONFIG_ADDR + shift, data, lenth);
            break;
        case SYS_SET_ADR:
            hal_drv_flash_read(SYS_SET_ADDR + shift, data, lenth);
            break;
        case SENSEOR_CALIBRATION_ADR:
            hal_drv_flash_read(SENSOR_CALIBRATION_DATA_ADDR + shift, data, lenth);
            break;
        case SHEEP_FOLD_P_ADR:
            hal_drv_flash_read(SHEEP_FOLD_POINT_ADDR + shift, data, lenth);
            break;
        case FORBIDDEN_ZONE_P_ADR:
            hal_drv_flash_read(FORBIDDEN_ZONE_POINT_ADDR + shift, data, lenth);
            break;
        default:
            break;
    }
    return OK;
}

int flash_partition_size(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case DEV_APP_ADR:
            if(ota_fd > 0) {
                return ql_fsize(ota_fd);
            } else {
                ota_fd = ql_fopen(OTA_FILE, "rb+");
                if(ota_fd < 0) {
                    LOG_E("%s open fail", OTA_FILE);
                    return  FAIL;
                }
                return ql_fsize(ota_fd);
            }  
        break; 
        default:
        break;
    }
    return OK;
}

void flash_ota_close()
{
    ql_fclose(ota_fd);
    ota_fd = -1;
}

void sys_param_save(FLASH_PARTITION flash_part)
{
    switch(flash_part) {
        case SYS_CONFIG_ADR:
            LOG_I("SYS_CONFIG_ADR");
            sys_config.magic = IOT_MAGIC;
            sys_config.crc32 = GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4);
            flash_partition_erase(SYS_CONFIG_ADR);
            flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
        break;
        case BACK_SYS_CONFIG_ADR:
            LOG_I("BACK_SYS_CONFIG_ADR");
            sys_config_back = sys_config;
            flash_partition_erase(BACK_SYS_CONFIG_ADR);
            flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        break;
        case SYS_SET_ADR:
            LOG_I("SYS_SET_ADR");
            sys_param_set.magic = IOT_MAGIC;
            sys_param_set.crc32 = GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4);
            flash_partition_erase(SYS_SET_ADR);
            flash_partition_write(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
            flash_partition_read(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
            LOG_I("total_fence_sw:%d", sys_param_set.total_fence_sw);
        break;
        case SENSEOR_CALIBRATION_ADR:
            LOG_I("SENSEOR_CALIBRATION_ADR");
            sensor_calibration_data.magic = IOT_MAGIC;
            sensor_calibration_data.crc32 = GetCrc32((uint8_t *)&sensor_calibration_data, sizeof(sensor_calibration_data) - 4);	
            flash_partition_erase(SENSEOR_CALIBRATION_ADR);
            flash_partition_write(SENSEOR_CALIBRATION_ADR, (void *)&sensor_calibration_data, sizeof(sensor_calibration_data) , 0);
        break;
        case SHEEP_FOLD_P_ADR:
            LOG_I("SHEEP_FOLD_P_ADR");
            sheepfang_data.magic = IOT_MAGIC;
            sheepfang_data.crc32 = GetCrc32((uint8_t *)&sheepfang_data, sizeof(sheepfang_data) - 4);
            flash_partition_erase(SHEEP_FOLD_P_ADR);
            flash_partition_write(SHEEP_FOLD_P_ADR, (void *)&sheepfang_data, sizeof(sheepfang_data) , 0);
        break;
        case FORBIDDEN_ZONE_P_ADR:
            LOG_I("FORBIDDEN_ZONE_P_ADR");
            sheepfang_data.magic = IOT_MAGIC;
            forbidden_zone_data.crc32 = GetCrc32((uint8_t *)&forbidden_zone_data, sizeof(forbidden_zone_data) - 4);
            flash_partition_erase(FORBIDDEN_ZONE_P_ADR);
            flash_partition_write(FORBIDDEN_ZONE_P_ADR, (void *)&forbidden_zone_data, sizeof(forbidden_zone_data) , 0);
        break;
        default:
        break;
    }
}

void debug_data_printf(char *str_tag, const uint8_t *in_data, uint16_t data_len)
{
    uint16_t i, len;
    char data_str[4];
    char *str;
    if(data_len == 0 || in_data == NULL || str_tag == NULL || (strlen(str_tag) > 20)) return;
    str = malloc(1024);
    if(str == NULL){
        return;
    }
    len = data_len > 300?300:data_len;
    sprintf(str, "%s[%d]:", str_tag, len);
    for(i = 0; i < len; i++){
        sprintf(data_str, "%02x ", in_data[i]);
        strncat(str, data_str, strlen(data_str));
    }
    LOG_I("%s", str);
    free(str);
}
void system_enter_ship_mode(CAR_CMD_Q car_cmd_q)
{
    uint8_t buf[256];
    uint16_t buf_len;
    LOG_I("enter ship mode");
    /*开锁状态不进入船运模式*/
    if(car_info.lock_sta == CAR_UNLOCK_STA) {
        if(car_cmd_q.src == NET_CAR_CMD_SER){
            buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x02, 0);
            net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
        } else if (car_cmd_q.src == BLUE_CAR_CMD_SER){
            ble_cmd_ship_mode_ask(0x02);
        }
        return;
    }
    hal_drv_write_gpio_value(O_BLE_POWER, LOW_L); //ble power off
    hal_drv_write_gpio_value(SENSOR_POWER, LOW_L); //sensor power off
    if(car_cmd_q.src == NET_CAR_CMD_SER){
        buf_len = net_engwe_cmdId_operate_respos(buf, car_cmd_q.net_car_control, 0x01, 0);
        net_engwe_pack_seq_up(OPERATION_FEEDBACK_UP, buf, buf_len, car_cmd_q.net_car_control.seq); 
    } else if (car_cmd_q.src == BLUE_CAR_CMD_SER){
        ble_cmd_ship_mode_ask(0x01);
    }
    MCU_CMD_MARK(CMD_SHIP_MODE_INDEX); 
}

int64_t systm_tick_diff(int64_t time)
{
    int64_t sec;
    const int64_t cur_t = def_rtos_get_system_tick();
    sec = cur_t - time;
    return sec;
}



void sensor_input_handler()
{
    LOG_I("algo is week");
    if(shock_cent == 0){
        shock_time_out = def_rtos_get_system_tick();
    }
    shock_cent++;
    if(shock_cent >= sys_param_set.shock_sensitivity) {
        shock_cent = 0;
        car_info.move_alarm = 1;
        week_time("sensor", 30);
    } 
    if(def_rtos_get_system_tick() - shock_time_out > 500){
        shock_cent = 0;
    }
}


void ble_connect_handler()
{
    LOG_I("ble connect");
    week_time("ble", -1);
    sys_info.ble_connect = 1;
}

void power_36v_handler()
{
    LOG_I("36V power on");
//    sys_info.power_36v = 1;
    week_time("sys", 30); 
}
static void hal_drv_init()
{
    hal_drv_set_gpio_irq(I_SENSOR_IN, RISING_EDGE, DOWN_MODE, sensor_input_handler);
    hal_drv_gpio_init(O_RED_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_WHITE_IND, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
  //  hal_drv_gpio_init(O_BAT_CHARGE_CON, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_MCU_CONEC, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);
    hal_drv_gpio_init(SENSOR_POWER, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);
    hal_drv_gpio_init(O_AUDIO_SD, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);
    hal_drv_set_gpio_irq(I_BLE_CON_SIG, RISING_EDGE, DOWN_MODE, ble_connect_handler);
 //   hal_drv_gpio_init(I_BLE_CON_SIG, IO_INPUT, DOWN_MODE, LOW_L);

    hal_drv_gpio_init(O_KEY_HIGH, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_KEY_LOW, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(O_BLE_WEEK_SIG, IO_OUTPUT, PULL_NONE_MODE, LOW_L);
    hal_drv_gpio_init(I_MCU_WEEK, IO_INPUT, DOWN_MODE, L_NONE);
    hal_drv_gpio_init(I_36VPOWER_DET, IO_INPUT, UP_MODE, L_NONE);

    hal_drv_set_gpio_irq(I_36VPOWER_DET, FALL_EDGE, UP_MODE, power_36v_handler);
    hal_drv_gpio_init(O_BLE_POWER, IO_OUTPUT, PULL_NONE_MODE, HIGH_L);

    hal_drv_uart_init(BLE_UART, BLE_BAUD, BLE_PARITY);
    hal_drv_uart_init(MCU_UART, MCU_BAUD, MCU_PARITY);
    hal_drv_write_gpio_value(O_MCU_CONEC, HIGH_L);
    hal_virt_at_init();
 //   hal_drv_write_gpio_value(O_BAT_CHARGE_CON, HIGH_L);
    hal_drv_write_gpio_value(O_BLE_WEEK_SIG, LOW_L);
    hal_drv_write_gpio_value(SENSOR_POWER, HIGH_L);
    hal_drv_write_gpio_value(O_AUDIO_SD, HIGH_L);

    LOG_I("hal_drv_init is ok");
}



static void sys_param_set_default_init()
{
    memset(&sys_param_set, 0, sizeof(sys_param_set));
    sys_param_set.magic = IOT_MAGIC;
    sys_param_set.ota_cnt = 0;
    SETBIT(sys_param_set.net_heart_sw, LOCK_HEART_SW);
    SETBIT(sys_param_set.net_heart_sw, UNLOCK_HEART_SW);
    sys_param_set.alive_flag = 1;
    sys_param_set.net_engwe_offline_opearte_push_cmdId = OFFLINE_OPERATE_PUSH_DEFAULT;
    sys_param_set.net_engwe_state_push_cmdId = STATE_PUSH_DEFAULT;
    sys_param_set.unlock_car_heart_interval = 10;
    sys_param_set.net_heart_interval = 300;
    sys_param_set.lock_car_heart_interval = 900;
    sys_param_set.internal_battry_work_interval = 3600;
    sys_param_set.lock_car_heart2_interval = 100;
    sys_param_set.unlock_car_heart2_interval = 10;
    sys_param_set.net_engwe_report_time1_cmdId = 7;
    sys_param_set.net_engwe_report_time2_cmdId = 7;
    sys_param_set.shock_sensitivity = 3;
    sys_param_set.bms_charge_current = 8;
    sys_param_set.shock_sw = 1;
    sys_param_set.hid_lock_sw = 1;
    sys_param_set.navigation_quit_time = 15;
    sys_param_set.crc32 = GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4);
    flash_partition_erase(SYS_SET_ADR);
    flash_partition_write(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
}

static void sys_param_set_init()
{
    flash_partition_read(SYS_SET_ADR, (void *)&sys_param_set, sizeof(sys_param_set), 0);
    if(sys_param_set.magic != IOT_MAGIC || sys_param_set.crc32 != GetCrc32((uint8_t *)&sys_param_set, sizeof(sys_param_set) - 4)) {
        sys_param_set_default_init();
        LOG_E("sys_set_param save is fail!");
    }
    LOG_I("ota_cnt:%d", sys_param_set.ota_cnt);
}

static void sheepfang_data_init()
{
    uint8_t i = 0;
    flash_partition_read(SHEEP_FOLD_P_ADR, (void *)&sheepfang_data, sizeof(sheepfang_data), 0);
    if((sheepfang_data.magic == IOT_MAGIC) && (sheepfang_data.crc32 == GetCrc32((uint8_t *)&sheepfang_data, sizeof(sheepfang_data) - 4))){
        if(sheepfang_data.shape_type == CIRCLE){
            LOG_I("CIRCLE");
            LOG_I("center:%f, %f, radius:%f", sheepfang_data.circle.center.lat, sheepfang_data.circle.center.lon, sheepfang_data.circle.radius);
        } else if(sheepfang_data.shape_type == POLYGON) {
            LOG_I("POLYGON");
            for(i = 0; i < sheepfang_data.polygon.point_num; i++) {
                LOG_I("P[%d]:%f, %f",i, sheepfang_data.polygon.p[i].lat, sheepfang_data.polygon.p[i].lon);
            }
        }
    } else {
        LOG_E("SHEEP_FOLD_P_ADR is invalid");
        sheepfang_data.magic = IOT_MAGIC;
        sheepfang_data.shape_type = FENCE_NONE;
        sheepfang_data.crc32 = GetCrc32((uint8_t *)&sheepfang_data, sizeof(sheepfang_data) - 4);
        flash_partition_erase(SHEEP_FOLD_P_ADR);
        flash_partition_write(SHEEP_FOLD_P_ADR, (void *)&sheepfang_data, sizeof(sheepfang_data), 0);
    }
}

static void forbidden_zone_data_init()
{
    uint8_t i = 0;
    flash_partition_read(FORBIDDEN_ZONE_P_ADR, (void *)&forbidden_zone_data, sizeof(forbidden_zone_data), 0);
    if((forbidden_zone_data.magic == IOT_MAGIC) && (forbidden_zone_data.crc32 == GetCrc32((uint8_t *)&forbidden_zone_data, sizeof(forbidden_zone_data) - 4))){
        if(forbidden_zone_data.shape_type == CIRCLE){
            LOG_I("CIRCLE");
            LOG_I("center:%f, %f, radius:%f", forbidden_zone_data.circle.center.lat, forbidden_zone_data.circle.center.lon, forbidden_zone_data.circle.radius);
        } else if(forbidden_zone_data.shape_type == POLYGON) {
            LOG_I("POLYGON");
            for(i = 0; i < forbidden_zone_data.polygon.point_num; i++) {
                LOG_I("P[%d]:%f, %f",i, forbidden_zone_data.polygon.p[i].lat, forbidden_zone_data.polygon.p[i].lon);
            }
        }
    } else {
        LOG_E("FORBIDDEN_ZONE_P_ADR is invalid");
        memset(&forbidden_zone_data, 0, sizeof(forbidden_zone_data));
        forbidden_zone_data.magic = IOT_MAGIC;
        forbidden_zone_data.shape_type = FENCE_NONE;
        forbidden_zone_data.crc32 = GetCrc32((uint8_t *)&forbidden_zone_data, sizeof(forbidden_zone_data) - 4);
        flash_partition_erase(FORBIDDEN_ZONE_P_ADR);
        flash_partition_write(FORBIDDEN_ZONE_P_ADR, (void *)&forbidden_zone_data, sizeof(forbidden_zone_data), 0);
        flash_partition_read(FORBIDDEN_ZONE_P_ADR, (void *)&forbidden_zone_data, sizeof(forbidden_zone_data), 0);
    }
}

static void sys_config_default_init()
{
    memset(&sys_config, 0, sizeof(sys_config));
    memcpy(&sys_config.manufacturer[0], DEFAULT_MANUFACTURER, strlen(DEFAULT_MANUFACTURER));
    memcpy(&sys_config.sn[0], DEFAULT_SN, strlen(DEFAULT_SN));
    memcpy(&sys_config.dev_type, DEFAULT_DEV_TYPE, 6);
    memcpy(&sys_config.apn, DEFAULT_APN, strlen(DEFAULT_APN));
    memcpy(&sys_config.ip, DEFAULT_IP, strlen(DEFAULT_IP));
    memcpy(&sys_config.DSN, DEFAULT_DNS, strlen(DEFAULT_DNS));
    sys_config.port = DEFAULT_PORT;
    sys_config.magic = IOT_MAGIC;
    sys_config.mqtt_qos = 1;
    sys_config.mqtt_will_en = 1;
    memcpy(sys_config.mqtt_will_topic, DEFAULT_MQTT_WILL_TOPIC, strlen(DEFAULT_MQTT_WILL_TOPIC));
    memcpy(sys_config.soft_ver, SOFTVER, strlen(SOFTVER));
    memcpy(sys_config.hw_ver, HWVER, strlen(HWVER));
    memcpy(sys_config.mqtt_client_user, DEFAULT_MQTT_USER, strlen(DEFAULT_MQTT_USER));
    memcpy(sys_config.mqtt_client_pass, DEFAULT_MQTT_PWD, strlen(DEFAULT_MQTT_PWD));
    memcpy(sys_config.mqtt_pub_topic, DEFAULT_MQTT_PUB_PRE, strlen(DEFAULT_MQTT_PUB_PRE));
    memcpy(sys_config.mqtt_sub_topic, DEFAULT_MQTT_SUB_PRE, strlen(DEFAULT_MQTT_SUB_PRE));
    sys_config.crc32 = GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4);
    flash_partition_erase(SYS_CONFIG_ADR);
    flash_partition_erase(BACK_SYS_CONFIG_ADR);
    flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
    flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
}

void sys_config_init()
{
    flash_partition_read(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
    if(sys_config.magic != IOT_MAGIC || sys_config.crc32 != GetCrc32((uint8_t *)&sys_config, sizeof(sys_config) - 4)) {
        flash_partition_read(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        if(sys_config_back.magic != IOT_MAGIC || sys_config_back.crc32 != GetCrc32((uint8_t *)&sys_config_back, sizeof(sys_config_back) - 4)) {
            LOG_E("sys_config save is fail!");
            sys_config_default_init();
        } else {
            sys_config = sys_config_back;
            flash_partition_erase(SYS_CONFIG_ADR);
            flash_partition_write(SYS_CONFIG_ADR, (void *)&sys_config, sizeof(sys_config), 0);
        }
    } else {
        flash_partition_read(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        if(sys_config_back.magic != IOT_MAGIC || sys_config_back.crc32 != GetCrc32((uint8_t *)&sys_config_back, sizeof(sys_config_back) - 4)) {
            sys_config_back = sys_config;
            flash_partition_erase(BACK_SYS_CONFIG_ADR);
            flash_partition_write(BACK_SYS_CONFIG_ADR, (void *)&sys_config_back, sizeof(sys_config_back), 0);
        }
    }
    LOG_I("APN:%s, DSN:%s, IP:%s, PORT:%d", sys_config.apn, sys_config.DSN, sys_config.ip, sys_config.port);
}

def_rtos_task_t app_system_task = NULL;
def_rtos_timer_t system_timer;
def_rtos_sem_t system_task_sem;


/*心跳设置和状态改变时调用*/
void regular_heart_update()
{
    if(sys_info.power_36v) {
        if(car_info.lock_sta == CAR_UNLOCK_STA) {
            if(CHECKBIT(sys_param_set.net_heart_sw, UNLOCK_HEART_SW)) {
                rtc_event_register(NET_REPORT_EVENT, sys_param_set.unlock_car_heart_interval, 1); 
            } else {
                rtc_event_unregister(NET_REPORT_EVENT);
            }
            if(CHECKBIT(sys_param_set.net_heart_sw, UNLOCK_HEART2_SW)) {
                rtc_event_register(NET_REPORT2_EVENT, sys_param_set.unlock_car_heart2_interval, 1);
            } else {
                rtc_event_unregister(NET_REPORT2_EVENT);
            }
        } else {
            if(CHECKBIT(sys_param_set.net_heart_sw, LOCK_HEART_SW)){
                rtc_event_register(NET_REPORT_EVENT, sys_param_set.lock_car_heart_interval, 1); 
            } else {
                rtc_event_unregister(NET_REPORT_EVENT);
            }
            if(CHECKBIT(sys_param_set.net_heart_sw, LOCK_HEART2_SW)){
                rtc_event_register(NET_REPORT_EVENT, sys_param_set.lock_car_heart2_interval, 1); 
            } else {
                rtc_event_unregister(NET_REPORT2_EVENT);
            }
        }
    } else {
        rtc_event_unregister(NET_REPORT2_EVENT);
        if(CHECKBIT(sys_param_set.net_heart_sw, INTERNAL_BAT_HEART_SW)){
            rtc_event_register(NET_REPORT_EVENT, sys_param_set.internal_battry_work_interval, 1);
        } else {
            rtc_event_unregister(NET_REPORT_EVENT);
        }
    }
}

int sys_mode_reinit(uint8_t mode)
{
    int res;
    switch(mode){
        case GPS_MODEL:
            res = GPS_reinit();
            if(res == 2 || res == 0) {
                return 0;
            }
        break;
        case BLE_MODEL:
            res = ble_reinit(); 
            if(res == 0) {
                return 0;
            }
        break;
        case SENSOR_MODEL:
            res = app_sensor_reinit();
            if(res == 0) {
                return 0;
            }
        break;
    }
    return 1;
}



 
void app_system_thread(void *param)
{
    def_rtosStaus res;
    int64_t csq_time_t = def_rtos_get_system_tick();
    int64_t ble_info_up_time_t = 0;
    int64_t shock_time_t = 0;
    int64_t trace_time_t = 0;
    uint8_t TEMP = 0;
    ble_heart_time_t = def_rtos_get_system_tick();
    if(sys_param_set.alive_flag) {  //已激活
        sys_info.iot_mode = IOT_ACTIVE_MODE;
    } else {    //未激活
        app_set_led_ind(LED_WAIT_ALIVE);
        if(sys_info.power_36v == 0) {
            if(sys_info.ble_connect == 0) {
                week_time("ble", 300);  //5分钟后休眠
            }
        }
    }
    while (1)
    {
  //      LOG_I("IS RUN");
        res = def_rtos_semaphore_wait(system_task_sem, RTOS_WAIT_FOREVER);
        if(res != RTOS_SUCEESS) {
            continue;
        }
        if(Gps.GpsPower == GPS_POWER_ON) {
            if(def_rtos_get_system_tick() - gps_resh_time_t > 15*1000 && (iot_error_check(IOT_ERROR_TYPE, GPS_ERROR) == 0)) {
                iot_error_set(IOT_ERROR_TYPE, GPS_ERROR);      
                Gps.init = 0;  //触发错误处理
                SETBIT(sys_info.mode_reinit_flag, GPS_MODEL);
            }   
        }
        app_bat_charge_check();
        if(def_rtos_get_system_tick() - ble_heart_time_t > 20*1000 && (iot_error_check(IOT_ERROR_TYPE, BLE_ERROR) == 0)) {
            iot_error_set(IOT_ERROR_TYPE, BLE_ERROR);
            ble_info.init = 0;
            SETBIT(sys_info.mode_reinit_flag, BLE_MODEL);
        }

        if(CHECKBIT(sys_info.mode_reinit_flag, GPS_MODEL)) {
            if(sys_mode_reinit(GPS_MODEL) == 0) {
                CLEARBIT(sys_info.mode_reinit_flag, GPS_MODEL);
            }
        }

        if(CHECKBIT(sys_info.mode_reinit_flag, BLE_MODEL)) {
            if(sys_mode_reinit(BLE_MODEL) == 0) {
                CLEARBIT(sys_info.mode_reinit_flag, BLE_MODEL);
            }
        }

        // if(CHECKBIT(sys_info.mode_reinit_flag, SENSOR_MODEL)) {
        //     if(sys_mode_reinit(SENSOR_MODEL) == 0) {
        //         CLEARBIT(sys_info.mode_reinit_flag, SENSOR_MODEL);
        //     }
        // }

        /*报错推送*/
        if(sys_info.last_car_error != sys_info.car_error || sys_info.iot_error != sys_info.last_iot_error) {
            sys_info.last_car_error = sys_info.car_error;
            sys_info.last_iot_error = sys_info.iot_error;
            net_engwe_cmd_push(STATUS_PUSH_UP, 0x00000200);
        }

        if(car_info.lock_sta == CAR_UNLOCK_STA && sys_info.ota_flag == 0) {
            iot_can_heart_fun();
        }   
        if(sys_set_var.sys_updata_falg != 0) {
            if(sys_set_var.sys_updata_falg & (1 << SYS_SET_SAVE)) {
                sys_set_var.sys_updata_falg &= ~(1 << SYS_SET_SAVE);
                sys_param_save(SYS_SET_ADR); 
            }
            if(sys_set_var.sys_updata_falg & (1 << SYS_CONFIG_SAVE)) {
                sys_set_var.sys_updata_falg &= ~(1 << SYS_CONFIG_SAVE);
                sys_param_save(SYS_CONFIG_ADR);
                sys_param_save(BACK_SYS_CONFIG_ADR);
            }
            if(sys_set_var.sys_updata_falg & (1 << SHEEP_DATA_SAVE)){
                sys_set_var.sys_updata_falg &= ~(1 << SHEEP_DATA_SAVE);
                LOG_I("SHEEP_DATA_SAVE");
                sys_param_save(SHEEP_FOLD_P_ADR);
            }
            if(sys_set_var.sys_updata_falg & (1 << FORBIDDEN_DATA_SAVE)){
                sys_set_var.sys_updata_falg &= ~(1 << FORBIDDEN_DATA_SAVE);
                LOG_I("FORBIDDEN_DATA_SAVE");
                sys_param_save(FORBIDDEN_ZONE_P_ADR);
            }
        }
        if(def_rtos_get_system_tick() - csq_time_t > 10*1000) {
            net_update_singal_csq();
            nw_info = hal_drv_get_operator_info();
            LOG_I("MCC:%d, mnc:%d, lac:%d, cid:%d", nw_info.mcc, nw_info.mnc, nw_info.lac, nw_info.cid);
            LOG_I("net_state:%d, act:%d, rsrp:%d, bit_error_rate:%d", nw_info.net_state, nw_info.act, nw_info.rsrp, nw_info.bit_error_rate);
            csq_time_t = def_rtos_get_system_tick();
            LOG_I("CSQ:%d", gsm_info.csq);
      //      LOG_I("pack_series_number:%d, pack_parallel_number:%d", car_info.bms_info[0].pack_series_number, car_info.bms_info[0].pack_parallel_number);   
        //    LOG_I("ptich:%.2f,roll:%.2f,yaw:%.2f",euler_angle[0],euler_angle[1],euler_angle[2]);
            // LOG_I("car_info.speed_limit:%d, car_info.pedal_speed:%d", car_info.speed_limit, car_info.pedal_speed);
            // LOG_I("car_info.total_odo:%d, car_info.single_odo:%d", car_info.total_odo, car_info.single_odo);
            // LOG_I("car_info.remain_odo:%d, car_info.wheel:%d", car_info.remain_odo, car_info.wheel);
            // LOG_I("car_info.current:%d, car_info.pedal_torque:%d", car_info.current, car_info.pedal_torque);
            // LOG_I("car_info.bus_voltage:%d, car_info.trip_calorie:%d", car_info.bus_voltage, car_info.ebike_calorie);
            // LOG_I("car_info.hmi_info.display_unit:%d, car_info.gear:%d", car_info.hmi_info.display_unit, car_info.gear);
            // LOG_I("car_info.speed:%d, car_info.current_limit:%d", car_info.speed, car_info.current_limit);
            // if(TEMP == 1) {
            //     imu_algo_timer_start();
            //     TEMP = 0;
            // }
            if(TEMP == 0) {
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
                #if 0
                voice_play_mark(VOICE_TEST); 
                #endif
          //      GPS_Start(GPS_MODE_CONT);
                MCU_CMD_MARK(CMD_MCU_ADC_DATA_INDEX);
             //   MCU_CMD_MARK(CMD_MCU_BAT_CHARGE_OFF_INDEX);
                // LOG_I("imu_algo_timer_stop");
            //     imu_algo_timer_stop();
                TEMP = 1;
            } 
      //      MCU_CMD_MARK(CMD_MCU_ADC_DATA_INDEX);
        }
        if(sys_set_var.car_power_en) {
            if(sys_set_var.car_power_en == 1) {

            } else if(sys_set_var.car_power_en == 2) {
                
            }
            sys_set_var.car_power_en = 0;
        }
        
        // if(sys_info.paltform_connect == 1){
        //     iot_mqtt_public((uint8_t *)test_net, strlen(test_net));
        // }
//        if(car_info.lock_sta == CAR_LOCK_STA) {
      //      car_heart_event();
 //       }
        /*电子围栏检测*/
        GPS_fence_detection();

        if(hal_drv_read_gpio_value(I_BLE_CON_SIG)) {   //蓝牙连接状态检测
            if(car_info.lock_sta == CAR_UNLOCK_STA) {
                ble_heart_event();
            } else {
                if(def_rtos_get_system_tick() - ble_info_up_time_t > 5000) {
                    if(sys_info.ble_can_trans_sw == 0) {
                        ble_heart_event();
                    }
                    ble_info_up_time_t = def_rtos_get_system_tick();
                }
            }   
        } else if(sys_info.ble_connect == 1) {   
            if(sys_param_set.alive_flag == 0) {  //未激活
                if(sys_info.power_36v == 0) {
                    week_time("ble", 300);  //5分钟后休眠
                }
            } else {
                week_time("ble", 60);   
            }
            sys_info.ble_log_sw = 0;
            sys_info.ble_connect = 0;
            LOG_I("BLE DISCONNECT!");
        }

        if(hal_drv_read_gpio_value(I_36VPOWER_DET) == 1 && sys_info.power_36v == 1) {    //36V电源检测
            sys_info.power_36v = 0;
            regular_heart_update();
            LOG_I("POWER ON OFF");
            sys_info.power_sta = BATTERY_OUT;
            net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            sys_info.power_sta = BATTERY_INNER;
            LOG_I("36V power off");
            car_info.bms_info[0].connect = 0; 
            if(car_info.lock_sta == CAR_UNLOCK_STA) {   //防止直接拔电池
                week_time("lock", 30);
                voice_play_mark(LOCK_VOICE);   
                car_info.lock_sta = CAR_LOCK_STA;
            }
        } else if(hal_drv_read_gpio_value(I_36VPOWER_DET) == 0 && sys_info.power_36v == 0) {
             can_png_quest(BMS_ADR, BMS_COMPREHENSIVE_DATA, 0);
             sys_info.power_36v = 1;
             sys_info.power_sta = BATTERY_IN;
             regular_heart_update();
             LOG_I("POWER ON STATUE");
             net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
             LOG_I("36V power on");
             sys_info.power_sta = BATTERY_USE;
        }
        // if(sys_info.bat_soc <= 5 && sys_info.power_36v == 0) {
        //     if(sys_info.ble_connect == 1)
        //         ble_cmd_mark(BLE_DISCONNECT_INDEX);
        // }
        
        if(car_info.last_lock_sta != car_info.lock_sta && (sys_info.ota_flag == 0)) {
            car_info.last_lock_sta = car_info.lock_sta;
            if(car_info.lock_sta == CAR_UNLOCK_STA) {
                GPS_Start(GPS_MODE_CONT);
                SETBIT(sys_param_set.net_engwe_report_time1_cmdId, RIDE_INFO_CMD);
                SETBIT(sys_param_set.net_engwe_report_time1_cmdId, BATTRY_INFO_CMD);
            } else {
                GPS_stop();//关锁后关GPS
                CLEARBIT(sys_param_set.net_engwe_report_time1_cmdId, RIDE_INFO_CMD);
                CLEARBIT(sys_param_set.net_engwe_report_time1_cmdId, BATTRY_INFO_CMD);
            }
            regular_heart_update();
            net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
        }

   //     LOG_I("total_fence_sw:%d, sheepfang_sta:%d, fence_sta:%d", sys_param_set.total_fence_sw, sys_info.sheepfang_sta, sys_info.fence_sta);
        /*开锁运动状态检测*/
        if(car_info.lock_sta == CAR_UNLOCK_STA && (sys_info.ota_flag == 0)) {
            if(car_info.speed == 0 && car_info.last_speed) {  //运动到静止
                car_info.car_unlock_state = CAR_UNLOCK_TO_STILL;  //需要推送
                LOG_I("CAR_UNLOCK_TO_STILL");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            } else if(car_info.last_speed == 0 && car_info.speed) {
                car_info.car_unlock_state = CAR_UNLOCK_TO_MOVE;  //需要推送
                LOG_I("CAR_UNLOCK_TO_MOVE");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            } else if(car_info.last_speed == 0 && car_info.speed == 0) {
                car_info.car_unlock_state = CAR_UNLOCK_STILL;
            } else if(car_info.last_speed && car_info.speed) {
                car_info.car_unlock_state = CAR_UNLOCK_MOVE;
            }
            car_info.last_speed = car_info.speed;

            // if(car_info.last_gear != car_info.gear){
            //     net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
            // }
            // car_info.last_gear = car_info.gear;

            // if(car_info.hmi_info.display_unit != car_info.last_unit) {
            //     net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
            // }
            // car_info.last_unit = car_info.hmi_info.display_unit;

            if(car_info.last_speed_limit != car_info.speed_limit) {
                net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
            }
            car_info.last_speed_limit = car_info.speed_limit;

            if(car_info.last_headlight_sta != car_info.headlight_sta) {
                net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
            }
            car_info.last_headlight_sta = car_info.headlight_sta;
        }  
        /*====================震动检测========================*/
        if(car_info.lock_sta == CAR_LOCK_STA && (sys_info.ota_flag == 0)) {
            if(car_info.move_alarm && car_info.car_lock_state == CAR_LOCK_STILL) {
                car_info.move_alarm = 0;
                car_info.car_lock_state = CAR_LOCK_TO_SHOCK;
                shock_time_t = def_rtos_get_system_tick();
                trace_time_t = def_rtos_get_system_tick();
                GPS_Start(GPS_MODE_TM);  //震动获取一次定位  
                if(sys_param_set.shock_sw) {
                    #if 1
                    voice_play_mark(ALARM_VOICE); 
                    #endif
                    if(sys_info.iot_error == 0){
                        app_set_led_ind(LED_ARARM);
                    }
                    LOG_I("CAR_LOCK_TO_SHOCK"); 
                    net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
                }
                imu_algo_timer_start();
                car_info.car_lock_state = CAR_LOCK_SHOCK;
            } else if(car_info.car_lock_state == CAR_LOCK_SHOCK && car_info.move_alarm) {
                if(sys_param_set.shock_sw) {
                    if(sys_info.iot_error == 0 && sys_info.led_type_cur != LED_ARARM){
                        app_set_led_ind(LED_ARARM);
                    } 
                    if(sys_info.voice_type_cur != ALARM_VOICE) {
                        #if 1
                        voice_play_mark(ALARM_VOICE); 
                        #endif
                    }
                }
                car_info.move_alarm = 0;
                shock_time_t = def_rtos_get_system_tick();
            } else if(def_rtos_get_system_tick() - shock_time_t > 5000 && car_info.car_lock_state != CAR_LOCK_STILL) {
                car_info.car_lock_state = CAR_LOCK_STILL;
                imu_algo_timer_stop();
                trace_time_t = def_rtos_get_system_tick();
            } 
            if(def_rtos_get_system_tick() - trace_time_t > 30000 && sys_info.track_mode == 0 && car_info.car_lock_state != CAR_LOCK_STILL) {   /*触发GPS追踪模式，防盗模式*/
                week_time("track", -1);
                sys_info.track_mode = 1;
                GPS_Start(GPS_MODE_CONT);
                rtc_event_register(GPS_TRACK_EVENT, 10, 1);
            }
            if(sys_info.track_mode == 1 && def_rtos_get_system_tick() - trace_time_t > 180000) {/*3分钟后解除*/
                week_time("track", 30);
                GPS_stop();
                sys_info.track_mode = 0;
                rtc_event_unregister(GPS_TRACK_EVENT);
            }
        } else {
            if(sys_info.track_mode == 1) {
                sys_info.track_mode = 0;
                rtc_event_unregister(GPS_TRACK_EVENT);
                week_time("track", 30);
            }
            imu_algo_timer_stop();
            car_info.car_lock_state = CAR_LOCK_STILL;
            car_info.move_alarm = 0;
        }
        
        if(sys_info.shock_sw_state != sys_param_set.shock_sw) {
            sys_info.shock_sw_state = sys_param_set.shock_sw;
            net_engwe_cmd_push(OPERATION_PUSH_UP, sys_param_set.net_engwe_offline_opearte_push_cmdId);
        }


        /*充电检测*/
        if(car_info.quick_charger_det && car_info.quick_charger_flag == 0){
            car_control_cmd(CAR_BMS_CHARGE_CURRENT_SET);
            car_info.quick_charger_flag = 1;
            car_info.charger_state = CHARGER_PLUG_IN;
            net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            car_info.charger_state = CHARGER_QUICK_IN;
            LOG_I("CHARGER_QUICK_IN");
            net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            car_info.charger_state = CHARGER_STATE;
        } 
        if(car_info.bms_info[0].charge_det && car_info.charger_det_flag == 0){
            car_info.charger_det_flag = 1;
            if(car_info.charger_state != CHARGER_STATE) {
                car_info.charger_state = CHARGER_PLUG_IN;
                LOG_I("CHARGER_PLUG_IN");
                net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            }
            rtc_event_register(BAT_MCU_ADC_GET_EVENT, BMS_CHARGE_RTC_CHECK_TIME, 1);
            car_info.charger_state = CHARGER_STATE;
        }
        if(car_info.bms_info[0].charge_det == 0 && car_info.charger_det_flag == 1){
            car_info.charger_det_flag = 0;
            car_info.quick_charger_det = 0;
            car_info.quick_charger_flag = 0;
            car_info.charger_state = CHARGER_PUG_OUT;
            LOG_I("CHARGER_PUG_OUT");
            rtc_event_register(BAT_MCU_ADC_GET_EVENT, BMS_DISCHARGE_RTC_CHECK_TIME, 1);
            net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            car_info.charger_state = CHARGER_NONE_STA;
        }
        if(car_info.bms_info[0].chargefull_sta && car_info.charger_full_flag == 0){
            car_info.charger_full_flag = 1;
            car_info.charger_state = CHARGER_FULL;
            LOG_I("CHARGER_FULL");
            net_engwe_cmd_push(STATUS_PUSH_UP, sys_param_set.net_engwe_state_push_cmdId);
            car_info.charger_state = CHARGER_NONE_STA;
        }
        if(car_info.bms_info[0].chargefull_sta == 0){
            car_info.charger_full_flag = 0;
        }
    }
    def_rtos_task_delete(NULL);
}


def_rtos_sem_t log_sem_t;
#define SYS_LOG_LEN    4096
struct rt_ringbuffer *sys_log_buf;

void sys_log_out(const char *fmt, ...)
{
    uint8_t *buf;
    va_list args;
    va_start(args, fmt);
    buf = malloc(512);
    if(buf == NULL) return;
    memset(buf, 0, 512);
    vsnprintf((char *)buf, 512, fmt, args);
    va_end(args);
    rt_ringbuffer_put(sys_log_buf, buf, strlen((char *)buf));
    def_rtos_smaphore_release(log_sem_t);
    free(buf);
}

void app_system_log_out_thread(void *param)
{
    uint16_t buf_len, read_len;
    uint8_t buf[128];
    while(1){
        if(def_rtos_semaphore_wait(log_sem_t, RTOS_WAIT_FOREVER) == RTOS_SUCEESS) {
            buf_len = rt_ringbuffer_data_len(sys_log_buf);
            while(buf_len > 0){
                read_len = MIN(buf_len, 128);
                buf_len -= read_len;
                rt_ringbuffer_get(sys_log_buf, buf, read_len);
                if(sys_info.ble_log_sw) {
                    ble_log_out(buf, read_len);
                }
            }  
        }
    }
    def_rtos_task_delete(NULL);
}

void system_timer_fun()
{
    def_rtos_smaphore_release(system_task_sem);
}

void system_timer_stop()
{
    def_rtos_timer_stop(system_timer);
}

void system_timer_start()
{
    def_rtos_timer_start(system_timer, 1000, 1);
}

void sensor_cali_param_init()
{
    flash_partition_read(SENSEOR_CALIBRATION_ADR,  (void *)&sensor_calibration_data, \
    sizeof(struct sensor_calibration_data_stu), 0);
    if(sensor_calibration_data.magic == IOT_MAGIC\
	&& sensor_calibration_data.crc32 == GetCrc32((uint8_t *)&sensor_calibration_data, sizeof(sensor_calibration_data) - 4)) {
		sys_info.static_cali_flag = 1;
        for(uint8_t i = 0; i < 3; i++) {
            LOG_I("static_offset_acc[%d]:%f, static_offset_gyro[%d]:%f",i,\
            sensor_calibration_data.static_offset_acc[i], i, sensor_calibration_data.static_offset_gyro[i]);
        }
	}
}

void sys_param_init()
{
    ota_fd = -1;
    memset(&sys_info, 0, sizeof(sys_info));
    memset(&sys_set_var, 0, sizeof(sys_set_var));
    sys_config_init();
    sys_param_set_init();
    sheepfang_data_init();
    forbidden_zone_data_init();
    sensor_cali_param_init();
    sys_info.power_adc.sys_power_rate = 24.2556;
    sys_info.power_adc.bat_val_rate = 2.0;
    sys_info.bat_charge_state = BAT_CHARGE_OFF;
    sys_info.adc_charge_get_interval = 30*60; 
    sys_info.adc_discharge_get_interval = 60*60; 
    sys_info.shock_sw_state = sys_param_set.shock_sw;
    LOG_I("OTA_CNT:%d", sys_param_set.ota_cnt);   
}




void ota_test()
{
    char ota_str[] = "123456789";
    char ota_s[4] = {0};
    if(flash_partition_erase(DEV_APP_ADR) < 0) {
        LOG_E("ota erase is error");
    }

    if(flash_partition_write(DEV_APP_ADR, ota_str, sizeof(ota_str), 0) < 0) {
        LOG_E("ota write is error");
    }
    if(flash_partition_read(DEV_APP_ADR, ota_s, 3, 6) < 0) {


        
        LOG_E("ota read is error");
    } else {
        LOG_I("%s", ota_s);
    }
}
extern void electron_fence_test();

void app_sys_init()
{
    hal_drv_init();
    app_led_init();
    app_rtc_init();
    net_control_init();
    sys_param_init();
    can_protocol_init();
    app_http_ota_init();
    net_engwe_init();
    low_power_init();
    app_sensor_init();
    register_module("sys");
    register_module("ble");
    register_module("lock");
    register_module("sensor");
    register_module("track");
    register_module("ota");
    register_module("mcu");
    register_module("net");
    sys_log_buf = rt_ringbuffer_create(SYS_LOG_LEN);
    week_time("sys", 30); 
    shock_cent = 0;
    def_rtos_semaphore_create(&log_sem_t, 0);
    
   // sys_info.static_cali_flag = 1;
  //   app_set_led_ind(LED_TEST);
    rtc_event_register(BAT_MCU_ADC_GET_EVENT, BMS_DISCHARGE_RTC_CHECK_TIME, 1);
    rtc_event_register(NET_HEART_EVENT, sys_param_set.net_heart_interval, 1);
  //  rtc_event_register(SENSOR_CHECK_EVENT, SENSOR_FUNC_RTC_CHECK_TIME, 1);  /*2小时检测一次传感器*/
    def_rtos_semaphore_create(&system_task_sem, 0);
    def_rtos_timer_create(&system_timer, app_system_task, system_timer_fun, NULL);
    def_rtos_timer_start(system_timer, 1000, 1);
    LOG_I("app_sys_init is ok");
}
