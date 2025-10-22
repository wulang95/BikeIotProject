#include "app_system.h"
#include "hal_drv_net.h"
#include "sockets.h"
// #include "lwip/ip_addr.h"
// #include "lwip/ip6_addr.h"
// #include "lwip/netdb.h"
// #include "lwip/netif.h"
// #include "lwip/inet.h"
// #include "lwip/tcp.h"
#include "ql_mqttclient.h"
#include "ql_api_datacall.h"
#include "ql_ssl.h"
#define DBG_TAG         "net_control"

#ifdef NET_CONTROL_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"

#define MIN_FUN     0
#define ALL_FUN     1
#define DIS_FUN     4

struct gsm_info_stu gsm_info;
static char ip4_adr_str[16];

#define HAND_SELECT_PLMN_ENABLE    1

typedef enum
{
    PDP_NOT_ACTIVATED = 0,
    PDP_CONDITIONS_CHECK ,
    PDP_START_ACTIVATION ,
    PDP_ACTIVATION_SUCCESS ,
    PDP_NTP_SYNC,
    PDP_ACTIVATION_FAILURE,
    PDP_GET_OPERT_INFO,
    PDP_HAND_SELELCT_OPERT,
    PDP_HAND_SW_PLMN,
    PDP_ACTIVE_DETECT,
} pdp_active_state_type;


typedef enum
{
    SOCKET_STATUS_IDLE = 0,
    SOCKET_STATUS_CREATE = 1,
    SOCKET_STATUS_CONNECTTING = 2,
    SOCKET_STATUS_DATA_PROC = 3,
    SOCKET_STATUS_DISCONNECT =4,
} socket_state_type;

typedef struct {
    socket_state_type socket_state;
    struct sockaddr_in local4;
    struct sockaddr_in server_ipv4;
    int socket_fd;
} SOCKET_CON_INFO_STU;

typedef struct {

    pdp_active_state_type pdp_state;
    uint8_t pdp_is_active;
    int profile_idx;
} PDP_ACTIVE_INFO_STU;

PDP_ACTIVE_INFO_STU pdp_active_info;
SOCKET_CON_INFO_STU socket_con_info;
typedef struct {
    uint8_t state;
    uint16_t sim_cid;
    mqtt_client_t mqtt_cli;
    struct mqtt_connect_client_info_t client_info;
    int mqtt_connected;
    int sub_res;
    char *url;
    char *sub_topic;
    def_rtos_sem_t mqtt_disconnet_sem_t;
    char pub_topic[128];
}MQTT_CON_INFO;

MQTT_CON_INFO mqtt_con_info;

struct black_plmn_info_s {
    uint8_t plmn_num;
    char plmn[7][9];
};

static struct black_plmn_info_s black_plmn_info;
static char s_plmn[9];

enum {
    MQTT_BIND_SIM_AND_PROFILE = 1,
    MQTT_CLIENT_INIT,
    MQTT_CONNECT,
    MQTT_CONNECT_RES,
    MQTT_SET_INPUB,
    MQTT_SUB,
    MQTT_SUB_RES,
    MQTT_CONNECT_DET,
    MQTT_UNSUB_RES,
    MQTT_DISCONNECT,
};
#define USE_CRT_BUFFER 1

void net_control_init()
{
    memset(&gsm_info, 0, sizeof(gsm_info));
    memset(&pdp_active_info, 0, sizeof(pdp_active_info));
    memset(&socket_con_info, 0, sizeof(socket_con_info));
    memset(&black_plmn_info, 0, sizeof(black_plmn_info));
    pdp_active_info.pdp_is_active = 0;
    pdp_active_info.profile_idx = 1;
    pdp_active_info.pdp_state = PDP_NOT_ACTIVATED;
    socket_con_info.socket_fd = -1;
    socket_con_info.socket_state = SOCKET_STATUS_IDLE;
    LOG_I("net_control_init is ok");
}

void net_update_singal_csq()
{
    hal_drv_get_signal(&gsm_info.csq);
}



static int add_plmn_to_black_table(char *plmn)
{
    if(plmn != NULL && strlen(plmn) <= 9) {
        LOG_I("plmn:%s", plmn);
    } else {
        LOG_E("plmn is error");
        return -1;
    }
    if(black_plmn_info.plmn_num == 7) {
        LOG_E("back plmn is full");
        return -1;
    }
    for(int i = 0; i < 7; i++) {
        if(strlen(black_plmn_info.plmn[i]) != 0) {
            if(strcmp(black_plmn_info.plmn[i], plmn) == 0){
                return 1;
            }
        } else {
            LOG_I("add plmn:%s", plmn);
            memcpy(black_plmn_info.plmn[i], plmn, strlen(plmn));
            black_plmn_info.plmn_num++;
            return 0;
        }
    }
    return -1;
}

void plmn_black_print()
{
    LOG_I("plmn_num:%d", black_plmn_info.plmn_num);
    for(int i = 0; i < black_plmn_info.plmn_num; i++){
        LOG_I("black plmn:%s", black_plmn_info.plmn[i]);
    }
}


static int select_plmn(char *plmn){
    if(plmn == NULL) {
        LOG_E("plmn is error");
        return -1;
    }
    for(int i = 0; i < black_plmn_info.plmn_num; i++) {
        if(strcmp(black_plmn_info.plmn[i], plmn) == 0) {
            LOG_I("plmn is in");
            return 1;
        }
    }
    LOG_I("select_plmn:%s", plmn);
    return 0;
}

//uint8_t debug_flag = 1;

static void pdp_active_state_machine(void)
{
    uint8_t c_fun;
    uint8_t cpin = 0;
    uint8_t net_reg = 0;
    static uint8_t count = 0;
    static int64_t check_csq_timeout = 0;
    static int64_t check_pdp_timeout = 0;
    static int64_t check_sync_timeout = 0;
    switch (pdp_active_info.pdp_state) {
        case PDP_NOT_ACTIVATED:
            LOG_I("PDP_NOT_ACTIVATED");
            week_time("pdp", -1); 
         //   ql_volte_set(0, 0);
            sys_info.pdp_reg = 0;
            pdp_active_info.pdp_is_active = 0;
            sys_info.paltform_connect = 0;
            pdp_active_info.pdp_state  = PDP_CONDITIONS_CHECK;
            mqtt_con_info.state = MQTT_BIND_SIM_AND_PROFILE;
            check_csq_timeout = def_rtos_get_system_tick();
        break;
        case PDP_CONDITIONS_CHECK:
            LOG_I("PDP_CONDITIONS_CHECK");
            hal_drv_get_imei(gsm_info.imei, sizeof(gsm_info.imei));
            LOG_I("IMEI:%s", gsm_info.imei);
            c_fun = hal_dev_get_c_fun();
            if(c_fun != ALL_FUN) {
                hal_dev_set_c_fun(ALL_FUN, 0);
            }
            cpin = hal_drv_get_cpin();
            if(cpin) {
                hal_drv_get_iccid(gsm_info.iccid, sizeof(gsm_info.iccid));
                hal_drv_get_signal(&gsm_info.csq);
                LOG_I("CSQ:%d", gsm_info.csq);
                LOG_I("ICCID:%s", gsm_info.iccid);
            }
            net_reg = hal_drv_get_net_register_sta();
            LOG_I("cpin:%d, c_fun:%d, net_reg:%d", cpin, c_fun,net_reg);
            if(gsm_info.csq > 10 && gsm_info.csq != 99 && cpin == 1 && c_fun == ALL_FUN && net_reg == 1) {
                pdp_active_info.pdp_state  = PDP_START_ACTIVATION;
                check_pdp_timeout = def_rtos_get_system_tick();
                hal_drv_set_data_call_asyn_mode(pdp_active_info.profile_idx,1);
                hal_drv_start_data_call(pdp_active_info.profile_idx, sys_config.apn); 
            } else if (gsm_info.csq > 10 && gsm_info.csq != 99 && cpin == 1 && c_fun == ALL_FUN && net_reg == 0 && (def_rtos_get_system_tick() - check_csq_timeout)/1000 > 15*60) {
                pdp_active_info.pdp_state = PDP_GET_OPERT_INFO;
                count = 0;
            } else if((def_rtos_get_system_tick() - check_csq_timeout)/1000 > 30*60) {
                LOG_E("sys_reset...");
                if(cpin == 0) {
                    cat1_reset_reson_save(NET_RESET_NO_CARD);
                } else if(gsm_info.csq <= 10 || gsm_info.csq == 99) {
                    cat1_reset_reson_save(NET_RESET_CSQ_LOW);
                } else {
                    cat1_reset_reson_save(NET_RESET_NO_REG);
                }
                MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
            }
        break;
        case PDP_START_ACTIVATION:
            LOG_I("PDP_START_ACTIVATION");
            memset(ip4_adr_str, 0, sizeof(ip4_adr_str));
            if(hal_drv_get_data_call_res(pdp_active_info.profile_idx,ip4_adr_str)) {
                pdp_active_info.pdp_state  = PDP_NTP_SYNC;
            } else if((def_rtos_get_system_tick() - check_pdp_timeout)/1000 > 5*60) {
               pdp_active_info.pdp_state  = PDP_ACTIVATION_FAILURE;
            }
        break;
        case PDP_NTP_SYNC:
            LOG_I("PDP_NTP_SYNC");
            if(hal_net_ntp_sync(pdp_active_info.profile_idx) == 0){
                pdp_active_info.pdp_state = PDP_ACTIVATION_SUCCESS;
            } else if((def_rtos_get_system_tick() - check_sync_timeout)/1000 > 30){
                LOG_E("PDP_NTP_SYNC IS FAIL");
                pdp_active_info.pdp_state = PDP_ACTIVATION_SUCCESS;
            }
        break;
        case PDP_ACTIVATION_SUCCESS:
            LOG_I("PDP_ACTIVATION_SUCCESS");
            pdp_active_info.pdp_state  = PDP_ACTIVE_DETECT;
            pdp_active_info.pdp_is_active = 1;
            sys_info.pdp_reg = 1; 

        break;
        case PDP_ACTIVATION_FAILURE:
            LOG_I("PDP_ACTIVATION_FAILURE");
            #if HAND_SELECT_PLMN_ENABLE == 1
                pdp_active_info.pdp_state = PDP_GET_OPERT_INFO;
                count = 0;
            #else
                hal_drv_stop_data_call(pdp_active_info.profile_idx);
                hal_dev_set_c_fun(MIN_FUN, 0);
                pdp_active_info.pdp_state  = PDP_NOT_ACTIVATED;
                if(count++>=3) {
                    MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
                }
            #endif
        break;
        case PDP_GET_OPERT_INFO:
            LOG_I("PDP_GET_OPERT_INFO");
            if(hal_get_plmn_info() == 0) {
                memset(s_plmn, 0, 9);
                add_plmn_to_black_table(oper_info.c_plmn);
                plmn_black_print();
                pdp_active_info.pdp_state = PDP_HAND_SELELCT_OPERT;
            } else if(count++ >= 3) {
                if(net_reg == 0) {
                    cat1_reset_reson_save(NET_RESET_NO_REG);
                } else {
                    cat1_reset_reson_save(NET_RESET_PDP_FAIL);
                }
                MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
            } else {
                def_rtos_task_sleep_s(5);
            }
        break;
        case PDP_HAND_SELELCT_OPERT:
            LOG_I("PDP_HAND_SELELCT_OPERT");
            for(int i = 0; i < oper_info.vaild_oper_num; i++) {
                if(select_plmn(oper_info.v_oper_info[i].plmn) == 0) {
                    memcpy(s_plmn, oper_info.v_oper_info[i].plmn, 9);
                    pdp_active_info.pdp_state = PDP_HAND_SW_PLMN;
                    count = 0;
                    LOG_I("plmn:%s", s_plmn);
                    break;
                } 
            } 
            if(strlen(s_plmn) == 0) {
                if(net_reg == 0) {
                    cat1_reset_reson_save(NET_RESET_NO_REG);
                } else {
                    cat1_reset_reson_save(NET_RESET_PDP_FAIL);
                }
                MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
            }  
        break;
        case PDP_HAND_SW_PLMN:
            LOG_I("PDP_HAND_SW_PLMN");
            if (hal_select_oper(s_plmn) == 0) {
                pdp_active_info.pdp_state = PDP_NOT_ACTIVATED;
            } else {
                if(count++ >= 3) {
                    count = 0;
                    add_plmn_to_black_table(s_plmn);
                    plmn_black_print();
                    memset(s_plmn, 0, 9);
                    pdp_active_info.pdp_state = PDP_HAND_SELELCT_OPERT;
                } else {
                    def_rtos_task_sleep_s(5);
                }
            } 
        break;
        case PDP_ACTIVE_DETECT:
            LOG_I("PDP_ACTIVE_DETECT");
            week_time("pdp", 30);
            #if 0
            LOG_I("debug_flag:%d", debug_flag);
            if(debug_flag++ < 4) {
                def_rtos_task_sleep_s(20);
                pdp_active_info.pdp_state = PDP_ACTIVATION_FAILURE;
                break;
            }
            #endif
            hal_drv_pdp_detect_block();
            sys_info.net_disconnect_res = NET_DISCON_REG_FAIL;
            count = 0;
            memset(&black_plmn_info, 0, sizeof(black_plmn_info));
            pdp_active_info.pdp_state = PDP_NOT_ACTIVATED;
        break;
    }
} 

void pdp_active_thread(void *param)
{
    hal_drv_data_call_register_init();
    app_system_wait_active();
    while(1) {
    //    LOG_I("IS RUN");
        pdp_active_state_machine();
        def_rtos_task_sleep_ms(100);
    }
    def_rtos_task_delete(NULL);

}

void iot_socket_data_init()
{
    hal_drv_set_dns_addr(sys_config.DSN);
    memset(&socket_con_info.local4, 0x00, sizeof(struct sockaddr_in));
    socket_con_info.local4.sin_family = AF_INET;
    socket_con_info.local4.sin_port = 0;
    inet_aton(ip4_adr_str, &socket_con_info.local4.sin_addr);
    memset(&socket_con_info.server_ipv4, 0x00, sizeof(struct sockaddr_in));

} 


void iot_socket_create()
{
    uint8_t ret;	
    socket_con_info.socket_fd = socket(AF_INET, SOCK_STREAM, 0);
    if(socket_con_info.socket_fd < 0){
        LOG_E("socket is error");
        return;
    }
    ret = bind(socket_con_info.socket_fd, (struct sockaddr *)&socket_con_info.local4, sizeof(struct sockaddr));
    if(ret < 0){
        close(socket_con_info.socket_fd);
        LOG_E("bind is error");
        return;
    }
    socket_con_info.socket_state = SOCKET_STATUS_CONNECTTING;
}
/*
void iot_socket_connect()
{
    uint8_t ret;
    struct addrinfo *pres = NULL;	
	struct addrinfo *temp = NULL;
    static uint16_t socket_connect_delay = 1;
    ret = getaddrinfo_with_pcid(sys_config.ip, NULL, NULL, &pres, 1);
	if (ret < 0 || pres == NULL) 
	{
		LOG_E("DNS getaddrinfo failed! ret=%d; pres=%p!\n",ret,pres);
	}

    for(temp = pres; temp != NULL; temp = temp->ai_next) {
        struct sockaddr_in * sin_res = (struct sockaddr_in *)temp->ai_addr;
        socket_con_info.server_ipv4.sin_addr = sin_res->sin_addr;
        socket_con_info.server_ipv4.sin_family = AF_INET;
	    socket_con_info.server_ipv4.sin_port = htons(sys_config.port);
        ret = connect(socket_con_info.socket_fd, (struct sockaddr *)&socket_con_info.server_ipv4, sizeof(socket_con_info.server_ipv4));	
        if(ret == 0) {
            socket_con_info.socket_state = SOCKET_STATUS_DATA_PROC;
            return;
        }
    }
    if(socket_connect_delay > 512) {
        socket_connect_delay = 1;
    }
    def_rtos_task_sleep_s(socket_connect_delay);
    socket_connect_delay = socket_connect_delay << 1;
}
*/
void net_socket_send(uint8_t *data, uint16_t len)
{
    debug_data_printf("net_send", data, len);
    write(socket_con_info.socket_fd, data, len);
}

void net_socket_close()
{
    if(socket_con_info.socket_fd > 0) {
        close(socket_con_info.socket_fd);
        LOG_I("socket is close");
    }
    sys_info.paltform_connect = 0;
    socket_con_info.socket_fd = -1;
}

void socket_data_process()
{
    uint8_t buf[1024] = {0};
    uint16_t len;
    fd_set read_fds;
    fd_set exp_fds;
    int flags = 0;
    int fd_changed = 0;
    FD_ZERO(&read_fds);
	FD_ZERO(&exp_fds);

    flags |= O_NONBLOCK;
	fcntl(socket_con_info.socket_fd, F_SETFL, flags);

    FD_SET(socket_con_info.socket_fd, &read_fds);
    FD_SET(socket_con_info.socket_fd, &exp_fds);
    NET_CMD_MARK(NET_CMD_SIGN_IN_Q0);
    while(1) {
    //    LOG_I("IS RUN");
        fd_changed = select(socket_con_info.socket_fd+1, &read_fds, NULL, &exp_fds, NULL);
        LOG_I("fd_changed:%d", fd_changed);
        if(fd_changed > 0) {
            if(FD_ISSET(socket_con_info.socket_fd, &read_fds)){
                FD_CLR(socket_con_info.socket_fd, &read_fds);
                memset(buf, 0, sizeof(buf));
                len = read(socket_con_info.socket_fd, buf, 512);
                if(len <= 0) {
                    socket_con_info.socket_state = SOCKET_STATUS_DISCONNECT;
                    sys_info.paltform_connect = 0;
                    LOG_E("socket is error");
                    break;
                }
                FD_SET(socket_con_info.socket_fd, &read_fds);
              //  debug_data_printf("net_recv", buf, len);
                net_recv_data_parse(buf, len);
            } else if(FD_ISSET(socket_con_info.socket_fd, &exp_fds)) {
                FD_CLR(socket_con_info.socket_fd, &exp_fds);
                socket_con_info.socket_state = SOCKET_STATUS_DISCONNECT;
                sys_info.paltform_connect = 0;
                LOG_E("socket is error");
                break;
            } else {
                LOG_E("socket is error");
                socket_con_info.socket_state = SOCKET_STATUS_DISCONNECT;
                sys_info.paltform_connect = 0;
                break;
            }  
        }   
    }
}

// void socket_data_process()
// {
//     int  recv_len = 0;
//     uint8_t buf[1024] = {0};
//     net_protocol_cmd_send(NET_CMD_SIGN_IN_Q0);
//     while(1){
//         memset(buf, 0, sizeof(buf));
//         recv_len = read(socket_con_info.socket_fd, buf, 1024);
//         if(recv_len > 0) {
//              debug_data_printf("net_recv", buf, recv_len);
//              net_recv_data_parse(buf, recv_len);
//         } else {
//             LOG_E("SOCKET is error");
//             socket_con_info.socket_state = SOCKET_STATUS_IDLE;
//             sys_info.paltform_connect = 0;
//             break;
//         }
//     }
// }

static void socket_disconnect_handle(uint8_t disconnect_type) 
{
    static uint16_t socket_disconnect_delay = 1;
    if(disconnect_type == 1){
        def_rtos_task_sleep_s(socket_disconnect_delay);
        socket_disconnect_delay = socket_disconnect_delay << 1;
        if(socket_disconnect_delay > 512) {
            cat1_reset_reson_save(NET_RESET_MQTT_DISCONNECT_FAIL);
            sys_reset();
        }
    } else {
        socket_disconnect_delay = 1;
    }
    
}
/*
static void iot_server_socket_state_machine(void)
{
    static uint8_t connect_count = 0;
    if(pdp_active_info.pdp_is_active == 1 && sys_info.ota_flag == 0) {
        switch(socket_con_info.socket_state) {
            case SOCKET_STATUS_IDLE:
                LOG_I("SOCKET_STATUS_IDLE");
                sys_info.paltform_connect = 0;
          //      rtc_event_unregister(NET_HEART_EVENT);
                if(socket_con_info.socket_fd > 0) {
                    close(socket_con_info.socket_fd);
                }
                socket_con_info.socket_fd = -1;
                iot_socket_data_init();
                connect_count = 0;
                socket_con_info.socket_state = SOCKET_STATUS_CREATE;
            break;
            case SOCKET_STATUS_CREATE:
                LOG_I("SOCKET_STATUS_CREATE");
                iot_socket_create();
            break;
            case SOCKET_STATUS_CONNECTTING:
                LOG_I("SOCKET_STATUS_CONNECTTING");
                iot_socket_connect();
                if(connect_count++ >= 100) {
                    def_rtos_task_sleep_ms(5000);
                    sys_reset();
                }
            break;
            case SOCKET_STATUS_DATA_PROC:
                LOG_I("SOCKET_STATUS_DATA_PROC");
                sys_info.paltform_connect = 1;
                socket_data_process(); 
            break;
            case SOCKET_STATUS_DISCONNECT:
                LOG_I("SOCKET_STATUS_DISCONNECT");
                socket_disconnect_handle();
                socket_con_info.socket_state = SOCKET_STATUS_IDLE;
            break;
        }
    }
} */




static void iot_mqtt_connect_result_cb(mqtt_client_t *client, void *arg, mqtt_connection_status_e status)
{
    LOG_I("status:%d", status);
    if(status == 0) {
        mqtt_con_info.mqtt_connected = 1;
    }
}

static void iot_mqtt_state_exception_cb(mqtt_client_t *client)
{
    LOG_E("mqtt session abnormal disconnect");
    mqtt_con_info.mqtt_connected = 0;
    ql_rtos_semaphore_release(mqtt_con_info.mqtt_disconnet_sem_t);

}

static void iot_mqtt_requst_result_cb(mqtt_client_t *client, void *arg,int err)
{
    mqtt_con_info.sub_res = 1;
}

static void iot_mqtt_pub_result_cb(mqtt_client_t *client, void *arg,int err)
{
    LOG_I("err:%d", err);
}

void iot_mqtt_public(const uint8_t *data, uint16_t len)
{
    if(sys_info.paltform_connect == 0) {
        return;
    }
    LOG_I("pub_topic:%s", mqtt_con_info.pub_topic);
    debug_data_printf("pub_data", data, len);
    ql_mqtt_publish(&mqtt_con_info.mqtt_cli, mqtt_con_info.pub_topic, (void *)data, len, sys_config.mqtt_qos, 1, iot_mqtt_pub_result_cb, NULL);
}

static void iot_mqtt_inpub_data_cb(mqtt_client_t *client, void *arg, int pkt_id, const char *topic, const unsigned char *payload, unsigned short payload_len)
{
	LOG_I("sub_topic: %s", topic);
	LOG_I("payload: %s, payload_len:%d", payload, payload_len);
    week_time("net", 30); 
    net_engwe_data_parse((uint8_t *)payload, payload_len);
}

#define  MQTT_SSL_ENABLE   0
void iot_mqtt_state_machine()
{
    static uint32_t mqtt_bind_time = 1;
    static uint32_t mqtt_client_init_time = 1;
    static uint32_t mqtt_connect_ret_time = 1;
    static uint32_t mqtt_client_sub_time = 1;
    static uint64_t mqtt_connect_timeout = 0;
    static uint64_t mqtt_disconnect_timeout = 0;
    static uint64_t mqtt_sub_timeout = 0;
//    static uint16_t mqtt_count = 0;
    static int ret = MQTTCLIENT_SUCCESS;
    char will_msg[128] = {0};
    if(pdp_active_info.pdp_is_active == 1) {
        switch(mqtt_con_info.state) {
            case MQTT_BIND_SIM_AND_PROFILE:
                week_time("net", -1); 
                sys_info.paltform_connect = 0;
                ret = ql_mqtt_client_deinit(&mqtt_con_info.mqtt_cli);
                LOG_I("ret:%d", ret);
                if(QL_DATACALL_SUCCESS != ql_bind_sim_and_profile(0, 1, &mqtt_con_info.sim_cid)) {
                    LOG_E("MQTT_BIND_SIM_AND_PROFILE FAIL");
                    def_rtos_task_sleep_s(mqtt_bind_time);
                    mqtt_bind_time = mqtt_bind_time << 1;
                    if(mqtt_bind_time > 64) {
                        cat1_reset_reson_save(NTE_RESET_MQTT_BIND_SIM_AND_PROFILE_FAIL);
                        MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
                     //   sys_reset();
                    }
                } else {
                    mqtt_bind_time = 1;
                     mqtt_con_info.state = MQTT_CLIENT_INIT;
                    LOG_I("MQTT_BIND_SIM_AND_PROFILE SUCCESS");
                }
            break;
            case MQTT_CLIENT_INIT:
                LOG_I("MQTT_CLIENT_INIT");
                if(ql_mqtt_client_init(&mqtt_con_info.mqtt_cli, mqtt_con_info.sim_cid) != MQTTCLIENT_SUCCESS) {
                    LOG_E("MQTT_CLIENT_INIT FAIL:%d", mqtt_client_init_time);        
                    def_rtos_task_sleep_s(mqtt_client_init_time);
                    mqtt_client_init_time = mqtt_client_init_time << 1;
                    if(mqtt_client_init_time > 64) {
                     //   sys_reset();
                        cat1_reset_reson_save(NET_RESET_MQTT_CLIENT_INIT_FAIL);
                        MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
                    }
                } else {
                    mqtt_client_init_time = 1;
                    mqtt_con_info.state = MQTT_CONNECT;
                    LOG_I("MQTT_CLIENT_INIT SUCCESS");
                }  
            break;
            case MQTT_CONNECT:
                LOG_I("MQTT_CONNECT");
                sprintf(will_msg, "{\"clientId\":\"%s/%s\",\"imei\":\"%s\"}",gsm_info.imei, ip4_adr_str, gsm_info.imei);
                LOG_I("will_msg:%s", will_msg);
                mqtt_con_info.mqtt_connected = 0;
                mqtt_con_info.client_info.clean_session = 1;
                mqtt_con_info.client_info.pkt_timeout = 5;
                mqtt_con_info.client_info.retry_times = 3;
                mqtt_con_info.client_info.keep_alive = 60*5;
                mqtt_con_info.client_info.will_retain = 0;
                if(sys_config.mqtt_will_en == 1) {
                    mqtt_con_info.client_info.will_topic = sys_config.mqtt_will_topic;
                    mqtt_con_info.client_info.will_msg = will_msg;
                    mqtt_con_info.client_info.will_qos = sys_config.mqtt_qos;
                } else {
                    mqtt_con_info.client_info.will_topic = NULL;
                    mqtt_con_info.client_info.will_msg = NULL;
                    mqtt_con_info.client_info.will_qos = 0;
                }
                mqtt_con_info.client_info.client_id = gsm_info.imei;
                LOG_I("ID:%s", mqtt_con_info.client_info.client_id);
                mqtt_con_info.client_info.client_user = sys_config.mqtt_client_user;
                mqtt_con_info.client_info.client_pass = sys_config.mqtt_client_pass;
                mqtt_con_info.client_info.ssl_cfg = NULL;
                #if defined(MQTT_SSL_ENABLE) && (MQTT_SSL_ENABLE == 1)
                    struct mqtt_ssl_config_t quectel_ssl_cfg = {
                    .ssl_ctx_id = 1,
                     #if USE_CRT_BUFFER   
                    .verify_level = MQTT_SSL_VERIFY_NONE,
                    .cacert_path = "UFS:dev-mqtt.engweapp.cn.crt",
                    .client_cert_path = NULL,
                    .client_key_path = NULL,
                    .client_key_pwd = NULL,
                    #else
                    .verify_level = MQTT_SSL_VERIFY_NONE,
                    .cacert_path = NULL,
                    .client_cert_path = NULL,
                    .client_key_path = NULL,
                    #endif

                    
                    .ssl_version = QL_SSL_VERSION_ALL,
                    .sni_enable = 0,
                    .ssl_negotiate_timeout = QL_SSL_NEGOTIATE_TIME_DEF,
                    .ignore_invalid_certsign = 0,
                    .ignore_multi_certchain_verify = 0,
                    .ignore_certitem = MBEDTLS_X509_BADCERT_NOT_TRUSTED|MBEDTLS_X509_BADCERT_EXPIRED|MBEDTLS_X509_BADCERT_FUTURE,
                    };
                    mqtt_con_info.client_info.ssl_cfg = &quectel_ssl_cfg;
                #endif
                sprintf(mqtt_con_info.pub_topic, "%s%s", sys_config.mqtt_pub_topic, gsm_info.imei);
                if(mqtt_con_info.url == NULL) {
                    mqtt_con_info.url = malloc(256);
                    memset(mqtt_con_info.url, 0, 256);
                    sprintf(mqtt_con_info.url, "%s:%lu", sys_config.ip, sys_config.port);
                }
               
                LOG_I("MQTT USER:%s", mqtt_con_info.client_info.client_user);
                LOG_I("MQTT PASS:%s", mqtt_con_info.client_info.client_pass);
                LOG_I("MQTT URL:%s", mqtt_con_info.url);
                ret = ql_mqtt_connect(&mqtt_con_info.mqtt_cli, mqtt_con_info.url, iot_mqtt_connect_result_cb, NULL,\
                 (const struct mqtt_connect_client_info_t *)&mqtt_con_info.client_info, iot_mqtt_state_exception_cb);
                mqtt_connect_timeout = def_rtos_get_system_tick();
                mqtt_con_info.state = MQTT_CONNECT_RES;
            break;
            case MQTT_CONNECT_RES:
                LOG_I("ret:%d, connect:%d", ret,mqtt_con_info.mqtt_connected);
                if(ret == MQTTCLIENT_WOUNDBLOCK && mqtt_con_info.mqtt_connected == 1) {
                    // if(mqtt_con_info.url) {
                    //     free(mqtt_con_info.url);
                    // }
                    mqtt_con_info.state = MQTT_SET_INPUB;
                //    mqtt_count= 0;
                    mqtt_connect_ret_time = 1;
                    LOG_I("MQTT_CONNECT SUCCESS");
                } else if(def_rtos_get_system_tick() - mqtt_connect_timeout > 12000) {
                    LOG_E("MQTT CONNECT FAIL:%d", mqtt_connect_ret_time);
                    mqtt_con_info.state = MQTT_BIND_SIM_AND_PROFILE;
                    def_rtos_task_sleep_s(mqtt_connect_ret_time);
                    mqtt_connect_ret_time = mqtt_connect_ret_time << 1;
                    if(mqtt_connect_ret_time > 512) {
                        mqtt_connect_ret_time = 1;
                    //    if(mqtt_count++ == 3) {
                            cat1_reset_reson_save(NET_RESET_MQTT_CONNECT_SERVER_FAIL);
                            MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
                       //     sys_reset();
                    //    }
                    } 
                }
            break;
            case MQTT_SET_INPUB:
                LOG_I("MQTT_SET_INPUB");
                ql_mqtt_set_inpub_callback(&mqtt_con_info.mqtt_cli, iot_mqtt_inpub_data_cb, NULL);
                sys_info.paltform_connect = 1;
                mqtt_con_info.state = MQTT_SUB;
                mqtt_disconnect_timeout = def_rtos_get_system_tick();
            break;
            case MQTT_SUB:
                mqtt_con_info.sub_res = 0;
                if(mqtt_con_info.sub_topic == NULL) {
                    mqtt_con_info.sub_topic = malloc(256);
                    memset(mqtt_con_info.sub_topic, 0, 256);
                }
                sprintf(mqtt_con_info.sub_topic, "%s%s", sys_config.mqtt_sub_topic, gsm_info.imei);
                LOG_I("MQTT SUB TOPIC:%s", mqtt_con_info.sub_topic);
                ret = ql_mqtt_sub_unsub(&mqtt_con_info.mqtt_cli, mqtt_con_info.sub_topic, sys_config.mqtt_qos, iot_mqtt_requst_result_cb, NULL, 1);
                mqtt_sub_timeout = def_rtos_get_system_tick();
                mqtt_con_info.state = MQTT_SUB_RES;
            break;
            case MQTT_SUB_RES:
                if(ret == MQTTCLIENT_WOUNDBLOCK && mqtt_con_info.sub_res == 1) {
                    mqtt_client_sub_time = 0;
                    // if(mqtt_con_info.sub_topic) {
                    //     free(mqtt_con_info.sub_topic);
                    // }
                    mqtt_con_info.state = MQTT_CONNECT_DET;
                    net_engwe_signed();
                    net_engwe_cmd_push(CONFIG_FEEDBACK_UP, 0x00000400);
                //    检测OTA成功 版本不一样说明更新成功
                    LOG_I("last_ver:%s, cur_ver:%s", sys_config.soft_ver, SOFTVER);
                    if(sys_param_set.ota_flag == 1){
                        if(strcmp(sys_config.soft_ver, SOFTVER) != 0) {
                            LOG_I("OTA SUCCESS");
                            net_engwe_fota_state_push(FOTA_UPDATE_SUCCESS);
                            memset(sys_config.soft_ver, 0, sizeof(sys_config.soft_ver));
                            memcpy(sys_config.soft_ver, SOFTVER, strlen(SOFTVER));
                            SETBIT(sys_set_var.sys_updata_falg, SYS_CONFIG_SAVE);
                        } else {
                            LOG_I("OTA FAIL");
                            net_engwe_fota_state_push(FOTA_VER_ERROR);
                        }
                        sys_param_set.ota_flag = 0;
                        SETBIT(sys_set_var.sys_updata_falg, SYS_SET_SAVE);
                    }  
                } else if(def_rtos_get_system_tick() - mqtt_sub_timeout > 12000) {  
                    LOG_E("mqtt sub fail:%d", mqtt_client_sub_time);
                    def_rtos_task_sleep_s(mqtt_client_sub_time);
                    mqtt_client_sub_time =  mqtt_client_sub_time << 1;
                    mqtt_con_info.state = MQTT_BIND_SIM_AND_PROFILE;
                    if(mqtt_client_sub_time > 512) {
                        mqtt_client_sub_time = 0;
                    //    if(mqtt_count++ > 3) {
                    //        sys_reset();
                        cat1_reset_reson_save(NET_RESET_MQTT_SUB_FAIL);
                        MCU_CMD_MARK(CMD_CAT_REPOWERON_INDEX);
                    //    }
                    }
                }
            break;
            case MQTT_CONNECT_DET:
                LOG_I("MQTT_CONNECT_DET");
                week_time("net", 30); 
                ql_rtos_semaphore_wait(mqtt_con_info.mqtt_disconnet_sem_t, QL_WAIT_FOREVER);
                if(def_rtos_get_system_tick() - mqtt_disconnect_timeout < 60000) {      /*防止掉线频繁连接*/
                    socket_disconnect_handle(1);
                } else {
                    socket_disconnect_handle(0);
                }
                if(hal_get_at_ceer(&gsm_info.at_ceer_p) != 0) {
                    gsm_info.at_ceer_p = 0xffff; //无效
                }
                LOG_E("mqtt connect is fail");
                mqtt_con_info.state = MQTT_BIND_SIM_AND_PROFILE;
                if(pdp_active_info.pdp_state != PDP_NOT_ACTIVATED) {
                    sys_info.net_disconnect_res = NET_DISCON_SERVICE_DISCON;
                }
            break;
        }
    }
}


void iot_mqtt_init()
{
    ql_rtos_semaphore_create(&mqtt_con_info.mqtt_disconnet_sem_t, 0);
    mqtt_con_info.url = NULL;
    mqtt_con_info.sub_topic = NULL;
    mqtt_con_info.state = MQTT_BIND_SIM_AND_PROFILE;
}

void net_socket_thread(void *param)
{
    iot_mqtt_init();
    app_system_wait_active();
    while (1)
    {
        LOG_I("IS RUN");
        iot_mqtt_state_machine();
        def_rtos_task_sleep_ms(200);
     //   def_rtos_task_sleep_s(1);
     //   iot_server_socket_state_machine();
    }
    def_rtos_task_delete(NULL);
}


