#include "app_system.h"
#include "hal_drv_net.h"
#include "sockets.h"
#include "lwip/ip_addr.h"
#include "lwip/ip6_addr.h"

#include "lwip/netdb.h"
#include "lwip/netif.h"
#include "lwip/inet.h"
#include "lwip/tcp.h"

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

typedef enum
{
    PDP_NOT_ACTIVATED = 0,
    PDP_CONDITIONS_CHECK ,
    PDP_START_ACTIVATION ,
    PDP_ACTIVATION_SUCCESS ,
    PDP_ACTIVATION_FAILURE ,
    PDP_ACTIVE_DETECT ,
} pdp_active_state_type;


typedef enum
{
    SOCKET_STATUS_IDLE = 0,
    SOCKET_STATUS_CREATE = 1,
    SOCKET_STATUS_CONNECTTING = 2,
    SOCKET_STATUS_DATA_PROC = 7,
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
} PDP_ACTIVE_INFO_STU;

PDP_ACTIVE_INFO_STU pdp_active_info;
SOCKET_CON_INFO_STU socket_con_info;

void net_control_init()
{
    memset(&gsm_info, 0, sizeof(gsm_info));
    memset(&pdp_active_info, 0, sizeof(pdp_active_info));
    memset(&socket_con_info, 0, sizeof(socket_con_info));
    pdp_active_info.pdp_is_active = 0;
    pdp_active_info.pdp_state = PDP_NOT_ACTIVATED;
    socket_con_info.socket_fd = -1;
    socket_con_info.socket_state = SOCKET_STATUS_IDLE;
    LOG_I("net_control_init is ok");
}

void net_update_singal_csq()
{
    hal_drv_get_signal(&gsm_info.csq);
}

static void pdp_active_state_machine(void)
{
    uint8_t c_fun;
    uint8_t cpin;
    uint8_t net_reg;
    static int64_t check_csq_timeout = 0;
    static int64_t check_pdp_timeout = 0;
    switch (pdp_active_info.pdp_state) {
        case PDP_NOT_ACTIVATED:
            LOG_I("PDP_NOT_ACTIVATED");
            sys_info.pdp_reg = 0;
            pdp_active_info.pdp_is_active = 0;
            pdp_active_info.pdp_state  = PDP_CONDITIONS_CHECK;
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
                hal_drv_set_data_call_asyn_mode(1);
                hal_drv_start_data_call(sys_config.apn); 
            }
            if((def_rtos_get_system_tick() - check_csq_timeout)/1000 > 30*60) {
                LOG_E("sys_reset...");
                def_rtos_task_sleep_ms(5000);
                sys_reset();
            }
        break;
        case PDP_START_ACTIVATION:
            LOG_I("PDP_START_ACTIVATION");
            memset(ip4_adr_str, 0, sizeof(ip4_adr_str));
            if(hal_drv_get_data_call_res(ip4_adr_str)) {
                pdp_active_info.pdp_state  = PDP_ACTIVATION_SUCCESS;
            }
            if((check_pdp_timeout - def_rtos_get_system_tick())/1000 > 5*60) {
               pdp_active_info.pdp_state  = PDP_ACTIVATION_FAILURE;
            }
        break;
        case PDP_ACTIVATION_SUCCESS:
            LOG_I("PDP_ACTIVATION_SUCCESS");
            pdp_active_info.pdp_state  = PDP_ACTIVE_DETECT;
            pdp_active_info.pdp_is_active = 1;
            sys_info.pdp_reg = 1; 
        break;
        case PDP_ACTIVATION_FAILURE:
            hal_dev_set_c_fun(MIN_FUN, 0);
            LOG_I("PDP_ACTIVATION_FAILURE");
            pdp_active_info.pdp_state  = PDP_NOT_ACTIVATED;
        break;
        case PDP_ACTIVE_DETECT:
            LOG_I("PDP_ACTIVE_DETECT");
            hal_drv_pdp_detect_block();
            pdp_active_info.pdp_state = PDP_NOT_ACTIVATED;
        break;
    }
} 

void pdp_active_thread(void *param)
{
    hal_drv_data_call_register_init();
    while(1) {
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
        LOG_E("socket is erro");
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
        LOG_I("select");
        fd_changed = select(socket_con_info.socket_fd+1, &read_fds, NULL, &exp_fds, NULL);
        LOG_I("fd_changed:%d", fd_changed);
        if(fd_changed > 0) {
            if(FD_ISSET(socket_con_info.socket_fd, &read_fds)){
                FD_CLR(socket_con_info.socket_fd, &read_fds);
                memset(buf, 0, sizeof(buf));
                len = read(socket_con_info.socket_fd, buf, 512);
                if(len <= 0) {
                    socket_con_info.socket_state = SOCKET_STATUS_IDLE;
                    sys_info.paltform_connect = 0;
                    LOG_E("socket is error");
                    break;
                }
                FD_SET(socket_con_info.socket_fd, &read_fds);
                debug_data_printf("net_recv", buf, len);
                net_recv_data_parse(buf, len);
            } else if(FD_ISSET(socket_con_info.socket_fd, &exp_fds)) {
                FD_CLR(socket_con_info.socket_fd, &exp_fds);
                socket_con_info.socket_state = SOCKET_STATUS_IDLE;
                sys_info.paltform_connect = 0;
                LOG_E("socket is error");
                break;
            } else {
                LOG_E("socket is error");
                socket_con_info.socket_state = SOCKET_STATUS_IDLE;
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
        }
    }
}

void net_socket_thread(void *param)
{
    while (1)
    {
        iot_server_socket_state_machine();
        def_rtos_task_sleep_ms(100);
    }
    def_rtos_task_delete(NULL);
}



