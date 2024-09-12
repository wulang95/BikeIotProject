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

static char ip4_adr_str[16];

typedef enum
{
    PDP_NOT_ACTIVATED = 0,
    PDP_CONDITIONS_CHECK = 1,
    PDP_START_ACTIVATION = 2,
    PDP_ACTIVATION_SUCCESS = 3,
    PDP_ACTIVATION_FAILURE = 4,
    JUDGMENT_ACTIVATION = 5,
} pdp_active_state_type;


typedef enum
{
    SOCKET_STATUS_IDLE = 0,
    SOCKET_STATUS_CREATE = 1,
    SOCKET_STATUS_CONNECTTING = 2,
    SOCKET_STATUS_LOGIN = 3,
    SOCKET_STATUS_LOGIN_OK = 4,
    SOCKET_STATUS_LOGIN_FAIL = 5,
    JUDGMENT_LOGIN_STATUS = 6,
    SOCKET_STATUS_DATA_PROC = 7,
} socket_state_type;

typedef struct {
    char imei[20];
    char iccid[24];
    uint8_t csq;
    int rssi;
    uint8_t online;  
}GSM_INFO_STU;

GSM_INFO_STU gsm_info;

static uint8_t socket_en;
static void pdp_active_state_machine(void)
{
    uint8_t c_fun;
    uint8_t cpin;
    static int64_t check_csq_timeout = 0;
    static int64_t check_pdp_timeout = 0;
    static pdp_active_state_type pdp_state = PDP_NOT_ACTIVATED;
    if(socket_en == 1) return;
    switch (pdp_state) {
        case PDP_NOT_ACTIVATED:
            LOG_I("PDP_NOT_ACTIVATED");
            pdp_state = PDP_CONDITIONS_CHECK;
            check_csq_timeout = def_rtos_get_system_tick();
            socket_en = 0;
        break;
        case PDP_CONDITIONS_CHECK:
            LOG_I("PDP_CONDITIONS_CHECK");
            hal_drv_get_imei(gsm_info.imei, sizeof(gsm_info.imei));
            c_fun = hal_dev_get_c_fun();
            if(c_fun != ALL_FUN) {
                hal_dev_set_c_fun(ALL_FUN, 0);
            }
            cpin = hal_drv_get_cpin();
            if(cpin) {
                hal_drv_get_iccid(gsm_info.iccid, sizeof(gsm_info.iccid));
                hal_drv_get_signal(&gsm_info.csq, &gsm_info.rssi);
            }
            LOG_I("CSQ:%d, rssi:%d", gsm_info.csq, gsm_info.rssi);
            LOG_I("IMEI:%s", gsm_info.imei);
            LOG_I("ICCID:%s", gsm_info.iccid);
            if(gsm_info.csq > 10 && gsm_info.csq != 99 && cpin == 1 && c_fun == ALL_FUN) {
                pdp_state = PDP_START_ACTIVATION;
                check_pdp_timeout = def_rtos_get_system_tick();
            }
            if((check_csq_timeout - def_rtos_get_system_tick())/1000 > 30*60) {
                def_rtos_task_sleep_ms(5000);
                sys_reset();
            }
        break;
        case PDP_START_ACTIVATION:
            LOG_I("PDP_START_ACTIVATION");
            memset(ip4_adr_str, 0, sizeof(ip4_adr_str));
            hal_drv_start_data_call(sys_config.apn);
            if(hal_drv_get_data_call_res(ip4_adr_str)) {
                pdp_state = PDP_ACTIVATION_SUCCESS;
            }
            if((check_pdp_timeout - def_rtos_get_system_tick())/1000 > 5*60) {
               pdp_state = PDP_ACTIVATION_FAILURE;
            }
        break;
        case PDP_ACTIVATION_SUCCESS:
            LOG_I("PDP_ACTIVATION_SUCCESS");
            pdp_state = JUDGMENT_ACTIVATION;
        break;
        case PDP_ACTIVATION_FAILURE:
            hal_dev_set_c_fun(MIN_FUN, 0);
            LOG_I("PDP_ACTIVATION_FAILURE");
            pdp_state = PDP_NOT_ACTIVATED;
        break;
        case JUDGMENT_ACTIVATION:
            socket_en = 1;
            LOG_I("JUDGMENT_ACTIVATION");
        break;
    }

} 

static int socket_fd;

static void jbd_server_socket_state_machine(void)
{
    int ret;
    struct sockaddr_in local4, server_ipv4;	
 //   struct addrinfo *pres = NULL;	
//	struct addrinfo *temp = NULL;
    static int64_t check_socket_connect_timeout = 0;
   // static int64_t check_login_timeout = 0;
    static socket_state_type socket_state = SOCKET_STATUS_IDLE;
    if(socket_en) {
        switch(socket_state) {
            case SOCKET_STATUS_IDLE:
                LOG_I("SOCKET_STATUS_IDLE");
                socket_state = SOCKET_STATUS_CREATE;
                check_socket_connect_timeout = def_rtos_get_system_tick();
            break;
            case SOCKET_STATUS_CREATE:
                LOG_I("SOCKET_STATUS_CREATE");
                hal_drv_set_dns_addr();
                memset(&local4, 0x00, sizeof(struct sockaddr_in));
	            local4.sin_family = AF_INET;
	            local4.sin_port = 0;
	            inet_aton(ip4_adr_str, &local4.sin_addr);
                socket_fd = socket(AF_INET, SOCK_STREAM, 0);
                if(socket_fd < 0) {
                    LOG_E("socket is error");
                    break;
                }	
                ret = bind(socket_fd,(struct sockaddr *)&local4,sizeof(struct sockaddr));
                if(ret < 0) {
                    close(socket_fd);
                    LOG_E("bind is error");
                    break;
                }
                socket_state = SOCKET_STATUS_CONNECTTING;
            break;
            case SOCKET_STATUS_CONNECTTING:
                LOG_I("SOCKET_STATUS_CONNECTTING");
                memset(&server_ipv4, 0x00, sizeof(struct sockaddr_in));
                inet_aton(sys_config.ip, &server_ipv4.sin_addr);
                server_ipv4.sin_family = AF_INET;
			    server_ipv4.sin_port = htons(sys_config.port);
                ret = connect(socket_fd, (struct sockaddr *)&server_ipv4, sizeof(server_ipv4));	
                if(ret == 0) {
                    socket_state = SOCKET_STATUS_LOGIN;
                } else {
                    LOG_E("connect is error");
                    break;
                }
                if(def_rtos_get_system_tick() - check_socket_connect_timeout > 12*60*60*1000) {
                    def_rtos_task_sleep_ms(5000);
                    sys_reset();
                }
            break;
            case SOCKET_STATUS_LOGIN:
                LOG_I("SOCKET_STATUS_LOGIN");
                socket_state = SOCKET_STATUS_LOGIN_OK;
            break;
            case SOCKET_STATUS_LOGIN_OK:
                LOG_I("SOCKET_STATUS_LOGIN_OK");
                socket_state = SOCKET_STATUS_LOGIN_FAIL;
            break;
            case SOCKET_STATUS_LOGIN_FAIL:
                LOG_I("SOCKET_STATUS_LOGIN_FAIL");
                socket_state = JUDGMENT_LOGIN_STATUS;
            break;
            case JUDGMENT_LOGIN_STATUS:
                LOG_I("JUDGMENT_LOGIN_STATUS");
                socket_state = SOCKET_STATUS_DATA_PROC;
            break;
            case SOCKET_STATUS_DATA_PROC:
                LOG_I("SOCKET_STATUS_DATA_PROC");
            break;
        }
    }
}
void net_connect_thread(void *param)
{
    while (1)
    {
        pdp_active_state_machine();
        jbd_server_socket_state_machine();
        def_rtos_task_sleep_ms(100);
    }
    def_rtos_task_delete(NULL);
    
}



