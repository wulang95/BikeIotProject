#include "hal_drv_net.h"
#include "ql_api_sim.h"
#include "ql_api_dev.h"
#include "ql_api_nw.h"
#include "ql_power.h"

#define DBG_TAG         "hal_drv_net"

#ifdef HAL_DRV_NET_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"



void hal_drv_get_imei(char *data, uint16_t len)
{
    ql_dev_get_imei(data, (size_t)len, 0);
}

void hal_drv_get_iccid(char *data, uint16_t len)
{
    ql_sim_get_iccid(0, data, (size_t)len);
}

void hal_drv_get_signal(uint8_t *csq, int *rssi)
{
     ql_nw_signal_strength_info_s pt_info;
    ql_nw_get_csq(0, csq);
    ql_nw_get_signal_strength(0, &pt_info);
    *rssi = pt_info.rssi;
}

uint8_t hal_drv_get_net_register_sta()
{
    uint8_t res = 0;
    ql_nw_reg_status_info_s nw_info = {0};
    ql_nw_get_reg_status(0, &nw_info);
    if((QL_NW_REG_STATE_HOME_NETWORK != nw_info.data_reg.state) && (QL_NW_REG_STATE_ROAMING != nw_info.data_reg.state)) {
        res = 1;
    }
    return res;
}

void hal_drv_set_data_call_asyn_mode(uint8_t mode)
{
    ql_set_data_call_asyn_mode(0, 1, mode);
}

uint8_t hal_drv_get_data_call_res(char *ip4_adr)
{
    uint8_t res = 0;
    ql_data_call_info_s info ={0};
    ql_get_data_call_info(0, 1, &info);
    LOG_I("info.profile_idx: %d, info.ip_version: %d", info.profile_idx, info.ip_version);
	LOG_I("info->v4.state: %d, info.v6.state: %d", info.v4.state, info.v6.state);
    if(info.v4.state)
    {
		LOG_I("info.v4.addr.ip: %s", ip4addr_ntoa(&(info.v4.addr.ip)));
		LOG_I("info.v4.addr.pri_dns: %s", ip4addr_ntoa(&(info.v4.addr.pri_dns)));
		LOG_I("info.v4.addr.sec_dns: %s", ip4addr_ntoa(&(info.v4.addr.sec_dns)));
        sprintf(ip4_adr, "%s", ip4addr_ntoa(&(info.v4.addr.ip)));
    }
    if(info.v4.state == 1){
        res = 1;
    }
    return res;
}

void hal_drv_start_data_call(char *apn)
{
    ql_start_data_call(0, 1, QL_PDP_TYPE_IP, apn, NULL, NULL, 0);
}

uint8_t hal_drv_get_cpin()
{
    uint8_t res = 0;
    ql_sim_status_e card_status;
    ql_sim_get_card_status(0, &card_status);
    LOG_I("card_status:%d", card_status);
    if(card_status == QL_SIM_STATUS_READY) {
        res = 1;
    }
    return res;
}

void hal_dev_set_c_fun(uint8_t fun, uint8_t rst)
{
     ql_dev_set_modem_fun(fun, rst, 0);
}

uint8_t hal_dev_get_c_fun()
{
    uint8_t fun;
    ql_dev_get_modem_fun(&fun, 0);
    return fun;
}

void sys_reset()
{
    ql_power_reset(RESET_NORMAL);
}

void hal_drv_set_dns_addr()
{
    ql_datacall_dns_info_s dns_pri = {0};
	ql_datacall_dns_info_s dns_sec = {0};

    ql_datacall_get_dns_addr(0, 1, &dns_pri, &dns_sec);
    memset(&dns_pri, 0x00, sizeof(ql_datacall_dns_info_s));
    dns_pri.type = QL_PDP_TYPE_IP;
	dns_sec.type = QL_PDP_TYPE_IP;
    ip4addr_aton("114.114.114.114", &(dns_pri.ip4));
	ip4addr_aton("8.8.8.8", &(dns_sec.ip4));
	ql_datacall_set_dns_addr(0, 1, &dns_pri, &dns_sec);
}

