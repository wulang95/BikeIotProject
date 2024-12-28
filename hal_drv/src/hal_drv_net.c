#include "hal_drv_net.h"
#include "rtos_port_def.h"
#include "ql_api_sim.h"
#include "ql_api_dev.h"
#include "ql_api_nw.h"
#include "ql_power.h"
#include "ql_osi_def.h"


#define DBG_TAG         "hal_drv_net"

#ifdef HAL_DRV_NET_DEBUG
#define DBG_LVL    DBG_LOG
#else
#define DBG_LVL   DBG_INFO
#endif

#include "log_port.h"

def_rtos_sem_t pdp_active_det_sem;

void hal_drv_get_imei(char *data, uint16_t len)
{
    ql_dev_get_imei(data, (size_t)len, 0);
}

void hal_drv_get_iccid(char *data, uint16_t len)
{
    ql_sim_get_iccid(0, data, (size_t)len);
}

void hal_drv_get_signal(uint8_t *csq)
{
    ql_nw_get_csq(0, csq);
}

uint8_t hal_drv_get_net_register_sta()
{
    uint8_t res = 1;
    ql_nw_reg_status_info_s nw_info = {0};
    ql_nw_get_reg_status(0, &nw_info);
    LOG_I("nw_info.data_reg.state:%d", nw_info.data_reg.state);
    if((QL_NW_REG_STATE_HOME_NETWORK != nw_info.data_reg.state) && (QL_NW_REG_STATE_ROAMING != nw_info.data_reg.state)) {
        res = 0;
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
    uint8_t res;
    res = ql_start_data_call(0, 1, QL_PDP_TYPE_IP, apn, NULL, NULL, 0);
    if(res != 0) {
        LOG_E("error");
    }
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

void hal_drv_set_dns_addr(char *dns)
{
    ql_datacall_dns_info_s dns_pri = {0};
	ql_datacall_dns_info_s dns_sec = {0};

    ql_datacall_get_dns_addr(0, 1, &dns_pri, &dns_sec);
    memset(&dns_pri, 0x00, sizeof(ql_datacall_dns_info_s));
    dns_pri.type = QL_PDP_TYPE_IP;
	dns_sec.type = QL_PDP_TYPE_IP;
    ip4addr_aton(dns, &(dns_pri.ip4));
	ip4addr_aton("8.8.8.8", &(dns_sec.ip4));
	ql_datacall_set_dns_addr(0, 1, &dns_pri, &dns_sec);
}


void ql_datacall_ind_callback(uint8_t sim_id, unsigned int ind_type, int profile_idx, bool result, void *ctx)
{
    uint8_t pdp_state = 0xff;
    static uint8_t last_pdp_state = 0xff;
    LOG_I("nSim = %d, profile_idx=%d, ind_type=0x%x, result=%d", sim_id, profile_idx, ind_type, result);
    if((profile_idx < PROFILE_IDX_MIN) || (profile_idx > PROFILE_IDX_MAX))
    {
        return;
    }
    switch(ind_type)
    {
        case QUEC_DATACALL_ACT_RSP_IND:  //only received in asyn mode
        {
            pdp_state = (result == true)? QL_PDP_ACTIVED:QL_PDP_DEACTIVED;
            break;
        }
        case QUEC_DATACALL_DEACT_RSP_IND:  //only received in asyn mode
        {
            pdp_state = (result == true)? QL_PDP_DEACTIVED:QL_PDP_ACTIVED;
            break;
        }
        case QUEC_DATACALL_PDP_DEACTIVE_IND:  //received in both asyn mode and sync mode
        {
            pdp_state = QL_PDP_DEACTIVED;
            break;
        }
    }  
    if(pdp_state == QL_PDP_DEACTIVED && last_pdp_state == QL_PDP_ACTIVED) {
        def_rtos_smaphore_release(pdp_active_det_sem);
    }
    last_pdp_state = pdp_state;
}

void hal_drv_pdp_detect_block()
{
    def_rtos_semaphore_wait(pdp_active_det_sem, RTOS_WAIT_FOREVER);
}

void hal_drv_data_call_register_init()
{
    def_rtos_semaphore_create(&pdp_active_det_sem, 0);
    ql_datacall_register_cb(0, 1, ql_datacall_ind_callback, NULL);
}

NET_NW_INFO hal_drv_get_operator_info()
{
    NET_NW_INFO nw_info;
    ql_nw_operator_info_s oper_i;
    ql_nw_reg_status_info_s reg_info;
    ql_nw_signal_strength_info_s pt_info;
    ql_nw_mode_type_e nw_mode;
    uint8_t csq;
    ql_nw_get_operator_name(0, &oper_i);
    ql_nw_get_reg_status(0, &reg_info);
    ql_nw_get_signal_strength(0, &pt_info);
    ql_nw_get_mode(0, &nw_mode);
    ql_nw_get_csq(0, &csq);
    nw_info.mcc = atoi(oper_i.mcc);
    nw_info.mnc = atoi(oper_i.mnc);
    nw_info.lac = reg_info.data_reg.lac;
    nw_info.cid = reg_info.data_reg.cid;
    nw_info.net_state = (reg_info.data_reg.state == QL_NW_REG_STATE_HOME_NETWORK) ? 1:(reg_info.data_reg.state == QL_NW_REG_STATE_ROAMING)?0:0xff;
    nw_info.act = (reg_info.data_reg.act == QL_NW_ACCESS_TECH_GSM) ? 0:(reg_info.data_reg.act == QL_NW_ACCESS_TECH_E_UTRAN)?1:0xff;
    nw_info.csq = csq;
    nw_info.bit_error_rate = pt_info.bitErrorRate;
    // LOG_I("MCC:%s, MNC:%s", oper_i.mcc, oper_i.mnc);
    // LOG_I("lac:%d,cid:%d", reg_info.data_reg.lac, reg_info.data_reg.cid);
    // LOG_I("CSQ:%d, bitErrorRate:%d", pt_info.rssi, pt_info.bitErrorRate);
    // LOG_I("nw_neg:%d, act:%d", reg_info.data_reg.state, reg_info.data_reg.act);
    // LOG_I("nw_mode:%d", nw_mode);
    return nw_info;
}


