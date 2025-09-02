#include "hal_drv_net.h"
#include "rtos_port_def.h"
#include "ql_api_sim.h"
#include "ql_api_dev.h"
#include "ql_api_nw.h"
#include "ql_power.h"
#include "ql_osi_def.h"
#include "ql_ntp_client.h"
#include "ql_api_rtc.h"
#include "hal_virt_at.h"
#include <string.h>
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

void hal_drv_set_data_call_asyn_mode(int pdp_index, uint8_t mode)
{
    ql_set_data_call_asyn_mode(0, pdp_index, mode);
}

uint8_t hal_drv_get_data_call_res(int pdp_index, char *ip4_adr)
{
    uint8_t res = 0;
    ql_data_call_info_s info ={0};
    ql_get_data_call_info(0, pdp_index, &info);
    LOG_I("info.profile_idx: %d, info.ip_version: %d", info.profile_idx, info.ip_version);
	LOG_I("info->v4.state: %d, info.v6.state: %d", info.v4.state, info.v6.state);
    if(info.v4.state)
    {
		LOG_I("info.v4.addr.ip: %s", ip4addr_ntoa(&(info.v4.addr.ip)));
		LOG_I("info.v4.addr.pri_dns: %s", ip4addr_ntoa(&(info.v4.addr.pri_dns)));
		LOG_I("info.v4.addr.sec_dns: %s", ip4addr_ntoa(&(info.v4.addr.sec_dns)));
        if(ip4_adr) {
            sprintf(ip4_adr, "%s", ip4addr_ntoa(&(info.v4.addr.ip)));
        }
    }
    if(info.v4.state == 1){
        res = 1;
    }
    return res;
}

void hal_drv_start_data_call(int pdp_index, char *apn)
{
    uint8_t res;
    res = ql_start_data_call(0, pdp_index, QL_PDP_TYPE_IP, apn, NULL, NULL, 0);
    if(res != 0) {
        LOG_E("ql_start_data_call is error");
    }
}


void hal_drv_stop_data_call(int pdp_index)
{
    if(ql_stop_data_call(0, pdp_index) != 0){
        LOG_E("ql_stop_data_call is error");
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
    LOG_I("sys_reset...");
    def_rtos_task_sleep_s(5);
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

uint8_t hal_get_rsrp()
{
    uint8_t rsrp_t = 0;
    ql_nw_signal_strength_info_s pt_info;
    ql_nw_get_signal_strength(0, &pt_info);
    if(pt_info.rsrp <= -113) {      /*待确认*/
        rsrp_t = 99;
    } else {
        rsrp_t = (pt_info.rsrp+113)/2;
    }
    return rsrp_t;
}

NET_NW_INFO hal_drv_get_operator_info()
{
    NET_NW_INFO nw_info;
    ql_nw_operator_info_s oper_i;
    ql_nw_reg_status_info_s reg_info;
    ql_nw_signal_strength_info_s pt_info;
    uint8_t csq, d_len = 0,total_len = 0;
    char at_buf[64] = {0};
    static uint8_t last_band = 0xff;
    ql_nw_get_operator_name(0, &oper_i);
    ql_nw_get_reg_status(0, &reg_info);
    ql_nw_get_signal_strength(0, &pt_info);
    ql_nw_get_csq(0, &csq);
    nw_info.mcc = atoi(oper_i.mcc);
    nw_info.mnc = atoi(oper_i.mnc);
    nw_info.lac = reg_info.data_reg.lac;
    nw_info.cid = reg_info.data_reg.cid;
    nw_info.net_state = (reg_info.data_reg.state == QL_NW_REG_STATE_HOME_NETWORK) ? 1:(reg_info.data_reg.state == QL_NW_REG_STATE_ROAMING)?0:0xff;
    nw_info.act = (reg_info.data_reg.act == QL_NW_ACCESS_TECH_GSM) ? 0:(reg_info.data_reg.act == QL_NW_ACCESS_TECH_E_UTRAN)?1:0xff;
    nw_info.rsrp = pt_info.rsrp;
    LOG_I("rsrp:%d", pt_info.rsrp);
    nw_info.bit_error_rate = pt_info.bitErrorRate;
    hal_virt_flush_recv();
    hal_virt_at_write("AT+QNWINFO\r\n");
  //      hal_virt_at_write("AT+QENG=\"SERVINGCELL\"\r\n");
    d_len = hal_virt_at_read(at_buf, 64, 2000);
    total_len += d_len;
    if(total_len > 0) {
        d_len = hal_virt_at_recv_buf(&at_buf[total_len], 64 - d_len, 5000);
    }
    total_len += d_len;
    LOG_I("at_buf:%s", at_buf);
    if(strstr(at_buf, "LTE BAND 3") != NULL){
        nw_info.fre_band = 0x02;
    } else if(strstr(at_buf, "LTE BAND 1") != NULL) {
        nw_info.fre_band = 0x00;
    } else if(strstr(at_buf, "LTE BAND 20") != NULL) {
        nw_info.fre_band = 0x0C;
    } else if(strstr(at_buf, "LTE BAND 5") != NULL) {
        nw_info.fre_band = 0x04;
    } else if(strstr(at_buf, "LTE BAND 7") != NULL) {
        nw_info.fre_band = 0x06;
    } else if(strstr(at_buf, "LTE BAND 8") != NULL) {
        nw_info.fre_band = 0x07;
    } else if(strstr(at_buf, "LTE BAND 28") != NULL) {
        nw_info.fre_band = 0x0F;
    } else if(strstr(at_buf, "LTE BAND 2") != NULL) {
        nw_info.fre_band = 0x01;
    } else if(strstr(at_buf, "GSM 850") != NULL) {
        nw_info.fre_band = 0x17;
    } else if(strstr(at_buf, "GSM 900") != NULL) {
        nw_info.fre_band = 0x18;
    } else if(strstr(at_buf, "GSM 1800") != NULL) {
        nw_info.fre_band = 0x19;
    } else if(strstr(at_buf, "GSM 1900") != NULL) {
        nw_info.fre_band = 0x1A;
    } else {
        nw_info.fre_band = last_band;
    }
    LOG_I("at:%s", at_buf);
    last_band = nw_info.fre_band;
    // LOG_I("MCC:%s, MNC:%s", oper_i.mcc, oper_i.mnc);
    // LOG_I("lac:%d,cid:%d", reg_info.data_reg.lac, reg_info.data_reg.cid);
    // LOG_I("CSQ:%d, bitErrorRate:%d", pt_info.rssi, pt_info.bitErrorRate);
    // LOG_I("nw_neg:%d, act:%d", reg_info.data_reg.state, reg_info.data_reg.act);
    // LOG_I("nw_mode:%d", nw_mode);
    return nw_info;
}

static ntp_client_id  ntp_cli_id = 0;

static void ntp_result_cb(ntp_client_id cli_id, int result, struct tm *sync_time, void *arg)
{
    ql_rtc_time_t time;
	if(ntp_cli_id != cli_id)
		return;

	if(result == QL_NTP_SUCCESS){
		char time_str[256] = {0};

		snprintf(time_str, 256, "%04d/%02d/%02d,%02d:%02d:%02d",sync_time->tm_year + 1900, sync_time->tm_mon + 1, sync_time->tm_mday,
                       sync_time->tm_hour, sync_time->tm_min, sync_time->tm_sec);
		LOG_I("ntp sync time:%s", time_str);
	    time.tm_year = sync_time->tm_year+ 1900;
	    time.tm_mon  = sync_time->tm_mon + 1;
	    time.tm_mday = sync_time->tm_mday;
	    time.tm_wday = sync_time->tm_wday;
	    time.tm_hour = sync_time->tm_hour;
	    time.tm_min  = sync_time->tm_min;
	    time.tm_sec  = sync_time->tm_sec;
		if(ql_rtc_set_time(&time) != QL_RTC_SUCCESS)
		{
			LOG_E("ntp set RTC time failed");
		}
	}else{
		LOG_E("ntp sync failed :%d", result);
	}
}

int hal_net_ntp_sync(int pdp_index)
{
    ql_ntp_sync_option  sync_option;
    int error_num = 0;
    sync_option.pdp_cid = pdp_index;
    sync_option.sim_id = 0;
    sync_option.retry_cnt = 3;
    sync_option.retry_interval_tm = 60;

    ntp_cli_id = ql_ntp_sync("ntp.aliyun.com", &sync_option, ntp_result_cb, NULL, &error_num);
    return error_num;
}



void hal_net_info_print()
{
    ql_nw_operator_info_s oper_info;
    ql_nw_cell_info_s cell_info;
    ql_nw_get_operator_name(0, &oper_info);
    LOG_I("mcc:%s, mnc:%s, long_oper_name:%s, short_oper_name:%s", oper_info.mcc, oper_info.mnc,oper_info.long_oper_name, oper_info.short_oper_name);
    ql_nw_get_cell_info(0, &cell_info);
    LOG_I("gsm_info_valid:%d, gsm_info_num:%d, lte_info_valid:%d, lte_info_num:%d", cell_info.gsm_info_valid, cell_info.gsm_info_num, cell_info.lte_info_valid, cell_info.lte_info_num);
    for(int i = 0; i < cell_info.gsm_info_num; i++) {
        LOG_I("i:%d,mmc:%d,mnc:%d,rssi:%d,flag:%d,arfcn:%d,RX_dBm:%d", i, cell_info.gsm_info[i].mcc, cell_info.gsm_info[i].mnc, cell_info.gsm_info[i].rssi, \
            cell_info.gsm_info[i].flag, cell_info.gsm_info[i].arfcn, cell_info.gsm_info[i].RX_dBm);
    }
    for(int i = 0; i < cell_info.lte_info_num; i++){
        LOG_I("i:%d, mcc:%d, mnc:%d, rssi:%d, flag:%d, earfcn:%d, RX_dBm:%d", i, cell_info.lte_info[i].mcc, cell_info.lte_info[i].mnc, cell_info.lte_info[i].rssi, \
        cell_info.lte_info[i].flag, cell_info.lte_info[i].earfcn, cell_info.lte_info[i].RX_dBm);
    }
}

struct oper_info_s oper_info;


static void hal_plmn_info_print(struct oper_info_s oper_info_t)
{
    LOG_I("c_plmn:%s, c_oper_name:%s, vaild_oper_num:%d", oper_info_t.c_plmn, oper_info_t.c_oper_name, oper_info_t.vaild_oper_num);
    for(int i = 0; i < oper_info_t.vaild_oper_num; i++){
        LOG_I("index:%d, plmn:%s, oper_name:%s", i, oper_info_t.v_oper_info[i].plmn, oper_info_t.v_oper_info[i].oper_name);
    }
}

int hal_plmn_one_prase(char *buf, uint16_t len)
{
    char *p = buf, *p1;
    char data_str[36] = {0};
    uint8_t v_func = 0;
    uint8_t step = 0;
    LOG_I("buf:%s", buf);
    for(;;) {
        switch(step){
            case 0:
                p1 = strchr(p, ',');
                if(p1 != NULL){
                    memcpy(data_str, p, p1 - p);
                } else {
                    LOG_E("error");
                    return -1;
                }
                v_func = atoi(data_str);
                step = 1;
                p = p1 + 1;
                memset(data_str, 0, 36);
            break;
            case 1:
                p1 = strchr(p, ',');
                if(p1 != NULL) {
                    memcpy(data_str, p, p1 - p);
                } else {
                    LOG_E("error");
                    return -1;
                }
                if(v_func == 2) {
                    memcpy(oper_info.c_oper_name, data_str, strlen(data_str));
                    oper_info.c_oper_name[strlen(data_str)] = '\0';
                } else if(v_func == 1) {
                    memcpy(oper_info.v_oper_info[oper_info.vaild_oper_num].oper_name, data_str, strlen(data_str));
                    oper_info.v_oper_info[oper_info.vaild_oper_num].oper_name[strlen(data_str)] ='\0';
                } else {
                    LOG_E("error");
                    return -2;
                }
                step = 2;
                p = p1 + 1;
                memset(data_str, 0, 36);
            break;
            case 2:
                p1 = strchr(p, ',');
                if(p1 == NULL) {
                    LOG_E("error");
                    return -1;
                } 
                step = 3;
                p = p1 + 1;
                LOG_I("p:%s", p);
            break;
            case 3:
                p1 = strchr(p, ',');
                if(p1 != NULL){
                    memcpy(data_str, p, p1 - p);
                } else {
                    LOG_E("error");
                    return -1;
                }
                if(v_func == 2) {
                    memcpy(oper_info.c_plmn, data_str, strlen(data_str));
                    oper_info.c_plmn[strlen(data_str)] = '\0';
                } else if(v_func == 1) {
                    memcpy(oper_info.v_oper_info[oper_info.vaild_oper_num].plmn, data_str, strlen(data_str));
                    oper_info.v_oper_info[oper_info.vaild_oper_num].plmn[strlen(data_str)] = '\0';
                    oper_info.vaild_oper_num++;
                }   
                return 0; 
            break;
        }
        
    }
}
int hal_plmn_info_prase(char *info, uint16_t total_len)
{
    uint8_t step = 0;
    char buf[64] = {0};
    uint8_t buf_len = 0;

    char *p = info,*p1 = NULL, *p2 = NULL;
    LOG_I("info:%s", info);
    p1 = strchr(p, '(');
    if(p1 == NULL) {
        LOG_E("data is error");
        return -1;
    }
    for(;;){
        switch (step)
        {
            case 0:
                p1 = strchr(p, '(');
                if(p1 == NULL) {
                    return 0;
                }
                step = 1;
                p = p1;
            break;
            case 1:
                p2 = strchr(p, ')');
                if(p2 != NULL) {
                    buf_len = p2 - (p+1);
                    if(buf_len >= 64) {
                        LOG_E("buf err");
                        return -1;
                    }
                    memcpy(buf, p + 1, buf_len);
                    if(oper_info.vaild_oper_num >= 7) {
                        return 0;
                    } else {
                        if(buf_len >= 18) {
                            if(hal_plmn_one_prase(buf, buf_len) != 0) {
                                LOG_E("hal_plmn_one_prase is error");
                                return -1;
                            }
                        }
                        p = p2;
                        step = 0;
                        memset(buf, 0, 64);
                    }
                } else {
                    LOG_E("plmn prase error");
                    return -1;
                }
            break;
        }  
        LOG_I("step:%d", step);
        LOG_I("p:%s", p);
    }
}



int hal_get_plmn_info()
{
    int res = 0;
    char *at_buf =NULL;
    uint16_t total_len = 0, d_len = 0;
    at_buf = (char*) malloc(1024);
    if(at_buf == NULL) {
        LOG_E("at_buf malloc is error");
        return -2;
    }
    memset(at_buf, 0, 1024);
    memset(&oper_info, 0, sizeof(oper_info));
    hal_virt_flush_recv();
    hal_virt_at_write("AT+COPS=?\r\n");
    d_len = hal_virt_at_read(at_buf, 1024, 2000);
    total_len += d_len;
    if(total_len > 0) {
        d_len = hal_virt_at_recv_buf(&at_buf[total_len], 1024 - d_len, 180000);
    }
    total_len += d_len;
    if(total_len > 0) {
        LOG_I("at_buf:%s, ", at_buf);
        if(hal_plmn_info_prase(at_buf, total_len) == 0){
            hal_plmn_info_print(oper_info);
        } else {
            LOG_E("hal_plmn_info_prase is error");
            res = -1;
        }
    } else {
        res = -3;
    }
   free(at_buf);
   return res;
}


int hal_select_oper(char *plmn)
{
    int res = 0;
    char buf_str[64] = {0};
    char *rec_buf;
    uint16_t d_len, total_len = 0;
    if(plmn == NULL) {
        LOG_E("plmn is error");
        res = -1;
    }
    rec_buf = malloc(64);
    if(rec_buf == NULL) {
        LOG_E("rec_buf is malloc error");
        return -3;
    }
    hal_virt_flush_recv();
    sprintf(buf_str, "AT+COPS=1,2,%s\r\n", plmn);
    hal_virt_at_write(buf_str);
    d_len = hal_virt_at_read(rec_buf, 64, 2000);
    total_len += d_len;
    if(total_len > 0) {
        d_len = hal_virt_at_recv_buf(&rec_buf[total_len], 64 - d_len, 5000);
    }
    total_len += d_len;
    if(total_len > 0) {
        LOG_I("rec_buf:%s, ", rec_buf);
        if(strstr(rec_buf, "OK") == NULL) {
            res = -2;
        }
    } else {
        res = -3;
    }
    free(rec_buf);
    return res;
}