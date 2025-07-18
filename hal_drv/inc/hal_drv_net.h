#ifndef __HAL_DRV_NET_H
#define __HAL_DRV_NET_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>

typedef enum {
    NET_ROAM = 0,
    NET_HOME,
}NET_STATE;

typedef enum {
    GSM_NET = 0,
    LTE_NET
} ACCESS_NET;


typedef struct {
    uint16_t mcc;
    uint16_t mnc;
    uint16_t lac;
    uint32_t cid;
    NET_STATE net_state;
    ACCESS_NET act;
    uint8_t fre_band;
    int rsrp;
    uint8_t bit_error_rate;
}NET_NW_INFO;


struct v_oper_info_s {
    char oper_name[35];
    char plmn[9]; 
};

struct oper_info_s{
    char c_plmn[9];   //当前用的plmn
    char c_oper_name[35];  //当前用运营商名称
    uint8_t vaild_oper_num;  //其它可用运营商数量
    struct v_oper_info_s v_oper_info[7];  //其它当前可用的运营商信息
};
extern struct oper_info_s oper_info;

void hal_drv_get_imei(char *data, uint16_t len);
void hal_drv_get_iccid(char *data, uint16_t len);
void hal_drv_get_signal(uint8_t *csq);
uint8_t hal_drv_get_net_register_sta();
void hal_drv_set_data_call_asyn_mode(int pdp_index, uint8_t mode);
uint8_t hal_drv_get_data_call_res(int pdp_index, char *ip4_adr);
void hal_drv_start_data_call(int pdp_index, char *apn);
uint8_t hal_drv_get_cpin();
void hal_dev_set_c_fun(uint8_t fun, uint8_t rst);
uint8_t hal_dev_get_c_fun();
void sys_reset();
void hal_drv_set_dns_addr(char *dns);
void hal_drv_data_call_register_init();
void hal_drv_pdp_detect_block();
NET_NW_INFO hal_drv_get_operator_info();
int hal_net_ntp_sync(int pdpd_index);
void hal_drv_stop_data_call(int pdp_index);
void hal_net_info_print();
int hal_get_plmn_info();
int hal_select_oper(char *plmn);






#endif