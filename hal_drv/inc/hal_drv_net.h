#ifndef __HAL_DRV_NET_H
#define __HAL_DRV_NET_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>


void hal_drv_get_imei(char *data, uint16_t len);
void hal_drv_get_iccid(char *data, uint16_t len);
void hal_drv_get_signal(uint8_t *csq, int *rssi);
uint8_t hal_drv_get_net_register_sta();
void hal_drv_set_data_call_asyn_mode(uint8_t mode);
uint8_t hal_drv_get_data_call_res(char *ip4_adr);
void hal_drv_start_data_call(char *apn);
uint8_t hal_drv_get_cpin();
void hal_dev_set_c_fun(uint8_t fun, uint8_t rst);
uint8_t hal_dev_get_c_fun();
void sys_reset();
void hal_drv_set_dns_addr();












#endif