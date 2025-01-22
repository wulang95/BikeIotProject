#ifndef  __HAL_DRV_IIC_H
#define  __HAL_DRV_IIC_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



int hal_drv_iic_init();
int hal_drv_iic_read(uint8_t salve, uint8_t adress, uint8_t *buf, uint32_t len);
int hal_drv_iic_write(uint8_t salve, uint8_t adress, uint8_t *buf, uint32_t len);
int hal_drv_iic_release();











#endif