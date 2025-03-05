#ifndef __HAL_DRV_FLASH_H
#define __HAL_DRV_FLASH_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>






int hal_drv_flash_write(uint32_t addr, void *data, size_t len);
int hal_drv_flash_read(uint32_t addr, void *data, size_t len);
int hal_drv_flash_erase(uint32_t addr, size_t len);







#endif
