#ifndef     __HAL_VIRT_AT_H
#define     __HAL_VIRT_AT_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>




void hal_virt_at_init();
uint16_t hal_virt_at_read(char *buf, uint16_t len, uint32_t timeout);
void hal_virt_at_write(char *buf);












#endif