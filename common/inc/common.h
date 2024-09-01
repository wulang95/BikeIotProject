#ifndef  __COMMON_H
#define  __COMMON_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define MIN(a,b) ((a) < (b) ? (a) : (b))



uint16_t Package_CheckSum(uint8_t* pdata, uint32_t len);
uint16_t  HexSrt_To_Value(uint8_t *Hex,  char *HexSrt, uint16_t HexSrtLen);








#endif