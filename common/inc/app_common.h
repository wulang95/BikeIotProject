#ifndef  __COMMON_H
#define  __COMMON_H
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#define MIN(a,b) ((a) < (b) ? (a) : (b))

#define OK      0
#define FAIL    -1

#define ARRAY_SIZE(Z)   sizeof(Z)/sizeof(Z[0])
#define SETBIT(dat, bit)  (dat) |= 1<<(bit)
#define CLEARBIT(dat, bit) (dat) &= ~(1<<(bit))
#define CHECKBIT(dat, bit) (((dat) >> (bit))&0x01)


unsigned int GetCrc32(const unsigned char* pData, unsigned int Len);
uint16_t Package_CheckSum(uint8_t* pdata, uint32_t len);
uint16_t  HexSrt_To_Value(uint8_t *Hex,  char *HexSrt, uint16_t HexSrtLen);
uint16_t drv_modbus_crc16(uint8_t *puchMsg, uint16_t usDataLen);
uint8_t String_Para_Intercept(char *StrData, char *pPara[], uint8_t PaprNum, uint8_t *EndFlag, char InterChar, char EndChar);
unsigned int GetCrc32_cum(const unsigned char* pData, unsigned int Len, unsigned int CRC32);



#endif