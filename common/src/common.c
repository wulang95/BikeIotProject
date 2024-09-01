#include "common.h"



/****************************************************
**校验
********************************************************/
uint16_t Package_CheckSum(uint8_t* pdata, uint32_t len)
{
    uint16_t sum = 0;
    uint32_t i;

    for(i = 0; i < len; i++)
        sum += pdata[i];
    sum = sum ^ 0xFFFF;             
    return sum;
}


/**
** ¹¦ÄÜ    : Ê®Áù½øÖÆ×Ö·û´®×ªÊýÖµ
** ÈÕÆÚ    : 2018-5-15 19:48:26
** ÐÞ¸Ä¼ÇÂ¼:
**/
uint16_t  HexSrt_To_Value(uint8_t *Hex,  char *HexSrt, uint16_t HexSrtLen)
{
    uint16_t i, HexLen;
    uint8_t Byte;

    const char Hex4Bit[23] = {0x00, 0x01, 0x02, 0x03, 0x04, 0x05, 0x06, 0x07, 0x08, 0x09, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0x0A, 0x0B, 0x0C, 0x0D, 0x0E, 0x0F};

    HexLen = 0;

    for(i = 0; i < HexSrtLen; i += 2)
    {
        Byte = 0;
        if(((*HexSrt >= '0') && (*HexSrt <= '9')) || ((*HexSrt >= 'A') && (*HexSrt <= 'F')))
            Byte = Hex4Bit[(*HexSrt) - '0'];
        else continue;
		
        HexSrt++;
		
        if(((*HexSrt >= '0') && (*HexSrt <= '9')) || ((*HexSrt >= 'A') && (*HexSrt <= 'F')))
            Byte = (Byte << 4) | Hex4Bit[(*HexSrt) - '0'];
        else continue;
        HexSrt++;

        Hex[HexLen++] = Byte;
    }
    return HexLen;
}