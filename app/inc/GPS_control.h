#ifndef  __GPS_CONTROL_H
#define  __GPS_CONTROL_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>



enum {
    GPS_MODE_TM = 0,
    GPS_MODE_CONT
};

enum {
    GPS_POWER_OFF = 0,
    GPS_POWER_ON,
};


typedef struct GPS_DATA_STRUCT
{
    char     Time1[12];          		//定位时间1 时分秒 hhmmss.00
    uint8_t  GpsFlag;            		//获取到GPS数据标志
    char  GPSValidFlag;       		    //有效定位标志 0：无有效定位 1：
    char     LatLongData[48];    		//经纬度信息  ddmm.mmm,N/S, dddmm.mmm,E/W 2238.07773,N,11407.55384,E 
    char     SateNumStr[6];      		//GPS卫星数量
    char     HDOP[6];            		//水平精度因子   0.5~99.99
    char     Time2[12];          		//定位时间1 年 月 日 ddmmyy
    char     SeaLevelH[12];      	    //海平面高度  （-9999.9 - 9999.9），M
    char     Mode[3];            		//定位模式 A=自主定位，D=差分，E=估算，N=无效
    uint8_t  SolType;            		//当前定位类型
    char    Ground_speed[12];                //地速
    char   Yaw[12];                         //航向角
    double Latitude;                    //纬度数据 小数格式
    double Longitude;                   //经度数据 小数格式
	uint8_t  RefreshFlag;               //定位刷新标志       
} GPS_DATA;




typedef struct GPS_INFO_STRUCT
{
    uint8_t  GpsPower;           		
	uint32_t Status;             		
    uint8_t  GpsMode;            		
    uint8_t  SendCmdFlag;        		

    int64_t  Gps_Tm_timeout;
    uint8_t  SateNum;            		
    uint16_t GetGPSNum;          		
	
	uint8_t  GpsPowerFlag;
	
	uint8_t  GpsPowerSta:2;          	
	uint8_t  Cgpscold:1;           
	uint8_t  reserved:5;             	
	
	uint32_t GPS_BAUD;              
	uint8_t  CaptureNum;
	char     CaptureKey[16];
	
    char     PosData[80];        		
} GPS_INFO;

extern GPS_INFO  Gps;
extern int64_t gps_resh_time_t;
void GPS_Init();
void gps_data_trans(uint8_t *data, uint16_t len);
void gps_control_thread(void *param);
void GPS_Start(uint8_t Mode);
void GPS_stop();


















#ifdef __cplusplus
}
#endif

#endif