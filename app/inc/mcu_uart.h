#ifndef  __MCU_UART_H
#define  __MCU_UART_H
#ifdef __cplusplus
extern "C" {
#endif
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
typedef struct
{
    uint8_t DLC          : 4;       ///< Data length code
    uint8_t RESERVED0    : 2;       ///< Ignore
    uint8_t RTR          : 1;       ///< Remote transmission request
    uint8_t IDE          : 1;       ///< IDentifier extension
}stc_can_rxcontrol_t;

typedef struct
{
    uint8_t RESERVED0    : 4;       ///< Ignore
    uint8_t TX           : 1;       ///< TX is set to 1 if the loop back mode is activated
    uint8_t KOER         : 3;       ///< Kind of error
}stc_can_status_t;

typedef struct
{
    stc_can_rxcontrol_t Control_f;      ///< @ref stc_can_rxcontrol_t
    stc_can_status_t    Status_f;       ///< @ref stc_can_status_t
    uint16_t            CycleTime;      ///< TTCAN cycletime
}stc_can_cst_t;


typedef struct stc_can_rxframe
{
    union
    {
        uint32_t RBUF32_0;              ///< Ignore
        uint32_t StdID;                 ///< Standard ID
        uint32_t ExtID;                 ///< Extended ID
    };
    union
    {
        uint32_t        RBUF32_1;       ///< Ignore
        stc_can_cst_t   Cst;            ///< @ref stc_can_cst_t
    };
    union
    {
        uint32_t RBUF32_2[2];           ///< Ignore
        uint8_t  Data[8];               ///< CAN data
    };

}stc_can_rxframe_t;

enum{
    GPS_SINGAL = 0,
    GPS_TRACK
};


enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_GPS_POWERON = 0X0E,
    CMD_GPS_POWEROFF = 0X0D,
    CMD_GPS_DATA = 0X0F,
	CMD_GPS_TRANS = 0X0B,
	CMD_GPS_DEEPSLEEP = 0X0A,
	CMD_GPS_HOST_START = 0X09,
    CMD_CAT_REPOWERON = 0X08,
};
enum {
    CMD_CAN_TRANS_INDEX = 0,
    CMD_GPS_POWERON_INDEX,
    CMD_GPS_POWEROFF_INDEX,
    CMD_GPS_DATA_INDEX,
    CMD_GPS_TRANS_INDEX,
    CMD_GPS_DEEPSLEEP_INDEX,
    CMD_GPS_HOST_START_INDEX,
    CMD_CAT_REPOWERON_INDEX,
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
    double   Latitude;           		//维度数据  小数格式
    double   Longitude;          		//经度数据  小数格式

	uint8_t  RefreshFlag;               //定位刷新标志       
} GPS_DATA;

extern uint8_t GPS_MODE;
extern GPS_DATA gps_info;
extern int64_t gps_resh_time_t;
void mcu_uart_init();
uint8_t can_data_recv(stc_can_rxframe_t *can_rxframe, uint32_t time_out);
void can_data_send(stc_can_rxframe_t can_txframe);
void mcu_uart_recv_thread(void *param);
void mcu_uart_send_thread(void *param);













#ifdef __cplusplus
}
#endif

#endif