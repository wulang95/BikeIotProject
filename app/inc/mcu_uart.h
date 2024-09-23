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



struct gps_info_stu{
    uint8_t valid;
    uint8_t starNum;
};

enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_GPS_START = 0X0E,
    CMD_GPS_STOP = 0X0D,
};
extern struct gps_info_stu gps_info;
void mcu_uart_init();
uint8_t can_data_recv(stc_can_rxframe_t *can_rxframe, uint32_t time_out);
void can_data_send(stc_can_rxframe_t can_txframe);
void mcu_uart_recv_thread(void *param);














#ifdef __cplusplus
}
#endif

#endif