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



enum {
    CMD_CAN_TRANS = 0X0C,
    CMD_GPS_POWERON = 0X0E,
    CMD_GPS_POWEROFF = 0X0D,
    CMD_GPS_DATA = 0X0F,
	CMD_GPS_TRANS = 0X0B,
	CMD_GPS_DEEPSLEEP = 0X0A,
	CMD_GPS_HOST_START = 0X09,
    CMD_CAT_REPOWERON = 0X08,
    CMD_CRC_ERROR = 0X07,
    CMD_CAN_OTA_DATA = 0X06,
    CMD_CAN_OTA_START = 0X05,
    CMD_CAN_OTA_END = 0X04,
    CMD_CAN_OTA_DATA_FINISH = 0X03,
    CMD_CAN_LOCK_CAR = 0X10,
    CMD_CAN_UNLOCK_CAR = 0X11,
    CMD_CAN_CAR_CONTROL = 0X12,
    CMD_SHIP_MODE = 0X13,
    CMD_MCU_OTA_START = 0X14,
    CMD_MCU_OTA_DATA = 0X15,
    CMD_MCU_OTA_END = 0X16,
    CMD_MCU_VER = 0X17,
    CMD_MCU_ADC_DATA = 0X18,
    CMD_MCU_BAT_CHARGE_ON = 0X19,
    CMD_MCU_BAT_CHARGE_OFF = 0X1A,
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
    CMD_CRC_ERROR_INDEX,
    CMD_CAN_OTA_DATA_INDEX,
    CMD_CAN_OTA_START_INDEX,
    CMD_CAN_OTA_END_INDEX,
    CMD_CAN_OTA_DATA_FINISH_INDEX,
    CMD_CAN_LOCK_CAR_INDEX,
    CMD_CAN_UNLOCK_CAR_INDEX,
    CMD_CAN_CAR_CONTROL_INDEX,
    CMD_SHIP_MODE_INDEX,
    CMD_MCU_OTA_START_INDEX,
    CMD_MCU_OTA_DATA_INDEX,
    CMD_MCU_OTA_END_INDEX,
    CMD_MCU_VER_INDEX,
    CMD_MCU_ADC_DATA_INDEX,
    CMD_MCU_BAT_CHARGE_ON_INDEX,
    CMD_MCU_BAT_CHARGE_OFF_INDEX,
    CMD_INDEX_MAX,
};


void mcu_uart_init();
uint8_t can_data_recv(stc_can_rxframe_t *can_rxframe, uint32_t time_out);
void can_data_send(stc_can_rxframe_t can_txframe);
void mcu_uart_recv_thread(void *param);
void mcu_uart_send_thread(void *param);
void MCU_CMD_MARK(uint8_t cmd);
void mcu_data_pack(uint8_t cmd, uint8_t *data, uint16_t data_len, uint8_t *buf, uint16_t *lenth);
void mcu_uart_send(uint8_t *data, uint16_t len);
int mcu_ota_task();









#ifdef __cplusplus
}
#endif

#endif