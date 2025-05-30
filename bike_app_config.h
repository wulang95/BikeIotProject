#ifndef  __CONFIG_H
#define  __CONFIG_H
#include "ql_gpio.h"
#include "ql_adc.h"
#include "hal_drv_uart.h"

#define DBG_ENABLE   1
#define BLE_UART    UART2
#define BLE_BAUD    115200
#define BLE_PARITY  PARITY_NONE

#define MCU_UART    UART1
#define MCU_BAUD    115200
#define MCU_PARITY  PARITY_NONE
#define OTA_BAUD    9600
//#define MCU_WEEK

#define BAT_ADC_VAL        QL_ADC1_CHANNEL 
#define TEMP_ADC_VAL       QL_ADC0_CHANNEL

#define I_SENSOR_IN           GPIO_1     //传感器输入中断   OK
#define O_RED_IND             GPIO_3     //红灯 OK   0427交换红白灯
#define O_WHITE_IND           GPIO_2     //白灯 OK
#define O_KEY_LOW             GPIO_4     //低压驱动
#define O_MCU_CONEC           GPIO_5     //cat1运行正常指示 OK
#define SENSOR_POWER          GPIO_8
#define O_AUDIO_SD            GPIO_25    //音频备用
#define I_BLE_CON_SIG         GPIO_14    //蓝牙连接状态 OK
#define O_BLE_POWER           GPIO_15   //蓝牙电源控制  OK
#define O_KEY_HIGH            GPIO_18   //KEY输出
#define O_BLE_WEEK_SIG        GPIO_19       //唤醒蓝牙 
#define I_MCU_WEEK            GPIO_20       //MCU唤醒 ok
#define I_36VPOWER_DET        GPIO_21       //36V电源检测 OK    

#define FLASH_SECTOR_SIZE  4096

#define  OM_NET_PROTOCOL
/*novolte*/
#define     FLASH_BASE                  0x602D0000 
//521K
#define     SYS_CONFIG_ADDR             FLASH_BASE
#define     SYS_CONFIG_SIZE             FLASH_SECTOR_SIZE  
#define     BACK_SYS_CONFIG_ADDR        (SYS_CONFIG_ADDR + SYS_CONFIG_SIZE)
#define     BACK_SYS_CONFIG_SIZE        FLASH_SECTOR_SIZE
#define     SYS_SET_ADDR                (BACK_SYS_CONFIG_ADDR + BACK_SYS_CONFIG_SIZE)
#define     SYS_SET_SIZE                FLASH_SECTOR_SIZE
#define     SENSOR_CALIBRATION_DATA_ADDR    (SYS_SET_ADDR + SYS_SET_SIZE)
#define     SENSOR_CALIBRATION_DATA_SIZE    FLASH_SECTOR_SIZE
#define     SHEEP_FOLD_POINT_ADDR           (SENSOR_CALIBRATION_DATA_ADDR + SENSOR_CALIBRATION_DATA_SIZE)
#define     SHEEP_FOLD_POINT_SIZE           FLASH_SECTOR_SIZE
#define     FORBIDDEN_ZONE_POINT_ADDR       (SHEEP_FOLD_POINT_ADDR + SHEEP_FOLD_POINT_SIZE)
#define     FORBIDDEN_ZONE_SIZE             FLASH_SECTOR_SIZE


//#define USE_LOCAL_GEOFENCE
















#endif // !1    