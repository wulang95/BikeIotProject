#ifndef  __CONFIG_H
#define  __CONFIG_H
#include "ql_gpio.h"
#include "ql_adc.h"

#define DBG_ENABLE   1
#define BLE_UART    UART2
#define BLE_BAUD    115200
#define BLE_PARITY  PARITY_NONE

#define MCU_UART    UART1
#define MCU_BAUD    115200
#define MCU_PARITY  PARITY_EVENT
//#define MCU_WEEK

#define BAT_ADC_VAL        QL_ADC1_CHANNEL 

#define I_SENSOR_IN           GPIO_1
#define O_RED_IND             GPIO_2
#define O_WHITE_IND           GPIO_3
#define O_BAT_CHARGE_CON      GPIO_4
#define O_MCU_CONEC           GPIO_5
#define I_DOG_WEEK            GPIO_8
#define O_BLE_RST             GPIO_13
#define I_BLE_CON_SIG         GPIO_14 
#define O_LCD_POWER           GPIO_15
#define O_KEY_HIGH            GPIO_18
#define O_BLE_WEEK_SIG        GPIO_19
#define I_MCU_WEEK            GPIO_20   
#define I_36VPOWER_DET        GPIO_21
#define I_DOG_DONE            GPIO_22
#define O_BLE_POWER           GPIO_24
#define O_KEY_LOW             GPIO_25

#define FLASH_SECTOR_SIZE  4096

#define     FLASH_BASE                  0x60290000 
#define     DEV_APP_ADDR                FLASH_BASE
//521K
#define     DEV_APP_SIZE                0x80000  
#define     SYS_CONFIG_ADDR             (DEV_APP_ADDR + DEV_APP_SIZE)
#define     SYS_CONFIG_SIZE             FLASH_SECTOR_SIZE  
#define     BACK_SYS_CONFIG_ADDR        (SYS_CONFIG_ADDR + SYS_CONFIG_SIZE)
#define     BACK_SYS_CONFIG_SIZE        FLASH_SECTOR_SIZE
#define     SYS_SET_ADDR                (BACK_SYS_CONFIG_ADDR + BACK_SYS_CONFIG_SIZE)
#define     SYS_SET_SIZE                FLASH_SECTOR_SIZE
#define     SENSOR_CALIBRATION_DATA_ADDR    (SYS_SET_ADDR + SYS_SET_SIZE)
#define     SENSOR_CALIBRATION_DATA_SEIZE    FLASH_SECTOR_SIZE



















#endif // !1    