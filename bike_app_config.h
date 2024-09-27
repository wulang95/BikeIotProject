#ifndef  __CONFIG_H
#define  __CONFIG_H
#include "ql_gpio.h"

#define DBG_ENABLE   1
#define BLE_UART    UART1
#define BLE_BAUD    115200
#define BLE_PARITY  PARITY_NONE

#define MCU_UART    UART2
#define MCU_BAUD    115200
#define MCU_PARITY  PARITY_EVENT
//#define MCU_WEEK



#define I_SENSOR_IN           GPIO_1
#define O_RED_IND             GPIO_2
#define O_WHITE_IND           GPIO_3
#define O_BAT_CHARGE_CON      GPIO_4
#define I_MCU_CONEC           GPIO_5
#define I_DOG_WEEK            GPIO_8
#define O_BLE_RST             GPIO_13
#define I_BLE_CON_SIG         GPIO_14 
#define O_LCD_POWER           GPIO_15
#define O_KEY_HIGH            GPIO_18
#define O_BLE_WEEK_SIG        GPIO_19
#define O_MCU_WEEK            GPIO_20
#define I_36VPOWER_DET        GPIO_21
#define I_DOG_DONE            GPIO_22
#define O_BLE_POWER           GPIO_24
#define O_KEY_LOW             GPIO_25


#define     BOOT_ADDR               0x60250000
#define     BOOT_SIZE               0x2000
#define     APP1_ADDR               (BOOT_ADDR + BOOT_SIZE)   
#define     APP1_SIZE               (0x40000)   
#define     APP2_ADDR               (APP1_ADDR + APP1_SIZE)
#define     APP2_SIZE               (0x40000)
#define     BOOT_CONFIG_ADDR        (APP2_ADDR + APP2_SIZE)
#define     BOOT_CONFIG_SIZE        0x1000
#define     SYS_CONFIG_ADDR           (BOOT_CONFIG_ADDR + BOOT_CONFIG_SIZE)
#define     SYS_CONFIG_SIZE           0x1000
#define     BACK_SYS_CONFIG_ADDR      (SYS_CONFIG_ADDR + SYS_CONFIG_SIZE)
#define     BACK_SYS_CONFIG_SIZE      0x1000
#define     CAR_SET_ADD             (BACK_SYS_CONFIG_ADDR + BACK_SYS_CONFIG_SIZE)
#define     CAR_SET_SIZE            0x1000



















#endif // !1    