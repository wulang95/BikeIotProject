#ifndef     __APP_ERROR_H
#define     __APP_ERROR_H

#ifdef __cplusplus
extern "C" {
#endif

typedef  enum{
    CTRL_PHASE_OC_FAULT = 0,    //控制器相线过流故障
    CTRL_BUS_OC_FAULT,  //控制器母线过流故障
    CTRL_HALL_FAULT,    //控制器HALL故障
    CTRL_TEMP_FAULT,    //控制器温度故障
    COMM_FAULT,     //通讯故障
    MOTOR_TEMP_FAULT,   //电机温度故障
    BRAKE_FAULT,    //刹车故障
    UNDEFINED_FAULT,    //其它未定义故障
    CTRL_COMM_FAULT,    //控制器通讯故障
    CTRL_UV_OV_FAULT,   //控制器过压，欠压
    CTRL_NEW_EURO_STAND_FAULT,  //控制器新欧标故障
    BASIS_VOL_FAULT,    //基准电压故障 
    TORQUE_VOL_FAULT,   //力矩电压故障
    SPEED_SENSOR_FAULT, //车速传感器故障 
    TEMP_SENSOR_FAULT,  //温度传感器故障
    CUR_FEEDBACK_CIRCUIT_FAULT, //电流反馈电路故障
    DRIVE_VOL_FAULT,    //驱动电压故障
    ABNORMAL_CURRENT_FAULT, //电流异常故障
    MOTOR_PHASE_LOSS_FAULT, //电机缺相故障
    TORUQE_VOL_DETEC_CIR_FAULT, //力矩电压检测电路异常
    CHARGE_OV,  //充电过压
    CHARGE_OC,  //充电过流
    CHARGE_OT, //充电过温
    CHARGE_UT, //充电低温
    DIFF_OT,  //温差过大
    DIFF_OU,    //压差过大
    SIGNAL_UNDER_VOL, //单体欠压
    SIGNAL_OVER_VOL, //单体过压
    CHARGE_MOS_OT, //充电MOSFET过温
    DISCHARGE_MOS_OT, //放电MOSFET过温
    PRE_MOS_OT, //预充MOSFET过温
    RESERVE,  //预留
    DISCHARGE_OVER_LOAD,  //放电过载
} CAR_ERROR;

typedef enum {
    SENSOR_ERROR = 0,
    GPS_ERROR,
    BLE_ERROR,
    MCU_CONN_ERROR,
    AUDIO_ERROR,
    IOT_MAX_ERROR,
}IOT_ERROR;

enum {
    CAR_ERROR_TYPE = 0,
    IOT_ERROR_TYPE,
};

void iot_error_set(uint8_t err_type, uint8_t error);
void iot_error_clean(uint8_t err_type, uint8_t error);
uint8_t iot_error_check(uint8_t err_type, uint8_t error);









#ifdef __cplusplus
}
#endif

#endif