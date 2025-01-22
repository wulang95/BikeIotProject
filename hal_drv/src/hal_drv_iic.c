#include "hal_drv_iic.h"
#include "ql_i2c.h"


int hal_drv_iic_init()
{
    return ql_I2cInit(i2c_1, STANDARD_MODE); 
}

int hal_drv_iic_read(uint8_t salve, uint8_t adress, uint8_t *buf, uint32_t len)
{
    return ql_I2cRead(i2c_1, salve, adress, buf, len);
}

int hal_drv_iic_write(uint8_t salve, uint8_t adress, uint8_t *buf, uint32_t len)
{
    return ql_I2cWrite(i2c_1, salve, adress, buf, len);
}

int hal_drv_iic_release()
{
    return ql_I2cRelease(i2c_1);
}