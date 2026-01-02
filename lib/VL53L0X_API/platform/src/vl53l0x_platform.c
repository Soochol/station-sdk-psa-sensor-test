/**
 * @file vl53l0x_platform.c
 * @brief VL53L0X Platform Abstraction Layer Implementation
 */

#include "vl53l0x_platform.h"
#include "hal/i2c_handler.h"
#include "config.h"
#include "stm32h7xx_hal.h"

/*============================================================================*/
/* Private Defines                                                            */
/*============================================================================*/

#define VL53L0X_I2C_TIMEOUT     500     /* ms - increased for calibration */

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

VL53L0X_Error VL53L0X_WriteReg(VL53L0X_Dev_t *Dev, uint8_t index, uint8_t data)
{
    HAL_StatusTypeDef status;
    
    status = I2C_Handler_Write8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                 index, &data, 1, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteReg16(VL53L0X_Dev_t *Dev, uint8_t index, uint16_t data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[2];
    
    /* Big-endian */
    buffer[0] = (uint8_t)(data >> 8);
    buffer[1] = (uint8_t)(data & 0xFF);
    
    status = I2C_Handler_Write8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                 index, buffer, 2, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteReg32(VL53L0X_Dev_t *Dev, uint8_t index, uint32_t data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[4];
    
    /* Big-endian */
    buffer[0] = (uint8_t)(data >> 24);
    buffer[1] = (uint8_t)(data >> 16);
    buffer[2] = (uint8_t)(data >> 8);
    buffer[3] = (uint8_t)(data & 0xFF);
    
    status = I2C_Handler_Write8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                 index, buffer, 4, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_WriteMulti(VL53L0X_Dev_t *Dev, uint8_t index,
                                   uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status;
    
    status = I2C_Handler_Write8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                 index, pdata, (uint16_t)count, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadReg(VL53L0X_Dev_t *Dev, uint8_t index, uint8_t *data)
{
    HAL_StatusTypeDef status;
    
    status = I2C_Handler_Read8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                index, data, 1, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadReg16(VL53L0X_Dev_t *Dev, uint8_t index, uint16_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[2];
    
    status = I2C_Handler_Read8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                index, buffer, 2, VL53L0X_I2C_TIMEOUT);
    
    if (status == HAL_OK) {
        /* Big-endian */
        *data = ((uint16_t)buffer[0] << 8) | buffer[1];
    }
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadReg32(VL53L0X_Dev_t *Dev, uint8_t index, uint32_t *data)
{
    HAL_StatusTypeDef status;
    uint8_t buffer[4];
    
    status = I2C_Handler_Read8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                index, buffer, 4, VL53L0X_I2C_TIMEOUT);
    
    if (status == HAL_OK) {
        /* Big-endian */
        *data = ((uint32_t)buffer[0] << 24) |
                ((uint32_t)buffer[1] << 16) |
                ((uint32_t)buffer[2] << 8) |
                buffer[3];
    }
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_ReadMulti(VL53L0X_Dev_t *Dev, uint8_t index,
                                  uint8_t *pdata, uint32_t count)
{
    HAL_StatusTypeDef status;
    
    status = I2C_Handler_Read8(VL53L0X_I2C_BUS, Dev->I2cDevAddr,
                                index, pdata, (uint16_t)count, VL53L0X_I2C_TIMEOUT);
    
    return (status == HAL_OK) ? VL53L0X_ERROR_NONE : VL53L0X_ERROR_CONTROL_INTERFACE;
}

VL53L0X_Error VL53L0X_PollingDelay(VL53L0X_Dev_t *Dev)
{
    (void)Dev;
    HAL_Delay(1);
    return VL53L0X_ERROR_NONE;
}
