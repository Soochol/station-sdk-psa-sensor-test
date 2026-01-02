/**
 * @file i2c_handler.c
 * @brief I2C communication handler implementation
 */

#include "hal/i2c_handler.h"
#include <string.h>

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static I2C_HandleTypeDef* i2c_handles[I2C_BUS_COUNT] = {NULL};

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

HAL_StatusTypeDef I2C_Handler_Init(I2C_BusID_t bus_id, I2C_HandleTypeDef* hi2c)
{
    if (bus_id >= I2C_BUS_COUNT || hi2c == NULL) {
        return HAL_ERROR;
    }
    
    i2c_handles[bus_id] = hi2c;
    return HAL_OK;
}

HAL_StatusTypeDef I2C_Handler_IsDeviceReady(I2C_BusID_t bus_id, uint8_t dev_addr,
                                             uint32_t timeout_ms)
{
    if (bus_id >= I2C_BUS_COUNT || i2c_handles[bus_id] == NULL) {
        return HAL_ERROR;
    }
    
    /* HAL expects 8-bit address (left-shifted by 1) */
    uint16_t addr_8bit = (uint16_t)(dev_addr << 1);
    
    return HAL_I2C_IsDeviceReady(i2c_handles[bus_id], addr_8bit, 3, timeout_ms);
}

HAL_StatusTypeDef I2C_Handler_Read16(I2C_BusID_t bus_id, uint8_t dev_addr,
                                      uint16_t reg_addr, uint8_t* data,
                                      uint16_t len, uint32_t timeout_ms)
{
    if (bus_id >= I2C_BUS_COUNT || i2c_handles[bus_id] == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    uint16_t addr_8bit = (uint16_t)(dev_addr << 1);
    
    return HAL_I2C_Mem_Read(i2c_handles[bus_id], addr_8bit, reg_addr,
                            I2C_MEMADD_SIZE_16BIT, data, len, timeout_ms);
}

HAL_StatusTypeDef I2C_Handler_Write16(I2C_BusID_t bus_id, uint8_t dev_addr,
                                       uint16_t reg_addr, const uint8_t* data,
                                       uint16_t len, uint32_t timeout_ms)
{
    if (bus_id >= I2C_BUS_COUNT || i2c_handles[bus_id] == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    uint16_t addr_8bit = (uint16_t)(dev_addr << 1);
    
    return HAL_I2C_Mem_Write(i2c_handles[bus_id], addr_8bit, reg_addr,
                             I2C_MEMADD_SIZE_16BIT, (uint8_t*)data, len, timeout_ms);
}

HAL_StatusTypeDef I2C_Handler_Read8(I2C_BusID_t bus_id, uint8_t dev_addr,
                                     uint8_t reg_addr, uint8_t* data,
                                     uint16_t len, uint32_t timeout_ms)
{
    if (bus_id >= I2C_BUS_COUNT || i2c_handles[bus_id] == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    uint16_t addr_8bit = (uint16_t)(dev_addr << 1);
    
    return HAL_I2C_Mem_Read(i2c_handles[bus_id], addr_8bit, reg_addr,
                            I2C_MEMADD_SIZE_8BIT, data, len, timeout_ms);
}

HAL_StatusTypeDef I2C_Handler_Write8(I2C_BusID_t bus_id, uint8_t dev_addr,
                                      uint8_t reg_addr, const uint8_t* data,
                                      uint16_t len, uint32_t timeout_ms)
{
    if (bus_id >= I2C_BUS_COUNT || i2c_handles[bus_id] == NULL || data == NULL) {
        return HAL_ERROR;
    }
    
    uint16_t addr_8bit = (uint16_t)(dev_addr << 1);
    
    return HAL_I2C_Mem_Write(i2c_handles[bus_id], addr_8bit, reg_addr,
                             I2C_MEMADD_SIZE_8BIT, (uint8_t*)data, len, timeout_ms);
}

I2C_HandleTypeDef* I2C_Handler_GetHandle(I2C_BusID_t bus_id)
{
    if (bus_id >= I2C_BUS_COUNT) {
        return NULL;
    }
    return i2c_handles[bus_id];
}
