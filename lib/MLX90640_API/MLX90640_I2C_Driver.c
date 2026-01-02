/**
 * @file MLX90640_I2C_Driver.c
 * @brief MLX90640 I2C Driver Implementation for STM32H7
 */

#include "MLX90640_I2C_Driver.h"
#include "hal/i2c_handler.h"
#include "config.h"

/*============================================================================*/
/* Private Defines                                                            */
/*============================================================================*/

#define MLX90640_I2C_TIMEOUT    500     /* ms - increased for large EEPROM reads */

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void MLX90640_I2CInit(void)
{
    /* I2C is initialized by CubeMX and I2C_Handler */
}

int MLX90640_I2CRead(uint8_t slaveAddr, uint16_t startAddress,
                      uint16_t nMemAddressRead, uint16_t *data)
{
    if (data == NULL || nMemAddressRead == 0) {
        return -1;
    }
    
    /* MLX90640 uses big-endian 16-bit words */
    uint8_t* raw_data = (uint8_t*)data;
    uint16_t byte_count = nMemAddressRead * 2;
    
    HAL_StatusTypeDef status = I2C_Handler_Read16(
        MLX90640_I2C_BUS,
        slaveAddr,
        startAddress,
        raw_data,
        byte_count,
        MLX90640_I2C_TIMEOUT
    );
    
    if (status != HAL_OK) {
        return -1;
    }
    
    /* Swap bytes (MLX90640 sends MSB first) */
    for (uint16_t i = 0; i < nMemAddressRead; i++) {
        uint8_t temp = raw_data[i * 2];
        raw_data[i * 2] = raw_data[i * 2 + 1];
        raw_data[i * 2 + 1] = temp;
    }
    
    return 0;
}

int MLX90640_I2CWrite(uint8_t slaveAddr, uint16_t writeAddress, uint16_t data)
{
    /* Prepare data in big-endian format */
    uint8_t write_buf[2];
    write_buf[0] = (uint8_t)(data >> 8);
    write_buf[1] = (uint8_t)(data & 0xFF);
    
    HAL_StatusTypeDef status = I2C_Handler_Write16(
        MLX90640_I2C_BUS,
        slaveAddr,
        writeAddress,
        write_buf,
        2,
        MLX90640_I2C_TIMEOUT
    );
    
    return (status == HAL_OK) ? 0 : -1;
}

void MLX90640_I2CFreqSet(int freq)
{
    /* I2C frequency is configured by CubeMX */
    (void)freq;
}
