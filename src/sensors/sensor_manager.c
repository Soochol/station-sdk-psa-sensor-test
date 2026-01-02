/**
 * @file sensor_manager.c
 * @brief Sensor manager implementation
 */

#include "sensors/sensor_manager.h"
#include "sensors/vl53l0x.h"
#include "sensors/mlx90640.h"
#include <string.h>

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static const SensorDriver_t* sensors[MAX_SENSORS];
static uint8_t sensor_count = 0;

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void SensorManager_Init(void)
{
    /* Clear registry */
    sensor_count = 0;
    memset(sensors, 0, sizeof(sensors));

    /* Register built-in drivers */
    SensorManager_Register(&VL53L0X_Driver);
    SensorManager_Register(&MLX90640_Driver);
}

void SensorManager_InitSensors(void)
{
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i]->init != NULL) {
            sensors[i]->init();
        }
    }
}

HAL_StatusTypeDef SensorManager_Register(const SensorDriver_t* driver)
{
    if (driver == NULL) {
        return HAL_ERROR;
    }
    
    if (sensor_count >= MAX_SENSORS) {
        return HAL_ERROR;
    }
    
    /* Check for duplicate ID */
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i]->id == driver->id) {
            return HAL_ERROR;
        }
    }
    
    sensors[sensor_count++] = driver;
    return HAL_OK;
}

const SensorDriver_t* SensorManager_GetByID(SensorID_t id)
{
    for (uint8_t i = 0; i < sensor_count; i++) {
        if (sensors[i]->id == id) {
            return sensors[i];
        }
    }
    return NULL;
}

const SensorDriver_t* SensorManager_GetByIndex(uint8_t index)
{
    if (index >= sensor_count) {
        return NULL;
    }
    return sensors[index];
}

uint8_t SensorManager_GetCount(void)
{
    return sensor_count;
}

bool SensorManager_IsValidID(SensorID_t id)
{
    return SensorManager_GetByID(id) != NULL;
}
