/**
 * @file sensor_manager.h
 * @brief Sensor driver interface and management
 * 
 * Provides a unified interface for sensor drivers and manages
 * registration and lookup of sensor instances.
 */

#ifndef SENSOR_MANAGER_H
#define SENSOR_MANAGER_H

#ifdef __cplusplus
extern "C" {
#endif

#include "sensors/sensor_types.h"
#include "config.h"

/*============================================================================*/
/* Sensor Driver Interface                                                    */
/*============================================================================*/

/**
 * @brief Sensor driver structure
 * 
 * Defines the interface that all sensor drivers must implement.
 * Function pointers can be NULL if functionality is not applicable.
 */
typedef struct SensorDriver {
    /* Identification */
    SensorID_t      id;             /* Unique sensor ID */
    const char*     name;           /* Human-readable name */

    /* Lifecycle */
    HAL_StatusTypeDef   (*init)(void);      /* Initialize sensor hardware */
    void                (*deinit)(void);    /* Deinitialize sensor */

    /* Specification */
    void                (*set_spec)(const SensorSpec_t* spec);  /* Set test spec */
    void                (*get_spec)(SensorSpec_t* spec);        /* Get current spec */
    bool                (*has_spec)(void);                      /* Check if spec is set */

    /* Test Execution */
    TestStatus_t        (*run_test)(SensorResult_t* result);    /* Execute test (requires spec) */
    TestStatus_t        (*read_sensor)(SensorResult_t* result); /* Read sensor (no spec required) */

    /* Serialization */
    uint8_t             (*serialize_spec)(const SensorSpec_t* spec, uint8_t* buffer);
    uint8_t             (*parse_spec)(const uint8_t* buffer, SensorSpec_t* spec);
    uint8_t             (*serialize_result)(const SensorResult_t* result, uint8_t* buffer);
} SensorDriver_t;

/*============================================================================*/
/* Sensor Manager API                                                         */
/*============================================================================*/

/**
 * @brief Initialize sensor manager and register all drivers
 */
void SensorManager_Init(void);

/**
 * @brief Initialize all registered sensors
 */
void SensorManager_InitSensors(void);

/**
 * @brief Register a sensor driver
 * @param driver Pointer to driver structure
 * @return HAL_OK on success, HAL_ERROR if registry is full
 */
HAL_StatusTypeDef SensorManager_Register(const SensorDriver_t* driver);

/**
 * @brief Get sensor driver by ID
 * @param id Sensor ID
 * @return Pointer to driver or NULL if not found
 */
const SensorDriver_t* SensorManager_GetByID(SensorID_t id);

/**
 * @brief Get sensor driver by index
 * @param index 0-based index
 * @return Pointer to driver or NULL if index out of range
 */
const SensorDriver_t* SensorManager_GetByIndex(uint8_t index);

/**
 * @brief Get count of registered sensors
 * @return Number of registered sensors
 */
uint8_t SensorManager_GetCount(void);

/**
 * @brief Check if sensor ID is valid (registered)
 * @param id Sensor ID
 * @return true if valid, false otherwise
 */
bool SensorManager_IsValidID(SensorID_t id);

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_MANAGER_H */
