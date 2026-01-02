/**
 * @file sensor_types.h
 * @brief Common sensor type definitions
 * 
 * Defines sensor IDs, test status codes, and data structures
 * for sensor specifications and results.
 */

#ifndef SENSOR_TYPES_H
#define SENSOR_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "stm32h7xx_hal.h"

/*============================================================================*/
/* Sensor IDs                                                                 */
/*============================================================================*/

typedef enum {
    SENSOR_ID_NONE          = 0x00,
    SENSOR_ID_VL53L0X       = 0x01,     /* ToF Distance Sensor */
    SENSOR_ID_MLX90640      = 0x02,     /* IR Thermal Array Sensor */
    SENSOR_ID_MAX           = 0x03,
} SensorID_t;

/*============================================================================*/
/* Test Status Codes                                                          */
/*============================================================================*/

typedef enum {
    STATUS_PASS             = 0x00,     /* Test passed */
    STATUS_FAIL_NO_ACK      = 0x01,     /* No I2C acknowledgement */
    STATUS_FAIL_TIMEOUT     = 0x02,     /* Operation timeout */
    STATUS_FAIL_INVALID     = 0x03,     /* Invalid measurement (out of spec) */
    STATUS_FAIL_INIT        = 0x04,     /* Initialization failed */
    STATUS_FAIL_NO_SPEC     = 0x05,     /* Specification not set */
    STATUS_NOT_TESTED       = 0xFF,     /* Not tested (skipped) */
} TestStatus_t;

/*============================================================================*/
/* Sensor Specification (Union for different sensor types)                    */
/*============================================================================*/

/**
 * @brief Sensor specification union
 *
 * Contains target values and tolerances for each sensor type.
 * The raw[] array provides byte-level access for serialization.
 */
typedef union {
    /* VL53L0X Specification */
    struct {
        uint16_t    target_dist;    /* Target distance in mm (30 ~ 2000) */
        uint16_t    tolerance;      /* Tolerance in mm */
    } vl53l0x;

    /* MLX90640 Specification */
    struct {
        int16_t     target_temp;    /* Target temperature in 0.1°C units (e.g., 250 = 25.0°C) */
        int16_t     tolerance;      /* Tolerance in 0.1°C units */
        uint8_t     pixel_x;        /* Target pixel X (0-31) or 0xFF for average */
        uint8_t     pixel_y;        /* Target pixel Y (0-23) or 0xFF for average */
    } mlx90640;

    /* Raw bytes for serialization */
    uint8_t raw[8];
} SensorSpec_t;

/*============================================================================*/
/* Sensor Result (Union for different sensor types)                           */
/*============================================================================*/

/**
 * @brief Sensor result union
 *
 * Contains measured values and comparison data for each sensor type.
 * The raw[] array provides byte-level access for serialization.
 */
typedef union {
    /* VL53L0X Result */
    struct {
        uint16_t    measured;       /* Measured distance in mm */
        uint16_t    target;         /* Spec target distance in mm */
        uint16_t    tolerance;      /* Spec tolerance in mm */
        uint16_t    diff;           /* |measured - target| in mm */
    } vl53l0x;

    /* MLX90640 Result */
    struct {
        int16_t     measured;       /* Measured temperature in 0.1°C units */
        int16_t     target;         /* Spec target temperature in 0.1°C units */
        int16_t     tolerance;      /* Spec tolerance in 0.1°C units */
        int16_t     diff;           /* |measured - target| in 0.1°C units */
        int16_t     ambient;        /* Ambient temperature in 0.1°C units */
        int16_t     min_temp;       /* Min pixel temperature in 0.1°C units */
        int16_t     max_temp;       /* Max pixel temperature in 0.1°C units */
    } mlx90640;

    /* Raw bytes for serialization */
    uint8_t raw[16];
} SensorResult_t;

#ifdef __cplusplus
}
#endif

#endif /* SENSOR_TYPES_H */
