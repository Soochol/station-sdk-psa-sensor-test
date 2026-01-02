/**
 * @file config.h
 * @brief Global configuration and constants for PSA Sensor Test firmware
 * 
 * This file contains all configurable parameters for the firmware including:
 * - Firmware version information
 * - Timeout settings
 * - Sensor I2C configuration
 * - Protocol settings
 * - Debug configuration
 */

#ifndef CONFIG_H
#define CONFIG_H

#ifdef __cplusplus
extern "C" {
#endif

/*============================================================================*/
/* Firmware Version                                                           */
/*============================================================================*/

#define FW_VERSION_MAJOR        1
#define FW_VERSION_MINOR        0
#define FW_VERSION_PATCH        0

/*============================================================================*/
/* Timeout Settings (milliseconds)                                            */
/*============================================================================*/

#define TIMEOUT_SENSOR_TEST_MS      5000    /* Per-sensor test timeout */
#define TIMEOUT_UART_TX_MS          1000    /* UART transmit timeout */
#define TIMEOUT_I2C_MS              100     /* I2C communication timeout */

/*============================================================================*/
/* I2C Bus Configuration                                                      */
/*============================================================================*/

typedef enum {
    I2C_BUS_1 = 0,      /* I2C1 - VL53L0X (PB6: SCL, PB7: SDA) */
    I2C_BUS_4 = 1,      /* I2C4 - MLX90640 (PB8: SCL, PB9: SDA) */
    I2C_BUS_COUNT
} I2C_BusID_t;

/*============================================================================*/
/* Sensor I2C Configuration                                                   */
/*============================================================================*/

/* VL53L0X - I2C1 */
#define VL53L0X_I2C_BUS             I2C_BUS_1
#define VL53L0X_I2C_ADDR            0x29

/* MLX90640 - I2C4 */
#define MLX90640_I2C_BUS            I2C_BUS_4
#define MLX90640_I2C_ADDR           0x33

/*============================================================================*/
/* Protocol Settings                                                          */
/*============================================================================*/

#define PROTOCOL_STX                0x02
#define PROTOCOL_ETX                0x03
#define PROTOCOL_MAX_PAYLOAD        64
#define PROTOCOL_RX_BUFFER_SIZE     128

/*============================================================================*/
/* Sensor Manager Settings                                                    */
/*============================================================================*/

#define MAX_SENSORS                 8

/*============================================================================*/
/* UART Buffer Settings                                                       */
/*============================================================================*/

#define UART_RX_BUFFER_SIZE         256
#define UART_TX_BUFFER_SIZE         256

/*============================================================================*/
/* Debug Configuration                                                        */
/*============================================================================*/

#define DEBUG_ENABLED               0

/*============================================================================*/
/* Watchdog Configuration                                                     */
/*============================================================================*/

#define WATCHDOG_ENABLED            1   /* Enable Independent Watchdog (IWDG) */
#define WATCHDOG_TIMEOUT_MS         10000 /* Approximate timeout in milliseconds (10 seconds) */

/*============================================================================*/
/* VL53L0X Configuration                                                      */
/*============================================================================*/

#define VL53L0X_RANGE_MIN_MM        30
#define VL53L0X_RANGE_MAX_MM        2000
#define VL53L0X_MEASUREMENT_MODE    1       /* 0=Single, 1=Continuous */
#define VL53L0X_TIMING_BUDGET_US    33000   /* Measurement timing budget */

/*============================================================================*/
/* MLX90640 Configuration                                                     */
/*============================================================================*/

#define MLX90640_REFRESH_RATE       4       /* 0=0.5Hz, 1=1Hz, 2=2Hz, 3=4Hz, 4=8Hz, 5=16Hz, 6=32Hz, 7=64Hz */
#define MLX90640_RESOLUTION         19      /* ADC resolution: 16, 17, 18, or 19 bits */
#define MLX90640_EMISSIVITY         0.95f   /* Default emissivity */
#define MLX90640_WARMUP_TIME_MS     6000    /* Warmup time after init before valid readings (measured: ~5-6s) */

#ifdef __cplusplus
}
#endif

#endif /* CONFIG_H */
