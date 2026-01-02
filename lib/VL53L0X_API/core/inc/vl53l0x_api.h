/**
 * @file vl53l0x_api.h
 * @brief VL53L0X ToF Sensor API
 * 
 * Simplified implementation based on ST VL53L0X API.
 * Provides distance measurement functionality.
 */

#ifndef VL53L0X_API_H
#define VL53L0X_API_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>

/*============================================================================*/
/* Error Codes                                                                */
/*============================================================================*/

typedef int8_t VL53L0X_Error;

#define VL53L0X_ERROR_NONE                      0
#define VL53L0X_ERROR_CALIBRATION_WARNING      -1
#define VL53L0X_ERROR_MIN_CLIPPED              -2
#define VL53L0X_ERROR_UNDEFINED                -3
#define VL53L0X_ERROR_INVALID_PARAMS           -4
#define VL53L0X_ERROR_NOT_SUPPORTED            -5
#define VL53L0X_ERROR_RANGE_ERROR              -6
#define VL53L0X_ERROR_TIME_OUT                 -7
#define VL53L0X_ERROR_MODE_NOT_SUPPORTED       -8
#define VL53L0X_ERROR_BUFFER_TOO_SMALL         -9
#define VL53L0X_ERROR_GPIO_NOT_EXISTING       -10
#define VL53L0X_ERROR_GPIO_FUNCTIONALITY_NOT_SUPPORTED -11
#define VL53L0X_ERROR_CONTROL_INTERFACE       -20
#define VL53L0X_ERROR_INVALID_COMMAND         -30
#define VL53L0X_ERROR_DIVISION_BY_ZERO        -40
#define VL53L0X_ERROR_REF_SPAD_INIT           -50
#define VL53L0X_ERROR_NOT_IMPLEMENTED         -99

/*============================================================================*/
/* Device Modes                                                               */
/*============================================================================*/

typedef uint8_t VL53L0X_DeviceModes;

#define VL53L0X_DEVICEMODE_SINGLE_RANGING           0
#define VL53L0X_DEVICEMODE_CONTINUOUS_RANGING       1
#define VL53L0X_DEVICEMODE_CONTINUOUS_TIMED_RANGING 3

/*============================================================================*/
/* Range Status                                                               */
/*============================================================================*/

#define VL53L0X_RANGESTATUS_RANGEVALID              0
#define VL53L0X_RANGESTATUS_SIGMA                   1
#define VL53L0X_RANGESTATUS_SIGNAL                  2
#define VL53L0X_RANGESTATUS_MINRANGE                3
#define VL53L0X_RANGESTATUS_PHASE                   4
#define VL53L0X_RANGESTATUS_HW                      5

/*============================================================================*/
/* Limit Check IDs                                                            */
/*============================================================================*/

#define VL53L0X_CHECKENABLE_SIGMA_FINAL_RANGE           0
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_FINAL_RANGE     1
#define VL53L0X_CHECKENABLE_SIGNAL_REF_CLIP             2
#define VL53L0X_CHECKENABLE_RANGE_IGNORE_THRESHOLD      3
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_MSRC            4
#define VL53L0X_CHECKENABLE_SIGNAL_RATE_PRE_RANGE       5
#define VL53L0X_CHECKENABLE_NUMBER_OF_CHECKS            6

/*============================================================================*/
/* Fixed Point Types                                                          */
/*============================================================================*/

typedef uint32_t FixPoint1616_t;

/*============================================================================*/
/* Types                                                                      */
/*============================================================================*/

/**
 * @brief Device structure
 */
typedef struct {
    uint8_t I2cDevAddr;         /* I2C device address */
    uint8_t comms_type;         /* Communication type (1=I2C) */
    uint16_t comms_speed_khz;   /* Communication speed */
    /* Internal state */
    uint8_t Data[256];          /* Device data/calibration */
} VL53L0X_Dev_t;

/**
 * @brief Ranging measurement data
 */
typedef struct {
    uint32_t TimeStamp;
    uint32_t MeasurementTimeUsec;
    uint16_t RangeMilliMeter;
    uint16_t RangeDMaxMilliMeter;
    uint32_t SignalRateRtnMegaCps;
    uint32_t AmbientRateRtnMegaCps;
    uint16_t EffectiveSpadRtnCount;
    uint8_t ZoneId;
    uint8_t RangeFractionalPart;
    uint8_t RangeStatus;
} VL53L0X_RangingMeasurementData_t;

/*============================================================================*/
/* API Functions                                                              */
/*============================================================================*/

/**
 * @brief Initialize data structures
 * @param Dev Device handle
 * @return Error code
 */
VL53L0X_Error VL53L0X_DataInit(VL53L0X_Dev_t *Dev);

/**
 * @brief Perform static initialization
 * @param Dev Device handle
 * @return Error code
 */
VL53L0X_Error VL53L0X_StaticInit(VL53L0X_Dev_t *Dev);

/**
 * @brief Perform reference SPAD management
 * @param Dev Device handle
 * @param refSpadCount Output: reference SPAD count
 * @param isApertureSpads Output: aperture SPAD flag
 * @return Error code
 */
VL53L0X_Error VL53L0X_PerformRefSpadManagement(VL53L0X_Dev_t *Dev,
                                                 uint32_t *refSpadCount,
                                                 uint8_t *isApertureSpads);

/**
 * @brief Perform reference calibration
 * @param Dev Device handle
 * @param pVhvSettings Output: VHV settings
 * @param pPhaseCal Output: Phase calibration
 * @return Error code
 */
VL53L0X_Error VL53L0X_PerformRefCalibration(VL53L0X_Dev_t *Dev,
                                              uint8_t *pVhvSettings,
                                              uint8_t *pPhaseCal);

/**
 * @brief Set device mode
 * @param Dev Device handle
 * @param DeviceMode Device mode
 * @return Error code
 */
VL53L0X_Error VL53L0X_SetDeviceMode(VL53L0X_Dev_t *Dev,
                                      VL53L0X_DeviceModes DeviceMode);

/**
 * @brief Set measurement timing budget
 * @param Dev Device handle
 * @param MeasurementTimingBudgetMicroSeconds Timing budget in microseconds
 * @return Error code
 */
VL53L0X_Error VL53L0X_SetMeasurementTimingBudgetMicroSeconds(VL53L0X_Dev_t *Dev,
                                                               uint32_t MeasurementTimingBudgetMicroSeconds);

/**
 * @brief Perform single ranging measurement
 * @param Dev Device handle
 * @param pRangingMeasurementData Output: measurement data
 * @return Error code
 */
VL53L0X_Error VL53L0X_PerformSingleRangingMeasurement(VL53L0X_Dev_t *Dev,
                                                        VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);

/**
 * @brief Start measurement
 * @param Dev Device handle
 * @return Error code
 */
VL53L0X_Error VL53L0X_StartMeasurement(VL53L0X_Dev_t *Dev);

/**
 * @brief Stop measurement
 * @param Dev Device handle
 * @return Error code
 */
VL53L0X_Error VL53L0X_StopMeasurement(VL53L0X_Dev_t *Dev);

/**
 * @brief Get measurement data ready
 * @param Dev Device handle
 * @param pMeasurementDataReady Output: ready flag
 * @return Error code
 */
VL53L0X_Error VL53L0X_GetMeasurementDataReady(VL53L0X_Dev_t *Dev,
                                                uint8_t *pMeasurementDataReady);

/**
 * @brief Get ranging measurement data
 * @param Dev Device handle
 * @param pRangingMeasurementData Output: measurement data
 * @return Error code
 */
VL53L0X_Error VL53L0X_GetRangingMeasurementData(VL53L0X_Dev_t *Dev,
                                                  VL53L0X_RangingMeasurementData_t *pRangingMeasurementData);

/**
 * @brief Clear interrupt mask
 * @param Dev Device handle
 * @param InterruptMask Interrupt mask
 * @return Error code
 */
VL53L0X_Error VL53L0X_ClearInterruptMask(VL53L0X_Dev_t *Dev, uint32_t InterruptMask);

/**
 * @brief Enable/Disable a specific limit check
 * @param Dev Device handle
 * @param LimitCheckId Limit check identifier (VL53L0X_CHECKENABLE_*)
 * @param LimitCheckEnable 1 to enable, 0 to disable
 * @return Error code
 */
VL53L0X_Error VL53L0X_SetLimitCheckEnable(VL53L0X_Dev_t *Dev,
                                           uint16_t LimitCheckId,
                                           uint8_t LimitCheckEnable);

/**
 * @brief Set limit check threshold value
 * @param Dev Device handle
 * @param LimitCheckId Limit check identifier (VL53L0X_CHECKENABLE_*)
 * @param LimitCheckValue Threshold value in FixPoint1616 format
 * @return Error code
 */
VL53L0X_Error VL53L0X_SetLimitCheckValue(VL53L0X_Dev_t *Dev,
                                          uint16_t LimitCheckId,
                                          FixPoint1616_t LimitCheckValue);

/**
 * @brief Get current limit check value
 * @param Dev Device handle
 * @param LimitCheckId Limit check identifier
 * @param pLimitCheckValue Output: current threshold value
 * @return Error code
 */
VL53L0X_Error VL53L0X_GetLimitCheckCurrent(VL53L0X_Dev_t *Dev,
                                            uint16_t LimitCheckId,
                                            FixPoint1616_t *pLimitCheckCurrent);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_API_H */
