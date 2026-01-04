/**
 * @file vl53l0x_simple.h
 * @brief Simple VL53L0X driver based on Pololu library (C port)
 * @note Based on MarcelMG/VL53L0X-STM32F103, adapted for STM32H7
 *
 * Original: https://github.com/MarcelMG/VL53L0X-STM32F103
 * License: MIT (Pololu) / GPL-3.0 (MarcelMG)
 */

#ifndef VL53L0X_SIMPLE_H
#define VL53L0X_SIMPLE_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>

/*============================================================================*/
/* Register Addresses (from VL53L0X API vl53l0x_device.h)                     */
/*============================================================================*/

typedef enum {
    SYSRANGE_START                              = 0x00,
    SYSTEM_THRESH_HIGH                          = 0x0C,
    SYSTEM_THRESH_LOW                           = 0x0E,
    SYSTEM_SEQUENCE_CONFIG                      = 0x01,
    SYSTEM_RANGE_CONFIG                         = 0x09,
    SYSTEM_INTERMEASUREMENT_PERIOD              = 0x04,
    SYSTEM_INTERRUPT_CONFIG_GPIO                = 0x0A,
    GPIO_HV_MUX_ACTIVE_HIGH                     = 0x84,
    SYSTEM_INTERRUPT_CLEAR                      = 0x0B,
    RESULT_INTERRUPT_STATUS                     = 0x13,
    RESULT_RANGE_STATUS                         = 0x14,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_RTN       = 0xBC,
    RESULT_CORE_RANGING_TOTAL_EVENTS_RTN        = 0xC0,
    RESULT_CORE_AMBIENT_WINDOW_EVENTS_REF       = 0xD0,
    RESULT_CORE_RANGING_TOTAL_EVENTS_REF        = 0xD4,
    RESULT_PEAK_SIGNAL_RATE_REF                 = 0xB6,
    ALGO_PART_TO_PART_RANGE_OFFSET_MM           = 0x28,
    I2C_SLAVE_DEVICE_ADDRESS                    = 0x8A,
    MSRC_CONFIG_CONTROL                         = 0x60,
    PRE_RANGE_CONFIG_MIN_SNR                    = 0x27,
    PRE_RANGE_CONFIG_VALID_PHASE_LOW            = 0x56,
    PRE_RANGE_CONFIG_VALID_PHASE_HIGH           = 0x57,
    PRE_RANGE_MIN_COUNT_RATE_RTN_LIMIT          = 0x64,
    FINAL_RANGE_CONFIG_MIN_SNR                  = 0x67,
    FINAL_RANGE_CONFIG_VALID_PHASE_LOW          = 0x47,
    FINAL_RANGE_CONFIG_VALID_PHASE_HIGH         = 0x48,
    FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT = 0x44,
    PRE_RANGE_CONFIG_SIGMA_THRESH_HI            = 0x61,
    PRE_RANGE_CONFIG_SIGMA_THRESH_LO            = 0x62,
    PRE_RANGE_CONFIG_VCSEL_PERIOD               = 0x50,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI          = 0x51,
    PRE_RANGE_CONFIG_TIMEOUT_MACROP_LO          = 0x52,
    SYSTEM_HISTOGRAM_BIN                        = 0x81,
    HISTOGRAM_CONFIG_INITIAL_PHASE_SELECT       = 0x33,
    HISTOGRAM_CONFIG_READOUT_CTRL               = 0x55,
    FINAL_RANGE_CONFIG_VCSEL_PERIOD             = 0x70,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI        = 0x71,
    FINAL_RANGE_CONFIG_TIMEOUT_MACROP_LO        = 0x72,
    CROSSTALK_COMPENSATION_PEAK_RATE_MCPS       = 0x20,
    MSRC_CONFIG_TIMEOUT_MACROP                  = 0x46,
    SOFT_RESET_GO2_SOFT_RESET_N                 = 0xBF,
    IDENTIFICATION_MODEL_ID                     = 0xC0,
    IDENTIFICATION_REVISION_ID                  = 0xC2,
    OSC_CALIBRATE_VAL                           = 0xF8,
    GLOBAL_CONFIG_VCSEL_WIDTH                   = 0x32,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_0            = 0xB0,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_1            = 0xB1,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_2            = 0xB2,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_3            = 0xB3,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_4            = 0xB4,
    GLOBAL_CONFIG_SPAD_ENABLES_REF_5            = 0xB5,
    GLOBAL_CONFIG_REF_EN_START_SELECT           = 0xB6,
    DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD         = 0x4E,
    DYNAMIC_SPAD_REF_EN_START_OFFSET            = 0x4F,
    POWER_MANAGEMENT_GO1_POWER_FORCE            = 0x80,
    VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV           = 0x89,
    ALGO_PHASECAL_LIM                           = 0x30,
    ALGO_PHASECAL_CONFIG_TIMEOUT                = 0x30,
} VL53L0X_RegAddr_t;

typedef enum {
    VcselPeriodPreRange,
    VcselPeriodFinalRange
} VL53L0X_VcselPeriodType_t;

/*============================================================================*/
/* Device Structure                                                           */
/*============================================================================*/

typedef struct {
    uint8_t  address;                     /* I2C address (7-bit) */
    uint8_t  last_status;                 /* Status of last I2C transmission */
    bool     io_2v8;                      /* Use 2.8V I/O mode */
    uint32_t io_timeout;                  /* I/O timeout in ms */
    bool     did_timeout;                 /* Timeout flag */
    uint32_t timeout_start_ms;            /* Timeout start tick */
    uint8_t  stop_variable;               /* StopVariable from DataInit */
    uint32_t measurement_timing_budget_us;
} VL53L0X_Dev_Simple_t;

typedef struct {
    bool tcc, msrc, dss, pre_range, final_range;
} VL53L0X_SequenceStepEnables_t;

typedef struct {
    uint16_t pre_range_vcsel_period_pclks, final_range_vcsel_period_pclks;
    uint16_t msrc_dss_tcc_mclks, pre_range_mclks, final_range_mclks;
    uint32_t msrc_dss_tcc_us, pre_range_us, final_range_us;
} VL53L0X_SequenceStepTimeouts_t;

/*============================================================================*/
/* Public API Functions                                                       */
/*============================================================================*/

/**
 * @brief Initialize the VL53L0X sensor
 * @param dev Pointer to device structure
 * @return true on success, false on failure
 */
bool VL53L0X_Simple_Init(VL53L0X_Dev_Simple_t* dev);

/**
 * @brief Perform a single range measurement
 * @param dev Pointer to device structure
 * @return Distance in millimeters, or 65535 on timeout
 */
uint16_t VL53L0X_Simple_ReadRangeSingleMillimeters(VL53L0X_Dev_Simple_t* dev);

/**
 * @brief Check if a timeout occurred
 * @param dev Pointer to device structure
 * @return true if timeout occurred
 */
bool VL53L0X_Simple_TimeoutOccurred(VL53L0X_Dev_Simple_t* dev);

/**
 * @brief Set the measurement timing budget
 * @param dev Pointer to device structure
 * @param budget_us Timing budget in microseconds (minimum 20000)
 * @return true on success
 */
bool VL53L0X_Simple_SetMeasurementTimingBudget(VL53L0X_Dev_Simple_t* dev, uint32_t budget_us);

/**
 * @brief Get the measurement timing budget
 * @param dev Pointer to device structure
 * @return Timing budget in microseconds
 */
uint32_t VL53L0X_Simple_GetMeasurementTimingBudget(VL53L0X_Dev_Simple_t* dev);

/**
 * @brief Set I/O timeout
 * @param dev Pointer to device structure
 * @param timeout_ms Timeout in milliseconds (0 to disable)
 */
void VL53L0X_Simple_SetTimeout(VL53L0X_Dev_Simple_t* dev, uint32_t timeout_ms);

/*============================================================================*/
/* Low-level I/O Functions (for advanced use)                                 */
/*============================================================================*/

void VL53L0X_Simple_WriteReg(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t value);
void VL53L0X_Simple_WriteReg16Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint16_t value);
void VL53L0X_Simple_WriteReg32Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint32_t value);
uint8_t VL53L0X_Simple_ReadReg(VL53L0X_Dev_Simple_t* dev, uint8_t reg);
uint16_t VL53L0X_Simple_ReadReg16Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg);
uint32_t VL53L0X_Simple_ReadReg32Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg);
void VL53L0X_Simple_WriteMulti(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t* src, uint8_t count);
void VL53L0X_Simple_ReadMulti(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t* dst, uint8_t count);

#ifdef __cplusplus
}
#endif

#endif /* VL53L0X_SIMPLE_H */
