/**
 * @file vl53l0x_simple.c
 * @brief Simple VL53L0X driver based on Pololu library (C port)
 * @note Based on MarcelMG/VL53L0X-STM32F103, adapted for STM32H7
 *
 * Most of the functionality of this library is based on the VL53L0X API
 * provided by ST (STSW-IMG005), and some of the explanatory comments are quoted
 * or paraphrased from the API source code, API user manual (UM2039), and the
 * VL53L0X datasheet.
 *
 * Original: https://github.com/MarcelMG/VL53L0X-STM32F103
 * License: MIT (Pololu) / GPL-3.0 (MarcelMG)
 */

#include "vl53l0x_simple.h"
#include "hal/i2c_handler.h"
#include "config.h"
#include "main.h"
#include <string.h>

/*============================================================================*/
/* Macros                                                                     */
/*============================================================================*/

/* Decode VCSEL (vertical cavity surface emitting laser) pulse period in PCLKs
   from register value */
#define decodeVcselPeriod(reg_val) (((reg_val) + 1) << 1)

/* Encode VCSEL pulse period register value from period in PCLKs */
#define encodeVcselPeriod(period_pclks) (((period_pclks) >> 1) - 1)

/* Calculate macro period in *nanoseconds* from VCSEL period in PCLKs */
#define calcMacroPeriod(vcsel_period_pclks) \
    ((((uint32_t)2304 * (vcsel_period_pclks) * 1655) + 500) / 1000)

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static bool getSpadInfo(VL53L0X_Dev_Simple_t* dev, uint8_t* count, bool* type_is_aperture);
static void getSequenceStepEnables(VL53L0X_Dev_Simple_t* dev, VL53L0X_SequenceStepEnables_t* enables);
static void getSequenceStepTimeouts(VL53L0X_Dev_Simple_t* dev, VL53L0X_SequenceStepEnables_t* enables,
                                     VL53L0X_SequenceStepTimeouts_t* timeouts);
static bool performSingleRefCalibration(VL53L0X_Dev_Simple_t* dev, uint8_t vhv_init_byte);
static uint16_t decodeTimeout(uint16_t value);
static uint16_t encodeTimeout(uint16_t timeout_mclks);
static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks);
static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks);
static uint8_t getVcselPulsePeriod(VL53L0X_Dev_Simple_t* dev, VL53L0X_VcselPeriodType_t type);
static bool setSignalRateLimit(VL53L0X_Dev_Simple_t* dev, float limit_Mcps);
static void startTimeout(VL53L0X_Dev_Simple_t* dev);
static bool checkTimeoutExpired(VL53L0X_Dev_Simple_t* dev);
static uint16_t readRangeContinuousMillimeters(VL53L0X_Dev_Simple_t* dev);

/*============================================================================*/
/* Low-Level I/O Functions                                                    */
/*============================================================================*/

void VL53L0X_Simple_WriteReg(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t value)
{
    dev->last_status = I2C_Handler_Write8(VL53L0X_I2C_BUS, dev->address, reg, &value, 1, TIMEOUT_I2C_MS);
}

void VL53L0X_Simple_WriteReg16Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint16_t value)
{
    uint8_t buf[2];
    buf[0] = (uint8_t)(value >> 8);
    buf[1] = (uint8_t)(value & 0xFF);
    dev->last_status = I2C_Handler_Write8(VL53L0X_I2C_BUS, dev->address, reg, buf, 2, TIMEOUT_I2C_MS);
}

void VL53L0X_Simple_WriteReg32Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint32_t value)
{
    uint8_t buf[4];
    buf[0] = (uint8_t)(value >> 24);
    buf[1] = (uint8_t)(value >> 16);
    buf[2] = (uint8_t)(value >> 8);
    buf[3] = (uint8_t)(value & 0xFF);
    dev->last_status = I2C_Handler_Write8(VL53L0X_I2C_BUS, dev->address, reg, buf, 4, TIMEOUT_I2C_MS);
}

uint8_t VL53L0X_Simple_ReadReg(VL53L0X_Dev_Simple_t* dev, uint8_t reg)
{
    uint8_t value = 0;
    dev->last_status = I2C_Handler_Read8(VL53L0X_I2C_BUS, dev->address, reg, &value, 1, TIMEOUT_I2C_MS);
    return value;
}

uint16_t VL53L0X_Simple_ReadReg16Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg)
{
    uint8_t buf[2];
    dev->last_status = I2C_Handler_Read8(VL53L0X_I2C_BUS, dev->address, reg, buf, 2, TIMEOUT_I2C_MS);
    return ((uint16_t)buf[0] << 8) | (uint16_t)buf[1];
}

uint32_t VL53L0X_Simple_ReadReg32Bit(VL53L0X_Dev_Simple_t* dev, uint8_t reg)
{
    uint8_t buf[4];
    dev->last_status = I2C_Handler_Read8(VL53L0X_I2C_BUS, dev->address, reg, buf, 4, TIMEOUT_I2C_MS);
    return ((uint32_t)buf[0] << 24) | ((uint32_t)buf[1] << 16) |
           ((uint32_t)buf[2] << 8) | (uint32_t)buf[3];
}

void VL53L0X_Simple_WriteMulti(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t* src, uint8_t count)
{
    dev->last_status = I2C_Handler_Write8(VL53L0X_I2C_BUS, dev->address, reg, src, count, TIMEOUT_I2C_MS);
}

void VL53L0X_Simple_ReadMulti(VL53L0X_Dev_Simple_t* dev, uint8_t reg, uint8_t* dst, uint8_t count)
{
    dev->last_status = I2C_Handler_Read8(VL53L0X_I2C_BUS, dev->address, reg, dst, count, TIMEOUT_I2C_MS);
}

/*============================================================================*/
/* Timeout Functions                                                          */
/*============================================================================*/

static void startTimeout(VL53L0X_Dev_Simple_t* dev)
{
    dev->timeout_start_ms = HAL_GetTick();
}

static bool checkTimeoutExpired(VL53L0X_Dev_Simple_t* dev)
{
    return (dev->io_timeout > 0) &&
           ((HAL_GetTick() - dev->timeout_start_ms) > dev->io_timeout);
}

void VL53L0X_Simple_SetTimeout(VL53L0X_Dev_Simple_t* dev, uint32_t timeout_ms)
{
    dev->io_timeout = timeout_ms;
}

bool VL53L0X_Simple_TimeoutOccurred(VL53L0X_Dev_Simple_t* dev)
{
    bool tmp = dev->did_timeout;
    dev->did_timeout = false;
    return tmp;
}

/*============================================================================*/
/* Initialization                                                             */
/*============================================================================*/

bool VL53L0X_Simple_Init(VL53L0X_Dev_Simple_t* dev)
{
    /* Set defaults */
    if (dev->address == 0) {
        dev->address = VL53L0X_I2C_ADDR;
    }
    dev->io_2v8 = true;
    dev->io_timeout = 500;  /* 500ms default timeout */
    dev->did_timeout = false;
    dev->stop_variable = 0;
    dev->measurement_timing_budget_us = 0;

    /* VL53L0X_DataInit() begin */

    /* Sensor uses 1V8 mode for I/O by default; switch to 2V8 mode if necessary */
    if (dev->io_2v8) {
        VL53L0X_Simple_WriteReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV,
            VL53L0X_Simple_ReadReg(dev, VHV_CONFIG_PAD_SCL_SDA__EXTSUP_HV) | 0x01);
    }

    /* Set I2C standard mode */
    VL53L0X_Simple_WriteReg(dev, 0x88, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0x80, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x00);
    dev->stop_variable = VL53L0X_Simple_ReadReg(dev, 0x91);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x00);

    /* Disable SIGNAL_RATE_MSRC (bit 1) and SIGNAL_RATE_PRE_RANGE (bit 4) limit checks */
    VL53L0X_Simple_WriteReg(dev, MSRC_CONFIG_CONTROL,
        VL53L0X_Simple_ReadReg(dev, MSRC_CONFIG_CONTROL) | 0x12);

    /* Set final range signal rate limit to 0.25 MCPS */
    setSignalRateLimit(dev, 0.25f);

    VL53L0X_Simple_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xFF);

    /* VL53L0X_DataInit() end */

    /* VL53L0X_StaticInit() begin */

    uint8_t spad_count;
    bool spad_type_is_aperture;
    if (!getSpadInfo(dev, &spad_count, &spad_type_is_aperture)) {
        return false;
    }

    /* The SPAD map (RefGoodSpadMap) is read by VL53L0X_get_info_from_device() in
       the API, but the same data seems to be more easily readable from
       GLOBAL_CONFIG_SPAD_ENABLES_REF_0 through _6, so read it from there */
    uint8_t ref_spad_map[6];
    VL53L0X_Simple_ReadMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* VL53L0X_set_reference_spads() begin (assume NVM values are valid) */

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, DYNAMIC_SPAD_REF_EN_START_OFFSET, 0x00);
    VL53L0X_Simple_WriteReg(dev, DYNAMIC_SPAD_NUM_REQUESTED_REF_SPAD, 0x2C);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, GLOBAL_CONFIG_REF_EN_START_SELECT, 0xB4);

    uint8_t first_spad_to_enable = spad_type_is_aperture ? 12 : 0;
    uint8_t spads_enabled = 0;

    for (uint8_t i = 0; i < 48; i++) {
        if (i < first_spad_to_enable || spads_enabled == spad_count) {
            ref_spad_map[i / 8] &= ~(1 << (i % 8));
        } else if ((ref_spad_map[i / 8] >> (i % 8)) & 0x1) {
            spads_enabled++;
        }
    }

    VL53L0X_Simple_WriteMulti(dev, GLOBAL_CONFIG_SPAD_ENABLES_REF_0, ref_spad_map, 6);

    /* VL53L0X_set_reference_spads() end */

    /* VL53L0X_load_tuning_settings() begin (DefaultTuningSettings from vl53l0x_tuning.h) */

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x09, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x10, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x11, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0x24, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x25, 0xFF);
    VL53L0X_Simple_WriteReg(dev, 0x75, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x4E, 0x2C);
    VL53L0X_Simple_WriteReg(dev, 0x48, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x30, 0x20);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x30, 0x09);
    VL53L0X_Simple_WriteReg(dev, 0x54, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x31, 0x04);
    VL53L0X_Simple_WriteReg(dev, 0x32, 0x03);
    VL53L0X_Simple_WriteReg(dev, 0x40, 0x83);
    VL53L0X_Simple_WriteReg(dev, 0x46, 0x25);
    VL53L0X_Simple_WriteReg(dev, 0x60, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x27, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x50, 0x06);
    VL53L0X_Simple_WriteReg(dev, 0x51, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x52, 0x96);
    VL53L0X_Simple_WriteReg(dev, 0x56, 0x08);
    VL53L0X_Simple_WriteReg(dev, 0x57, 0x30);
    VL53L0X_Simple_WriteReg(dev, 0x61, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x62, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x64, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x65, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x66, 0xA0);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x22, 0x32);
    VL53L0X_Simple_WriteReg(dev, 0x47, 0x14);
    VL53L0X_Simple_WriteReg(dev, 0x49, 0xFF);
    VL53L0X_Simple_WriteReg(dev, 0x4A, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x7A, 0x0A);
    VL53L0X_Simple_WriteReg(dev, 0x7B, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x78, 0x21);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x23, 0x34);
    VL53L0X_Simple_WriteReg(dev, 0x42, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x44, 0xFF);
    VL53L0X_Simple_WriteReg(dev, 0x45, 0x26);
    VL53L0X_Simple_WriteReg(dev, 0x46, 0x05);
    VL53L0X_Simple_WriteReg(dev, 0x40, 0x40);
    VL53L0X_Simple_WriteReg(dev, 0x0E, 0x06);
    VL53L0X_Simple_WriteReg(dev, 0x20, 0x1A);
    VL53L0X_Simple_WriteReg(dev, 0x43, 0x40);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x34, 0x03);
    VL53L0X_Simple_WriteReg(dev, 0x35, 0x44);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x31, 0x04);
    VL53L0X_Simple_WriteReg(dev, 0x4B, 0x09);
    VL53L0X_Simple_WriteReg(dev, 0x4C, 0x05);
    VL53L0X_Simple_WriteReg(dev, 0x4D, 0x04);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x44, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x45, 0x20);
    VL53L0X_Simple_WriteReg(dev, 0x47, 0x08);
    VL53L0X_Simple_WriteReg(dev, 0x48, 0x28);
    VL53L0X_Simple_WriteReg(dev, 0x67, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x70, 0x04);
    VL53L0X_Simple_WriteReg(dev, 0x71, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x72, 0xFE);
    VL53L0X_Simple_WriteReg(dev, 0x76, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x77, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x0D, 0x01);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x01, 0xF8);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x8E, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x00);

    /* VL53L0X_load_tuning_settings() end */

    /* Set interrupt config to new sample ready */
    /* VL53L0X_SetGpioConfig() begin */

    VL53L0X_Simple_WriteReg(dev, SYSTEM_INTERRUPT_CONFIG_GPIO, 0x04);
    VL53L0X_Simple_WriteReg(dev, GPIO_HV_MUX_ACTIVE_HIGH,
        VL53L0X_Simple_ReadReg(dev, GPIO_HV_MUX_ACTIVE_HIGH) & ~0x10);  /* active low */
    VL53L0X_Simple_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    /* VL53L0X_SetGpioConfig() end */

    dev->measurement_timing_budget_us = VL53L0X_Simple_GetMeasurementTimingBudget(dev);

    /* Disable MSRC and TCC by default */
    /* VL53L0X_SetSequenceStepEnable() begin */

    VL53L0X_Simple_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_SetSequenceStepEnable() end */

    /* Recalculate timing budget */
    VL53L0X_Simple_SetMeasurementTimingBudget(dev, dev->measurement_timing_budget_us);

    /* VL53L0X_StaticInit() end */

    /* VL53L0X_PerformRefCalibration() begin */

    /* VL53L0X_perform_vhv_calibration() begin */

    VL53L0X_Simple_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x01);
    if (!performSingleRefCalibration(dev, 0x40)) {
        return false;
    }

    /* VL53L0X_perform_vhv_calibration() end */

    /* VL53L0X_perform_phase_calibration() begin */

    VL53L0X_Simple_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0x02);
    if (!performSingleRefCalibration(dev, 0x00)) {
        return false;
    }

    /* VL53L0X_perform_phase_calibration() end */

    /* Restore the previous Sequence Config */
    VL53L0X_Simple_WriteReg(dev, SYSTEM_SEQUENCE_CONFIG, 0xE8);

    /* VL53L0X_PerformRefCalibration() end */

    return true;
}

/*============================================================================*/
/* Range Measurement                                                          */
/*============================================================================*/

static uint16_t readRangeContinuousMillimeters(VL53L0X_Dev_Simple_t* dev)
{
    startTimeout(dev);
    while ((VL53L0X_Simple_ReadReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired(dev)) {
            dev->did_timeout = true;
            return 65535;
        }
    }

    /* Assumptions: Linearity Corrective Gain is 1000 (default);
       fractional ranging is not enabled */
    uint16_t range = VL53L0X_Simple_ReadReg16Bit(dev, RESULT_RANGE_STATUS + 10);

    VL53L0X_Simple_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);

    return range;
}

uint16_t VL53L0X_Simple_ReadRangeSingleMillimeters(VL53L0X_Dev_Simple_t* dev)
{
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x91, dev->stop_variable);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x00);

    VL53L0X_Simple_WriteReg(dev, SYSRANGE_START, 0x01);

    /* Wait until start bit has been cleared */
    startTimeout(dev);
    while (VL53L0X_Simple_ReadReg(dev, SYSRANGE_START) & 0x01) {
        if (checkTimeoutExpired(dev)) {
            dev->did_timeout = true;
            return 65535;
        }
    }

    return readRangeContinuousMillimeters(dev);
}

/*============================================================================*/
/* Timing Budget Functions                                                    */
/*============================================================================*/

bool VL53L0X_Simple_SetMeasurementTimingBudget(VL53L0X_Dev_Simple_t* dev, uint32_t budget_us)
{
    VL53L0X_SequenceStepEnables_t enables;
    VL53L0X_SequenceStepTimeouts_t timeouts;

    const uint16_t StartOverhead      = 1320;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    const uint32_t MinTimingBudget = 20000;

    if (budget_us < MinTimingBudget) {
        return false;
    }

    uint32_t used_budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(dev, &enables);
    getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        used_budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        used_budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        used_budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        used_budget_us += FinalRangeOverhead;

        if (used_budget_us > budget_us) {
            return false;
        }

        uint32_t final_range_timeout_us = budget_us - used_budget_us;

        uint16_t final_range_timeout_mclks = timeoutMicrosecondsToMclks(
            final_range_timeout_us, timeouts.final_range_vcsel_period_pclks);

        if (enables.pre_range) {
            final_range_timeout_mclks += timeouts.pre_range_mclks;
        }

        VL53L0X_Simple_WriteReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI,
            encodeTimeout(final_range_timeout_mclks));

        dev->measurement_timing_budget_us = budget_us;
    }
    return true;
}

uint32_t VL53L0X_Simple_GetMeasurementTimingBudget(VL53L0X_Dev_Simple_t* dev)
{
    VL53L0X_SequenceStepEnables_t enables;
    VL53L0X_SequenceStepTimeouts_t timeouts;

    const uint16_t StartOverhead     = 1910;
    const uint16_t EndOverhead        = 960;
    const uint16_t MsrcOverhead       = 660;
    const uint16_t TccOverhead        = 590;
    const uint16_t DssOverhead        = 690;
    const uint16_t PreRangeOverhead   = 660;
    const uint16_t FinalRangeOverhead = 550;

    uint32_t budget_us = StartOverhead + EndOverhead;

    getSequenceStepEnables(dev, &enables);
    getSequenceStepTimeouts(dev, &enables, &timeouts);

    if (enables.tcc) {
        budget_us += (timeouts.msrc_dss_tcc_us + TccOverhead);
    }

    if (enables.dss) {
        budget_us += 2 * (timeouts.msrc_dss_tcc_us + DssOverhead);
    } else if (enables.msrc) {
        budget_us += (timeouts.msrc_dss_tcc_us + MsrcOverhead);
    }

    if (enables.pre_range) {
        budget_us += (timeouts.pre_range_us + PreRangeOverhead);
    }

    if (enables.final_range) {
        budget_us += (timeouts.final_range_us + FinalRangeOverhead);
    }

    dev->measurement_timing_budget_us = budget_us;
    return budget_us;
}

/*============================================================================*/
/* Helper Functions                                                           */
/*============================================================================*/

static bool setSignalRateLimit(VL53L0X_Dev_Simple_t* dev, float limit_Mcps)
{
    if (limit_Mcps < 0 || limit_Mcps > 511.99f) {
        return false;
    }

    /* Q9.7 fixed point format (9 integer bits, 7 fractional bits) */
    VL53L0X_Simple_WriteReg16Bit(dev, FINAL_RANGE_CONFIG_MIN_COUNT_RATE_RTN_LIMIT,
        (uint16_t)(limit_Mcps * (1 << 7)));
    return true;
}

static bool getSpadInfo(VL53L0X_Dev_Simple_t* dev, uint8_t* count, bool* type_is_aperture)
{
    uint8_t tmp;

    VL53L0X_Simple_WriteReg(dev, 0x80, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x00);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x06);
    VL53L0X_Simple_WriteReg(dev, 0x83, VL53L0X_Simple_ReadReg(dev, 0x83) | 0x04);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x07);
    VL53L0X_Simple_WriteReg(dev, 0x81, 0x01);

    VL53L0X_Simple_WriteReg(dev, 0x80, 0x01);

    VL53L0X_Simple_WriteReg(dev, 0x94, 0x6b);
    VL53L0X_Simple_WriteReg(dev, 0x83, 0x00);
    startTimeout(dev);
    while (VL53L0X_Simple_ReadReg(dev, 0x83) == 0x00) {
        if (checkTimeoutExpired(dev)) {
            return false;
        }
    }
    VL53L0X_Simple_WriteReg(dev, 0x83, 0x01);
    tmp = VL53L0X_Simple_ReadReg(dev, 0x92);

    *count = tmp & 0x7f;
    *type_is_aperture = (tmp >> 7) & 0x01;

    VL53L0X_Simple_WriteReg(dev, 0x81, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x06);
    VL53L0X_Simple_WriteReg(dev, 0x83, VL53L0X_Simple_ReadReg(dev, 0x83) & ~0x04);
    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x01);
    VL53L0X_Simple_WriteReg(dev, 0x00, 0x01);

    VL53L0X_Simple_WriteReg(dev, 0xFF, 0x00);
    VL53L0X_Simple_WriteReg(dev, 0x80, 0x00);

    return true;
}

static void getSequenceStepEnables(VL53L0X_Dev_Simple_t* dev, VL53L0X_SequenceStepEnables_t* enables)
{
    uint8_t sequence_config = VL53L0X_Simple_ReadReg(dev, SYSTEM_SEQUENCE_CONFIG);

    enables->tcc          = (sequence_config >> 4) & 0x1;
    enables->dss          = (sequence_config >> 3) & 0x1;
    enables->msrc         = (sequence_config >> 2) & 0x1;
    enables->pre_range    = (sequence_config >> 6) & 0x1;
    enables->final_range  = (sequence_config >> 7) & 0x1;
}

static void getSequenceStepTimeouts(VL53L0X_Dev_Simple_t* dev,
                                     VL53L0X_SequenceStepEnables_t* enables,
                                     VL53L0X_SequenceStepTimeouts_t* timeouts)
{
    timeouts->pre_range_vcsel_period_pclks = getVcselPulsePeriod(dev, VcselPeriodPreRange);

    timeouts->msrc_dss_tcc_mclks = VL53L0X_Simple_ReadReg(dev, MSRC_CONFIG_TIMEOUT_MACROP) + 1;
    timeouts->msrc_dss_tcc_us = timeoutMclksToMicroseconds(
        timeouts->msrc_dss_tcc_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->pre_range_mclks = decodeTimeout(
        VL53L0X_Simple_ReadReg16Bit(dev, PRE_RANGE_CONFIG_TIMEOUT_MACROP_HI));
    timeouts->pre_range_us = timeoutMclksToMicroseconds(
        timeouts->pre_range_mclks, timeouts->pre_range_vcsel_period_pclks);

    timeouts->final_range_vcsel_period_pclks = getVcselPulsePeriod(dev, VcselPeriodFinalRange);

    timeouts->final_range_mclks = decodeTimeout(
        VL53L0X_Simple_ReadReg16Bit(dev, FINAL_RANGE_CONFIG_TIMEOUT_MACROP_HI));

    if (enables->pre_range) {
        timeouts->final_range_mclks -= timeouts->pre_range_mclks;
    }

    timeouts->final_range_us = timeoutMclksToMicroseconds(
        timeouts->final_range_mclks, timeouts->final_range_vcsel_period_pclks);

    (void)enables;  /* Suppress unused warning when pre_range check is the only use */
}

static bool performSingleRefCalibration(VL53L0X_Dev_Simple_t* dev, uint8_t vhv_init_byte)
{
    VL53L0X_Simple_WriteReg(dev, SYSRANGE_START, 0x01 | vhv_init_byte);

    startTimeout(dev);
    while ((VL53L0X_Simple_ReadReg(dev, RESULT_INTERRUPT_STATUS) & 0x07) == 0) {
        if (checkTimeoutExpired(dev)) {
            return false;
        }
    }

    VL53L0X_Simple_WriteReg(dev, SYSTEM_INTERRUPT_CLEAR, 0x01);
    VL53L0X_Simple_WriteReg(dev, SYSRANGE_START, 0x00);

    return true;
}

static uint8_t getVcselPulsePeriod(VL53L0X_Dev_Simple_t* dev, VL53L0X_VcselPeriodType_t type)
{
    if (type == VcselPeriodPreRange) {
        return decodeVcselPeriod(VL53L0X_Simple_ReadReg(dev, PRE_RANGE_CONFIG_VCSEL_PERIOD));
    } else if (type == VcselPeriodFinalRange) {
        return decodeVcselPeriod(VL53L0X_Simple_ReadReg(dev, FINAL_RANGE_CONFIG_VCSEL_PERIOD));
    } else {
        return 255;
    }
}

static uint16_t decodeTimeout(uint16_t reg_val)
{
    /* Format: (LSByte * 2^MSByte) + 1 */
    return (uint16_t)((reg_val & 0x00FF) << (uint16_t)((reg_val & 0xFF00) >> 8)) + 1;
}

static uint16_t encodeTimeout(uint16_t timeout_mclks)
{
    /* Format: (LSByte * 2^MSByte) + 1 */
    uint32_t ls_byte = 0;
    uint16_t ms_byte = 0;

    if (timeout_mclks > 0) {
        ls_byte = timeout_mclks - 1;

        while ((ls_byte & 0xFFFFFF00) > 0) {
            ls_byte >>= 1;
            ms_byte++;
        }

        return (ms_byte << 8) | (ls_byte & 0xFF);
    } else {
        return 0;
    }
}

static uint32_t timeoutMclksToMicroseconds(uint16_t timeout_period_mclks, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return ((timeout_period_mclks * macro_period_ns) + (macro_period_ns / 2)) / 1000;
}

static uint32_t timeoutMicrosecondsToMclks(uint32_t timeout_period_us, uint8_t vcsel_period_pclks)
{
    uint32_t macro_period_ns = calcMacroPeriod(vcsel_period_pclks);
    return (((timeout_period_us * 1000) + (macro_period_ns / 2)) / macro_period_ns);
}
