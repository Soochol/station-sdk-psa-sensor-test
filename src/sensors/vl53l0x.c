/**
 * @file vl53l0x.c
 * @brief VL53L0X ToF distance sensor driver implementation
 * @note Uses simple Pololu-based library (MarcelMG/VL53L0X-STM32F103 port)
 */

#include "sensors/vl53l0x.h"
#include "vl53l0x_simple.h"
#include "hal/i2c_handler.h"
#include "config.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>

/*============================================================================*/
/* Debug Configuration                                                        */
/*============================================================================*/

/**
 * @brief Enable/disable VL53L0X debug output
 * Set to 1 to enable debug output via UART, 0 to disable
 */
#define VL53L0X_DEBUG_ENABLE   0

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static SensorSpec_t current_spec;
static bool spec_set = false;
static bool initialized = false;

/* VL53L0X device handle */
static VL53L0X_Dev_Simple_t vl53l0x_dev;

/* External debug variable defined in main.cpp */
extern volatile int dbg_vl53l0x_step;

/* Local debug variables */
volatile int dbg_vl53l0x_api_err = 0;
volatile uint16_t dbg_vl53l0x_measured = 0;
volatile uint8_t  dbg_vl53l0x_range_status = 0xFF;
volatile uint16_t dbg_vl53l0x_target = 0;
volatile uint16_t dbg_vl53l0x_tolerance = 0;
volatile uint16_t dbg_vl53l0x_diff = 0;
volatile int      dbg_vl53l0x_test_step = 0;

/*============================================================================*/
/* Debug Functions (conditionally compiled)                                   */
/*============================================================================*/

#if VL53L0X_DEBUG_ENABLE

extern UART_HandleTypeDef huart4;

static void dbg_print(const char* msg)
{
    HAL_UART_Transmit(&huart4, (uint8_t*)msg, strlen(msg), 100);
}

static void dbg_printf(const char* fmt, ...)
{
    char buf[128];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buf, sizeof(buf), fmt, args);
    va_end(args);
    dbg_print(buf);
}

#define DBG_PRINT(msg)      dbg_print(msg)
#define DBG_PRINTF(...)     dbg_printf(__VA_ARGS__)

#else /* !VL53L0X_DEBUG_ENABLE */

#define DBG_PRINT(msg)      ((void)0)
#define DBG_PRINTF(...)     ((void)0)

#endif /* VL53L0X_DEBUG_ENABLE */

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static HAL_StatusTypeDef VL53L0X_Init_Driver(void);
static void VL53L0X_Deinit(void);
static void VL53L0X_SetSpec(const SensorSpec_t* spec);
static void VL53L0X_GetSpec(SensorSpec_t* spec);
static bool VL53L0X_HasSpec(void);
static TestStatus_t VL53L0X_RunTest(SensorResult_t* result);
static uint8_t VL53L0X_SerializeSpec(const SensorSpec_t* spec, uint8_t* buffer);
static uint8_t VL53L0X_ParseSpec(const uint8_t* buffer, SensorSpec_t* spec);
static uint8_t VL53L0X_SerializeResult(const SensorResult_t* result, uint8_t* buffer);

/*============================================================================*/
/* Driver Instance                                                            */
/*============================================================================*/

const SensorDriver_t VL53L0X_Driver = {
    .id             = SENSOR_ID_VL53L0X,
    .name           = "VL53L0X",
    .init           = VL53L0X_Init_Driver,
    .deinit         = VL53L0X_Deinit,
    .set_spec       = VL53L0X_SetSpec,
    .get_spec       = VL53L0X_GetSpec,
    .has_spec       = VL53L0X_HasSpec,
    .run_test       = VL53L0X_RunTest,
    .serialize_spec = VL53L0X_SerializeSpec,
    .parse_spec     = VL53L0X_ParseSpec,
    .serialize_result = VL53L0X_SerializeResult,
};

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

static HAL_StatusTypeDef VL53L0X_Init_Driver(void)
{
    HAL_StatusTypeDef hal_status;

    dbg_vl53l0x_step = 10;
    DBG_PRINT("\r\n[VL53L0X] Init start (simple driver)\r\n");

    if (initialized) {
        DBG_PRINT("[VL53L0X] Already initialized\r\n");
        return HAL_OK;
    }

    /* NOTE: XSHUT power-up sequence is handled in main.cpp:
     *   - MX_GPIO_Init(): 12V OFF, 2s delay, XSHUT LOW
     *   - main(): 12V ON, 100ms delay, XSHUT HIGH
     * No need to repeat here - sensor is already booted when this is called.
     */

    /* Check device presence first */
    DBG_PRINTF("[VL53L0X] I2C check addr=0x%02X...", VL53L0X_I2C_ADDR);
    hal_status = I2C_Handler_IsDeviceReady(VL53L0X_I2C_BUS, VL53L0X_I2C_ADDR, TIMEOUT_I2C_MS);
    if (hal_status != HAL_OK) {
        DBG_PRINTF("FAIL (hal=%d)\r\n", hal_status);
        dbg_vl53l0x_step = -12;
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Wait for sensor boot - check model ID */
    dbg_vl53l0x_step = 15;
    DBG_PRINT("[VL53L0X] Wait for boot (model_id=0xEE)...");

    memset(&vl53l0x_dev, 0, sizeof(vl53l0x_dev));
    vl53l0x_dev.address = VL53L0X_I2C_ADDR;
    vl53l0x_dev.io_timeout = 500;  /* 500ms timeout */

    uint8_t model_id = 0;
    for (int retry = 0; retry < 100; retry++) {
        model_id = VL53L0X_Simple_ReadReg(&vl53l0x_dev, IDENTIFICATION_MODEL_ID);
        if (model_id == 0xEE) break;
        HAL_Delay(5);
    }
    if (model_id != 0xEE) {
        DBG_PRINTF("FAIL (got 0x%02X)\r\n", model_id);
        dbg_vl53l0x_step = -15;
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Initialize using simple driver */
    dbg_vl53l0x_step = 20;
    DBG_PRINT("[VL53L0X] Simple_Init...");

    if (!VL53L0X_Simple_Init(&vl53l0x_dev)) {
        DBG_PRINT("FAIL\r\n");
        dbg_vl53l0x_step = -20;
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Set measurement timing budget */
    dbg_vl53l0x_step = 30;
    DBG_PRINTF("[VL53L0X] Set timing budget=%uus...", VL53L0X_TIMING_BUDGET_US);
    if (!VL53L0X_Simple_SetMeasurementTimingBudget(&vl53l0x_dev, VL53L0X_TIMING_BUDGET_US)) {
        DBG_PRINT("FAIL\r\n");
        /* Non-fatal, continue with default */
    } else {
        DBG_PRINT("OK\r\n");
    }

    dbg_vl53l0x_step = 100;
    initialized = true;
    DBG_PRINT("[VL53L0X] Init complete!\r\n");
    return HAL_OK;
}

static void VL53L0X_Deinit(void)
{
    initialized = false;
}

static void VL53L0X_SetSpec(const SensorSpec_t* spec)
{
    if (spec != NULL) {
        current_spec = *spec;
        spec_set = true;
    }
}

static void VL53L0X_GetSpec(SensorSpec_t* spec)
{
    if (spec != NULL) {
        *spec = current_spec;
    }
}

static bool VL53L0X_HasSpec(void)
{
    return spec_set;
}

static TestStatus_t VL53L0X_RunTest(SensorResult_t* result)
{
    uint16_t measured_mm;

    dbg_vl53l0x_test_step = 100;
    DBG_PRINT("\r\n[VL53L0X] RunTest start\r\n");

    if (result == NULL) {
        DBG_PRINT("[VL53L0X] ERROR: result=NULL\r\n");
        return STATUS_FAIL_INVALID;
    }

    if (!spec_set) {
        DBG_PRINT("[VL53L0X] ERROR: no spec set\r\n");
        dbg_vl53l0x_test_step = -110;
        return STATUS_FAIL_NO_SPEC;
    }

    dbg_vl53l0x_target = current_spec.vl53l0x.target_dist;
    dbg_vl53l0x_tolerance = current_spec.vl53l0x.tolerance;
    DBG_PRINTF("[VL53L0X] Spec: target=%umm, tol=%umm\r\n",
               current_spec.vl53l0x.target_dist, current_spec.vl53l0x.tolerance);

    if (!initialized) {
        dbg_vl53l0x_test_step = 120;
        DBG_PRINT("[VL53L0X] Not initialized, calling init...\r\n");
        if (VL53L0X_Init_Driver() != HAL_OK) {
            DBG_PRINT("[VL53L0X] Init failed!\r\n");
            return STATUS_FAIL_INIT;
        }
    }

    /* Perform single ranging measurement */
    dbg_vl53l0x_test_step = 130;
    DBG_PRINT("[VL53L0X] ReadRangeSingleMillimeters...");

    measured_mm = VL53L0X_Simple_ReadRangeSingleMillimeters(&vl53l0x_dev);

    if (VL53L0X_Simple_TimeoutOccurred(&vl53l0x_dev)) {
        DBG_PRINT("TIMEOUT\r\n");
        dbg_vl53l0x_test_step = -130;
        return STATUS_FAIL_TIMEOUT;
    }
    DBG_PRINTF("OK (%umm)\r\n", measured_mm);

    dbg_vl53l0x_test_step = 140;
    dbg_vl53l0x_measured = measured_mm;
    DBG_PRINTF("[VL53L0X] Measured: %umm\r\n", measured_mm);

    /* Fill result structure */
    result->vl53l0x.measured = measured_mm;
    result->vl53l0x.target = current_spec.vl53l0x.target_dist;
    result->vl53l0x.tolerance = current_spec.vl53l0x.tolerance;

    /* Calculate difference */
    dbg_vl53l0x_test_step = 160;
    int16_t diff = (int16_t)measured_mm - (int16_t)current_spec.vl53l0x.target_dist;
    if (diff < 0) diff = -diff;
    result->vl53l0x.diff = (uint16_t)diff;
    dbg_vl53l0x_diff = (uint16_t)diff;
    DBG_PRINTF("[VL53L0X] Diff: %umm (tolerance: %umm)\r\n",
               result->vl53l0x.diff, current_spec.vl53l0x.tolerance);

    /* Check against tolerance */
    dbg_vl53l0x_test_step = 170;
    if (result->vl53l0x.diff > current_spec.vl53l0x.tolerance) {
        DBG_PRINT("[VL53L0X] FAIL: out of tolerance\r\n");
        dbg_vl53l0x_test_step = -170;
        return STATUS_FAIL_INVALID;
    }

    dbg_vl53l0x_test_step = 200;
    DBG_PRINT("[VL53L0X] PASS\r\n");
    return STATUS_PASS;
}

static uint8_t VL53L0X_SerializeSpec(const SensorSpec_t* spec, uint8_t* buffer)
{
    if (spec == NULL || buffer == NULL) {
        return 0;
    }

    /* Format: [target_dist_hi][target_dist_lo][tolerance_hi][tolerance_lo] */
    buffer[0] = (uint8_t)(spec->vl53l0x.target_dist >> 8);
    buffer[1] = (uint8_t)(spec->vl53l0x.target_dist & 0xFF);
    buffer[2] = (uint8_t)(spec->vl53l0x.tolerance >> 8);
    buffer[3] = (uint8_t)(spec->vl53l0x.tolerance & 0xFF);

    return 4;
}

static uint8_t VL53L0X_ParseSpec(const uint8_t* buffer, SensorSpec_t* spec)
{
    if (buffer == NULL || spec == NULL) {
        return 0;
    }

    /* Parse big-endian format */
    spec->vl53l0x.target_dist = (uint16_t)((buffer[0] << 8) | buffer[1]);
    spec->vl53l0x.tolerance = (uint16_t)((buffer[2] << 8) | buffer[3]);

    return 4;
}

static uint8_t VL53L0X_SerializeResult(const SensorResult_t* result, uint8_t* buffer)
{
    if (result == NULL || buffer == NULL) {
        return 0;
    }

    /* Format: [measured][target][tolerance][diff] - all 16-bit big-endian */
    buffer[0] = (uint8_t)(result->vl53l0x.measured >> 8);
    buffer[1] = (uint8_t)(result->vl53l0x.measured & 0xFF);
    buffer[2] = (uint8_t)(result->vl53l0x.target >> 8);
    buffer[3] = (uint8_t)(result->vl53l0x.target & 0xFF);
    buffer[4] = (uint8_t)(result->vl53l0x.tolerance >> 8);
    buffer[5] = (uint8_t)(result->vl53l0x.tolerance & 0xFF);
    buffer[6] = (uint8_t)(result->vl53l0x.diff >> 8);
    buffer[7] = (uint8_t)(result->vl53l0x.diff & 0xFF);

    return 8;
}

/*============================================================================*/
/* Direct Init for Debugging (bypasses function pointer)                      */
/*============================================================================*/

__attribute__((noinline)) void VL53L0X_SimpleTest(void)
{
    dbg_vl53l0x_step = 1;
}

__attribute__((noinline)) HAL_StatusTypeDef VL53L0X_DirectInit(void)
{
    dbg_vl53l0x_step = 5;
    return VL53L0X_Init_Driver();
}
