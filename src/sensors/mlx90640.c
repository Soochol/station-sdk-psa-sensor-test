/**
 * @file mlx90640.c
 * @brief MLX90640 IR thermal array sensor driver implementation
 */

#include "sensors/mlx90640.h"
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "hal/i2c_handler.h"
#include "config.h"
#include "main.h"
#include <string.h>
#include <stdio.h>
#include <stdarg.h>
#include <math.h>

/*============================================================================*/
/* Debug Configuration                                                        */
/*============================================================================*/

/**
 * @brief Enable/disable MLX90640 debug output
 * Set to 1 to enable debug output via UART, 0 to disable
 */
#define MLX90640_DEBUG_ENABLE   0

/*============================================================================*/
/* Private Variables                                                          */
/*============================================================================*/

static SensorSpec_t current_spec;
static bool spec_set = false;
static bool initialized = false;

/* MLX90640 data buffers */
static paramsMLX90640 mlx_params;
static uint16_t eeData[832];
static uint16_t frameData[834];
static float mlxTemperatures[768];

/*============================================================================*/
/* Debug Functions (conditionally compiled)                                   */
/*============================================================================*/

#if MLX90640_DEBUG_ENABLE

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

/** @brief Print thermal image grid with min/max/avg summary */
static void Debug_PrintThermalImage(const float* temps, float min_t, float max_t, float avg_t)
{
    int max_x = 0, max_y = 0;

    dbg_print("\r\n[MLX90640] Thermal Image (32x24):\r\n");
    for (int y = 0; y < 24; y++) {
        for (int x = 0; x < 32; x++) {
            int idx = y * 32 + x;
            float t = temps[idx];
            int t_int = (int)t;
            int t_dec = ((int)(t * 100) % 100 + 100) % 100;
            dbg_printf("%2d.%02d ", t_int, t_dec);
            if (temps[idx] == max_t) {
                max_x = x;
                max_y = y;
            }
        }
        dbg_print("\r\n");
    }
    dbg_printf("[MLX90640] Min: %d.%02dC  Max: %d.%02dC  Avg: %d.%02dC\r\n",
               (int)min_t, ((int)(min_t * 100) % 100 + 100) % 100,
               (int)max_t, ((int)(max_t * 100) % 100 + 100) % 100,
               (int)avg_t, ((int)(avg_t * 100) % 100 + 100) % 100);
    dbg_printf("[MLX90640] Max at pixel(%d,%d)\r\n", max_x, max_y);
}

/** @brief Print frame data diagnostics */
static void Debug_PrintFrameInfo(const uint16_t* frame, const paramsMLX90640* params, int subpage)
{
    dbg_printf("OK (subpage=%d)\r\n", subpage);
    dbg_printf("[MLX90640] Frame[768]=%u, Frame[800]=%u\r\n", frame[768], frame[800]);
    dbg_printf("[MLX90640] Frame[810]=%u (Vdd raw)\r\n", frame[810]);
    dbg_printf("[MLX90640] vPTAT25=%u, KtPTAT=%d.%d\r\n",
               params->vPTAT25,
               (int)params->KtPTAT,
               ((int)(params->KtPTAT * 100) % 100 + 100) % 100);
    dbg_printf("[MLX90640] alphaPTAT=%d.%d, KvPTAT=%d.%d\r\n",
               (int)params->alphaPTAT,
               ((int)(params->alphaPTAT * 10) % 10 + 10) % 10,
               (int)(params->KvPTAT * 1000),
               ((int)(params->KvPTAT * 10000) % 10 + 10) % 10);

    int resolutionRAM = (frame[832] & 0x0C00) >> 10;
    dbg_printf("[MLX90640] resolutionRAM=%d, resolutionEE=%d\r\n",
               resolutionRAM, params->resolutionEE);

    float vdd = MLX90640_GetVdd((uint16_t*)frame, params);
    dbg_printf("[MLX90640] Vdd=%d.%02dV\r\n", (int)vdd, ((int)(vdd * 100) % 100 + 100) % 100);
}

/** @brief Print EEPROM and calibration info */
static void Debug_PrintCalibration(const uint16_t* ee, const paramsMLX90640* params)
{
    dbg_printf("[MLX90640] EEPROM[0-3]: 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
               ee[0], ee[1], ee[2], ee[3]);
    dbg_printf("[MLX90640] EEPROM[48-51]: 0x%04X 0x%04X 0x%04X 0x%04X\r\n",
               ee[48], ee[49], ee[50], ee[51]);
    dbg_printf("[MLX90640] gainEE=%d, vdd25=%d, kVdd=%d\r\n",
               params->gainEE, params->vdd25, params->kVdd);
    dbg_printf("[MLX90640] resolutionEE=%d (EEPROM[56]=0x%04X)\r\n",
               params->resolutionEE, ee[56]);
}

#define DBG_PRINT(msg)                          dbg_print(msg)
#define DBG_PRINTF(...)                         dbg_printf(__VA_ARGS__)
#define DBG_THERMAL_IMAGE(t, min, max, avg)     Debug_PrintThermalImage(t, min, max, avg)
#define DBG_FRAME_INFO(f, p, s)                 Debug_PrintFrameInfo(f, p, s)
#define DBG_CALIBRATION(e, p)                   Debug_PrintCalibration(e, p)

#else /* !MLX90640_DEBUG_ENABLE */

#define DBG_PRINT(msg)                          ((void)0)
#define DBG_PRINTF(...)                         ((void)0)
#define DBG_THERMAL_IMAGE(t, min, max, avg)     ((void)0)
#define DBG_FRAME_INFO(f, p, s)                 ((void)0)
#define DBG_CALIBRATION(e, p)                   ((void)0)

#endif /* MLX90640_DEBUG_ENABLE */

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static HAL_StatusTypeDef MLX90640_Init_Driver(void);
static void MLX90640_Deinit(void);
static void MLX90640_SetSpec(const SensorSpec_t* spec);
static void MLX90640_GetSpec(SensorSpec_t* spec);
static bool MLX90640_HasSpec(void);
static TestStatus_t MLX90640_RunTest(SensorResult_t* result);
static uint8_t MLX90640_SerializeSpec(const SensorSpec_t* spec, uint8_t* buffer);
static uint8_t MLX90640_ParseSpec(const uint8_t* buffer, SensorSpec_t* spec);
static uint8_t MLX90640_SerializeResult(const SensorResult_t* result, uint8_t* buffer);

/*============================================================================*/
/* Driver Instance                                                            */
/*============================================================================*/

const SensorDriver_t MLX90640_Driver = {
    .id             = SENSOR_ID_MLX90640,
    .name           = "MLX90640",
    .init           = MLX90640_Init_Driver,
    .deinit         = MLX90640_Deinit,
    .set_spec       = MLX90640_SetSpec,
    .get_spec       = MLX90640_GetSpec,
    .has_spec       = MLX90640_HasSpec,
    .run_test       = MLX90640_RunTest,
    .serialize_spec = MLX90640_SerializeSpec,
    .parse_spec     = MLX90640_ParseSpec,
    .serialize_result = MLX90640_SerializeResult,
};

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

static HAL_StatusTypeDef MLX90640_Init_Driver(void)
{
    HAL_StatusTypeDef hal_status;
    int mlx_status;

    DBG_PRINT("\r\n[MLX90640] Init start\r\n");

    if (initialized) {
        DBG_PRINT("[MLX90640] Already initialized\r\n");
        return HAL_OK;
    }

    /* Check device presence */
    DBG_PRINTF("[MLX90640] I2C check addr=0x%02X...", MLX90640_I2C_ADDR);
    hal_status = I2C_Handler_IsDeviceReady(MLX90640_I2C_BUS, MLX90640_I2C_ADDR, TIMEOUT_I2C_MS);
    if (hal_status != HAL_OK) {
        DBG_PRINTF("FAIL (hal=%d)\r\n", hal_status);
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Initialize I2C driver */
    MLX90640_I2CInit();

    /* Set refresh rate */
    DBG_PRINTF("[MLX90640] Set refresh rate=%d...", MLX90640_REFRESH_RATE);
    mlx_status = MLX90640_SetRefreshRate(MLX90640_I2C_ADDR, MLX90640_REFRESH_RATE);
    if (mlx_status != 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Set resolution */
    DBG_PRINTF("[MLX90640] Set resolution=%d...", MLX90640_RESOLUTION);
    mlx_status = MLX90640_SetResolution(MLX90640_I2C_ADDR, MLX90640_RESOLUTION);
    if (mlx_status != 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Read EEPROM */
    DBG_PRINT("[MLX90640] Dump EEPROM...");
    mlx_status = MLX90640_DumpEE(MLX90640_I2C_ADDR, eeData);
    if (mlx_status != 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Extract calibration parameters */
    DBG_PRINT("[MLX90640] Extract params...");
    mlx_status = MLX90640_ExtractParameters(eeData, &mlx_params);
    if (mlx_status != 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        return HAL_ERROR;
    }
    DBG_PRINT("OK\r\n");

    /* Debug: Print EEPROM and calibration info */
    DBG_CALIBRATION(eeData, &mlx_params);

    /* CRITICAL: Set sensor resolution to match EEPROM calibration resolution */
    /* resolutionEE: 0=16bit, 1=17bit, 2=18bit, 3=19bit */
    uint8_t calibResolution = mlx_params.resolutionEE + 16;
    DBG_PRINTF("[MLX90640] Setting resolution to %d (from EEPROM)\r\n", calibResolution);
    mlx_status = MLX90640_SetResolution(MLX90640_I2C_ADDR, calibResolution);
    if (mlx_status != 0) {
        DBG_PRINTF("[MLX90640] SetResolution FAIL (err=%d)\r\n", mlx_status);
        return HAL_ERROR;
    }

    initialized = true;
    DBG_PRINT("[MLX90640] Init complete!\r\n");
    return HAL_OK;
}

static void MLX90640_Deinit(void)
{
    initialized = false;
}

static void MLX90640_SetSpec(const SensorSpec_t* spec)
{
    if (spec != NULL) {
        current_spec = *spec;
        spec_set = true;
    }
}

static void MLX90640_GetSpec(SensorSpec_t* spec)
{
    if (spec != NULL) {
        *spec = current_spec;
    }
}

static bool MLX90640_HasSpec(void)
{
    return spec_set;
}

static TestStatus_t MLX90640_RunTest(SensorResult_t* result)
{
    int mlx_status;
    float ta, tr;
    float min_temp, max_temp, avg_temp;
    float measured_temp;

    DBG_PRINT("\r\n[MLX90640] RunTest start\r\n");

    if (result == NULL) {
        DBG_PRINT("[MLX90640] ERROR: result=NULL\r\n");
        return STATUS_FAIL_INVALID;
    }

    if (!spec_set) {
        DBG_PRINT("[MLX90640] ERROR: no spec set\r\n");
        return STATUS_FAIL_NO_SPEC;
    }

    DBG_PRINTF("[MLX90640] Spec: target=%d.%dC, tol=%d.%dC, pixel=(%d,%d)\r\n",
               current_spec.mlx90640.target_temp / 10,
               (current_spec.mlx90640.target_temp < 0 ? -current_spec.mlx90640.target_temp : current_spec.mlx90640.target_temp) % 10,
               current_spec.mlx90640.tolerance / 10,
               current_spec.mlx90640.tolerance % 10,
               current_spec.mlx90640.pixel_x,
               current_spec.mlx90640.pixel_y);

    if (!initialized) {
        DBG_PRINT("[MLX90640] Not initialized, calling init...\r\n");
        if (MLX90640_Init_Driver() != HAL_OK) {
            DBG_PRINT("[MLX90640] Init failed!\r\n");
            return STATUS_FAIL_INIT;
        }
    }

    /* Get frame data (need 2 subpages for full frame) */
    DBG_PRINT("[MLX90640] GetFrameData...");
    mlx_status = MLX90640_GetFrameData(MLX90640_I2C_ADDR, frameData);
    if (mlx_status < 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        return STATUS_FAIL_TIMEOUT;
    }

    /* Debug: Print frame diagnostics */
    DBG_FRAME_INFO(frameData, &mlx_params, MLX90640_GetSubPageNumber(frameData));

    ta = MLX90640_GetTa(frameData, &mlx_params);
    tr = ta - 8.0f;  /* Reflected temperature approximation */
    DBG_PRINTF("[MLX90640] Ta=%d.%dC\r\n", (int)ta, ((int)(ta * 10) % 10 + 10) % 10);

    /* Calculate object temperatures */
    DBG_PRINT("[MLX90640] CalculateTo...");
    MLX90640_CalculateTo(frameData, &mlx_params, MLX90640_EMISSIVITY, tr, mlxTemperatures);
    DBG_PRINTF("OK (pixel[0]=%d.%d, pixel[1]=%d.%d)\r\n",
               (int)mlxTemperatures[0], ((int)(mlxTemperatures[0] * 10) % 10 + 10) % 10,
               (int)mlxTemperatures[1], ((int)(mlxTemperatures[1] * 10) % 10 + 10) % 10);

    /* Wait for next subpage (8Hz = 125ms per frame) */
    HAL_Delay(150);

    /* Get second subpage for complete frame (with retry) */
    DBG_PRINT("[MLX90640] GetFrameData...");
    for (int retry = 0; retry < 5; retry++) {
        mlx_status = MLX90640_GetFrameData(MLX90640_I2C_ADDR, frameData);
        if (mlx_status >= 0) break;
        HAL_Delay(50);
    }
    if (mlx_status < 0) {
        DBG_PRINTF("FAIL (err=%d)\r\n", mlx_status);
        DBG_PRINT("[MLX90640] Using first subpage only\r\n");
    } else {
        DBG_PRINTF("OK (subpage=%d)\r\n", MLX90640_GetSubPageNumber(frameData));
        /* Calculate second subpage */
        MLX90640_CalculateTo(frameData, &mlx_params, MLX90640_EMISSIVITY, tr, mlxTemperatures);
        DBG_PRINTF("[MLX90640] After 2nd: pixel[0]=%d.%d, pixel[1]=%d.%d\r\n",
                   (int)mlxTemperatures[0], ((int)(mlxTemperatures[0] * 10) % 10 + 10) % 10,
                   (int)mlxTemperatures[1], ((int)(mlxTemperatures[1] * 10) % 10 + 10) % 10);
    }

    /* Find min/max/avg temperatures */
    min_temp = mlxTemperatures[0];
    max_temp = mlxTemperatures[0];
    avg_temp = 0;

    for (int i = 0; i < 768; i++) {
        if (mlxTemperatures[i] < min_temp) min_temp = mlxTemperatures[i];
        if (mlxTemperatures[i] > max_temp) max_temp = mlxTemperatures[i];
        avg_temp += mlxTemperatures[i];
    }
    avg_temp /= 768.0f;

    DBG_PRINTF("[MLX90640] Temp: min=%d.%dC, max=%d.%dC, avg=%d.%dC\r\n",
               (int)min_temp, ((int)(min_temp * 10) % 10 + 10) % 10,
               (int)max_temp, ((int)(max_temp * 10) % 10 + 10) % 10,
               (int)avg_temp, ((int)(avg_temp * 10) % 10 + 10) % 10);

    /* Debug: Print thermal image */
    DBG_THERMAL_IMAGE(mlxTemperatures, min_temp, max_temp, avg_temp);

    /* Get measured temperature based on spec */
    if (current_spec.mlx90640.pixel_x == 0xFF || current_spec.mlx90640.pixel_y == 0xFF) {
        /* Use average temperature */
        measured_temp = avg_temp;
        DBG_PRINTF("[MLX90640] Using average: %d.%02dC\r\n",
                   (int)measured_temp, ((int)(measured_temp * 100) % 100 + 100) % 100);
    } else {
        /* Use specific pixel */
        int idx = current_spec.mlx90640.pixel_y * 32 + current_spec.mlx90640.pixel_x;
        if (idx >= 0 && idx < 768) {
            measured_temp = mlxTemperatures[idx];
            DBG_PRINTF("[MLX90640] Pixel(%d,%d): %d.%02dC\r\n",
                       current_spec.mlx90640.pixel_x, current_spec.mlx90640.pixel_y,
                       (int)measured_temp, ((int)(measured_temp * 100) % 100 + 100) % 100);
        } else {
            measured_temp = avg_temp;
            DBG_PRINT("[MLX90640] Invalid pixel, using average\r\n");
        }
    }

    /* Fill result structure (in 0.1°C units) */
    result->mlx90640.measured = (int16_t)(measured_temp * 10);
    result->mlx90640.target = current_spec.mlx90640.target_temp;
    result->mlx90640.tolerance = current_spec.mlx90640.tolerance;
    result->mlx90640.ambient = (int16_t)(ta * 10);
    result->mlx90640.min_temp = (int16_t)(min_temp * 10);
    result->mlx90640.max_temp = (int16_t)(max_temp * 10);

    /* Calculate difference */
    int16_t diff = result->mlx90640.measured - result->mlx90640.target;
    if (diff < 0) diff = -diff;
    result->mlx90640.diff = diff;

    DBG_PRINTF("[MLX90640] Diff: %d.%dC (tolerance: %d.%dC)\r\n",
               diff / 10, diff % 10,
               current_spec.mlx90640.tolerance / 10, current_spec.mlx90640.tolerance % 10);

    /* Check against tolerance */
    if (result->mlx90640.diff > current_spec.mlx90640.tolerance) {
        DBG_PRINT("[MLX90640] FAIL: out of tolerance\r\n");
        return STATUS_FAIL_INVALID;
    }

    DBG_PRINT("[MLX90640] PASS\r\n");
    return STATUS_PASS;
}

static uint8_t MLX90640_SerializeSpec(const SensorSpec_t* spec, uint8_t* buffer)
{
    if (spec == NULL || buffer == NULL) {
        return 0;
    }

    /* Format: [target_temp_hi][target_temp_lo][tolerance_hi][tolerance_lo][pixel_x][pixel_y] */
    buffer[0] = (uint8_t)(spec->mlx90640.target_temp >> 8);
    buffer[1] = (uint8_t)(spec->mlx90640.target_temp & 0xFF);
    buffer[2] = (uint8_t)(spec->mlx90640.tolerance >> 8);
    buffer[3] = (uint8_t)(spec->mlx90640.tolerance & 0xFF);
    buffer[4] = spec->mlx90640.pixel_x;
    buffer[5] = spec->mlx90640.pixel_y;

    return 6;
}

static uint8_t MLX90640_ParseSpec(const uint8_t* buffer, SensorSpec_t* spec)
{
    if (buffer == NULL || spec == NULL) {
        return 0;
    }

    /* Parse big-endian format */
    spec->mlx90640.target_temp = (int16_t)((buffer[0] << 8) | buffer[1]);
    spec->mlx90640.tolerance = (int16_t)((buffer[2] << 8) | buffer[3]);
    spec->mlx90640.pixel_x = buffer[4];
    spec->mlx90640.pixel_y = buffer[5];

    return 6;
}

static uint8_t MLX90640_SerializeResult(const SensorResult_t* result, uint8_t* buffer)
{
    if (result == NULL || buffer == NULL) {
        return 0;
    }

    /* Format: [measured][target][tolerance][diff][ambient][min][max] - all 16-bit big-endian */
    buffer[0] = (uint8_t)(result->mlx90640.measured >> 8);
    buffer[1] = (uint8_t)(result->mlx90640.measured & 0xFF);
    buffer[2] = (uint8_t)(result->mlx90640.target >> 8);
    buffer[3] = (uint8_t)(result->mlx90640.target & 0xFF);
    buffer[4] = (uint8_t)(result->mlx90640.tolerance >> 8);
    buffer[5] = (uint8_t)(result->mlx90640.tolerance & 0xFF);
    buffer[6] = (uint8_t)(result->mlx90640.diff >> 8);
    buffer[7] = (uint8_t)(result->mlx90640.diff & 0xFF);
    buffer[8] = (uint8_t)(result->mlx90640.ambient >> 8);
    buffer[9] = (uint8_t)(result->mlx90640.ambient & 0xFF);
    buffer[10] = (uint8_t)(result->mlx90640.min_temp >> 8);
    buffer[11] = (uint8_t)(result->mlx90640.min_temp & 0xFF);
    buffer[12] = (uint8_t)(result->mlx90640.max_temp >> 8);
    buffer[13] = (uint8_t)(result->mlx90640.max_temp & 0xFF);

    return 14;
}
