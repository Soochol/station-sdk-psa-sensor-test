/**
 * @file main.cpp
 * @brief PSA Sensor Test Firmware - VL53L0X ToF Sensor Test
 *
 * Simplified firmware for testing VL53L0X ToF sensor only.
 *
 * Hardware Configuration:
 *   - MCU: STM32H723VGT6 @ 384MHz
 *   - I2C1: VL53L0X ToF sensor (PB6: SCL, PB7: SDA)
 *   - UART4: Host communication (115200 bps)
 *   - IWDG: Independent Watchdog (timeout ~10 seconds)
 */

extern "C" {
#include "main.h"
#include "config.h"
#include "hal/i2c_handler.h"
#include "hal/uart_handler.h"
#include "protocol/protocol.h"
#include "sensors/sensor_manager.h"
#include "sensors/vl53l0x.h"
#include "sensors/mlx90640.h"
#include "test/test_runner.h"
#include "MLX90640_API.h"
#include "vl53l0x_simple.h"

/* VL53L0X device handle from vl53l0x.c */
extern VL53L0X_Dev_Simple_t vl53l0x_dev;
}

#include <stdio.h>
#include <string.h>

/* RTT for debug output via SWD */
#include "SEGGER_RTT.h"

/*============================================================================*/
/* Global Variables (used by HAL MSP and IT)                                  */
/*============================================================================*/

I2C_HandleTypeDef hi2c1;
I2C_HandleTypeDef hi2c4;
UART_HandleTypeDef huart4;

#if WATCHDOG_ENABLED
static IWDG_HandleTypeDef hiwdg;
#endif

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static void App_Init(void);
static void App_MainLoop(void);
static void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_I2C4_Init(void);
static void MX_UART4_Init(void);
static void MPU_Config(void);

#if WATCHDOG_ENABLED
static void MX_IWDG_Init(void);
#endif

/*============================================================================*/
/* UART Receive Callback Override                                             */
/*============================================================================*/

extern "C" void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Handler_RxCpltCallback(huart);
}

/*============================================================================*/
/* Debug Variables                                                            */
/*============================================================================*/

volatile int dbg_i2c1_ready = -1;      /* 0=OK, 1=ERROR */
volatile int dbg_vl53l0x_init = -1;    /* 0=OK, 1=ERROR */
volatile uint16_t dbg_vl53l0x_dist = 0;  /* Distance in mm */
volatile int dbg_vl53l0x_step = 0;     /* VL53L0X init step (for debugging) */
volatile int dbg_test_status = -1;      /* Last test status */
volatile int dbg_measure_count = 0;     /* Measurement count */

/* I2C scan results */
volatile uint8_t dbg_i2c1_devices[8] = {0};
volatile int dbg_i2c1_count = 0;

/* Driver debug */
volatile uint32_t dbg_driver_ptr = 0;       /* Driver pointer address */
volatile uint32_t dbg_init_func_ptr = 0;    /* Init function pointer */
volatile int dbg_sensor_count = 0;          /* Registered sensor count */

/*============================================================================*/
/* Debug Test Functions                                                       */
/*============================================================================*/

/**
 * @brief Scan I2C1 bus for devices
 */
extern "C" void DBG_ScanI2C(void)
{
    dbg_i2c1_count = 0;

    /* Scan I2C1 (addresses 0x08 to 0x77) */
    for (uint8_t addr = 0x08; addr < 0x78; addr++) {
        if (HAL_I2C_IsDeviceReady(&hi2c1, (uint16_t)(addr << 1), 1, 10) == HAL_OK) {
            if (dbg_i2c1_count < 8) {
                dbg_i2c1_devices[dbg_i2c1_count++] = addr;
            }
        }
    }
}

/**
 * @brief Test VL53L0X device presence
 */
extern "C" void DBG_TestI2C(void)
{
    dbg_i2c1_ready = (I2C_Handler_IsDeviceReady(I2C_BUS_1, VL53L0X_I2C_ADDR, 100) == HAL_OK) ? 0 : 1;
}

/**
 * @brief Continuous measurement for watch window debugging
 * Tests all registered sensors
 */
extern "C" void DBG_ContinuousMeasure(void)
{
    uint8_t count = SensorManager_GetCount();

    for (uint8_t i = 0; i < count; i++) {
        const SensorDriver_t* driver = SensorManager_GetByIndex(i);
        if (driver != NULL && driver->run_test != NULL) {
            SensorResult_t result = {0};
            TestStatus_t status = driver->run_test(&result);

            if (driver->id == SENSOR_ID_VL53L0X) {
                dbg_test_status = (int)status;
                dbg_measure_count++;
                if (status == STATUS_PASS || status == STATUS_FAIL_INVALID) {
                    dbg_vl53l0x_dist = result.vl53l0x.measured;
                }
            }
            /* MLX90640 results are printed via dbg_printf in the driver */
        }
    }
}

/**
 * @brief Initialize and test VL53L0X sensor
 */
extern "C" void DBG_TestVL53L0X(void)
{
    dbg_vl53l0x_step = -99;  /* Entry marker */

    /* Get sensor count first */
    dbg_sensor_count = SensorManager_GetCount();

    const SensorDriver_t* driver = SensorManager_GetByID(SENSOR_ID_VL53L0X);
    if (driver == NULL) {
        dbg_vl53l0x_init = -2;
        dbg_vl53l0x_step = -98;  /* Driver not found */
        return;
    }

    /* Capture driver pointer info for debugging */
    dbg_driver_ptr = (uint32_t)driver;
    dbg_init_func_ptr = (uint32_t)(driver->init);

    /* Check if init function pointer is valid */
    if (driver->init == NULL) {
        dbg_vl53l0x_init = -3;
        dbg_vl53l0x_step = -95;  /* Init function is NULL */
        return;
    }

    /* Test simple function call first */
    dbg_vl53l0x_step = -98;  /* Before simple test */
    VL53L0X_SimpleTest();    /* Should set dbg_vl53l0x_step = 1 */
    dbg_vl53l0x_step = -97;  /* Before init() call */

    /* Use direct function call instead of function pointer for debugging */
    HAL_StatusTypeDef result = VL53L0X_DirectInit();
    /* Don't overwrite step on failure - preserve error step */
    if (result == HAL_OK) {
        dbg_vl53l0x_step = -96;  /* After init() call - success marker */
    }
    /* else: keep the step value from VL53L0X_Init_Driver for debugging */
    dbg_vl53l0x_init = (result == HAL_OK) ? 0 : 1;

    if (dbg_vl53l0x_init == 0) {
        /* Set a default spec and run test */
        SensorSpec_t spec;
        spec.vl53l0x.target_dist = 500;
        spec.vl53l0x.tolerance = 2000;
        driver->set_spec(&spec);

        SensorResult_t result_data;
        driver->run_test(&result_data);
        dbg_vl53l0x_dist = result_data.vl53l0x.measured;
    }
}

/*============================================================================*/
/* MLX90640 Stabilization Time Measurement                                    */
/*============================================================================*/

/* External MLX90640 API variables (from mlx90640.c) */
extern paramsMLX90640 mlx_params;
extern uint16_t eeData[832];
extern uint16_t frameData[834];
extern float mlxTemperatures[768];

/**
 * @brief Measure MLX90640 stabilization time after boot
 *
 * Continuously reads temperature data and outputs via RTT to determine
 * when the sensor stabilizes after power-on.
 */
extern "C" void DBG_MeasureMLX90640_StabilizationTime(void)
{
    int mlx_status;
    uint32_t start_time = HAL_GetTick();
    uint32_t elapsed;
    int sample_count = 0;
    const int MAX_SAMPLES = 100;  /* ~10 seconds at 100ms intervals */

    SEGGER_RTT_printf(0, "\r\n");
    SEGGER_RTT_printf(0, "========================================\r\n");
    SEGGER_RTT_printf(0, "MLX90640 Stabilization Time Measurement\r\n");
    SEGGER_RTT_printf(0, "========================================\r\n");
    SEGGER_RTT_printf(0, "Boot time: %lu ms\r\n", start_time);
    SEGGER_RTT_printf(0, "\r\n");

    /* Check I2C4 device presence first */
    SEGGER_RTT_printf(0, "[%lu ms] Checking MLX90640 on I2C4 @ 0x%02X...\r\n",
                      HAL_GetTick(), MLX90640_I2C_ADDR);

    HAL_StatusTypeDef hal_status = I2C_Handler_IsDeviceReady(MLX90640_I2C_BUS, MLX90640_I2C_ADDR, 100);
    if (hal_status != HAL_OK) {
        SEGGER_RTT_printf(0, "[ERROR] MLX90640 not found! hal_status=%d\r\n", hal_status);
        return;
    }
    SEGGER_RTT_printf(0, "[OK] MLX90640 detected\r\n\r\n");

    /* Initialize MLX90640 via sensor manager */
    const SensorDriver_t* mlx_driver = SensorManager_GetByID(SENSOR_ID_MLX90640);
    if (mlx_driver == NULL) {
        SEGGER_RTT_printf(0, "[ERROR] MLX90640 driver not registered!\r\n");
        return;
    }

    SEGGER_RTT_printf(0, "[%lu ms] Initializing MLX90640...\r\n", HAL_GetTick() - start_time);
    hal_status = mlx_driver->init();
    if (hal_status != HAL_OK) {
        SEGGER_RTT_printf(0, "[ERROR] MLX90640 init failed! status=%d\r\n", hal_status);
        return;
    }

    uint32_t init_time = HAL_GetTick() - start_time;
    SEGGER_RTT_printf(0, "[%lu ms] MLX90640 initialized (init took %lu ms)\r\n\r\n",
                      init_time, init_time);

    /* Print header for CSV-style output */
    SEGGER_RTT_printf(0, "Time(ms),Avg(C),Min(C),Max(C),Ta(C),Vdd(V),Center(C)\r\n");
    SEGGER_RTT_printf(0, "----------------------------------------------------\r\n");

    /* Continuous temperature measurement loop */
    while (sample_count < MAX_SAMPLES) {
        elapsed = HAL_GetTick() - start_time;

        /* Get frame data */
        mlx_status = MLX90640_GetFrameData(MLX90640_I2C_ADDR, frameData);
        if (mlx_status < 0) {
            SEGGER_RTT_printf(0, "[%lu ms] Frame error: %d\r\n", elapsed, mlx_status);
            HAL_Delay(100);
            continue;
        }

        /* Get ambient temperature and Vdd */
        float ta = MLX90640_GetTa(frameData, &mlx_params);
        float vdd = MLX90640_GetVdd(frameData, &mlx_params);
        float tr = ta - 8.0f;

        /* Calculate object temperatures */
        MLX90640_CalculateTo(frameData, &mlx_params, MLX90640_EMISSIVITY, tr, mlxTemperatures);

        /* Get second subpage for complete frame */
        HAL_Delay(130);  /* Wait for next subpage at 8Hz */
        mlx_status = MLX90640_GetFrameData(MLX90640_I2C_ADDR, frameData);
        if (mlx_status >= 0) {
            MLX90640_CalculateTo(frameData, &mlx_params, MLX90640_EMISSIVITY, tr, mlxTemperatures);
        }

        /* Calculate statistics */
        float min_temp = mlxTemperatures[0];
        float max_temp = mlxTemperatures[0];
        float avg_temp = 0;
        float center_temp = mlxTemperatures[12 * 32 + 16];  /* Center pixel (16, 12) */

        for (int i = 0; i < 768; i++) {
            if (mlxTemperatures[i] < min_temp) min_temp = mlxTemperatures[i];
            if (mlxTemperatures[i] > max_temp) max_temp = mlxTemperatures[i];
            avg_temp += mlxTemperatures[i];
        }
        avg_temp /= 768.0f;

        /* Output data via RTT (using integer formatting for float) */
        int avg_int = (int)avg_temp;
        int avg_dec = ((int)(avg_temp * 100) % 100);
        if (avg_dec < 0) avg_dec = -avg_dec;

        int min_int = (int)min_temp;
        int min_dec = ((int)(min_temp * 100) % 100);
        if (min_dec < 0) min_dec = -min_dec;

        int max_int = (int)max_temp;
        int max_dec = ((int)(max_temp * 100) % 100);
        if (max_dec < 0) max_dec = -max_dec;

        int ta_int = (int)ta;
        int ta_dec = ((int)(ta * 100) % 100);
        if (ta_dec < 0) ta_dec = -ta_dec;

        int vdd_int = (int)vdd;
        int vdd_dec = ((int)(vdd * 100) % 100);
        if (vdd_dec < 0) vdd_dec = -vdd_dec;

        int ctr_int = (int)center_temp;
        int ctr_dec = ((int)(center_temp * 100) % 100);
        if (ctr_dec < 0) ctr_dec = -ctr_dec;

        SEGGER_RTT_printf(0, "%lu,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d,%d.%02d\r\n",
                          elapsed,
                          avg_int, avg_dec,
                          min_int, min_dec,
                          max_int, max_dec,
                          ta_int, ta_dec,
                          vdd_int, vdd_dec,
                          ctr_int, ctr_dec);

        sample_count++;

        /* Brief delay before next sample */
        HAL_Delay(100);
    }

    SEGGER_RTT_printf(0, "\r\n");
    SEGGER_RTT_printf(0, "========================================\r\n");
    SEGGER_RTT_printf(0, "Measurement complete (%d samples)\r\n", sample_count);
    SEGGER_RTT_printf(0, "Total time: %lu ms\r\n", HAL_GetTick() - start_time);
    SEGGER_RTT_printf(0, "========================================\r\n");
}

/*============================================================================*/
/* Error Handler                                                              */
/*============================================================================*/

extern "C" void Error_Handler(void)
{
    __disable_irq();
    while (1) {
        /* Hang in infinite loop on error */
    }
}

/*============================================================================*/
/* Main Function                                                              */
/*============================================================================*/

int main(void)
{
    /* MPU Configuration */
    MPU_Config();

    /* MCU Configuration */
    HAL_Init();

    /* Initialize RTT for debug output */
    SEGGER_RTT_Init();
    SEGGER_RTT_printf(0, "\r\n\r\n=== RTT Debug Output Started ===\r\n");
    SEGGER_RTT_printf(0, "PSA Sensor Test Firmware v1.0\r\n");
    SEGGER_RTT_printf(0, "MCU: STM32H723VGT6 @ 384MHz\r\n\r\n");

    /* Configure the system clock */
    SystemClock_Config();

    /* Initialize all configured peripherals */
    MX_GPIO_Init();  /* Note: 12V OFF, 2s delay, XSHUT LOW at this point */

    /* Initialize I2C buses first (before powering sensors) */
    MX_I2C1_Init();
    MX_I2C4_Init();

    /* === Power-up sequence (matches psa-stm32-firmware) === */
    /* Step 1: Enable 12V power rail */
    HAL_GPIO_WritePin(DO_12VA_EN_GPIO_Port, DO_12VA_EN_Pin, GPIO_PIN_SET);  /* 12V ON */
    HAL_Delay(100);  /* Wait 100ms for 12V to stabilize */

    /* Step 2: Release VL53L0X from reset (XSHUT HIGH) */
    HAL_GPIO_WritePin(DO_TOF1_SHUT_GPIO_Port, DO_TOF1_SHUT_Pin, GPIO_PIN_SET);  /* XSHUT HIGH - boot sensor */
    HAL_Delay(100);  /* Wait 100ms for VL53L0X to boot (needs ~1.2ms, use 100ms for safety) */

    /* Initialize UART for communication */
    MX_UART4_Init();

    /* Early startup banner - test UART output */
    const char* banner = "\r\n\r\n=== PSA Sensor Test v1.0 ===\r\n";
    HAL_UART_Transmit(&huart4, (uint8_t*)banner, strlen(banner), 100);

    /* Initialize application */
    App_Init();

    /* === DEBUG TEST MODE === */
    /* I2C device scan */
    DBG_ScanI2C();

    /* Test VL53L0X presence */
    DBG_TestI2C();

    /* Test VL53L0X sensor */
    DBG_TestVL53L0X();

    /* If VL53L0X initialized successfully, continue to main loop */
    if (dbg_vl53l0x_init == 0) {
#if WATCHDOG_ENABLED
        MX_IWDG_Init();
#endif

        /* Main application loop - continuous measurement for debugging */
        while (1) {
#if WATCHDOG_ENABLED
            HAL_IWDG_Refresh(&hiwdg);  /* Feed watchdog */
#endif
            /* Process protocol and run async tests */
            App_MainLoop();
        }
    }

    /* If init failed, halt here for debugging */
    while (1) {
        __NOP();  /* Set breakpoint here to check results */
    }
}

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

/**
 * @brief Initialize application modules
 */
static void App_Init(void)
{
    /* Initialize I2C handler */
    I2C_Handler_Init(I2C_BUS_1, &hi2c1);
    I2C_Handler_Init(I2C_BUS_4, &hi2c4);

    /* Initialize UART handler */
    UART_Handler_Init(&huart4);

    /* Initialize sensor manager (registers VL53L0X, MLX90640 drivers) */
    SensorManager_Init();

    /* Debug: show registered sensors */
    {
        char buf[64];
        uint8_t count = SensorManager_GetCount();
        snprintf(buf, sizeof(buf), "\r\n[App] Registered sensors: %d\r\n", count);
        HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);

        for (uint8_t i = 0; i < count; i++) {
            const SensorDriver_t* drv = SensorManager_GetByIndex(i);
            if (drv) {
                snprintf(buf, sizeof(buf), "[App] Sensor[%d]: id=%d, name=%s\r\n", i, drv->id, drv->name);
                HAL_UART_Transmit(&huart4, (uint8_t*)buf, strlen(buf), 100);
            }
        }
    }

    /* Initialize all registered sensors */
    SensorManager_InitSensors();

    /* Set default specs for testing */
    {
        /* VL53L0X: target 500mm, tolerance 2000mm */
        const SensorDriver_t* vl53l0x = SensorManager_GetByID(SENSOR_ID_VL53L0X);
        if (vl53l0x) {
            SensorSpec_t spec;
            spec.vl53l0x.target_dist = 500;
            spec.vl53l0x.tolerance = 2000;
            vl53l0x->set_spec(&spec);
            HAL_UART_Transmit(&huart4, (uint8_t*)"[App] VL53L0X spec set\r\n", 24, 100);
        }

        /* MLX90640: target 25.0°C, tolerance 50.0°C, use average */
        const SensorDriver_t* mlx90640 = SensorManager_GetByID(SENSOR_ID_MLX90640);
        if (mlx90640) {
            SensorSpec_t spec;
            spec.mlx90640.target_temp = 250;   /* 25.0°C */
            spec.mlx90640.tolerance = 500;     /* 50.0°C */
            spec.mlx90640.pixel_x = 0xFF;      /* Use average */
            spec.mlx90640.pixel_y = 0xFF;
            mlx90640->set_spec(&spec);
            HAL_UART_Transmit(&huart4, (uint8_t*)"[App] MLX90640 spec set\r\n", 25, 100);
        } else {
            HAL_UART_Transmit(&huart4, (uint8_t*)"[App] MLX90640 NOT FOUND!\r\n", 27, 100);
        }
    }

    /* Initialize test runner */
    TestRunner_Init();

    /* Initialize protocol handler */
    Protocol_Init();
}

/**
 * @brief Main application loop
 */
static void App_MainLoop(void)
{
    static uint32_t last_rtt_tick = 0;

    /* Process protocol communications */
    Protocol_Process();

    /* Process async test execution (non-blocking) */
    TestRunner_ProcessAsync();

    /* RTT debug output every 1 second with live VL53L0X measurement */
    uint32_t now = HAL_GetTick();
    if (now - last_rtt_tick >= 1000) {
        last_rtt_tick = now;

        /* Live VL53L0X measurement */
        uint16_t live_dist = VL53L0X_Simple_ReadRangeSingleMillimeters(&vl53l0x_dev);
        bool timeout = VL53L0X_Simple_TimeoutOccurred(&vl53l0x_dev);

        /* Print sensor status via RTT */
        SEGGER_RTT_printf(0, "[%lu ms] VL53L0X: %d mm %s\r\n",
                          now, live_dist, timeout ? "(TIMEOUT)" : "");
    }

    /* Refresh watchdog timer */
#if WATCHDOG_ENABLED
    HAL_IWDG_Refresh(&hiwdg);
#endif
}

/*============================================================================*/
/* Peripheral Initialization Functions                                        */
/*============================================================================*/

/**
 * @brief System Clock Configuration
 */
static void SystemClock_Config(void)
{
    RCC_OscInitTypeDef RCC_OscInitStruct = {0};
    RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

    HAL_PWREx_ConfigSupply(PWR_LDO_SUPPLY);
    __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

    while(!__HAL_PWR_GET_FLAG(PWR_FLAG_VOSRDY)) {}

    RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
    RCC_OscInitStruct.HSEState = RCC_HSE_ON;
    RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
    RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
    RCC_OscInitStruct.PLL.PLLM = 1;
    RCC_OscInitStruct.PLL.PLLN = 96;
    RCC_OscInitStruct.PLL.PLLP = 2;
    RCC_OscInitStruct.PLL.PLLQ = 125;
    RCC_OscInitStruct.PLL.PLLR = 2;
    RCC_OscInitStruct.PLL.PLLRGE = RCC_PLL1VCIRANGE_3;
    RCC_OscInitStruct.PLL.PLLVCOSEL = RCC_PLL1VCOWIDE;
    RCC_OscInitStruct.PLL.PLLFRACN = 0;
    if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
        Error_Handler();
    }

    RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                                |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2
                                |RCC_CLOCKTYPE_D3PCLK1|RCC_CLOCKTYPE_D1PCLK1;
    RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
    RCC_ClkInitStruct.SYSCLKDivider = RCC_SYSCLK_DIV1;
    RCC_ClkInitStruct.AHBCLKDivider = RCC_HCLK_DIV2;
    RCC_ClkInitStruct.APB3CLKDivider = RCC_APB3_DIV16;
    RCC_ClkInitStruct.APB1CLKDivider = RCC_APB1_DIV2;
    RCC_ClkInitStruct.APB2CLKDivider = RCC_APB2_DIV2;
    RCC_ClkInitStruct.APB4CLKDivider = RCC_APB4_DIV2;
    if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK) {
        Error_Handler();
    }

    /* I2C1 uses default APB1 PCLK (96MHz) as clock source
     * With timing 0x009032AE, this gives ~425kHz I2C speed
     * Same configuration as psa-stm32-firmware */
}

/**
 * @brief I2C1 Initialization Function (VL53L0X)
 */
static void MX_I2C1_Init(void)
{
    hi2c1.Instance = I2C1;
    hi2c1.Init.Timing = 0x009032AE;  /* ~400kHz (Fast Mode) */
    hi2c1.Init.OwnAddress1 = 0;
    hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c1.Init.OwnAddress2 = 0;
    hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c1) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief I2C4 Initialization Function (MLX90640)
 */
static void MX_I2C4_Init(void)
{
    hi2c4.Instance = I2C4;
    hi2c4.Init.Timing = 0x20C71027;  /* 400kHz Fast Mode */
    hi2c4.Init.OwnAddress1 = 0;
    hi2c4.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c4.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c4.Init.OwnAddress2 = 0;
    hi2c4.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
    hi2c4.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c4.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
    if (HAL_I2C_Init(&hi2c4) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigAnalogFilter(&hi2c4, I2C_ANALOGFILTER_ENABLE) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_I2CEx_ConfigDigitalFilter(&hi2c4, 0) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief UART4 Initialization Function
 */
static void MX_UART4_Init(void)
{
    huart4.Instance = UART4;
    huart4.Init.BaudRate = 115200;
    huart4.Init.WordLength = UART_WORDLENGTH_8B;
    huart4.Init.StopBits = UART_STOPBITS_1;
    huart4.Init.Parity = UART_PARITY_NONE;
    huart4.Init.Mode = UART_MODE_TX_RX;
    huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
    huart4.Init.OverSampling = UART_OVERSAMPLING_16;
    huart4.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
    huart4.Init.ClockPrescaler = UART_PRESCALER_DIV1;
    huart4.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
    if (HAL_UART_Init(&huart4) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetTxFifoThreshold(&huart4, UART_TXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_SetRxFifoThreshold(&huart4, UART_RXFIFO_THRESHOLD_1_8) != HAL_OK) {
        Error_Handler();
    }
    if (HAL_UARTEx_DisableFifoMode(&huart4) != HAL_OK) {
        Error_Handler();
    }
}

/**
 * @brief GPIO Initialization Function
 */
static void MX_GPIO_Init(void)
{
    GPIO_InitTypeDef GPIO_InitStruct = {0};

    /* GPIO Ports Clock Enable */
    __HAL_RCC_GPIOH_CLK_ENABLE();
    __HAL_RCC_GPIOA_CLK_ENABLE();
    __HAL_RCC_GPIOB_CLK_ENABLE();
    __HAL_RCC_GPIOC_CLK_ENABLE();

    /* Configure 12V Power Enable (DO_12VA_EN) as output - PC13 */
    HAL_GPIO_WritePin(DO_12VA_EN_GPIO_Port, DO_12VA_EN_Pin, GPIO_PIN_RESET);  /* Start with 12V OFF */
    GPIO_InitStruct.Pin = DO_12VA_EN_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DO_12VA_EN_GPIO_Port, &GPIO_InitStruct);

    /* Keep 12V OFF and wait 2 seconds (discharge capacitors, ensure clean power-up) */
    HAL_GPIO_WritePin(DO_12VA_EN_GPIO_Port, DO_12VA_EN_Pin, GPIO_PIN_RESET);
    HAL_Delay(2000);  /* 2 second delay with 12V OFF */

    /* Configure VL53L0X XSHUT (DO_TOF1_SHUT) as output - keep LOW until 12V is stable */
    HAL_GPIO_WritePin(DO_TOF1_SHUT_GPIO_Port, DO_TOF1_SHUT_Pin, GPIO_PIN_RESET);  /* XSHUT LOW - hold in reset */
    GPIO_InitStruct.Pin = DO_TOF1_SHUT_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
    HAL_GPIO_Init(DO_TOF1_SHUT_GPIO_Port, &GPIO_InitStruct);

    /* Configure VL53L0X GPIO1 (DO_TOF1_GPIO) as input for interrupt */
    GPIO_InitStruct.Pin = DO_TOF1_GPIO_Pin;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(DO_TOF1_GPIO_GPIO_Port, &GPIO_InitStruct);

    /* NOTE: XSHUT release and 12V enable happen in main() after I2C init */
}

#if WATCHDOG_ENABLED
/**
 * @brief IWDG Initialization Function
 */
static void MX_IWDG_Init(void)
{
    hiwdg.Instance = IWDG1;
    hiwdg.Init.Prescaler = IWDG_PRESCALER_256;
    hiwdg.Init.Window = 4095;
    hiwdg.Init.Reload = 1250;  /* ~10 second timeout */

    if (HAL_IWDG_Init(&hiwdg) != HAL_OK) {
        Error_Handler();
    }
}
#endif

/**
 * @brief MPU Configuration
 */
static void MPU_Config(void)
{
    MPU_Region_InitTypeDef MPU_InitStruct = {0};

    HAL_MPU_Disable();

    MPU_InitStruct.Enable = MPU_REGION_ENABLE;
    MPU_InitStruct.Number = MPU_REGION_NUMBER0;
    MPU_InitStruct.BaseAddress = 0x0;
    MPU_InitStruct.Size = MPU_REGION_SIZE_4GB;
    MPU_InitStruct.SubRegionDisable = 0x87;
    MPU_InitStruct.TypeExtField = MPU_TEX_LEVEL0;
    MPU_InitStruct.AccessPermission = MPU_REGION_NO_ACCESS;
    MPU_InitStruct.DisableExec = MPU_INSTRUCTION_ACCESS_DISABLE;
    MPU_InitStruct.IsShareable = MPU_ACCESS_SHAREABLE;
    MPU_InitStruct.IsCacheable = MPU_ACCESS_NOT_CACHEABLE;
    MPU_InitStruct.IsBufferable = MPU_ACCESS_NOT_BUFFERABLE;

    HAL_MPU_ConfigRegion(&MPU_InitStruct);
    HAL_MPU_Enable(MPU_PRIVILEGED_DEFAULT);
}
