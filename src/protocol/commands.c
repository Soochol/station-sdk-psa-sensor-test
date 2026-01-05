/**
 * @file commands.c
 * @brief Command handler implementation with async test support
 *
 * Uses TestRunner's async API to prevent blocking during long sensor tests.
 * Busy state is managed by TestRunner_IsBusy() - single source of truth.
 */

#include "protocol/commands.h"
#include "protocol/protocol.h"
#include "sensors/sensor_manager.h"
#include "test/test_runner.h"
#include <string.h>

/*============================================================================*/
/* Private Function Prototypes                                                */
/*============================================================================*/

static void Handle_Ping(const Frame_t* request, Frame_t* response);
static void Handle_TestAll(const Frame_t* request, Frame_t* response);
static void Handle_TestSingle(const Frame_t* request, Frame_t* response);
static void Handle_GetSensorList(const Frame_t* request, Frame_t* response);
static void Handle_ReadSensor(const Frame_t* request, Frame_t* response);
static void Handle_SetSpec(const Frame_t* request, Frame_t* response);
static void Handle_GetSpec(const Frame_t* request, Frame_t* response);

/*============================================================================*/
/* Public Functions                                                           */
/*============================================================================*/

void Commands_Init(void)
{
    /* Nothing to initialize - busy state managed by TestRunner */
}

bool Commands_Process(const Frame_t* request, Frame_t* response)
{
    if (request == NULL || response == NULL) {
        return false;
    }

    /* Dispatch based on command code */
    switch (request->cmd) {
        case CMD_PING:
            Handle_Ping(request, response);
            return true;

        case CMD_TEST_ALL:
            Handle_TestAll(request, response);
            return true;

        case CMD_TEST_SINGLE:
            Handle_TestSingle(request, response);
            return true;

        case CMD_GET_SENSOR_LIST:
            Handle_GetSensorList(request, response);
            return true;

        case CMD_READ_SENSOR:
            Handle_ReadSensor(request, response);
            return true;

        case CMD_SET_SPEC:
            Handle_SetSpec(request, response);
            return true;

        case CMD_GET_SPEC:
            Handle_GetSpec(request, response);
            return true;

        default:
            Commands_BuildNAK(response, ERR_UNKNOWN_CMD);
            return true;
    }
}

void Commands_BuildNAK(Frame_t* response, ErrorCode_t error_code)
{
    Frame_Init(response, CMD_NAK);
    Frame_AddByte(response, (uint8_t)error_code);
}

/*============================================================================*/
/* Private Functions                                                          */
/*============================================================================*/

static void Handle_Ping(const Frame_t* request, Frame_t* response)
{
    (void)request;

    /* PONG response with firmware version */
    Frame_Init(response, CMD_PONG);
    Frame_AddByte(response, FW_VERSION_MAJOR);
    Frame_AddByte(response, FW_VERSION_MINOR);
    Frame_AddByte(response, FW_VERSION_PATCH);
}

static void Handle_TestAll(const Frame_t* request, Frame_t* response)
{
    (void)request;

    /* Check busy state from single source of truth */
    if (TestRunner_IsBusy()) {
        Commands_BuildNAK(response, ERR_BUSY);
        return;
    }

    /* Run all sensor tests (blocking for backward compatibility) */
    TestReport_t report;
    TestRunner_RunAll(&report);

    /* Build response */
    Frame_Init(response, CMD_TEST_RESULT);

    /* Serialize test report */
    uint8_t report_buffer[PROTOCOL_MAX_PAYLOAD];
    uint16_t report_len = TestRunner_SerializeReport(&report, report_buffer);

    if (report_len <= PROTOCOL_MAX_PAYLOAD) {
        Frame_AddBytes(response, report_buffer, (uint8_t)report_len);
    }
}

static void Handle_TestSingle(const Frame_t* request, Frame_t* response)
{
    /* Payload: [sensor_id] */
    if (request->payload_len < 1) {
        Commands_BuildNAK(response, ERR_INVALID_PAYLOAD);
        return;
    }

    SensorID_t sensor_id = (SensorID_t)request->payload[0];

    /* Validate sensor ID */
    if (!SensorManager_IsValidID(sensor_id)) {
        Commands_BuildNAK(response, ERR_INVALID_SENSOR_ID);
        return;
    }

    /* Check busy state from single source of truth */
    if (TestRunner_IsBusy()) {
        Commands_BuildNAK(response, ERR_BUSY);
        return;
    }

    /* Run single sensor test (blocking for backward compatibility) */
    TestReport_t report;
    TestRunner_RunSingle(sensor_id, &report);

    /* Build response */
    Frame_Init(response, CMD_TEST_RESULT);

    /* Serialize test report */
    uint8_t report_buffer[PROTOCOL_MAX_PAYLOAD];
    uint16_t report_len = TestRunner_SerializeReport(&report, report_buffer);

    if (report_len <= PROTOCOL_MAX_PAYLOAD) {
        Frame_AddBytes(response, report_buffer, (uint8_t)report_len);
    }
}

static void Handle_GetSensorList(const Frame_t* request, Frame_t* response)
{
    (void)request;

    Frame_Init(response, CMD_SENSOR_LIST);

    uint8_t count = SensorManager_GetCount();
    Frame_AddByte(response, count);

    /* Add each sensor's ID and name length + name */
    for (uint8_t i = 0; i < count; i++) {
        const SensorDriver_t* driver = SensorManager_GetByIndex(i);
        if (driver != NULL) {
            Frame_AddByte(response, (uint8_t)driver->id);

            /* Add name length and name (limited) */
            const char* name = driver->name;
            uint8_t name_len = 0;
            while (name[name_len] != '\0' && name_len < 16) {
                name_len++;
            }
            Frame_AddByte(response, name_len);
            Frame_AddBytes(response, (const uint8_t*)name, name_len);
        }
    }
}

static void Handle_ReadSensor(const Frame_t* request, Frame_t* response)
{
    /* Payload: [sensor_id] */
    if (request->payload_len < 1) {
        Commands_BuildNAK(response, ERR_INVALID_PAYLOAD);
        return;
    }

    SensorID_t sensor_id = (SensorID_t)request->payload[0];

    /* Get sensor driver */
    const SensorDriver_t* driver = SensorManager_GetByID(sensor_id);
    if (driver == NULL) {
        Commands_BuildNAK(response, ERR_INVALID_SENSOR_ID);
        return;
    }

    /* Check busy state */
    if (TestRunner_IsBusy()) {
        Commands_BuildNAK(response, ERR_BUSY);
        return;
    }

    /* Run sensor measurement (reuse run_test, ignore pass/fail) */
    SensorResult_t result;
    memset(&result, 0, sizeof(result));

    TestStatus_t status = STATUS_NOT_TESTED;
    if (driver->run_test != NULL) {
        status = driver->run_test(&result);
    }

    /* Build response */
    Frame_Init(response, CMD_SENSOR_DATA);
    Frame_AddByte(response, (uint8_t)sensor_id);
    Frame_AddByte(response, (uint8_t)status);

    /* Serialize raw result data */
    if (driver->serialize_result != NULL) {
        uint8_t result_buffer[8];
        uint8_t result_len = driver->serialize_result(&result, result_buffer);
        Frame_AddBytes(response, result_buffer, result_len);
    }
}

static void Handle_SetSpec(const Frame_t* request, Frame_t* response)
{
    /* Payload: [sensor_id][spec_data...] */
    if (request->payload_len < 2) {
        Commands_BuildNAK(response, ERR_INVALID_PAYLOAD);
        return;
    }

    SensorID_t sensor_id = (SensorID_t)request->payload[0];

    /* Get sensor driver */
    const SensorDriver_t* driver = SensorManager_GetByID(sensor_id);
    if (driver == NULL) {
        Commands_BuildNAK(response, ERR_INVALID_SENSOR_ID);
        return;
    }

    /* Parse specification */
    SensorSpec_t spec;
    if (driver->parse_spec != NULL) {
        uint8_t parsed = driver->parse_spec(&request->payload[1], &spec);
        if (parsed == 0) {
            Commands_BuildNAK(response, ERR_INVALID_PAYLOAD);
            return;
        }
    } else {
        /* Copy raw bytes */
        memcpy(spec.raw, &request->payload[1],
               (request->payload_len - 1 > 4) ? 4 : (request->payload_len - 1));
    }

    /* Set specification */
    if (driver->set_spec != NULL) {
        driver->set_spec(&spec);
    }

    /* ACK response */
    Frame_Init(response, CMD_SPEC_ACK);
    Frame_AddByte(response, (uint8_t)sensor_id);
}

static void Handle_GetSpec(const Frame_t* request, Frame_t* response)
{
    /* Payload: [sensor_id] */
    if (request->payload_len < 1) {
        Commands_BuildNAK(response, ERR_INVALID_PAYLOAD);
        return;
    }

    SensorID_t sensor_id = (SensorID_t)request->payload[0];

    /* Get sensor driver */
    const SensorDriver_t* driver = SensorManager_GetByID(sensor_id);
    if (driver == NULL) {
        Commands_BuildNAK(response, ERR_INVALID_SENSOR_ID);
        return;
    }

    /* Check if spec is set */
    if (driver->has_spec != NULL && !driver->has_spec()) {
        Commands_BuildNAK(response, ERR_NO_SPEC);
        return;
    }

    /* Get specification */
    SensorSpec_t spec;
    if (driver->get_spec != NULL) {
        driver->get_spec(&spec);
    } else {
        memset(&spec, 0, sizeof(spec));
    }

    /* Build response */
    Frame_Init(response, CMD_SPEC_DATA);
    Frame_AddByte(response, (uint8_t)sensor_id);

    /* Serialize specification */
    if (driver->serialize_spec != NULL) {
        uint8_t spec_buffer[8];
        uint8_t spec_len = driver->serialize_spec(&spec, spec_buffer);
        Frame_AddBytes(response, spec_buffer, spec_len);
    } else {
        Frame_AddBytes(response, spec.raw, 4);
    }
}
