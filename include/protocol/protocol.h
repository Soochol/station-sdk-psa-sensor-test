/**
 * @file protocol.h
 * @brief UART communication protocol constants and types
 * 
 * Defines the frame format and command codes for host-MCU communication.
 * 
 * Frame Format:
 *   [STX][LEN][CMD][PAYLOAD...][CRC][ETX]
 *   - STX: 0x02 (Start of frame)
 *   - LEN: Payload length (0-64)
 *   - CMD: Command code
 *   - PAYLOAD: Command-specific data
 *   - CRC: XOR checksum of LEN+CMD+PAYLOAD
 *   - ETX: 0x03 (End of frame)
 */

#ifndef PROTOCOL_H
#define PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stdbool.h>
#include "config.h"

/*============================================================================*/
/* Command Codes                                                              */
/*============================================================================*/

typedef enum {
    /* Host → MCU (Request) */
    CMD_PING                = 0x01,     /* Ping request */
    CMD_TEST_ALL            = 0x10,     /* Test all registered sensors */
    CMD_TEST_SINGLE         = 0x11,     /* Test specific sensor (payload: sensor_id) */
    CMD_GET_SENSOR_LIST     = 0x12,     /* Get list of registered sensors */
    CMD_READ_SENSOR         = 0x13,     /* Read sensor raw data (no spec comparison) */
    CMD_SET_SPEC            = 0x20,     /* Set sensor specification */
    CMD_GET_SPEC            = 0x21,     /* Get sensor specification */

    /* MCU → Host (Response) */
    CMD_PONG                = 0x01,     /* Ping response (same as PING) */
    CMD_TEST_RESULT         = 0x80,     /* Test result response */
    CMD_SENSOR_LIST         = 0x81,     /* Sensor list response */
    CMD_SPEC_ACK            = 0x82,     /* Specification set acknowledgement */
    CMD_SPEC_DATA           = 0x83,     /* Specification data response */
    CMD_SENSOR_DATA         = 0x84,     /* Raw sensor data response */
    CMD_NAK                 = 0xFE,     /* Negative acknowledgement (error) */
} CommandCode_t;

/*============================================================================*/
/* Error Codes                                                                */
/*============================================================================*/

typedef enum {
    ERR_NONE                = 0x00,     /* No error */
    ERR_UNKNOWN_CMD         = 0x01,     /* Unknown command code */
    ERR_INVALID_SENSOR_ID   = 0x02,     /* Invalid sensor ID */
    ERR_INVALID_PAYLOAD     = 0x03,     /* Invalid payload format/length */
    ERR_BUSY                = 0x04,     /* System busy (test in progress) */
    ERR_CRC_FAIL            = 0x05,     /* CRC verification failed */
    ERR_NO_SPEC             = 0x06,     /* Specification not set */
} ErrorCode_t;

/*============================================================================*/
/* Functions                                                                  */
/*============================================================================*/

/**
 * @brief Initialize protocol module
 * 
 * Initializes protocol internals and starts UART reception.
 */
void Protocol_Init(void);

/**
 * @brief Process protocol communications
 * 
 * Call this function from the main loop to process received frames
 * and send responses.
 */
void Protocol_Process(void);

/**
 * @brief Check if protocol is busy (test in progress)
 * @return true if busy, false if ready
 */
bool Protocol_IsBusy(void);

/**
 * @brief Feed data into protocol receive buffer
 *
 * Used to inject data from RTT or other sources into the protocol layer.
 *
 * @param data Pointer to data bytes
 * @param len Number of bytes to feed
 */
void Protocol_FeedData(const uint8_t* data, uint16_t len);

#ifdef __cplusplus
}
#endif

#endif /* PROTOCOL_H */
