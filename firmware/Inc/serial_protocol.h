/**
 * @file serial_protocol.h
 * @brief UART protocol for real-time tuning GUI communication.
 *
 * Frame format (binary, little-endian):
 *   [START 0xAA] [CMD 1B] [LEN 1B] [PAYLOAD LEN B] [CRC8 1B]
 *
 * Commands:
 *   0x01  SET_GAINS   payload: Kp(f32) Ki(f32) Kd(f32)
 *   0x02  SET_SETPOINT payload: setpoint(f32)
 *   0x03  GET_STATE    payload: (none)
 *   0x04  ENABLE       payload: enable(u8)
 *   0x05  RESET        payload: (none)
 *
 * Responses:
 *   0x81  STATE_REPORT payload: time(u32ms) setpoint(f32) rpm(f32) error(f32)
 *                               output(f32) Kp(f32) Ki(f32) Kd(f32)
 *   0x82  ACK          payload: cmd_acked(u8)
 *   0x83  NACK         payload: cmd_nacked(u8) error_code(u8)
 */
#ifndef SERIAL_PROTOCOL_H
#define SERIAL_PROTOCOL_H

#include <stdint.h>
#include <stdbool.h>

#define PROTO_START_BYTE  0xAA
#define PROTO_MAX_PAYLOAD 32

/* Command IDs */
#define CMD_SET_GAINS     0x01
#define CMD_SET_SETPOINT  0x02
#define CMD_GET_STATE     0x03
#define CMD_ENABLE        0x04
#define CMD_RESET         0x05

/* Response IDs */
#define RESP_STATE_REPORT 0x81
#define RESP_ACK          0x82
#define RESP_NACK         0x83

typedef struct {
    uint8_t  cmd;
    uint8_t  len;
    uint8_t  payload[PROTO_MAX_PAYLOAD];
    uint8_t  crc;
} Proto_Frame;

typedef enum {
    PARSE_WAIT_START,
    PARSE_WAIT_CMD,
    PARSE_WAIT_LEN,
    PARSE_WAIT_PAYLOAD,
    PARSE_WAIT_CRC
} Parse_State;

typedef struct {
    Parse_State state;
    Proto_Frame frame;
    uint8_t     payload_idx;
} Proto_Parser;

/**
 * Initialise parser state machine.
 */
void Proto_InitParser(Proto_Parser *parser);

/**
 * Feed one byte to the parser.
 * @return true when a complete valid frame is ready in parser->frame.
 */
bool Proto_FeedByte(Proto_Parser *parser, uint8_t byte);

/**
 * Build a frame into a buffer. Returns total frame length.
 */
uint8_t Proto_BuildFrame(uint8_t *buf, uint8_t cmd,
                         const uint8_t *payload, uint8_t len);

/**
 * CRC-8 (polynomial 0x07) over data.
 */
uint8_t Proto_CRC8(const uint8_t *data, uint8_t len);

#endif /* SERIAL_PROTOCOL_H */
