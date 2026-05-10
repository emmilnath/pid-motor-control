/**
 * @file serial_protocol.c
 * @brief Binary serial protocol for real-time PID tuning communication.
 */
#include "serial_protocol.h"
#include <string.h>

/* CRC-8 with polynomial 0x07 (CRC-8/ITU) */
uint8_t Proto_CRC8(const uint8_t *data, uint8_t len)
{
    uint8_t crc = 0x00;
    for (uint8_t i = 0; i < len; i++) {
        crc ^= data[i];
        for (uint8_t bit = 0; bit < 8; bit++) {
            if (crc & 0x80)
                crc = (crc << 1) ^ 0x07;
            else
                crc <<= 1;
        }
    }
    return crc;
}

void Proto_InitParser(Proto_Parser *parser)
{
    parser->state = PARSE_WAIT_START;
    parser->payload_idx = 0;
    memset(&parser->frame, 0, sizeof(Proto_Frame));
}

bool Proto_FeedByte(Proto_Parser *parser, uint8_t byte)
{
    switch (parser->state) {
    case PARSE_WAIT_START:
        if (byte == PROTO_START_BYTE) {
            parser->state = PARSE_WAIT_CMD;
            parser->payload_idx = 0;
        }
        break;

    case PARSE_WAIT_CMD:
        parser->frame.cmd = byte;
        parser->state = PARSE_WAIT_LEN;
        break;

    case PARSE_WAIT_LEN:
        parser->frame.len = byte;
        if (byte > PROTO_MAX_PAYLOAD) {
            /* Invalid length — reset */
            parser->state = PARSE_WAIT_START;
            break;
        }
        if (byte == 0) {
            parser->state = PARSE_WAIT_CRC;
        } else {
            parser->state = PARSE_WAIT_PAYLOAD;
            parser->payload_idx = 0;
        }
        break;

    case PARSE_WAIT_PAYLOAD:
        parser->frame.payload[parser->payload_idx++] = byte;
        if (parser->payload_idx >= parser->frame.len) {
            parser->state = PARSE_WAIT_CRC;
        }
        break;

    case PARSE_WAIT_CRC:
        parser->frame.crc = byte;
        parser->state = PARSE_WAIT_START;

        /* Verify CRC: computed over [cmd, len, payload...] */
        {
            uint8_t buf[2 + PROTO_MAX_PAYLOAD];
            buf[0] = parser->frame.cmd;
            buf[1] = parser->frame.len;
            memcpy(&buf[2], parser->frame.payload, parser->frame.len);
            uint8_t expected = Proto_CRC8(buf, 2 + parser->frame.len);
            if (expected == parser->frame.crc) {
                return true;  /* Valid frame ready */
            }
        }
        break;
    }

    return false;
}

uint8_t Proto_BuildFrame(uint8_t *buf, uint8_t cmd,
                         const uint8_t *payload, uint8_t len)
{
    uint8_t idx = 0;
    buf[idx++] = PROTO_START_BYTE;
    buf[idx++] = cmd;
    buf[idx++] = len;
    if (payload && len > 0) {
        memcpy(&buf[idx], payload, len);
        idx += len;
    }
    /* CRC over [cmd, len, payload] */
    uint8_t crc_buf[2 + PROTO_MAX_PAYLOAD];
    crc_buf[0] = cmd;
    crc_buf[1] = len;
    if (payload && len > 0) {
        memcpy(&crc_buf[2], payload, len);
    }
    buf[idx++] = Proto_CRC8(crc_buf, 2 + len);
    return idx;
}
