#pragma once
#include <cstring>
#include "scslink_types.h"

#define SCSLINK_SIGNATURE_LEN 13

// Secret key for signature (can be unique per system)
static const uint8_t scslink_secret_key[SCSLINK_SIGNATURE_LEN] = {
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x11, 0x22, 0x33, 0x44, 0x55};

// Function to serialize a scslink_message_t into a byte buffer
uint16_t scslink_msg_to_send_buffer(uint8_t *buffer, const scslink_message_t *msg)
{

    // Start with header
    buffer[0] = msg->sof;    // start-of-frame
    buffer[1] = msg->len;    // payload length
    buffer[2] = msg->seq;    // sequence number
    buffer[3] = msg->sysid;  // system id
    buffer[4] = msg->compid; // component id
    buffer[5] = msg->msgid;  // message id

    // Copy payload
    memcpy(buffer + SCSLINK_CORE_HEADER_LEN, msg->payload, msg->len);

    // Calculate CRC
    uint16_t crc = scslink_crc_calculate(buffer + 1, SCSLINK_CORE_HEADER_LEN - 1 + msg->len);

    // Append CRC
    buffer[SCSLINK_CORE_HEADER_LEN + msg->len] = crc & 0xFF;
    buffer[SCSLINK_CORE_HEADER_LEN + msg->len + 1] = (crc >> 8) & 0xFF;

    // Return total length to send
    return SCSLINK_CORE_HEADER_LEN + msg->len + SCSLINK_NUM_CHECKSUM_BYTES;
}

// Function to calculate CRC-16/X25 checksum
uint16_t scslink_crc_calculate(const uint8_t *buffer, uint16_t length) // CRC-16/X25
{
    uint16_t crc = 0xFFFF;
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)buffer[i];
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)
                crc = (crc >> 1) ^ 0x8408; // CRC-16/X25
            else
                crc >>= 1;
        }
    }
    return crc;
}

// Function to verify the signature of a received message

uint8_t scslink_parse_char(uint8_t byte, scslink_message_t *r_message, scslink_status_t *r_status)
{
    static scslink_message_t *parsed_msg;
    static scslink_status_t *status;

    switch (status->parse_state)
    {
    case SCSLINK_PARSE_STATE_IDLE:
        // Waiting for start-of-frame (SOF) byte
        if (byte == SCSLINK_SOF_MARKER)
        {
            parsed_msg->sof = byte;
            parsed_msg->len = 0;
            status->payload_index = 0;
            status->parse_state = SCSLINK_PARSE_STATE_GOT_SOF;
        }
        break;

    case SCSLINK_PARSE_STATE_GOT_SOF:
        parsed_msg->len = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_LENGTH;
        break;

    case SCSLINK_PARSE_STATE_GOT_LENGTH:
        parsed_msg->seq = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_SEQ;
        break;

    case SCSLINK_PARSE_STATE_GOT_SEQ:
        parsed_msg->sysid = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_SYSID;
        break;

    case SCSLINK_PARSE_STATE_GOT_SYSID:
        parsed_msg->compid = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_COMPID;
        break;

    case SCSLINK_PARSE_STATE_GOT_COMPID:
        parsed_msg->msgid = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_MSGID;
        break;

    case SCSLINK_PARSE_STATE_GOT_MSGID:
        parsed_msg->payload[status->payload_index++] = byte;
        if (status->payload_index == parsed_msg->len)
        {
            status->parse_state = SCSLINK_PARSE_STATE_GOT_PAYLOAD;
            status->payload_index = 0; // Reset for next stage
        }
        break;
    case SCSLINK_PARSE_STATE_GOT_PAYLOAD:
        parsed_msg->checksum[0] = byte; // Low byte
        status->parse_state = SCSLINK_PARSE_STATE_GOT_CHECKSUM1;
        break;
    case SCSLINK_PARSE_STATE_GOT_CHECKSUM1:
        parsed_msg->checksum[1] = byte; // High byte

        uint16_t computed_checksum = scslink_crc_calculate(((uint8_t *)parsed_msg) + 1, 6 + parsed_msg->len);

        if ((computed_checksum & 0xFF) == parsed_msg->checksum[0] && ((computed_checksum >> 8) & 0xFF) == parsed_msg->checksum[1])
        {
            status->parse_state = SCSLINK_PARSE_STATE_GOT_CHECKSUM2;
            status->signature_index = 0;

            if (!SCSLINK_REQUIRES_SIGNATURE)
            {
                status->msg_received++;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
                return 1; // Message successfully parsed
            }
        }
        else
        {
            status->parse_state = SCSLINK_PARSE_STATE_IDLE;
            status->parse_error++;
        }
        break;

    case SCSLINK_PARSE_STATE_GOT_CHECKSUM2:

        // Collect signature bytes
        parsed_msg->signature[status->signature_index++] = byte;

        // After receiving full signature, verify it
        if (status->signature_index == SCSLINK_SIGNATURE_LEN)
        {
            status->parse_state = SCSLINK_PARSE_STATE_GOT_SIGNATURE;

            uint8_t expected_signature[SCSLINK_SIGNATURE_LEN];

            uint16_t computed_checksum = scslink_crc_calculate(((uint8_t *)parsed_msg) + 1, 6 + parsed_msg->len);

            for (int i = 0; i < SCSLINK_SIGNATURE_LEN; i++)
            {
                expected_signature[i] = ((computed_checksum >> (8 * (i % 2))) & 0xFF) ^ scslink_secret_key[i];
            }

            // Verify signature
            if (parsed_msg->signature == expected_signature)
            {
                status->msg_received++;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
                return 1; // Message successfully parsed
            }
            else
            {
                status->signature_bad++;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
            }
        }
        break;
    }

    return 0; // Continue parsing
}