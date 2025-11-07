#pragma once
#include <stdint.h>

#define SCSLINK_SOF_MARKER 0xFD      // Start-of-frame marker
#define SCSLINK_MAX_PAYLOAD_SIZE 255 // Maximum payload size in bytes
#define SCSLINK_CORE_HEADER_LEN 6    // Length of core header (excluding payload)
#define SCSLINK_NUM_CHECKSUM_BYTES 2 // Checksum length in bytes
#define SCSLINK_SIGNATURE_LEN 13     // Signature length in bytes
#define SCSLINK_REQUIRES_SIGNATURE 0 // Set to 1 if signatures are required, 0 otherwise

typedef struct __scslink_message_t
{
    uint8_t sof;                                  // Start-of-frame marker (0xFD)
    uint8_t len;                                  // Payload length (0–255 bytes)
    uint8_t seq;                                  // Sequence number (0–255, increment per packet)
    uint8_t sysid;                                // System ID (unique per device)
    uint8_t compid;                               // Component ID (unique per subsystem)
    uint8_t msgid;                                // Message type ID
    uint8_t payload[SCSLINK_MAX_PAYLOAD_SIZE];    // Data for the message
    uint8_t checksum[SCSLINK_NUM_CHECKSUM_BYTES]; // CRC-16 (over header + payload)
    uint8_t signature[SCSLINK_SIGNATURE_LEN];     // Symmetric signature for authentication (if used)
} scslink_message_t;

typedef enum
{
    SCSLINK_PARSE_STATE_IDLE = 1,      // Waiting for start-of-frame
    SCSLINK_PARSE_STATE_GOT_SOF,       // Got start-of-frame
    SCSLINK_PARSE_STATE_GOT_LENGTH,    // Got payload length
    SCSLINK_PARSE_STATE_GOT_SEQ,       // Got sequence number
    SCSLINK_PARSE_STATE_GOT_SYSID,     // Got system ID
    SCSLINK_PARSE_STATE_GOT_COMPID,    // Got component ID
    SCSLINK_PARSE_STATE_GOT_MSGID,     // Got message ID
    SCSLINK_PARSE_STATE_GOT_PAYLOAD,   // Got payload
    SCSLINK_PARSE_STATE_GOT_CHECKSUM1, // Got checksum byte 1
    SCSLINK_PARSE_STATE_GOT_CHECKSUM2, // Got checksum byte 2
    SCSLINK_PARSE_STATE_GOT_SIGNATURE  // Got signature
} scslink_parse_state_t;

typedef struct __scslink_status
{
    uint8_t msg_received;              // Number of received messages
    uint8_t buffer_overrun;            // Number of buffer overruns
    uint8_t parse_error;               // Number of parse errors
    scslink_parse_state_t parse_state; // Parsing state machine
    uint8_t payload_index;             // Index of next byte to read in payload
    uint8_t current_rx_seq;            // Sequence number of current received packet
    uint8_t current_tx_seq;            // Sequence number of packets sent
    uint16_t packet_rx_success_count;  // Received packets
    uint16_t packet_rx_drop_count;     // Number of packet drops
    uint8_t signature_index;           // Current index in signature
    uint8_t signature_bad;             // Signature validation failures
} scslink_status_t;