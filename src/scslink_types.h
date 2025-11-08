#pragma once
#include <stdint.h>

/**
 * @brief Core type definitions and constants for the SCSLink protocol
 * 
 * This file contains the fundamental data structures and constants used throughout
 * the SCSLink communication protocol. It defines the message structure, parsing
 * states, and status tracking for reliable packet-based communication.
 */

// ============================================================================
// Protocol Constants
// ============================================================================

#define SCSLINK_SOF_MARKER 0xFD          // Start-of-frame marker byte (unique identifier)
#define SCSLINK_MAX_PAYLOAD_SIZE 255     // Maximum payload size in bytes (0-255)
#define SCSLINK_CORE_HEADER_LEN 6        // Core header length: SOF + LEN + SEQ + SYSID + COMPID + MSGID
#define SCSLINK_NUM_CHECKSUM_BYTES 2     // CRC-16 checksum length (2 bytes)
#define SCSLINK_SIGNATURE_LEN 13         // Optional signature length for authentication
#define SCSLINK_REQUIRES_SIGNATURE 0     // Enable (1) or disable (0) signature requirement

// Calculated constants
#define SCSLINK_MIN_PACKET_LEN (SCSLINK_CORE_HEADER_LEN + SCSLINK_NUM_CHECKSUM_BYTES)  // Minimum packet: 8 bytes
#define SCSLINK_MAX_PACKET_LEN (SCSLINK_CORE_HEADER_LEN + SCSLINK_MAX_PAYLOAD_SIZE + SCSLINK_NUM_CHECKSUM_BYTES + SCSLINK_SIGNATURE_LEN)  // Maximum packet size

// ============================================================================
// Message Structure
// ============================================================================

/**
 * @brief SCSLink message structure
 * 
 * This structure represents a complete SCSLink protocol message including header,
 * payload, checksum, and optional signature. The structure is designed to be
 * memory-efficient while providing all necessary fields for robust communication.
 * 
 * Message format:
 * [SOF(1)] [LEN(1)] [SEQ(1)] [SYSID(1)] [COMPID(1)] [MSGID(1)] [PAYLOAD(0-255)] [CRC(2)] [SIG(13)]
 * 
 * Total size: 6 (header) + 0-255 (payload) + 2 (CRC) + 0-13 (signature) = 8-276 bytes
 */
typedef struct __scslink_message_t
{
    uint8_t sof;                                  // Start-of-frame marker (always 0xFD)
    uint8_t len;                                  // Payload length in bytes (0-255)
    uint8_t seq;                                  // Sequence number (increments per packet, wraps at 256)
    uint8_t sysid;                                // System ID (identifies the sending device)
    uint8_t compid;                               // Component ID (identifies subsystem within device)
    uint8_t msgid;                                // Message type identifier (defines payload structure)
    uint8_t payload[SCSLINK_MAX_PAYLOAD_SIZE];    // Variable-length message data
    uint8_t checksum[SCSLINK_NUM_CHECKSUM_BYTES]; // CRC-16/X25 checksum (calculated over LEN through PAYLOAD)
    uint8_t signature[SCSLINK_SIGNATURE_LEN];     // Optional HMAC-like signature for message authentication
} scslink_message_t;

// ============================================================================
// Parser State Machine
// ============================================================================

/**
 * @brief Parser state machine enumeration
 * 
 * Defines the states for the byte-by-byte parser state machine. The parser
 * processes incoming bytes sequentially, transitioning through these states
 * to reconstruct complete messages from a serial byte stream.
 * 
 * State transitions:
 * IDLE -> GOT_SOF -> GOT_LENGTH -> GOT_SEQ -> GOT_SYSID -> GOT_COMPID -> 
 * GOT_MSGID -> GOT_PAYLOAD -> GOT_CHECKSUM1 -> GOT_CHECKSUM2 -> [GOT_SIGNATURE] -> IDLE
 */
typedef enum
{
    SCSLINK_PARSE_STATE_IDLE = 1,      // Waiting for start-of-frame marker (0xFD)
    SCSLINK_PARSE_STATE_GOT_SOF,       // Received SOF, waiting for length byte
    SCSLINK_PARSE_STATE_GOT_LENGTH,    // Received length, waiting for sequence number
    SCSLINK_PARSE_STATE_GOT_SEQ,       // Received sequence, waiting for system ID
    SCSLINK_PARSE_STATE_GOT_SYSID,     // Received system ID, waiting for component ID
    SCSLINK_PARSE_STATE_GOT_COMPID,    // Received component ID, waiting for message ID
    SCSLINK_PARSE_STATE_GOT_MSGID,     // Received message ID, collecting payload bytes
    SCSLINK_PARSE_STATE_GOT_PAYLOAD,   // Received complete payload, waiting for CRC low byte
    SCSLINK_PARSE_STATE_GOT_CHECKSUM1, // Received CRC low byte, waiting for CRC high byte
    SCSLINK_PARSE_STATE_GOT_CHECKSUM2, // Received CRC high byte, collecting signature (if enabled)
    SCSLINK_PARSE_STATE_GOT_SIGNATURE  // Received complete signature, message complete
} scslink_parse_state_t;

// ============================================================================
// Status and Statistics Tracking
// ============================================================================

/**
 * @brief SCSLink status and statistics structure
 * 
 * Maintains the current state of the parser and tracks communication statistics
 * for monitoring link quality and diagnosing issues. This structure should be
 * initialized to zero before first use.
 * 
 * Usage:
 * - parse_state: Current position in the state machine
 * - Counters: Track successful receptions, errors, and dropped packets
 * - Indices: Track current position within multi-byte fields
 */
typedef struct __scslink_status
{
    // Parser state
    scslink_parse_state_t parse_state; // Current state in the parsing state machine
    uint8_t payload_index;             // Current index while collecting payload bytes (0 to len-1)
    uint8_t signature_index;           // Current index while collecting signature bytes (0 to 12)
    
    // Sequence tracking
    uint8_t current_rx_seq;            // Last received sequence number
    uint8_t current_tx_seq;            // Next sequence number to transmit
    
    // Statistics counters
    uint8_t msg_received;              // Total valid messages received (wraps at 256)
    uint8_t parse_error;               // Number of CRC/checksum failures (wraps at 256)
    uint8_t signature_bad;             // Number of signature validation failures (wraps at 256)
    uint8_t buffer_overrun;            // Number of buffer overrun events (wraps at 256)
    uint16_t packet_rx_success_count;  // Total successful packet receptions (wraps at 65536)
    uint16_t packet_rx_drop_count;     // Total dropped packets due to errors (wraps at 65536)
} scslink_status_t;