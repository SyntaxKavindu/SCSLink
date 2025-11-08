#pragma once
#include <stdint.h>
#include "scslink_types.h"

/**
 * @brief Helper functions for SCSLink protocol operations
 * 
 * This file provides utility functions for:
 * - Message serialization (packing messages into byte buffers)
 * - CRC-16/X25 checksum calculation
 * - Byte-by-byte message parsing with state machine
 * - Optional signature-based authentication
 */

// ============================================================================
// Security Configuration
// ============================================================================

/**
 * @brief Secret key for message signature generation
 * 
 * This symmetric key is used to generate and verify message signatures.
 * IMPORTANT: Change this key for production systems and keep it secret!
 * Each system in the network should share the same key.
 */
static const uint8_t scslink_secret_key[SCSLINK_SIGNATURE_LEN] = {
    0x12, 0x34, 0x56, 0x78, 0x9A, 0xBC, 0xDE, 0xF0,
    0x11, 0x22, 0x33, 0x44, 0x55
};

// ============================================================================
// Message Serialization
// ============================================================================

/**
 * @brief Serialize a message structure into a byte buffer for transmission
 * 
 * This function takes a complete scslink_message_t structure and converts it
 * into a contiguous byte array ready for transmission over a serial link.
 * It automatically calculates and appends the CRC-16/X25 checksum.
 * 
 * @param buffer Pointer to output buffer (must be at least SCSLINK_CORE_HEADER_LEN + msg->len + 2 bytes)
 * @param msg    Pointer to the message structure to serialize
 * @return Total number of bytes written to buffer (header + payload + CRC)
 * 
 * @note This function does NOT add the optional signature. If signatures are
 *       required, they must be added separately.
 * 
 * Buffer layout:
 * [0]: SOF (0xFD)
 * [1]: Length
 * [2]: Sequence
 * [3]: System ID
 * [4]: Component ID
 * [5]: Message ID
 * [6..6+len-1]: Payload
 * [6+len]: CRC low byte
 * [6+len+1]: CRC high byte
 */
static inline uint16_t scslink_msg_to_send_buffer(uint8_t *buffer, const scslink_message_t *msg)
{
    // Pack header fields (6 bytes)
    buffer[0] = msg->sof;
    buffer[1] = msg->len;
    buffer[2] = msg->seq;
    buffer[3] = msg->sysid;
    buffer[4] = msg->compid;
    buffer[5] = msg->msgid;

    // Copy payload bytes (manual copy to avoid memcpy dependency)
    for (uint16_t i = 0; i < msg->len; i++)
    {
        buffer[SCSLINK_CORE_HEADER_LEN + i] = msg->payload[i];
    }

    // Calculate CRC over everything except SOF (from LEN through end of payload)
    uint16_t crc = scslink_crc_calculate(buffer + 1, SCSLINK_CORE_HEADER_LEN - 1 + msg->len);

    // Append CRC (little-endian: low byte first, then high byte)
    uint16_t crc_offset = SCSLINK_CORE_HEADER_LEN + msg->len;
    buffer[crc_offset] = crc & 0xFF;
    buffer[crc_offset + 1] = (crc >> 8) & 0xFF;

    // Return total packet length
    return SCSLINK_CORE_HEADER_LEN + msg->len + SCSLINK_NUM_CHECKSUM_BYTES;
}

// ============================================================================
// CRC Calculation
// ============================================================================

/**
 * @brief Calculate CRC-16/X25 checksum for data integrity verification
 * 
 * Implements the CRC-16/X25 algorithm (also known as CRC-16-CCITT with
 * polynomial 0x1021 in reversed form). This is the same CRC used by MAVLink.
 * 
 * Algorithm details:
 * - Polynomial: 0x8408 (reversed 0x1021)
 * - Initial value: 0xFFFF
 * - No final XOR
 * - Processes LSB first
 * 
 * @param buffer Pointer to data buffer to calculate CRC over
 * @param length Number of bytes to include in CRC calculation
 * @return 16-bit CRC value
 * 
 * @note This function is optimized for clarity over speed. For high-performance
 *       applications, consider using a table-based CRC implementation.
 */
static inline uint16_t scslink_crc_calculate(const uint8_t *buffer, uint16_t length)
{
    uint16_t crc = 0xFFFF;  // Initial CRC value
    
    // Process each byte
    for (uint16_t i = 0; i < length; i++)
    {
        crc ^= (uint16_t)buffer[i];  // XOR byte into CRC
        
        // Process each bit
        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1)  // If LSB is set
            {
                crc = (crc >> 1) ^ 0x8408;  // Shift right and XOR with polynomial
            }
            else
            {
                crc >>= 1;  // Just shift right
            }
        }
    }
    
    return crc;
}

// ============================================================================
// Message Parser
// ============================================================================

/**
 * @brief Parse incoming bytes into complete SCSLink messages
 * 
 * This is a byte-by-byte state machine parser that reconstructs complete messages
 * from a serial byte stream. Call this function for each received byte. When a
 * complete valid message is received, the function returns 1 and the message is
 * available in r_message.
 * 
 * Features:
 * - Handles byte synchronization (finds SOF marker)
 * - Validates message length and CRC
 * - Optional signature verification
 * - Tracks parsing statistics and errors
 * 
 * @param byte      The incoming byte to process
 * @param r_message Pointer to message structure to store completed message
 * @param r_status  Pointer to status structure for state tracking and statistics
 * @return 1 if a complete valid message was received, 0 if still parsing
 * 
 * @note This function uses static pointers that are initialized on first call.
 *       The r_message and r_status pointers are stored for subsequent calls.
 * 
 * Example usage:
 * @code
 * scslink_message_t msg;
 * scslink_status_t status = {0};
 * 
 * while (serial_data_available()) {
 *     uint8_t byte = serial_read_byte();
 *     if (scslink_parse_char(byte, &msg, &status)) {
 *         // Complete message received, process it
 *         handle_message(&msg);
 *     }
 * }
 * @endcode
 */
static inline uint8_t scslink_parse_char(uint8_t byte, scslink_message_t *r_message, scslink_status_t *r_status)
{
    static scslink_message_t *parsed_msg = NULL;
    static scslink_status_t *status = NULL;

    // Initialize static pointers on first call
    if (parsed_msg == NULL)
    {
        parsed_msg = r_message;
        status = r_status;
    }

    switch (status->parse_state)
    {
    case SCSLINK_PARSE_STATE_IDLE:
        // Waiting for start-of-frame (SOF) byte
        if (byte == SCSLINK_SOF_MARKER)
        {
            parsed_msg->sof = byte;
            parsed_msg->len = 0;
            status->payload_index = 0;
            status->signature_index = 0;
            status->parse_state = SCSLINK_PARSE_STATE_GOT_SOF;
        }
        // Ignore all other bytes while idle
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
        // Receive CRC low byte
        parsed_msg->checksum[0] = byte;
        status->parse_state = SCSLINK_PARSE_STATE_GOT_CHECKSUM1;
        break;
        
    case SCSLINK_PARSE_STATE_GOT_CHECKSUM1:
        // Receive CRC high byte and validate
        parsed_msg->checksum[1] = byte;

        // Calculate expected CRC over LEN through PAYLOAD
        uint16_t computed_checksum = scslink_crc_calculate(((uint8_t *)parsed_msg) + 1, SCSLINK_CORE_HEADER_LEN - 1 + parsed_msg->len);
        uint16_t received_checksum = parsed_msg->checksum[0] | (parsed_msg->checksum[1] << 8);

        if (computed_checksum == received_checksum)
        {
            // CRC is valid
            status->parse_state = SCSLINK_PARSE_STATE_GOT_CHECKSUM2;
            status->packet_rx_success_count++;

            // If signature not required, message is complete
            if (!SCSLINK_REQUIRES_SIGNATURE)
            {
                status->msg_received++;
                status->current_rx_seq = parsed_msg->seq;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
                return 1;  // Message successfully received
            }
        }
        else
        {
            // CRC mismatch - drop packet
            status->parse_error++;
            status->packet_rx_drop_count++;
            status->parse_state = SCSLINK_PARSE_STATE_IDLE;
        }
        break;

    case SCSLINK_PARSE_STATE_GOT_CHECKSUM2:
        // Collect signature bytes (only if SCSLINK_REQUIRES_SIGNATURE is enabled)
        parsed_msg->signature[status->signature_index++] = byte;

        // Check if we've received all signature bytes
        if (status->signature_index == SCSLINK_SIGNATURE_LEN)
        {
            status->parse_state = SCSLINK_PARSE_STATE_GOT_SIGNATURE;

            // Generate expected signature for verification
            uint8_t expected_signature[SCSLINK_SIGNATURE_LEN];
            uint16_t crc_for_sig = scslink_crc_calculate(((uint8_t *)parsed_msg) + 1, SCSLINK_CORE_HEADER_LEN - 1 + parsed_msg->len);

            // Simple signature: XOR CRC bytes with secret key
            for (uint8_t i = 0; i < SCSLINK_SIGNATURE_LEN; i++)
            {
                expected_signature[i] = ((crc_for_sig >> (8 * (i % 2))) & 0xFF) ^ scslink_secret_key[i];
            }

            // Verify signature byte by byte
            uint8_t signature_valid = 1;
            for (uint8_t i = 0; i < SCSLINK_SIGNATURE_LEN; i++)
            {
                if (parsed_msg->signature[i] != expected_signature[i])
                {
                    signature_valid = 0;
                    break;
                }
            }

            if (signature_valid)
            {
                // Signature is valid
                status->msg_received++;
                status->current_rx_seq = parsed_msg->seq;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
                return 1;  // Message successfully received
            }
            else
            {
                // Signature mismatch
                status->signature_bad++;
                status->packet_rx_drop_count++;
                status->parse_state = SCSLINK_PARSE_STATE_IDLE;
            }
        }
        break;
    default:
        // Unknown state - reset to idle
        status->parse_state = SCSLINK_PARSE_STATE_IDLE;
        break;
    }

    return 0;  // Continue parsing (message not yet complete)
}

// ============================================================================
// Utility Functions
// ============================================================================

/**
 * @brief Initialize SCSLink status structure
 * 
 * Resets all counters and sets the parser to idle state. Call this before
 * starting to parse messages.
 * 
 * @param status Pointer to status structure to initialize
 */
static inline void scslink_status_init(scslink_status_t *status)
{
    status->parse_state = SCSLINK_PARSE_STATE_IDLE;
    status->payload_index = 0;
    status->signature_index = 0;
    status->current_rx_seq = 0;
    status->current_tx_seq = 0;
    status->msg_received = 0;
    status->parse_error = 0;
    status->signature_bad = 0;
    status->buffer_overrun = 0;
    status->packet_rx_success_count = 0;
    status->packet_rx_drop_count = 0;
}

/**
 * @brief Get next sequence number for transmission
 * 
 * Returns the current TX sequence number and increments it for the next message.
 * Sequence numbers wrap around at 256.
 * 
 * @param status Pointer to status structure
 * @return Sequence number to use for the next transmitted message
 */
static inline uint8_t scslink_get_next_seq(scslink_status_t *status)
{
    return status->current_tx_seq++;
}