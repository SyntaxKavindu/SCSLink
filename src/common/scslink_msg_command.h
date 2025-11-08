#pragma once
#include <stdint.h>
#include "../scslink_types.h"
#include "../scslink_helpers.h"

/**
 * @brief Command message implementation for SCSLink protocol
 *
 * This file implements the command message type used to send control commands
 * and requests to other systems in the SCSLink network. Commands can be used
 * to request specific messages, set message intervals, or trigger actions.
 *
 * Message ID: 75 (SCSLINK_MSG_COMMAND)
 * Payload size: 6 bytes
 */

// ============================================================================
// Message Constants
// ============================================================================

#define SCSLINK_MSG_ID_COMMAND 75              // Message ID for command message
#define SCSLINK_MSG_ID_REQUEST_MESSAGE 76      // Reserved: Message ID for request message
#define SCSLINK_MSG_ID_SET_MESSAGE_INTERVAL 77 // Reserved: Message ID for set message interval
#define SCSLINK_MSG_COMMAND_LEN 6              // Length of command payload (1+1+1+1+1+1 bytes)
#define SCSLINK_MSG_COMMAND_CRC_EXTRA 12       // CRC extra seed for command message

// ============================================================================
// Message Structure
// ============================================================================

/**
 * @brief Command/Request message structure
 *
 * This structure represents a command or request sent to another system.
 * It can be used to request specific message types, set streaming intervals,
 * or send other control commands.
 *
 * Payload layout (6 bytes total):
 * [0]: command          - Command type/ID
 * [1]: message_id       - Message ID being requested or affected
 * [2]: param            - Additional parameter (e.g., interval, count)
 * [3]: target_system    - Target system ID (0 = broadcast)
 * [4]: target_component - Target component ID (0 = all components)
 * [5]: reserved         - Reserved for future use (set to 0)
 */
typedef struct __scslink_request_message_t
{
    uint8_t command;          // Command type identifier (1 byte)
    uint8_t message_id;       // Message ID being requested or affected (1 byte)
    uint8_t param;            // Additional parameter for command (1 byte)
    uint8_t target_system;    // Target system ID (0 = broadcast to all) (1 byte)
    uint8_t target_component; // Target component ID (0 = all components) (1 byte)
    uint8_t reserved;         // Reserved for future use, must be 0 (1 byte)
} scslink_request_message_t;

// ============================================================================
// Pack and Encode Functions
// ============================================================================

/**
 * @brief Pack a command message into the message buffer
 *
 * This function packs all command parameters into the SCSLink message structure
 * by serializing each field into the payload buffer.
 *
 * @param msg              Pointer to the message structure to pack data into
 * @param sequence_num     Sequence number for this message
 * @param system_id        ID of this system (sender)
 * @param component_id     ID of this component (sender)
 * @param command          Command type identifier
 * @param message_id       Message ID being requested or affected
 * @param param            Additional parameter (e.g., interval in Hz, retry count)
 * @param target_system    Target system ID (0 for broadcast)
 * @param target_component Target component ID (0 for all components)
 * @return Length of the payload (6 bytes)
 */
static inline uint16_t scslink_msg_command_pack(scslink_message_t *msg,
                                                uint8_t sequence_num,
                                                uint8_t system_id,
                                                uint8_t component_id,
                                                uint8_t command,
                                                uint8_t message_id,
                                                uint8_t param,
                                                uint8_t target_system,
                                                uint8_t target_component)
{
    // Fill message header
    msg->sof = SCSLINK_SOF_MARKER;
    msg->len = SCSLINK_MSG_COMMAND_LEN;
    msg->seq = sequence_num;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->msgid = SCSLINK_MSG_ID_COMMAND;

    uint8_t *payload = msg->payload;
    uint16_t offset = 0;

    // Pack command (1 byte)
    payload[offset++] = command;

    // Pack message_id (1 byte)
    payload[offset++] = message_id;

    // Pack param (1 byte)
    payload[offset++] = param;

    // Pack target_system (1 byte)
    payload[offset++] = target_system;

    // Pack target_component (1 byte)
    payload[offset++] = target_component;

    // Pack reserved byte (1 byte)
    payload[offset++] = 0;

    // Return payload length
    return SCSLINK_MSG_COMMAND_LEN;
}

/**
 * @brief Encode a command message from a structure
 *
 * This function takes a pre-filled command structure and encodes it into
 * the message buffer by calling the pack function with individual fields.
 *
 * @param msg           Pointer to the message structure to pack data into
 * @param sequence_num  Sequence number for this message
 * @param system_id     ID of this system (sender)
 * @param component_id  ID of this component (sender)
 * @param command_msg   Pointer to the command structure to encode
 * @return Length of the payload (6 bytes)
 */
static inline uint16_t scslink_msg_command_encode(scslink_message_t *msg,
                                                  uint8_t sequence_num,
                                                  uint8_t system_id,
                                                  uint8_t component_id,
                                                  const scslink_request_message_t *command_msg)
{
    return scslink_msg_command_pack(msg,
                                    sequence_num,
                                    system_id,
                                    component_id,
                                    command_msg->command,
                                    command_msg->message_id,
                                    command_msg->param,
                                    command_msg->target_system,
                                    command_msg->target_component);
}

// ============================================================================
// Unpack and Decode Functions
// ============================================================================

/**
 * @brief Unpack a command message from the message payload
 *
 * This function extracts all command fields from the message payload buffer
 * and stores them in the command structure.
 *
 * @param command_msg  Pointer to the command structure to unpack data into
 * @param msg          Pointer to the message structure containing the payload
 */
static inline void scslink_msg_command_unpack(scslink_request_message_t *command_msg,
                                              const scslink_message_t *msg)
{
    const uint8_t *payload = msg->payload;
    uint16_t offset = 0;

    // Unpack command (1 byte)
    command_msg->command = payload[offset++];

    // Unpack message_id (1 byte)
    command_msg->message_id = payload[offset++];

    // Unpack param (1 byte)
    command_msg->param = payload[offset++];

    // Unpack target_system (1 byte)
    command_msg->target_system = payload[offset++];

    // Unpack target_component (1 byte)
    command_msg->target_component = payload[offset++];

    // Unpack reserved byte (1 byte)
    command_msg->reserved = payload[offset++];
}

/**
 * @brief Decode a command message (wrapper for unpack)
 *
 * This function is a convenience wrapper that calls unpack to decode the message.
 * It provides API consistency with the encode function.
 *
 * @param command_msg  Pointer to the command structure to decode data into
 * @param msg          Pointer to the message structure containing the payload
 */
static inline void scslink_msg_command_decode(scslink_request_message_t *command_msg,
                                              const scslink_message_t *msg)
{
    scslink_msg_command_unpack(command_msg, msg);
}

// ============================================================================
// Accessor Functions
// ============================================================================
// These functions allow reading specific fields without unpacking the entire message

/**
 * @brief Get the command field from a command message
 *
 * Extracts the command type identifier from the message payload.
 *
 * @param msg  Pointer to the message structure
 * @return Command type identifier
 */
static inline uint8_t scslink_msg_command_get_command(const scslink_message_t *msg)
{
    return msg->payload[0];
}

/**
 * @brief Get the message_id field from a command message
 *
 * Extracts the message ID being requested or affected by this command.
 *
 * @param msg  Pointer to the message structure
 * @return Message ID being requested or affected
 */
static inline uint8_t scslink_msg_command_get_message_id(const scslink_message_t *msg)
{
    return msg->payload[1];
}

/**
 * @brief Get the param field from a command message
 *
 * Extracts the additional parameter value (e.g., interval, count, or other command-specific data).
 *
 * @param msg  Pointer to the message structure
 * @return Additional parameter value
 */
static inline uint8_t scslink_msg_command_get_param(const scslink_message_t *msg)
{
    return msg->payload[2];
}

/**
 * @brief Get the target_system field from a command message
 *
 * Extracts the target system ID. A value of 0 indicates broadcast to all systems.
 *
 * @param msg  Pointer to the message structure
 * @return Target system ID (0 = broadcast)
 */
static inline uint8_t scslink_msg_command_get_target_system(const scslink_message_t *msg)
{
    return msg->payload[3];
}

/**
 * @brief Get the target_component field from a command message
 *
 * Extracts the target component ID. A value of 0 indicates all components.
 *
 * @param msg  Pointer to the message structure
 * @return Target component ID (0 = all components)
 */
static inline uint8_t scslink_msg_command_get_target_component(const scslink_message_t *msg)
{
    return msg->payload[4];
}

/**
 * @brief Get the reserved field from a command message
 *
 * Extracts the reserved byte (currently unused, should be 0).
 *
 * @param msg  Pointer to the message structure
 * @return Reserved byte value
 */
static inline uint8_t scslink_msg_command_get_reserved(const scslink_message_t *msg)
{
    return msg->payload[5];
}