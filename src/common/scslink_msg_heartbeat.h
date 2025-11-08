#pragma once
#include <stdint.h>
#include "../scslink_types.h"

#define SCSLINK_MSG_ID_HEARTBEAT 0         // Message ID for heartbeat
#define SCSLINK_MSG_HEARTBEAT_LEN 9        // Length of heartbeat payload
#define SCSLINK_MSG_HEARTBEAT_CRC_EXTRA 50 // CRC extra for heartbeat message

/**
 * @brief Heartbeat message structure
 *
 * This message is emitted to announce the presence of a system and provide
 * basic status information. It allows other systems to detect the presence
 * and operational state of this system.
 *
 * Total payload size: 9 bytes
 */
typedef struct __scslink_heartbeat_t
{
    uint32_t custom_mode;    // Custom mode, can be defined by user/autopilot (4 bytes)
    uint8_t type;            // Type of the system (quadrotor, helicopter, ground rover, etc.) (1 byte)
    uint8_t autopilot;       // Autopilot type / class (1 byte)
    uint8_t base_mode;       // System mode bitmap (1 byte)
    uint8_t system_status;   // System status flag (1 byte)
    uint8_t scslink_version; // SCSLink version, not writable by user, gets added by protocol (1 byte)
} scslink_heartbeat_t;

/**
 * @brief Pack a heartbeat message into the message buffer
 *
 * This function packs all heartbeat parameters into the SCSLink message structure
 * by manually serializing each field into the payload buffer in little-endian format.
 *
 * @param msg              Pointer to the message structure to pack data into
 * @param sequence_num     Sequence number for this message
 * @param system_id        ID of this system (1-255)
 * @param component_id     ID of this component (0-255)
 * @param custom_mode      Custom mode, can be defined by user/autopilot
 * @param type             Type of the system (MAV_TYPE)
 * @param autopilot        Autopilot type / class (MAV_AUTOPILOT)
 * @param base_mode        System mode bitmap (MAV_MODE_FLAG)
 * @param system_status    System status flag (MAV_STATE)
 * @param scslink_version  SCSLink version (typically 3 for SCSLink v2)
 * @return Length of the payload (number of bytes in payload)
 */
static inline uint16_t scslink_msg_heartbeat_pack(scslink_message_t *msg,
                                                  uint8_t sequence_num,
                                                  uint8_t system_id,
                                                  uint8_t component_id,
                                                  uint32_t custom_mode,
                                                  uint8_t type,
                                                  uint8_t autopilot,
                                                  uint8_t base_mode,
                                                  uint8_t system_status,
                                                  uint8_t scslink_version)
{
    // Fill message header
    msg->sof = SCSLINK_SOF_MARKER;
    msg->len = SCSLINK_MSG_HEARTBEAT_LEN;
    msg->seq = sequence_num;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->msgid = SCSLINK_MSG_ID_HEARTBEAT;

    // Get pointer to payload for easier access
    uint8_t *payload = msg->payload;
    uint16_t offset = 0;

    // Pack custom_mode (4 bytes - little endian)
    payload[offset++] = (custom_mode >> 0) & 0xFF;
    payload[offset++] = (custom_mode >> 8) & 0xFF;
    payload[offset++] = (custom_mode >> 16) & 0xFF;
    payload[offset++] = (custom_mode >> 24) & 0xFF;

    // Pack type (1 byte)
    payload[offset++] = type;

    // Pack autopilot (1 byte)
    payload[offset++] = autopilot;

    // Pack base_mode (1 byte)
    payload[offset++] = base_mode;

    // Pack system_status (1 byte)
    payload[offset++] = system_status;

    // Pack scslink_version (1 byte)
    payload[offset++] = scslink_version;

    // Return payload length
    return SCSLINK_MSG_HEARTBEAT_LEN;
}

/**
 * @brief Encode a heartbeat message from a structure
 *
 * This function takes a pre-filled heartbeat structure and encodes it into
 * the message buffer by calling the pack function with individual fields.
 *
 * @param msg           Pointer to the message structure to pack data into
 * @param sequence_num  Sequence number for this message
 * @param system_id     ID of this system
 * @param component_id  ID of this component
 * @param heartbeat_msg Pointer to the heartbeat structure to encode
 * @return Length of the payload (number of bytes in payload)
 */
static inline uint16_t scslink_msg_heartbeat_encode(scslink_message_t *msg,
                                                    uint8_t sequence_num,
                                                    uint8_t system_id,
                                                    uint8_t component_id,
                                                    const scslink_heartbeat_t *heartbeat_msg)
{
    return scslink_msg_heartbeat_pack(msg,
                                      sequence_num,
                                      system_id,
                                      component_id,
                                      heartbeat_msg->custom_mode,
                                      heartbeat_msg->type,
                                      heartbeat_msg->autopilot,
                                      heartbeat_msg->base_mode,
                                      heartbeat_msg->system_status,
                                      heartbeat_msg->scslink_version);
}

/**
 * @brief Unpack a heartbeat message from the message payload
 *
 * This function extracts all heartbeat fields from the message payload buffer
 * and stores them in the heartbeat structure, handling little-endian byte order.
 *
 * @param heartbeat_msg Pointer to the heartbeat structure to unpack data into
 * @param msg           Pointer to the message structure containing the payload
 */
static inline void scslink_msg_heartbeat_unpack(scslink_heartbeat_t *heartbeat_msg,
                                                const scslink_message_t *msg)
{
    const uint8_t *payload = msg->payload;
    uint16_t offset = 0;

    // Unpack custom_mode (4 bytes - little endian)
    heartbeat_msg->custom_mode = 0;
    heartbeat_msg->custom_mode |= ((uint32_t)payload[offset++]) << 0;
    heartbeat_msg->custom_mode |= ((uint32_t)payload[offset++]) << 8;
    heartbeat_msg->custom_mode |= ((uint32_t)payload[offset++]) << 16;
    heartbeat_msg->custom_mode |= ((uint32_t)payload[offset++]) << 24;

    // Unpack type (1 byte)
    heartbeat_msg->type = payload[offset++];

    // Unpack autopilot (1 byte)
    heartbeat_msg->autopilot = payload[offset++];

    // Unpack base_mode (1 byte)
    heartbeat_msg->base_mode = payload[offset++];

    // Unpack system_status (1 byte)
    heartbeat_msg->system_status = payload[offset++];

    // Unpack scslink_version (1 byte)
    heartbeat_msg->scslink_version = payload[offset++];
}

/**
 * @brief Decode a heartbeat message (wrapper for unpack)
 *
 * This function is a convenience wrapper that calls unpack to decode the message.
 * It provides API consistency with the encode function.
 *
 * @param heartbeat_msg Pointer to the heartbeat structure to decode data into
 * @param msg           Pointer to the message structure containing the payload
 */
static inline void scslink_msg_heartbeat_decode(scslink_heartbeat_t *heartbeat_msg,
                                                const scslink_message_t *msg)
{
    scslink_msg_heartbeat_unpack(heartbeat_msg, msg);
}

// ============================================================================
// Accessor functions to get individual fields from a heartbeat message
// These functions allow reading specific fields without unpacking the entire message
// ============================================================================

/**
 * @brief Get the custom_mode field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return Custom mode value (4 bytes)
 */
static inline uint32_t scslink_msg_heartbeat_get_custom_mode(const scslink_message_t *msg)
{
    const uint8_t *payload = msg->payload;
    uint32_t custom_mode = 0;
    custom_mode |= ((uint32_t)payload[0]) << 0;
    custom_mode |= ((uint32_t)payload[1]) << 8;
    custom_mode |= ((uint32_t)payload[2]) << 16;
    custom_mode |= ((uint32_t)payload[3]) << 24;
    return custom_mode;
}

/**
 * @brief Get the type field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return Type of the system
 */
static inline uint8_t scslink_msg_heartbeat_get_type(const scslink_message_t *msg)
{
    return msg->payload[4];
}

/**
 * @brief Get the autopilot field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return Autopilot type / class
 */
static inline uint8_t scslink_msg_heartbeat_get_autopilot(const scslink_message_t *msg)
{
    return msg->payload[5];
}

/**
 * @brief Get the base_mode field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return System mode bitmap
 */
static inline uint8_t scslink_msg_heartbeat_get_base_mode(const scslink_message_t *msg)
{
    return msg->payload[6];
}

/**
 * @brief Get the system_status field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return System status flag
 */
static inline uint8_t scslink_msg_heartbeat_get_system_status(const scslink_message_t *msg)
{
    return msg->payload[7];
}

/**
 * @brief Get the scslink_version field from a heartbeat message
 *
 * @param msg Pointer to the message structure
 * @return SCSLink protocol version
 */
static inline uint8_t scslink_msg_heartbeat_get_scslink_version(const scslink_message_t *msg)
{
    return msg->payload[8];
}