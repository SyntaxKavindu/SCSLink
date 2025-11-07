#pragma once
#include <stdint.h>
#include <cstring>
#include "scslink_types.h"

#define SCSLINK_HEARTBEAT_MSG 0        // Message ID for heartbeat
#define SCSLINK_HEARTBEAT_LEN 9        // Length of heartbeat payload
#define SCSLINK_HEARTBEAT_CRC_EXTRA 50 // CRC extra for heartbeat message

// Heartbeat message structure
typedef struct __scslink_heartbeat_t
{
    uint32_t custom_mode;    // Flight mode
    uint8_t type;            // SCSLINK_TYPE
    uint8_t autopilot;       // SCSLINK_AUTOPILOT
    uint8_t base_mode;       // SCSLINK_MODE_FLAG
    uint8_t system_status;   // SCSLINK_STATE
    uint8_t scslink_version; // Always 3 for SCSLINK2
} scslink_heartbeat_t;

// Function to pack a heartbeat message
static inline uint16_t scslink_msg_heartbeat_pack(scslink_message_t *msg, uint8_t sequence_num, uint8_t system_id, uint8_t component_id, uint32_t custom_mode, uint8_t type, uint8_t autopilot, uint8_t base_mode, uint8_t system_status, uint8_t scslink_version)
{
    msg->sof = SCSLINK_SOF_MARKER;
    msg->len = SCSLINK_HEARTBEAT_LEN;
    msg->seq = sequence_num;
    msg->sysid = system_id;
    msg->compid = component_id;
    msg->msgid = SCSLINK_HEARTBEAT_MSG;

    scslink_heartbeat_t *heartbeat;
    heartbeat->custom_mode = custom_mode;
    heartbeat->type = type;
    heartbeat->autopilot = autopilot;
    heartbeat->base_mode = base_mode;
    heartbeat->system_status = system_status;
    heartbeat->scslink_version = scslink_version;

    // Copy payload
    memcpy(msg->payload, (const uint8_t *)heartbeat, SCSLINK_HEARTBEAT_LEN);

    // Return total message length
    return SCSLINK_HEARTBEAT_LEN;
}

// Function to encode a heartbeat message
static inline uint16_t scslink_msg_heartbeat_encode(scslink_message_t *msg, uint8_t sequence_num, uint8_t system_id, uint8_t component_id, const scslink_heartbeat_t *heartbeat_msg)
{
    return scslink_msg_heartbeat_pack(msg, sequence_num, system_id, component_id, heartbeat_msg->custom_mode, heartbeat_msg->type, heartbeat_msg->autopilot, heartbeat_msg->base_mode, heartbeat_msg->system_status, heartbeat_msg->scslink_version);
}

// Function to unpack a heartbeat message
static inline void scslink_msg_heartbeat_unpack(scslink_heartbeat_t *heartbeat_msg, const scslink_message_t *msg)
{
    memcpy(heartbeat_msg, msg->payload, SCSLINK_HEARTBEAT_LEN);
}

// Function to decode a heartbeat message
static inline void scslink_msg_heartbeat_decode(scslink_heartbeat_t *heartbeat_msg, const scslink_message_t *msg)
{
    scslink_msg_heartbeat_unpack(heartbeat_msg, msg);
}

// Accessor function to get the type field from a heartbeat message
static inline uint8_t scslink_msg_heartbeat_get_type(const scslink_message_t *msg)
{
    scslink_heartbeat_t heartbeat_msg;
    scslink_msg_heartbeat_unpack(&heartbeat_msg, msg);
    return heartbeat_msg.type;
}

// Accessor function to get the autopilot field from a heartbeat message
static inline uint8_t scslink_msg_heartbeat_get_autopilot(const scslink_message_t *msg)
{
    scslink_heartbeat_t heartbeat_msg;
    scslink_msg_heartbeat_unpack(&heartbeat_msg, msg);
    return heartbeat_msg.autopilot;
}

// Accessor function to get the base_mode field from a heartbeat message
static inline uint8_t scslink_msg_heartbeat_get_base_mode(const scslink_message_t *msg)
{
    scslink_heartbeat_t heartbeat_msg;
    scslink_msg_heartbeat_unpack(&heartbeat_msg, msg);
    return heartbeat_msg.base_mode;
}

// Accessor function to get the system_status field from a heartbeat message
static inline uint8_t scslink_msg_heartbeat_get_system_status(const scslink_message_t *msg)
{
    scslink_heartbeat_t heartbeat_msg;
    scslink_msg_heartbeat_unpack(&heartbeat_msg, msg);
    return heartbeat_msg.system_status;
}

// Accessor function to get the scslink_version field from a heartbeat message
static inline uint8_t scslink_msg_heartbeat_get_scslink_version(const scslink_message_t *msg)
{
    scslink_heartbeat_t heartbeat_msg;
    scslink_msg_heartbeat_unpack(&heartbeat_msg, msg);
    return heartbeat_msg.scslink_version;
}