#pragma once

/**
 * @brief Main header file for SCSLink protocol library
 * 
 * This is the primary include file for applications using the SCSLink protocol.
 * It provides a unified interface to all protocol functionality by including
 * the core types, helper functions, and all message definitions.
 * 
 * Usage:
 * Simply include this file in your application to access all SCSLink features:
 * @code
 * #include "scslink.h"
 * @endcode
 * 
 * @author SyntaxKavindu
 * @date 2025
 * @version 1.0
 */

// ============================================================================
// Core Protocol Files
// ============================================================================

#include "scslink_types.h"     // Core data structures and constants
#include "scslink_helpers.h"   // Utility functions (CRC, parsing, serialization)

// ============================================================================
// Message Definitions
// ============================================================================

#include "common/scslink_msg_heartbeat.h"          // Heartbeat/keepalive message
#include "common/scslink_msg_command.h"            // Command message
// Add additional message includes here as they are implemented

// ============================================================================
// Library Information
// ============================================================================

#define SCSLINK_VERSION_MAJOR 1
#define SCSLINK_VERSION_MINOR 0
#define SCSLINK_VERSION_PATCH 0

// Helper macro to create version number
#define SCSLINK_VERSION ((SCSLINK_VERSION_MAJOR << 16) | (SCSLINK_VERSION_MINOR << 8) | SCSLINK_VERSION_PATCH)
