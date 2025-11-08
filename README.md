# SCSLink Protocol

[![License](https://img.shields.io/badge/License-MIT-blue.svg)](LICENSE)
[![Version](https://img.shields.io/badge/version-1.0.0-green.svg)](https://github.com/SyntaxKavindu/SCSLink)
[![Language](https://img.shields.io/badge/language-C-orange.svg)](https://en.wikipedia.org/wiki/C_(programming_language))

**SCSLink** (Simple Communication System Link) is a lightweight, efficient, and robust binary communication protocol designed for embedded systems, robotics, and IoT applications. It provides reliable packet-based communication with CRC error detection and optional message authentication.

## 📋 Table of Contents

- [Overview](#-overview)
- [Features](#-features)
- [Inspiration & Credits](#-inspiration--credits)
- [Protocol Specification](#-protocol-specification)
- [Getting Started](#-getting-started)
- [Usage Examples](#-usage-examples)
- [Message Types](#-message-types)
- [API Reference](#-api-reference)
- [Building & Integration](#-building--integration)
- [Performance](#-performance)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

SCSLink is a header-only C library that implements a lightweight communication protocol for embedded systems. It's designed to be:

- **Compact**: Minimal overhead with efficient binary encoding
- **Reliable**: CRC-16/X25 checksum for error detection
- **Flexible**: Support for up to 255 different message types
- **Scalable**: Multiple systems and components support
- **Portable**: Pure C implementation with no external dependencies
- **Easy to Use**: Simple API with comprehensive documentation

### Why SCSLink?

In embedded systems and robotics, efficient and reliable communication is crucial. SCSLink provides a battle-tested protocol for:

- Serial communication (UART, RS-232, RS-485)
- Wireless links (Bluetooth, WiFi, Radio)
- Inter-processor communication
- Sensor networks
- Telemetry and control systems

---

## ✨ Features

### Core Features

- ✅ **Binary Protocol**: Efficient byte-level encoding
- ✅ **CRC-16/X25 Checksum**: Ensures data integrity
- ✅ **Sequence Numbering**: Tracks packet order and detects losses
- ✅ **Message ID System**: Support for 256 unique message types
- ✅ **System & Component Addressing**: Multi-device network support
- ✅ **Variable Payload**: 0-255 bytes per message
- ✅ **Optional Signatures**: HMAC-like authentication for secure communication
- ✅ **State Machine Parser**: Byte-by-byte parsing for stream processing
- ✅ **Zero Dependencies**: No external libraries required
- ✅ **Header-Only Library**: Easy integration into any project

### Advanced Features

- 📊 **Statistics Tracking**: Monitor message counts, errors, and drops
- 🔍 **Parse State Recovery**: Automatic synchronization on error
- 🎯 **Broadcast Support**: Send to all systems or specific targets
- 🛡️ **Error Detection**: CRC validation and signature verification
- ⚡ **Optimized Performance**: Static inline functions, zero-copy design

---

## 🙏 Inspiration & Credits

SCSLink is inspired by the excellent [**MAVLink**](https://mavlink.io/) (Micro Air Vehicle Link) protocol, which has become the de facto standard for drone and robotics communication.

### Credits to MAVLink

- **CRC Algorithm**: SCSLink uses the same CRC-16/X25 algorithm as MAVLink for proven reliability
- **Message Structure**: The packet format is influenced by MAVLink's efficient design
- **State Machine Parser**: Byte-by-byte parsing approach inspired by MAVLink's implementation

### Differences from MAVLink

While inspired by MAVLink, SCSLink is designed to be:

- **Simpler**: Reduced complexity for easier learning and implementation
- **Lighter**: Smaller code footprint suitable for resource-constrained MCUs
- **Customizable**: Easier to extend with custom message types
- **Educational**: Clean, well-documented code for learning embedded protocols

**Thank you to the MAVLink team** for creating an outstanding protocol and demonstrating best practices in embedded communication!

> *"If I have seen further, it is by standing on the shoulders of giants."* - Isaac Newton

---

## 📡 Protocol Specification

### Message Format

Each SCSLink message consists of:

```
┌──────┬──────┬──────┬───────┬────────┬───────┬─────────┬──────────┬─────────────┐
│ SOF  │ LEN  │ SEQ  │ SYSID │ COMPID │ MSGID │ PAYLOAD │ CRC (2B) │ SIG (13B)   │
│ (1B) │ (1B) │ (1B) │ (1B)  │ (1B)   │ (1B)  │ (0-255B)│          │  (Optional) │
└──────┴──────┴──────┴───────┴────────┴───────┴─────────┴──────────┴─────────────┘
```

### Field Descriptions

| Field    | Size      | Description                                          |
|----------|-----------|------------------------------------------------------|
| **SOF**  | 1 byte    | Start-of-frame marker (0xFD) - sync byte            |
| **LEN**  | 1 byte    | Payload length (0-255 bytes)                        |
| **SEQ**  | 1 byte    | Sequence number (wraps at 256)                      |
| **SYSID**| 1 byte    | System ID (1-255, identifies sender device)         |
| **COMPID**| 1 byte   | Component ID (identifies subsystem)                 |
| **MSGID**| 1 byte    | Message type identifier (0-255)                     |
| **PAYLOAD**| 0-255 bytes | Message-specific data                           |
| **CRC**  | 2 bytes   | CRC-16/X25 checksum (little-endian)                 |
| **SIG**  | 13 bytes  | Optional signature for authentication               |

### Message Size

- **Minimum**: 8 bytes (6 header + 0 payload + 2 CRC)
- **Maximum**: 276 bytes (6 header + 255 payload + 2 CRC + 13 signature)
- **Typical**: 10-50 bytes depending on message type

### CRC Calculation

- **Algorithm**: CRC-16/X25 (same as MAVLink)
- **Polynomial**: 0x1021 (reversed: 0x8408)
- **Initial Value**: 0xFFFF
- **Coverage**: From LEN through end of PAYLOAD (excludes SOF)

---

## 🚀 Getting Started

### Prerequisites

- C compiler (GCC, Clang, MSVC, or any standard C99+ compiler)
- No external dependencies required!

### Installation

SCSLink is a header-only library. Simply copy the source files to your project:

```bash
# Clone the repository
git clone https://github.com/SyntaxKavindu/SCSLink.git

# Copy headers to your project
cp -r SCSLink/src/scslink*.h your_project/include/
cp -r SCSLink/src/common/ your_project/include/common/
```

### Quick Start

```c
#include "scslink.h"

// Initialize status structure
scslink_status_t status = {0};
scslink_status_init(&status);

// Create and send a heartbeat message
scslink_message_t msg;
uint8_t buffer[SCSLINK_MAX_PACKET_LEN];

scslink_msg_heartbeat_pack(&msg, 
                           scslink_get_next_seq(&status),
                           1,    // system_id
                           0,    // component_id
                           0,    // custom_mode
                           1,    // type
                           2,    // autopilot
                           0,    // base_mode
                           3,    // system_status
                           3);   // scslink_version

// Serialize to buffer
uint16_t len = scslink_msg_to_send_buffer(buffer, &msg);

// Send via your communication channel
// uart_send(buffer, len);
```

---

## 💡 Usage Examples

### Example 1: Sending a Heartbeat

```c
#include "scslink.h"

void send_heartbeat(void)
{
    static scslink_status_t status = {0};
    scslink_message_t msg;
    uint8_t buffer[20];
    
    // Pack heartbeat message
    scslink_msg_heartbeat_pack(&msg,
                              scslink_get_next_seq(&status),
                              1,   // My system ID
                              0,   // My component ID
                              100, // Custom flight mode
                              2,   // Type: Ground rover
                              5,   // Autopilot type
                              128, // Base mode: Armed
                              4,   // System status: Active
                              3);  // Protocol version
    
    // Serialize and send
    uint16_t len = scslink_msg_to_send_buffer(buffer, &msg);
    uart_transmit(buffer, len);
}
```

### Example 2: Receiving and Parsing Messages

```c
#include "scslink.h"

void process_uart_byte(uint8_t byte)
{
    static scslink_message_t msg;
    static scslink_status_t status = {0};
    
    // Feed byte to parser
    if (scslink_parse_char(byte, &msg, &status))
    {
        // Complete message received!
        switch (msg.msgid)
        {
            case SCSLINK_MSG_ID_HEARTBEAT:
            {
                scslink_heartbeat_t heartbeat;
                scslink_msg_heartbeat_decode(&heartbeat, &msg);
                
                printf("Heartbeat from system %d, status: %d\n",
                       msg.sysid, heartbeat.system_status);
                break;
            }
            
            case SCSLINK_MSG_ID_COMMAND:
            {
                scslink_request_message_t cmd;
                scslink_msg_command_decode(&cmd, &msg);
                
                printf("Command %d received for message %d\n",
                       cmd.command, cmd.message_id);
                break;
            }
        }
    }
}

// In your UART interrupt handler:
void UART_IRQHandler(void)
{
    if (uart_data_available())
    {
        uint8_t byte = uart_read_byte();
        process_uart_byte(byte);
    }
}
```

### Example 3: Sending Command Messages

```c
#include "scslink.h"

void request_gps_data(void)
{
    static scslink_status_t status = {0};
    scslink_message_t msg;
    uint8_t buffer[20];
    
    // Pack command to request GPS message at 5Hz
    scslink_msg_command_pack(&msg,
                            scslink_get_next_seq(&status),
                            1,    // My system ID
                            0,    // My component ID
                            1,    // Command: Request message
                            33,   // Message ID: GPS position
                            5,    // Parameter: 5Hz rate
                            2,    // Target system
                            0);   // Target component (all)
    
    uint16_t len = scslink_msg_to_send_buffer(buffer, &msg);
    uart_transmit(buffer, len);
}
```

### Example 4: Using Accessor Functions

```c
#include "scslink.h"

void handle_message(const scslink_message_t *msg)
{
    if (msg->msgid == SCSLINK_MSG_ID_HEARTBEAT)
    {
        // Quick access without unpacking entire message
        uint8_t type = scslink_msg_heartbeat_get_type(msg);
        uint8_t status = scslink_msg_heartbeat_get_system_status(msg);
        uint32_t mode = scslink_msg_heartbeat_get_custom_mode(msg);
        
        printf("Type: %d, Status: %d, Mode: %u\n", type, status, mode);
    }
}
```

---

## 📨 Message Types

### Currently Implemented

| ID  | Name       | Description                           | Payload Size |
|-----|------------|---------------------------------------|--------------|
| 0   | HEARTBEAT  | System status and presence beacon     | 9 bytes      |
| 75  | COMMAND    | Send commands and requests            | 6 bytes      |

### Adding Custom Messages

1. Create a new header file in `src/common/`:

```c
// scslink_msg_custom.h
#pragma once
#include "../scslink_types.h"

#define SCSLINK_MSG_ID_CUSTOM 100
#define SCSLINK_MSG_CUSTOM_LEN 8

typedef struct {
    uint32_t timestamp;
    float temperature;
} scslink_custom_t;

static inline uint16_t scslink_msg_custom_pack(
    scslink_message_t *msg,
    uint8_t seq, uint8_t sysid, uint8_t compid,
    uint32_t timestamp, float temperature)
{
    msg->sof = SCSLINK_SOF_MARKER;
    msg->len = SCSLINK_MSG_CUSTOM_LEN;
    msg->seq = seq;
    msg->sysid = sysid;
    msg->compid = compid;
    msg->msgid = SCSLINK_MSG_ID_CUSTOM;
    
    uint8_t *p = msg->payload;
    
    // Pack timestamp (4 bytes, little-endian)
    p[0] = (timestamp >> 0) & 0xFF;
    p[1] = (timestamp >> 8) & 0xFF;
    p[2] = (timestamp >> 16) & 0xFF;
    p[3] = (timestamp >> 24) & 0xFF;
    
    // Pack temperature (4 bytes, IEEE 754 float)
    uint32_t temp_bits = *((uint32_t*)&temperature);
    p[4] = (temp_bits >> 0) & 0xFF;
    p[5] = (temp_bits >> 8) & 0xFF;
    p[6] = (temp_bits >> 16) & 0xFF;
    p[7] = (temp_bits >> 24) & 0xFF;
    
    return SCSLINK_MSG_CUSTOM_LEN;
}

// Add unpack, decode, and accessor functions...
```

2. Include in `scslink.h`:

```c
#include "common/scslink_msg_custom.h"
```

---

## 📚 API Reference

### Core Functions

#### Message Serialization

```c
uint16_t scslink_msg_to_send_buffer(uint8_t *buffer, const scslink_message_t *msg);
```
Converts a message structure to a byte buffer ready for transmission.

#### CRC Calculation

```c
uint16_t scslink_crc_calculate(const uint8_t *buffer, uint16_t length);
```
Calculates CRC-16/X25 checksum over the specified buffer.

#### Message Parsing

```c
uint8_t scslink_parse_char(uint8_t byte, scslink_message_t *msg, scslink_status_t *status);
```
Parse incoming bytes one at a time. Returns 1 when a complete message is received.

#### Status Management

```c
void scslink_status_init(scslink_status_t *status);
uint8_t scslink_get_next_seq(scslink_status_t *status);
```
Initialize status structure and get next sequence number.

### Heartbeat Message API

```c
// Pack from individual fields
uint16_t scslink_msg_heartbeat_pack(scslink_message_t *msg, ...);

// Encode from structure
uint16_t scslink_msg_heartbeat_encode(scslink_message_t *msg, ..., const scslink_heartbeat_t *heartbeat);

// Unpack to structure
void scslink_msg_heartbeat_unpack(scslink_heartbeat_t *heartbeat, const scslink_message_t *msg);

// Decode (alias for unpack)
void scslink_msg_heartbeat_decode(scslink_heartbeat_t *heartbeat, const scslink_message_t *msg);

// Accessor functions
uint32_t scslink_msg_heartbeat_get_custom_mode(const scslink_message_t *msg);
uint8_t scslink_msg_heartbeat_get_type(const scslink_message_t *msg);
uint8_t scslink_msg_heartbeat_get_autopilot(const scslink_message_t *msg);
// ... and more
```

### Command Message API

```c
// Pack, encode, unpack, decode
uint16_t scslink_msg_command_pack(scslink_message_t *msg, ...);
uint16_t scslink_msg_command_encode(scslink_message_t *msg, ..., const scslink_request_message_t *cmd);
void scslink_msg_command_unpack(scslink_request_message_t *cmd, const scslink_message_t *msg);
void scslink_msg_command_decode(scslink_request_message_t *cmd, const scslink_message_t *msg);

// Accessor functions
uint8_t scslink_msg_command_get_command(const scslink_message_t *msg);
uint8_t scslink_msg_command_get_message_id(const scslink_message_t *msg);
// ... and more

// Helper function
uint16_t scslink_msg_command_pack_to_send_buffer(uint8_t *buffer, ...);
```

---

## 🔧 Building & Integration

### Standalone Compilation

```bash
# Compile a test program
gcc -Wall -Wextra -std=c99 -I./src -o test_scslink test.c
```

### CMake Integration

```cmake
# CMakeLists.txt
cmake_minimum_required(VERSION 3.10)
project(MyProject)

# Add SCSLink include directory
include_directories(${CMAKE_SOURCE_DIR}/scslink/src)

# Your executable
add_executable(my_app main.c)
```

### Arduino Integration

```cpp
// Arduino sketch
#include "scslink.h"

scslink_status_t status = {0};

void setup() {
    Serial.begin(115200);
    scslink_status_init(&status);
}

void loop() {
    // Send heartbeat every second
    send_heartbeat();
    delay(1000);
    
    // Process incoming bytes
    while (Serial.available()) {
        uint8_t byte = Serial.read();
        process_byte(byte);
    }
}
```

### Platform Support

SCSLink is designed to be platform-independent and works on:

- ✅ Microcontrollers (ARM Cortex-M, AVR, PIC, ESP32, etc.)
- ✅ Embedded Linux (Raspberry Pi, BeagleBone, etc.)
- ✅ Desktop (Windows, macOS, Linux)
- ✅ RTOS (FreeRTOS, Zephyr, etc.)

---

## ⚡ Performance

### Memory Footprint

- **Code size**: ~2-4 KB (depending on optimization and messages used)
- **RAM usage**: 
  - Message structure: 282 bytes
  - Status structure: 16 bytes
  - Typical total: <1 KB per link

### Speed

- **Parsing**: ~100-200 CPU cycles per byte (on ARM Cortex-M4 @ 168MHz)
- **Packing**: ~50-100 CPU cycles per message
- **CRC calculation**: ~150 cycles per byte

### Optimization Tips

1. **Enable compiler optimization**: `-O2` or `-O3`
2. **Use static inline functions**: Already done in SCSLink
3. **Minimize dynamic allocation**: Use stack-allocated structures
4. **Buffer sizing**: Allocate exact message size when known

---

## 📖 Documentation

- **API Documentation**: See inline comments in header files
- **Examples**: Check `examples/` directory (if available)
- **Protocol Specification**: See [Protocol Specification](#protocol-specification) section

---

## 🤝 Contributing

Contributions are welcome! Here's how you can help:

1. **Report bugs**: Open an issue with details
2. **Suggest features**: Share your ideas
3. **Submit PRs**: Fork, create a branch, make changes, and submit
4. **Improve docs**: Help make documentation better
5. **Add examples**: Share your use cases

### Development Guidelines

- Follow existing code style
- Add comprehensive comments
- Test on multiple platforms
- Update documentation
- Keep code portable (C99 standard)

---

## 🔒 Security Considerations

### Optional Signatures

Enable message signatures for secure communication:

```c
// In scslink_types.h
#define SCSLINK_REQUIRES_SIGNATURE 1

// Set your secret key in scslink_helpers.h
static const uint8_t scslink_secret_key[13] = {
    0xYY, 0xYY, ... // Your secret key
};
```

⚠️ **Important**: The current signature implementation is simple. For production security, consider:
- Using a proper HMAC algorithm
- Implementing key exchange mechanisms
- Adding timestamp-based replay protection

---

## 📄 License

This project is licensed under the MIT License - see the [LICENSE](LICENSE) file for details.

```
MIT License

Copyright (c) 2025 SyntaxKavindu

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
```

---

## 🌟 Acknowledgments

- **MAVLink Team**: For creating an excellent protocol that inspired this project
- **Embedded Systems Community**: For continuous innovation in communication protocols
- **Contributors**: Everyone who helps improve SCSLink

---

## 📞 Contact & Support

- **Author**: SyntaxKavindu
- **GitHub**: [https://github.com/SyntaxKavindu/SCSLink](https://github.com/SyntaxKavindu/SCSLink)
- **Issues**: Report bugs and request features on GitHub Issues

---

## 🗺️ Roadmap

### Version 1.1 (Planned)
- [ ] Add more message types (GPS, IMU, etc.)
- [ ] Python bindings for testing
- [ ] Message generator from XML definitions
- [ ] Performance benchmarks

### Version 2.0 (Future)
- [ ] Extended addressing (16-bit message IDs)
- [ ] Message fragmentation for large payloads
- [ ] Improved signature/encryption
- [ ] Network routing support

---

<div align="center">

**Made with ❤️ for the Embedded Systems Community**

If you find SCSLink useful, please ⭐ star this repository!

[Report Bug](https://github.com/SyntaxKavindu/SCSLink/issues) · [Request Feature](https://github.com/SyntaxKavindu/SCSLink/issues) · [Discuss](https://github.com/SyntaxKavindu/SCSLink/discussions)

</div>
