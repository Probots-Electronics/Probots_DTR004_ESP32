/*
 * DTR004_Types.h — Shared types for the Probots DT-R series ESP32 library.
 *
 * Compatible with all Dingtian DT-R relay boards:
 *   DT-R002 (2 ch)  DT-R004 (4 ch)  DT-R008 (8 ch)
 *   DT-R016 (16 ch) DT-R032 (32 ch)
 *
 * Product page : https://probots.co.in
 * Support      : support@probots.co.in
 * License      : MIT — see LICENSE file
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Probots Electronics (probots.co.in)
 */

#ifndef DTR004_TYPES_H
#define DTR004_TYPES_H

#include <Arduino.h>

namespace DTR004 {

// Channel numbers for all DT-R board variants (2 / 4 / 8 / 16 / 32 ch).
// The constructor relayCount parameter determines which channels are valid
// for any given board.
enum class Channel : uint8_t {
    CH1  = 1,  CH2  = 2,  CH3  = 3,  CH4  = 4,
    CH5  = 5,  CH6  = 6,  CH7  = 7,  CH8  = 8,
    CH9  = 9,  CH10 = 10, CH11 = 11, CH12 = 12,
    CH13 = 13, CH14 = 14, CH15 = 15, CH16 = 16,
    CH17 = 17, CH18 = 18, CH19 = 19, CH20 = 20,
    CH21 = 21, CH22 = 22, CH23 = 23, CH24 = 24,
    CH25 = 25, CH26 = 26, CH27 = 27, CH28 = 28,
    CH29 = 29, CH30 = 30, CH31 = 31, CH32 = 32,
    ALL  = 0xFF
};

enum class RelayState : uint8_t {
    OFF = 0,
    ON  = 1
};

enum class Error : uint8_t {
    // Generic transport errors
    NONE              = 0,
    UNREACHABLE       = 1,
    TIMEOUT           = 2,
    PARSE_FAILED      = 3,
    INVALID_CHANNEL   = 4,
    BUSY              = 5,
    WIFI_DISCONNECTED = 6,
    // Modbus RS485 gateway exception codes (DTR004_ModbusTCP)
    DEVICE_NOT_FOUND  = 7,   // Exception 0x02/0x0B: RS485 slave not responding
    MB_ILLEGAL_FUNC   = 8,   // Exception 0x01: function code not supported
    MB_ILLEGAL_VALUE  = 9,   // Exception 0x03: value out of slave range
    MB_SLAVE_FAILURE  = 10,  // Exception 0x04: slave internal failure
    MB_EXCEPTION      = 11   // Any other Modbus exception (see getLastExceptionCode())
};

enum class AsyncState : uint8_t {
    IDLE        = 0,
    CONNECTING  = 1,
    SENDING     = 2,
    RECEIVING   = 3,
    DONE        = 4,
    ERROR_STATE = 5
};

enum class OpType : uint8_t {
    NONE        = 0,
    SET_RELAY   = 1,
    GET_INPUTS  = 2,
    GET_RELAYS  = 3,
    HEARTBEAT   = 4
};

struct RelayEvent {
    uint8_t         channel;
    RelayState      state;
    Error           err;
};

// bitmask supports up to 32 channels (bit 0 = CH1, bit 31 = CH32)
struct InputEvent {
    uint32_t bitmask;
    Error    err;
};

typedef void (*RelayCallback)     (RelayEvent evt);
typedef void (*InputCallback)     (InputEvent evt);
typedef void (*ConnectionCallback)(bool connected);

// Bitmask helpers — ch is 1-based (1 = CH1, 32 = CH32)
inline bool     isBitSet(uint32_t mask, uint8_t ch) { return (mask >> (ch - 1)) & 0x01u; }
inline uint32_t setBit  (uint32_t mask, uint8_t ch) { return mask | (1UL << (ch - 1)); }
inline uint32_t clearBit(uint32_t mask, uint8_t ch) { return mask & ~(1UL << (ch - 1)); }

} // namespace DTR004

#endif // DTR004_TYPES_H
