#ifndef DTR004_TYPES_H
#define DTR004_TYPES_H

#include <Arduino.h>

namespace DTR004 {

enum class Channel : uint8_t {
    CH1 = 1,
    CH2 = 2,
    CH3 = 3,
    CH4 = 4,
    ALL = 0xFF
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
    // Modbus RS485 gateway exception codes (from DTR004_ModbusTCP)
    DEVICE_NOT_FOUND  = 7,   // Exception 0x02 / 0x0B: RS485 slave not responding
    MB_ILLEGAL_FUNC   = 8,   // Exception 0x01: function code not supported by slave
    MB_ILLEGAL_VALUE  = 9,   // Exception 0x03: data value out of slave's range
    MB_SLAVE_FAILURE  = 10,  // Exception 0x04: slave device internal failure
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

struct InputEvent {
    uint8_t bitmask;
    Error   err;
};

typedef void (*RelayCallback)(RelayEvent evt);
typedef void (*InputCallback)(InputEvent evt);
typedef void (*ConnectionCallback)(bool connected);

inline bool isBitSet(uint8_t mask, uint8_t ch) { return (mask >> (ch - 1)) & 0x01; }
inline uint8_t setBit(uint8_t mask, uint8_t ch)   { return mask | (1 << (ch - 1)); }
inline uint8_t clearBit(uint8_t mask, uint8_t ch) { return mask & ~(1 << (ch - 1)); }

} // namespace DTR004

#endif
