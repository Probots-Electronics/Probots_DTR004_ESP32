# Probots\_DTR004\_ESP32 SDK

**Professional ESP32 driver for the DT-R004 4-Channel WiFi/Ethernet Relay Module**

[![Version](https://img.shields.io/badge/version-2.0.0-blue)](library.properties)
[![Platform](https://img.shields.io/badge/platform-ESP32-green)](https://www.espressif.com/en/products/socs/esp32)
[![Protocol](https://img.shields.io/badge/protocol-HTTP%20%7C%20Modbus%20TCP-orange)]()
[![License](https://img.shields.io/badge/license-MIT-brightgreen)](LICENSE)

---

## Features

- Type-safe relay control via `Channel` and `RelayState` enums
- `uint8_t` bitmask I/O for all relay and digital input reads
- Non-blocking async API — cooperative state machine, `loop()` never stalls
- Modbus TCP protocol layer — persistent connection, atomic 4-relay writes
- Network heartbeat with configurable interval and online/offline callback
- Rich error handling — `DTR004::Error` enum + per-instance success/failure counters

---

## Installation

**Arduino IDE (Library Manager)**
Search for `Probots_DTR004_ESP32` and click Install.

**Manual ZIP install**
1. Download this repository as a `.zip`.
2. Arduino IDE → **Sketch → Include Library → Add .ZIP Library…**

**PlatformIO**
```ini
lib_deps = https://github.com/Probots-Electronics/Probots_DTR004_ESP32
```

---

## Quick Start

### Synchronous API

```cpp
#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

Probots_DTR004_ESP32 relay("192.168.7.1");

void setup() {
    // ... WiFi.begin() ...

    // Type-safe relay control
    relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::ON);

    // Bitmask input read: bit 0 = DI1, bit 1 = DI2, …
    DTR004::Error err;
    uint8_t inputs = relay.getInputStates(&err);
    bool di1 = DTR004::isBitSet(inputs, 1);
}
```

### Async (Non-Blocking) API

```cpp
void onInputsDone(DTR004::InputEvent evt) {
    if (evt.err == DTR004::Error::NONE) {
        // evt.bitmask: bit 0 = DI1, bit 1 = DI2, …
        bool di2 = DTR004::isBitSet(evt.bitmask, 2);
    }
}

void loop() {
    relay.update();   // <-- must be called every loop()

    if (!relay.isBusy()) {
        relay.getInputsAsync(onInputsDone);
    }
}
```

### Modbus TCP Layer

```cpp
#include "DTR004_ModbusTCP.h"

DTR004_ModbusTCP modbus("192.168.7.1"); // default port 502

void setup() {
    modbus.connect();

    // Atomically set all 4 relays in one network round-trip
    modbus.setAllRelays(0b1010);  // CH2 + CH4 ON

    // Read digital inputs as bitmask
    uint8_t inputs = modbus.readInputStates();
}
```

### Network Heartbeat

```cpp
void onConnChange(bool online) {
    if (!online) activateLocalFailsafe();
}

void setup() {
    relay.onConnectionChange(onConnChange);
    relay.enableHeartbeat(10000); // probe every 10 s
}

void loop() {
    relay.update(); // heartbeat runs inside here
}
```

---

## API Reference

### `Probots_DTR004_ESP32`

#### Constructor
```cpp
Probots_DTR004_ESP32(const char* ip = "192.168.1.100", uint16_t port = 80);
```

#### Synchronous Methods

| Method | Returns | Description |
|---|---|---|
| `setRelay(Channel, RelayState)` | `Error` | Write one relay |
| `setRelayBitmask(uint8_t)` | `Error` | Write all 4 relays from bitmask |
| `getInputStates(Error*)` | `uint8_t` | Read all 4 digital inputs as bitmask |
| `getRelayStates(Error*)` | `uint8_t` | Read all 4 relay states as bitmask |

#### Async Methods

| Method | Returns | Description |
|---|---|---|
| `setRelayAsync(Channel, RelayState, cb)` | `bool` | Initiate non-blocking relay write |
| `getInputsAsync(InputCallback)` | `bool` | Initiate non-blocking input read |
| `update()` | `void` | Advance state machine — call every `loop()` |
| `isBusy()` | `bool` | True while an async op is in-flight |

#### Resiliency

| Method | Description |
|---|---|
| `enableHeartbeat(uint32_t ms)` | Enable periodic reachability probes |
| `disableHeartbeat()` | Stop heartbeat |
| `onConnectionChange(callback)` | Register online/offline callback |
| `isReachable()` | Blocking TCP probe |

#### Diagnostics
```cpp
relay.getSuccessCount();   // uint32_t
relay.getFailureCount();   // uint32_t
relay.getLastError();      // DTR004::Error
Probots_DTR004_ESP32::errorToString(err);
```

---

### `DTR004_ModbusTCP`

#### Constructor
```cpp
DTR004_ModbusTCP(const char* ip, uint16_t port = 502, uint8_t unitId = 1);
```

#### Relay / Input API

| Method | FC | Description |
|---|---|---|
| `connect()` / `disconnect()` | — | Manage persistent TCP connection |
| `setRelay(Channel, RelayState)` | 0x06 | Write one relay (reg `0x0036+N`) |
| `setAllRelays(uint8_t bitmask)` | 0x06 | Atomic 4-relay write (reg `0x0002`) |
| `readRelayStates(Error*)` | 0x03 | Read relay bitmask (reg `0x0001`) |
| `readInputStates(Error*)` | 0x03 | Read input bitmask (reg `0x000A`) |

#### Register / Sensor API

| Method | FC | Description |
|---|---|---|
| `readHoldingRegisters(addr, n, regs[])` | 0x03 | Read n 16-bit holding registers |
| `readInputRegisters(addr, n, regs[])` | 0x04 | Read n 16-bit input registers (live sensor values) |
| `writeSingleRegister(addr, value)` | 0x06 | Write one 16-bit holding register |

#### Diagnostics

| Method | Description |
|---|---|
| `getLastError()` | Last `DTR004::Error` code |
| `getLastExceptionCode()` | Raw Modbus exception byte (when error is `MB_*` or `DEVICE_NOT_FOUND`) |
| `errorToString(err)` | Human-readable error string |

---

### `DTR004::` Types

```cpp
enum class Channel   : uint8_t { CH1=1, CH2, CH3, CH4 };
enum class RelayState: uint8_t { OFF=0, ON=1 };
enum class Error     : uint8_t {
    // Transport errors
    NONE, UNREACHABLE, TIMEOUT, PARSE_FAILED, INVALID_CHANNEL,
    BUSY, WIFI_DISCONNECTED,
    // Modbus exception codes (DTR004_ModbusTCP only)
    DEVICE_NOT_FOUND,  // Exception 0x02/0x0B: RS485 slave not responding
    MB_ILLEGAL_FUNC,   // Exception 0x01: function code not supported
    MB_ILLEGAL_VALUE,  // Exception 0x03: value out of slave range
    MB_SLAVE_FAILURE,  // Exception 0x04: slave internal failure
    MB_EXCEPTION       // Any other Modbus exception (see getLastExceptionCode())
};

// Bitmask helpers
bool    isBitSet(uint8_t mask, uint8_t ch);  // ch = 1..4
uint8_t setBit  (uint8_t mask, uint8_t ch);
uint8_t clearBit(uint8_t mask, uint8_t ch);

// Callback signatures
void (*RelayCallback)   (RelayEvent evt);    // evt.channel, .state, .err
void (*InputCallback)   (InputEvent  evt);   // evt.bitmask, .err
void (*ConnectionCallback)(bool connected);
```

---

## DT-R004 Register Map (Modbus TCP)

Unit ID **0xFF** addresses the DT-R004's own relays and inputs.  
Unit IDs **1–247** are forwarded to the RS485 bus.

All DT-R004 registers use **Holding Registers** (FC 0x03 read / FC 0x06 write):

| Register | FC | Description |
|---|---|---|
| `0x0000` | 0x03 | Relay count (2 / 4 / 8 / 16 / 32) |
| `0x0001` | 0x03 | Relay status bitmask (bit 0 = Relay 1) |
| `0x0002` | 0x06 | Write all relays — high byte = update mask, low byte = new states |
| `0x0016` | 0x03 | Digital Input 1 state (0 = LOW, 1 = HIGH) |
| `0x0017` | 0x03 | Digital Input 2 state |
| `0x0018` | 0x03 | Digital Input 3 state |
| `0x0019` | 0x03 | Digital Input 4 state |
| `0x000A` | 0x03 | Input status bitmask (bit 0 = DI1) |
| `0x0036` | 0x03/0x06 | Relay 1 individual R/W (0 = OFF, 1 = ON) |
| `0x0037` | 0x03/0x06 | Relay 2 individual R/W |
| `0x0038` | 0x03/0x06 | Relay 3 individual R/W |
| `0x0039` | 0x03/0x06 | Relay 4 individual R/W |

---

## Examples

| Example | Covers |
|---|---|
| `01_BasicRelay` | Synchronous API, error codes, bitmask reads |
| `02_AsyncNonBlocking` | Cooperative state machine, callbacks |
| `03_ModbusTCP` | Modbus TCP layer, atomic writes, raw access |
| `04_NetworkResiliency` | Heartbeat, connection callbacks, failsafe logic |

---

## Requirements

- **Board**: ESP32 (any variant)
- **Framework**: Arduino
- **Arduino core**: `arduino-esp32` ≥ 2.0

---

*Built by [Probots Electronics](https://probots.co.in) — industrial IoT for everyone.*
