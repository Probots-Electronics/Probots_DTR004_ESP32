# Probots\_DTR004\_ESP32 SDK

**Professional ESP32 driver for the DT-R004 4-Channel WiFi/Ethernet Relay Module**

[![Version](https://img.shields.io/badge/version-2.0.0-blue)](library.properties)
[![Platform](https://img.shields.io/badge/platform-ESP32-green)](https://www.espressif.com/en/products/socs/esp32)
[![Protocol](https://img.shields.io/badge/protocol-HTTP%20%7C%20Modbus%20TCP-orange)]()
[![License](https://img.shields.io/badge/license-MIT-brightgreen)](LICENSE)

---

## Why this SDK vs. the v1 library?

| Feature | v1 (DTR004-Library) | v2 (this SDK) |
|---|---|---|
| Relay control | `setRelay(int, int)` | `setRelay(Channel, RelayState)` typed enum |
| Input reads | Returns `String` | Returns `uint8_t` bitmask |
| Async / Non-blocking | ✗ — blocks `loop()` | ✓ — cooperative state machine |
| Modbus TCP | ✗ | ✓ — dedicated `DTR004_ModbusTCP` class |
| Error handling | Serial print only | `DTR004::Error` enum, per-instance counters |
| Network heartbeat | ✗ | ✓ — configurable interval + callback |
| Batch relay write | ✗ | ✓ — `setRelayBitmask()` / `setAllRelays()` |

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

### Synchronous API (drop-in upgrade from v1)

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

| Method | Description |
|---|---|
| `connect()` / `disconnect()` | Manage persistent TCP connection |
| `setRelay(Channel, RelayState)` | FC 0x05 single coil write |
| `setAllRelays(uint8_t bitmask)` | FC 0x0F atomic multi-coil write |
| `readRelayStates(Error*)` | FC 0x01 read 4 coils → bitmask |
| `readInputStates(Error*)` | FC 0x02 read 4 discrete inputs → bitmask |

#### Raw Modbus API

| Method | FC | Description |
|---|---|---|
| `writeCoil(addr, bool)` | 0x05 | Write single coil |
| `writeMultipleCoils(addr, n, vals[])` | 0x0F | Write n coils |
| `readCoils(addr, n, buf[])` | 0x01 | Read n coils |
| `readDiscreteInputs(addr, n, buf[])` | 0x02 | Read n discrete inputs |

---

### `DTR004::` Types

```cpp
enum class Channel   : uint8_t { CH1=1, CH2, CH3, CH4 };
enum class RelayState: uint8_t { OFF=0, ON=1 };
enum class Error     : uint8_t { NONE, UNREACHABLE, TIMEOUT,
                                  PARSE_FAILED, INVALID_CHANNEL,
                                  BUSY, WIFI_DISCONNECTED };

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

| Type | Address | Description |
|---|---|---|
| Coil | 0x0000 | Relay 1 |
| Coil | 0x0001 | Relay 2 |
| Coil | 0x0002 | Relay 3 |
| Coil | 0x0003 | Relay 4 |
| Discrete Input | 0x0000 | Digital Input 1 |
| Discrete Input | 0x0001 | Digital Input 2 |
| Discrete Input | 0x0002 | Digital Input 3 |
| Discrete Input | 0x0003 | Digital Input 4 |

---

## Migrating from v1 (DTR004-Library)

```cpp
// v1
DTR004 relay("192.168.7.1");
relay.setRelay(1, 1);
String s = relay.getInputs();

// v2 — equivalent
Probots_DTR004_ESP32 relay("192.168.7.1");
relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::ON);
uint8_t inputs = relay.getInputStates();
```

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
