# Probots\_DTR004\_ESP32

**ESP32 driver for Dingtian DT-R series WiFi/Ethernet relay modules**

[![Version](https://img.shields.io/badge/version-2.1.0-blue)](library.properties)
[![Platform](https://img.shields.io/badge/platform-ESP32-green)](https://www.espressif.com/en/products/socs/esp32)
[![Protocol](https://img.shields.io/badge/protocol-HTTP%20%7C%20Modbus%20TCP-orange)]()
[![License](https://img.shields.io/badge/license-MIT-brightgreen)](LICENSE)

---

## Buy from Probots Electronics

> **[probots.co.in](https://probots.co.in)** — India's trusted source for industrial IoT and automation modules.

This library works with **all DT-R series relay boards** available at Probots:

| Board | Relays | Digital Inputs | Buy |
|---|---|---|---|
| DT-R002 | 2 | 2 | [probots.co.in/dtr002](https://probots.co.in) |
| **DT-R004** ★ | **4** | **4** | [probots.co.in/dtr004](https://probots.co.in) |
| DT-R008 | 8 | 8 | [probots.co.in/dtr008](https://probots.co.in) |
| DT-R016 | 16 | 8 | [probots.co.in/dtr016](https://probots.co.in) |
| DT-R032 | 32 | 8 | [probots.co.in/dtr032](https://probots.co.in) |

★ = library originally designed for this board; all others fully supported.

---

The Dingtian DT-R series spans relay boards from 2 to 32 channels, all sharing a common HTTP/Modbus TCP control interface over a built-in WiFi or Ethernet module. This makes them a practical fit for a wide range of projects: switching mains-voltage loads in home automation rigs, controlling solenoid valves in irrigation or HVAC systems, driving actuators in factory-floor automation, managing lab equipment power sequencing, or building smart building controllers where multiple independent output channels are needed in a compact DIN-rail form factor. This documentation covers the full software interface — HTTP-based synchronous and asynchronous relay control, Modbus TCP for atomic multi-relay writes and raw register access, digital input reading, and network resiliency with heartbeat and offline callbacks. It does not cover the board's onboard scheduling and timer configuration (accessible through the DT-R web UI), RS485 slave commissioning, or MQTT integration — those remain configured through the board's own interface. The value this library delivers is a clean, idiomatic Arduino/ESP32 API that removes the need to hand-craft HTTP query strings or assemble raw Modbus frames, letting you control up to 32 relay channels from a few lines of C++ without touching the wire protocol.

This library offers dual-protocol control — both the DT-R's native HTTP interface and a full Modbus TCP implementation — so you can choose the transport that best suits your architecture. A cooperative, non-blocking async API keeps your main `loop()` responsive while commands are in-flight, making it suitable for time-sensitive sketches running alongside sensor polling or display updates. Type-safe `Channel` and `RelayState` enums catch wiring mistakes at compile time, while `uint32_t` bitmasks give you a compact, efficient handle on all 32 channels at once. Built-in heartbeat monitoring and connection-change callbacks make it straightforward to implement failsafe logic in production deployments. Rich diagnostics — per-instance success/failure counters and a structured `Error` enum — give you visibility into reliability at runtime without adding external logging overhead. The library is most useful to makers prototyping home automation or IoT projects, engineers integrating relay control into ESP32-based products, and industrial developers who need a tested, MIT-licensed driver they can ship in commercial firmware without restriction.

---

## Features

- Works with **all DT-R board variants** — 2 / 4 / 8 / 16 / 32 channel
- Type-safe relay control via `Channel` enum (`CH1` … `CH32`) and `RelayState`
- `uint32_t` bitmask I/O — handles up to 32 relay and digital input channels
- **Non-blocking async API** — cooperative state machine, `loop()` never stalls
- **Modbus TCP** layer — persistent connection, atomic multi-relay writes
- Network heartbeat with configurable interval and online/offline callback
- Rich error handling — `DTR004::Error` enum + per-instance success/failure counters
- Standard MIT license — use freely in commercial and open-source projects

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

### 1 — Tell the library your board's channel count

```cpp
// DT-R004 (4 ch) — default, no change needed
Probots_DTR004_ESP32 relay("192.168.7.1");

// DT-R008 (8 ch)
Probots_DTR004_ESP32 relay("192.168.7.1", 80, 8);

// DT-R016 (16 ch)
Probots_DTR004_ESP32 relay("192.168.7.1", 80, 16);
```

### 2 — Synchronous API

```cpp
#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

Probots_DTR004_ESP32 relay("192.168.7.1");  // DT-R004 default IP

void setup() {
    // ... WiFi.begin() ...

    relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::ON);

    // Bitmask write: bit 0 = CH1, bit 1 = CH2, …
    relay.setRelayBitmask(0b0101);  // CH1 and CH3 ON

    DTR004::Error err;
    uint32_t inputs = relay.getInputStates(&err);
    bool di1 = DTR004::isBitSet(inputs, 1);
}
```

### 3 — Async (Non-Blocking) API

```cpp
void onInputsDone(DTR004::InputEvent evt) {
    if (evt.err == DTR004::Error::NONE) {
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

### 4 — Modbus TCP Layer

```cpp
#include "DTR004_ModbusTCP.h"

// DT-R008 (8 ch) — unit ID 0xFF targets the board itself
DTR004_ModbusTCP modbus("192.168.7.1", 502, 0xFF, 8);

void setup() {
    modbus.connect();

    // Atomically set all relays in one round-trip (≤ 8 ch boards)
    modbus.setAllRelays(0b10101010);  // CH2/CH4/CH6/CH8 ON

    uint32_t inputs = modbus.readInputStates();
}
```

### 5 — Network Heartbeat

```cpp
void onConnChange(bool online) {
    if (!online) activateLocalFailsafe();
}

void setup() {
    relay.onConnectionChange(onConnChange);
    relay.enableHeartbeat(10000);  // probe every 10 s
}

void loop() {
    relay.update();  // drives async state machine + heartbeat
}
```

---

## API Reference

### `Probots_DTR004_ESP32`

#### Constructor
```cpp
Probots_DTR004_ESP32(const char* ip         = "192.168.1.100",
                      uint16_t   port       = 80,
                      uint8_t    relayCount = 4);
```
Set `relayCount` to match your board (2, 4, 8, 16, or 32).

#### Synchronous Methods

| Method | Returns | Description |
|---|---|---|
| `setRelay(Channel, RelayState)` | `Error` | Write one relay |
| `setRelayBitmask(uint32_t)` | `Error` | Write all relays from bitmask |
| `getInputStates(Error*)` | `uint32_t` | Read all digital inputs as bitmask |
| `getRelayStates(Error*)` | `uint32_t` | Read all relay states as bitmask |

#### Async Methods

| Method | Returns | Description |
|---|---|---|
| `setRelayAsync(Channel, RelayState, cb)` | `bool` | Initiate non-blocking relay write |
| `getInputsAsync(InputCallback)` | `bool` | Initiate non-blocking input read |
| `update()` | `void` | Advance state machine — **call every `loop()`** |
| `isBusy()` | `bool` | `true` while an async op is in-flight |

#### Resiliency

| Method | Description |
|---|---|
| `enableHeartbeat(uint32_t ms)` | Enable periodic reachability probes |
| `disableHeartbeat()` | Stop heartbeat |
| `onConnectionChange(callback)` | Register online/offline callback |
| `isReachable()` | Blocking TCP probe, returns `bool` |

#### Diagnostics
```cpp
relay.getSuccessCount();                    // uint32_t
relay.getFailureCount();                    // uint32_t
relay.getLastError();                       // DTR004::Error
Probots_DTR004_ESP32::errorToString(err);  // const char*
```

---

### `DTR004_ModbusTCP`

#### Constructor
```cpp
DTR004_ModbusTCP(const char* ip,
                  uint16_t port       = 502,
                  uint8_t  unitId     = 0xFF,   // 0xFF = DT-R board itself; 1–247 = RS485 slaves
                  uint8_t  relayCount = 4);
```

#### Relay / Input API

| Method | FC | Description |
|---|---|---|
| `connect()` / `disconnect()` | — | Manage persistent TCP connection |
| `setRelay(Channel, RelayState)` | 0x06 | Write one relay via reg `0x0036+N` |
| `setAllRelays(uint32_t bitmask)` | 0x06 | Atomic write (≤8 ch) or sequential (>8 ch) |
| `readRelayStates(Error*)` | 0x03 | Relay bitmask from reg `0x0001` |
| `readInputStates(Error*)` | 0x03 | Input bitmask from reg `0x000A` |

#### Register / Sensor API

| Method | FC | Description |
|---|---|---|
| `readHoldingRegisters(addr, n, regs[])` | 0x03 | Read n 16-bit holding registers |
| `readInputRegisters(addr, n, regs[])` | 0x04 | Read n 16-bit input registers (sensor values) |
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
// Channel covers all DT-R board sizes
enum class Channel : uint8_t { CH1=1, CH2, …, CH32, ALL=0xFF };

enum class RelayState : uint8_t { OFF=0, ON=1 };

enum class Error : uint8_t {
    NONE, UNREACHABLE, TIMEOUT, PARSE_FAILED, INVALID_CHANNEL,
    BUSY, WIFI_DISCONNECTED,
    // Modbus exception codes (DTR004_ModbusTCP only):
    DEVICE_NOT_FOUND,   // Exception 0x02/0x0B: RS485 slave not responding
    MB_ILLEGAL_FUNC,    // Exception 0x01: function code not supported
    MB_ILLEGAL_VALUE,   // Exception 0x03: value out of slave range
    MB_SLAVE_FAILURE,   // Exception 0x04: slave internal failure
    MB_EXCEPTION        // Any other Modbus exception (see getLastExceptionCode())
};

// Bitmask helpers — ch is 1-based (1 = CH1, 32 = CH32)
bool     isBitSet(uint32_t mask, uint8_t ch);
uint32_t setBit  (uint32_t mask, uint8_t ch);
uint32_t clearBit(uint32_t mask, uint8_t ch);

// Callback types
void (*RelayCallback)     (RelayEvent evt);   // .channel, .state, .err
void (*InputCallback)     (InputEvent  evt);  // .bitmask (uint32_t), .err
void (*ConnectionCallback)(bool connected);
```

---

## DT-R Register Map (Modbus TCP)

Unit ID **`0xFF`** addresses the DT-R board's own relays and inputs.
Unit IDs **1–247** are forwarded to the RS485 bus.

All DT-R registers use Holding Registers (FC 0x03 read / FC 0x06 write):

| Register | FC | Description |
|---|---|---|
| `0x0000` | 0x03 | Relay count — reads actual channel count of the board |
| `0x0001` | 0x03 | Relay status bitmask (bit 0 = Relay 1, up to 16 bits) |
| `0x0002` | 0x06 | Write all relays — high byte = update mask, low byte = new states (≤ 8 ch) |
| `0x000A` | 0x03 | Digital input bitmask (bit 0 = DI1, up to 16 bits) |
| `0x0016`–`0x0035` | 0x03 | Digital Input 1–32 individual read |
| `0x0036`–`0x0055` | 0x03/0x06 | Relay 1–32 individual R/W (0 = OFF, 1 = ON) |

---

## Examples

| Example | Covers |
|---|---|
| `01_BasicRelay` | Synchronous API, error checking, bitmask reads |
| `02_AsyncNonBlocking` | Cooperative state machine, callbacks, non-blocking loop |
| `03_ModbusTCP` | Modbus TCP layer, atomic writes, raw register access |
| `04_NetworkResiliency` | Heartbeat, connection callbacks, failsafe logic |

---

## Requirements

- **Board**: ESP32 (any variant)
- **Framework**: Arduino
- **Arduino core**: `arduino-esp32` ≥ 2.0

---

## License

MIT License — see [LICENSE](LICENSE) file.
Copyright © 2024 [Probots Electronics](https://probots.co.in)

---

## Support & Resources

| | |
|---|---|
| **Store** | [probots.co.in](https://probots.co.in) |
| **Email** | support@probots.co.in |
| **Bug reports** | [GitHub Issues](https://github.com/Probots-Electronics/Probots_DTR004_ESP32/issues) |

> **Need more relay channels?** Probots stocks the full DT-R range — from 2 to 32 channels — with WiFi and Ethernet variants.
> **[Browse relay modules at probots.co.in →](https://probots.co.in)**
