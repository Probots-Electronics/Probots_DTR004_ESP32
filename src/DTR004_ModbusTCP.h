/*
 * DTR004_ModbusTCP — Modbus TCP client for Dingtian DT-R series relay modules.
 *
 * Implements the Modbus TCP application layer (MBAP + PDU) over a persistent
 * WiFiClient connection to the DT-R module's Modbus TCP server (port 502).
 *
 * Compatible with all DT-R board variants sold at probots.co.in:
 *   DT-R002 (2 ch)  DT-R004 (4 ch)  DT-R008 (8 ch)
 *   DT-R016 (16 ch) DT-R032 (32 ch)
 *
 * DT-R Modbus TCP register map (holding registers, FC 0x03 read / FC 0x06 write):
 *   0x0000           : Relay count register (read the board's actual channel count)
 *   0x0001           : Relay status bitmask (bit0=relay1, up to 16 relays per register)
 *   0x0002           : Write all relays (high-byte=update mask, low-byte=new states; ≤8 ch)
 *   0x000A           : Input status bitmask (bit0=input1)
 *   0x0016–0x0035    : Input 1–32 individual read (FC 0x03)
 *   0x0036–0x0055    : Relay 1–32 individual R/W (FC 0x03/0x06, 0=OFF 1=ON)
 *
 * IMPORTANT: Unit ID 0xFF addresses the DT-R module itself.
 *            Unit IDs 1–247 are forwarded to the RS485 bus.
 *
 * Advantages over HTTP layer:
 *   - Persistent TCP connection: no per-request handshake overhead
 *   - Atomic write-all via register 0x0002 (≤ 8 ch) or sequential individual writes
 *   - Standard Modbus TCP — works with any SCADA / PLC / HMI system
 *
 * Product page : https://probots.co.in
 * Support      : support@probots.co.in
 * License      : MIT — see LICENSE file
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Probots Electronics (probots.co.in)
 */

#ifndef DTR004_MODBUSTCP_H
#define DTR004_MODBUSTCP_H

#include <Arduino.h>
#include <WiFi.h>
#include "DTR004_Types.h"

#define DTR004_MODBUS_PORT      502
// Unit ID 0xFF addresses the DT-R module's own relays/inputs.
// Unit IDs 1–247 are forwarded to the RS485 bus as Modbus RTU slave addresses.
#define DTR004_MODBUS_UNIT_ID   0xFF
#define DTR004_MODBUS_TIMEOUT   2000UL

// DT-R holding register map
#define DTR004_REG_RELAY_COUNT  0x0000  // FC 0x03: relay count (2/4/8/16/32)
#define DTR004_REG_RELAY_STATUS 0x0001  // FC 0x03: relay bitmask, bit0=relay1
#define DTR004_REG_WRITE_RELAY  0x0002  // FC 0x06: high-byte=update mask, low-byte=new states (≤8 ch)
#define DTR004_REG_INPUT_STATUS 0x000A  // FC 0x03: input bitmask, bit0=input1
#define DTR004_REG_INPUT_BASE   0x0016  // FC 0x03: input1; input N = 0x0016+(N-1)
#define DTR004_REG_RELAY_BASE   0x0036  // FC 0x03/0x06: relay1; relay N = 0x0036+(N-1)

// Modbus function codes
#define MB_FC_READ_COILS        0x01
#define MB_FC_READ_DI           0x02
#define MB_FC_READ_HOLDING_REGS 0x03
#define MB_FC_READ_INPUT_REGS   0x04
#define MB_FC_WRITE_COIL        0x05
#define MB_FC_WRITE_SINGLE_REG  0x06
#define MB_FC_WRITE_MULTI_COILS 0x0F

class DTR004_ModbusTCP {
public:
    /**
     * @param ip         DT-R module IP address
     * @param port       Modbus TCP port (default 502)
     * @param unitId     Modbus unit ID — 0xFF for DT-R module itself; 1–247 for RS485 slaves
     * @param relayCount Number of relay channels on your board (2 / 4 / 8 / 16 / 32).
     *                   Controls channel validation and bitmask width.
     */
    explicit DTR004_ModbusTCP(const char* ip,
                               uint16_t port       = DTR004_MODBUS_PORT,
                               uint8_t  unitId     = DTR004_MODBUS_UNIT_ID,
                               uint8_t  relayCount = 4);

    ~DTR004_ModbusTCP() { disconnect(); }

    // Establish / tear down the persistent TCP connection.
    bool connect();
    void disconnect();
    bool isConnected() { return _client.connected(); }

    // -----------------------------------------------------------------------
    // High-level relay API
    // -----------------------------------------------------------------------

    /** Write a single relay. Channel must be within the board's relay count. */
    DTR004::Error setRelay(DTR004::Channel ch, DTR004::RelayState state);

    /**
     * Atomically write all relay channels from a bitmask (bit 0 = Relay 1).
     * For boards with ≤ 8 channels: single round-trip via register 0x0002.
     * For boards with > 8 channels: sequential individual writes via 0x0036+N.
     */
    DTR004::Error setAllRelays(uint32_t bitmask);

    /**
     * Read all relay states as a bitmask (bit 0 = Relay 1), 0 on error.
     * Reads register 0x0001 for boards with ≤ 16 channels.
     */
    uint32_t readRelayStates(DTR004::Error* err = nullptr);

    /**
     * Read all digital input states as a bitmask (bit 0 = DI1), 0 on error.
     * Reads register 0x000A for boards with ≤ 16 channels.
     */
    uint32_t readInputStates(DTR004::Error* err = nullptr);

    // -----------------------------------------------------------------------
    // RS485 Sensor / Raw Register API
    // -----------------------------------------------------------------------

    /**
     * FC 0x03 – Read Holding Registers.
     * Reads `count` 16-bit registers into regs[] (big-endian converted to host).
     * Returns number of registers read, -1 on error.
     */
    int16_t readHoldingRegisters(uint16_t startAddr, uint8_t count, uint16_t* regs);

    /**
     * FC 0x04 – Read Input Registers.
     * Returns number of registers read, -1 on error.
     * Use for live sensor values (voltage, current, temperature …).
     */
    int16_t readInputRegisters(uint16_t startAddr, uint8_t count, uint16_t* regs);

    /**
     * FC 0x06 – Write Single Holding Register.
     */
    DTR004::Error writeSingleRegister(uint16_t address, uint16_t value);

    // -----------------------------------------------------------------------
    // Raw coil / DI access (RS485 pass-through)
    // -----------------------------------------------------------------------

    /** FC 0x05 – Write Single Coil */
    DTR004::Error writeCoil(uint16_t address, bool value);

    /** FC 0x0F – Write Multiple Coils (packed LSB-first in values[]) */
    DTR004::Error writeMultipleCoils(uint16_t startAddr, uint8_t count, const uint8_t* values);

    /** FC 0x01 – Read Coils. Returns bytes placed in buf, -1 on error. */
    int16_t readCoils(uint16_t startAddr, uint8_t count, uint8_t* buf);

    /** FC 0x02 – Read Discrete Inputs. Returns bytes placed in buf, -1 on error. */
    int16_t readDiscreteInputs(uint16_t startAddr, uint8_t count, uint8_t* buf);

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------

    DTR004::Error getLastError()        const { return _lastError; }

    /**
     * Raw Modbus exception code from the last failed transaction.
     * Meaningful when getLastError() returns a MB_* or DEVICE_NOT_FOUND code.
     * Common codes: 0x01 Illegal Function, 0x02 Illegal Address / No Response,
     *               0x03 Illegal Value,    0x04 Slave Failure,
     *               0x0B Gateway Target Device Failed to Respond.
     */
    uint8_t getLastExceptionCode()      const { return _lastExceptionCode; }

    static const char* errorToString(DTR004::Error err);

private:
    const char*   _ip;
    uint16_t      _port;
    uint8_t       _unitId;
    uint8_t       _relayCount;
    uint16_t      _txId;
    DTR004::Error _lastError;
    uint8_t       _lastExceptionCode;
    WiFiClient    _client;

    DTR004::Error _transaction(const uint8_t* pdu, uint8_t pduLen,
                                uint8_t* resp,      uint8_t& respLen);

    bool _ensureConnected();
};

#endif // DTR004_MODBUSTCP_H
