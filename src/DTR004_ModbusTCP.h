#ifndef DTR004_MODBUSTCP_H
#define DTR004_MODBUSTCP_H

/*
 * DTR004_ModbusTCP
 *
 * Implements the Modbus TCP application layer (MBAP + PDU) over a persistent
 * WiFiClient connection to the DT-R004 module's Modbus TCP server (port 502).
 *
 * DT-R004 Modbus TCP register map (holding registers, verified from SDK manual):
 *   0x0001           : Relay status bitmask (FC 0x03, bit0=relay1)
 *   0x0002           : Write all relays (FC 0x06, high-byte=update mask, low-byte=new states)
 *   0x000A           : Input status bitmask (FC 0x03, bit0=input1)
 *   0x0036–0x0039    : Relay 1–4 individual R/W (FC 0x03/0x06, value 0=OFF 1=ON)
 *   0x0016–0x0019    : Input 1–4 individual read (FC 0x03, value 0=LOW 1=HIGH)
 *
 * IMPORTANT: Unit ID 0xFF addresses the DT-R004 itself.
 *            Unit IDs 1–247 are forwarded to the RS485 bus.
 *
 * Advantages over HTTP layer:
 *   - Persistent TCP connection: eliminates per-request handshake overhead
 *   - Atomic write-all via register 0x0002 (mask + new states in one FC 0x06)
 *   - Standard protocol — works with any Modbus master / SCADA system
 */

#include <Arduino.h>
#include <WiFi.h>
#include "DTR004_Types.h"

#define DTR004_MODBUS_PORT      502
// Unit ID 0xFF addresses the DT-R004's own relays/inputs over Modbus TCP.
// Unit IDs 1–247 are forwarded to the RS485 bus as Modbus RTU slave addresses.
#define DTR004_MODBUS_UNIT_ID   0xFF
#define DTR004_MODBUS_TIMEOUT   2000UL

// DT-R004 holding register map (verified from official SDK programming manual)
#define DTR004_REG_RELAY_COUNT  0x0000  // FC 0x03: relay count (2/4/8/16/32)
#define DTR004_REG_RELAY_STATUS 0x0001  // FC 0x03: relay bitmask, bit0=relay1
#define DTR004_REG_WRITE_RELAY  0x0002  // FC 0x06: high-byte=update mask, low-byte=new states
#define DTR004_REG_INPUT_STATUS 0x000A  // FC 0x03: input bitmask, bit0=input1 (inputs 1~16)
#define DTR004_REG_RELAY_BASE   0x0036  // FC 0x03/0x06: relay1; relay N = 0x0036+(N-1)
#define DTR004_REG_INPUT_BASE   0x0016  // FC 0x03: input1; input N = 0x0016+(N-1)

// Modbus function codes
#define MB_FC_READ_COILS        0x01   // RS485 pass-through only (not used by DT-R004 itself)
#define MB_FC_READ_DI           0x02   // RS485 pass-through only
#define MB_FC_READ_HOLDING_REGS 0x03   // DT-R004 relay/input registers + RS485 sensors
#define MB_FC_READ_INPUT_REGS   0x04   // RS485 sensors: live measurement values
#define MB_FC_WRITE_COIL        0x05   // RS485 pass-through only
#define MB_FC_WRITE_SINGLE_REG  0x06   // DT-R004 relay write + RS485 sensors
#define MB_FC_WRITE_MULTI_COILS 0x0F   // RS485 pass-through only

class DTR004_ModbusTCP {
public:
    /**
     * @param ip      DT-R004 IP address string
     * @param port    Modbus TCP port (default 502)
     * @param unitId  Modbus unit/slave ID (default 1)
     */
    explicit DTR004_ModbusTCP(const char* ip,
                               uint16_t port   = DTR004_MODBUS_PORT,
                               uint8_t  unitId = DTR004_MODBUS_UNIT_ID);

    ~DTR004_ModbusTCP() { disconnect(); }

    // Establish / tear down the persistent TCP connection.
    bool connect();
    void disconnect();
    bool isConnected() { return _client.connected(); }

    // -----------------------------------------------------------------------
    // High-level relay API (maps channels to coils)
    // -----------------------------------------------------------------------

    /** Write a single relay coil. */
    DTR004::Error setRelay(DTR004::Channel ch, DTR004::RelayState state);

    /**
     * Atomically write all four relay coils from a bitmask.
     * Bit 0 = Relay 1 … Bit 3 = Relay 4.
     * Uses FC 0x0F for a single network round-trip.
     */
    DTR004::Error setAllRelays(uint8_t bitmask);

    /**
     * Read all four relay coil states.
     * Returns bitmask (bit 0 = Relay 1 … bit 3 = Relay 4), 0 on error.
     */
    uint8_t readRelayStates(DTR004::Error* err = nullptr);

    /**
     * Read all four digital input states.
     * Returns bitmask (bit 0 = DI1 … bit 3 = DI4), 0 on error.
     */
    uint8_t readInputStates(DTR004::Error* err = nullptr);

    // -----------------------------------------------------------------------
    // RS485 Sensor / Register API
    // -----------------------------------------------------------------------

    /**
     * FC 0x03 – Read Holding Registers
     * Reads `count` 16-bit registers into regs[] (big-endian → host order done).
     * Returns number of registers read, -1 on error.
     * Use for: configuration registers, setpoints, device ID.
     */
    int16_t readHoldingRegisters(uint16_t startAddr, uint8_t count, uint16_t* regs);

    /**
     * FC 0x04 – Read Input Registers
     * Reads `count` 16-bit registers into regs[] (big-endian → host order done).
     * Returns number of registers read, -1 on error.
     * Use for: live measurement values (voltage, current, temperature…).
     */
    int16_t readInputRegisters(uint16_t startAddr, uint8_t count, uint16_t* regs);

    /**
     * FC 0x06 – Write Single Holding Register
     * Writes a 16-bit value to one register address.
     */
    DTR004::Error writeSingleRegister(uint16_t address, uint16_t value);

    // -----------------------------------------------------------------------
    // Raw Modbus coil / DI access
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

    DTR004::Error getLastError()         const { return _lastError; }

    /**
     * Raw Modbus exception code from the last failed transaction.
     * Only meaningful when getLastError() returns a MB_* or DEVICE_NOT_FOUND code.
     * Common codes: 0x01 Illegal Function, 0x02 Illegal Address / No Response,
     *               0x03 Illegal Value, 0x04 Slave Failure,
     *               0x0B Gateway Target Failed to Respond.
     */
    uint8_t getLastExceptionCode()       const { return _lastExceptionCode; }

    static const char* errorToString(DTR004::Error err);

private:
    const char* _ip;
    uint16_t    _port;
    uint8_t     _unitId;
    uint16_t    _txId;
    DTR004::Error _lastError;
    uint8_t       _lastExceptionCode;
    WiFiClient  _client;

    // Send PDU framed in MBAP header; receive response frame into resp[].
    // Returns Error::NONE and populates respLen on success.
    DTR004::Error _transaction(const uint8_t* pdu, uint8_t pduLen,
                                uint8_t* resp,       uint8_t& respLen);

    bool _ensureConnected();
};

#endif
