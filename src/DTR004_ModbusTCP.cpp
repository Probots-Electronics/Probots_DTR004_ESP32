/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Probots Electronics (probots.co.in)
 */

#include "DTR004_ModbusTCP.h"

// MBAP header layout (7 bytes):
//   [0-1] Transaction ID (big-endian, echoed by server)
//   [2-3] Protocol ID (always 0x0000)
//   [4-5] Remaining length = Unit ID (1 byte) + PDU bytes
//   [6]   Unit ID
// PDU starts at byte 7 of the response.

static void buildMBAP(uint8_t* frame, uint16_t txId, uint8_t unitId, uint8_t pduLen) {
    frame[0] = (txId >> 8) & 0xFF;
    frame[1] =  txId       & 0xFF;
    frame[2] = 0x00;
    frame[3] = 0x00;
    uint16_t len = 1u + pduLen;
    frame[4] = (len >> 8) & 0xFF;
    frame[5] =  len       & 0xFF;
    frame[6] = unitId;
}

// ---------------------------------------------------------------------------

DTR004_ModbusTCP::DTR004_ModbusTCP(const char* ip, uint16_t port,
                                    uint8_t unitId, uint8_t relayCount)
    : _ip(ip), _port(port), _unitId(unitId), _relayCount(relayCount),
      _txId(0), _lastError(DTR004::Error::NONE), _lastExceptionCode(0)
{}

bool DTR004_ModbusTCP::connect() {
    if (_client.connected()) return true;
    return _client.connect(_ip, _port);
}

void DTR004_ModbusTCP::disconnect() {
    _client.stop();
}

bool DTR004_ModbusTCP::_ensureConnected() {
    if (_client.connected()) return true;
    return _client.connect(_ip, _port);
}

// ---------------------------------------------------------------------------
// Core transaction engine
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::_transaction(const uint8_t* pdu, uint8_t pduLen,
                                               uint8_t* resp,      uint8_t& respLen) {
    if (!_ensureConnected()) {
        _lastError = DTR004::Error::UNREACHABLE;
        return _lastError;
    }

    uint8_t frame[7 + 256];
    buildMBAP(frame, ++_txId, _unitId, pduLen);
    memcpy(frame + 7, pdu, pduLen);
    _client.write(frame, 7 + pduLen);
    _client.flush();

    unsigned long deadline = millis() + DTR004_MODBUS_TIMEOUT;
    while (_client.available() < 8 && millis() < deadline && _client.connected()) {
        delay(1);
    }

    if (_client.available() < 8) {
        _client.stop();
        _lastError = DTR004::Error::TIMEOUT;
        return _lastError;
    }

    respLen = 0;
    while (_client.available() && respLen < 255) {
        resp[respLen++] = (uint8_t)_client.read();
    }

    // Validate transaction ID echo
    uint16_t echTx = ((uint16_t)resp[0] << 8) | resp[1];
    if (echTx != _txId) {
        _lastError = DTR004::Error::PARSE_FAILED;
        return _lastError;
    }

    // Modbus exception: FC byte has bit-7 set, exception code is byte 8.
    // We do NOT close the TCP socket on an exception — the gateway responded
    // correctly; the error is at the RS485 slave layer.
    if (respLen >= 9 && (resp[7] & 0x80)) {
        _lastExceptionCode = resp[8];
        switch (_lastExceptionCode) {
            case 0x01: _lastError = DTR004::Error::MB_ILLEGAL_FUNC;  break;
            case 0x02: _lastError = DTR004::Error::DEVICE_NOT_FOUND; break;
            case 0x03: _lastError = DTR004::Error::MB_ILLEGAL_VALUE; break;
            case 0x04: _lastError = DTR004::Error::MB_SLAVE_FAILURE; break;
            case 0x0A:
            case 0x0B: _lastError = DTR004::Error::DEVICE_NOT_FOUND; break;
            default:   _lastError = DTR004::Error::MB_EXCEPTION;     break;
        }
        return _lastError;
    }

    _lastExceptionCode = 0;
    _lastError = DTR004::Error::NONE;
    return DTR004::Error::NONE;
}

// ---------------------------------------------------------------------------
// High-level relay / input API
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::setRelay(DTR004::Channel ch, DTR004::RelayState state) {
    uint8_t n = (uint8_t)ch;
    if (n < 1 || n > _relayCount) return DTR004::Error::INVALID_CHANNEL;
    uint16_t reg = DTR004_REG_RELAY_BASE + (n - 1);
    return writeSingleRegister(reg, (state == DTR004::RelayState::ON) ? 0x0001u : 0x0000u);
}

DTR004::Error DTR004_ModbusTCP::setAllRelays(uint32_t bitmask) {
    if (_relayCount <= 8) {
        // Single round-trip: register 0x0002 — high-byte = update mask, low-byte = new states.
        // Build mask for the exact number of channels on this board.
        uint8_t  chanMask = (uint8_t)((1u << _relayCount) - 1u);
        uint16_t value    = (uint16_t)(chanMask << 8) | (uint8_t)(bitmask & chanMask);
        return writeSingleRegister(DTR004_REG_WRITE_RELAY, value);
    }

    // For > 8 channels, write each relay individually via 0x0036+N.
    DTR004::Error worst = DTR004::Error::NONE;
    for (uint8_t i = 1; i <= _relayCount; i++) {
        DTR004::RelayState s = (bitmask & (1UL << (i - 1)))
                               ? DTR004::RelayState::ON
                               : DTR004::RelayState::OFF;
        DTR004::Error e = setRelay((DTR004::Channel)i, s);
        if (e != DTR004::Error::NONE) worst = e;
    }
    return worst;
}

uint32_t DTR004_ModbusTCP::readRelayStates(DTR004::Error* err) {
    if (_relayCount <= 16) {
        uint16_t reg = 0;
        int16_t r = readHoldingRegisters(DTR004_REG_RELAY_STATUS, 1, &reg);
        if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
        if (r < 0) return 0;
        uint32_t mask = (1UL << _relayCount) - 1UL;
        return (uint32_t)reg & mask;
    }

    // > 16 channels: read two consecutive registers
    uint16_t regs[2] = {0, 0};
    int16_t r = readHoldingRegisters(DTR004_REG_RELAY_STATUS, 2, regs);
    if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
    if (r < 0) return 0;
    uint32_t raw  = ((uint32_t)regs[0]) | ((uint32_t)regs[1] << 16);
    uint32_t mask = (1UL << _relayCount) - 1UL;
    return raw & mask;
}

uint32_t DTR004_ModbusTCP::readInputStates(DTR004::Error* err) {
    if (_relayCount <= 16) {
        uint16_t reg = 0;
        int16_t r = readHoldingRegisters(DTR004_REG_INPUT_STATUS, 1, &reg);
        if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
        if (r < 0) return 0;
        uint32_t mask = (1UL << _relayCount) - 1UL;
        return (uint32_t)reg & mask;
    }

    uint16_t regs[2] = {0, 0};
    int16_t r = readHoldingRegisters(DTR004_REG_INPUT_STATUS, 2, regs);
    if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
    if (r < 0) return 0;
    uint32_t raw  = ((uint32_t)regs[0]) | ((uint32_t)regs[1] << 16);
    uint32_t mask = (1UL << _relayCount) - 1UL;
    return raw & mask;
}

// ---------------------------------------------------------------------------
// FC 0x03 – Read Holding Registers
// ---------------------------------------------------------------------------

int16_t DTR004_ModbusTCP::readHoldingRegisters(uint16_t startAddr, uint8_t count,
                                                uint16_t* regs) {
    uint8_t pdu[5] = {
        MB_FC_READ_HOLDING_REGS,
        (uint8_t)((startAddr >> 8) & 0xFF),
        (uint8_t)( startAddr       & 0xFF),
        0x00, count
    };
    uint8_t resp[9 + count * 2]; uint8_t rLen;
    DTR004::Error e = _transaction(pdu, 5, resp, rLen);
    if (e != DTR004::Error::NONE) return -1;

    uint8_t byteCount = resp[8];
    for (uint8_t i = 0; i < byteCount / 2 && i < count; i++) {
        regs[i] = ((uint16_t)resp[9 + i * 2] << 8) | resp[10 + i * 2];
    }
    return byteCount / 2;
}

// ---------------------------------------------------------------------------
// FC 0x04 – Read Input Registers
// ---------------------------------------------------------------------------

int16_t DTR004_ModbusTCP::readInputRegisters(uint16_t startAddr, uint8_t count,
                                              uint16_t* regs) {
    uint8_t pdu[5] = {
        MB_FC_READ_INPUT_REGS,
        (uint8_t)((startAddr >> 8) & 0xFF),
        (uint8_t)( startAddr       & 0xFF),
        0x00, count
    };
    uint8_t resp[9 + count * 2]; uint8_t rLen;
    DTR004::Error e = _transaction(pdu, 5, resp, rLen);
    if (e != DTR004::Error::NONE) return -1;

    uint8_t byteCount = resp[8];
    for (uint8_t i = 0; i < byteCount / 2 && i < count; i++) {
        regs[i] = ((uint16_t)resp[9 + i * 2] << 8) | resp[10 + i * 2];
    }
    return byteCount / 2;
}

// ---------------------------------------------------------------------------
// FC 0x06 – Write Single Holding Register
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::writeSingleRegister(uint16_t address, uint16_t value) {
    uint8_t pdu[5] = {
        MB_FC_WRITE_SINGLE_REG,
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)( address       & 0xFF),
        (uint8_t)((value   >> 8) & 0xFF),
        (uint8_t)( value         & 0xFF)
    };
    uint8_t resp[16]; uint8_t rLen;
    return _transaction(pdu, 5, resp, rLen);
}

// ---------------------------------------------------------------------------
// FC 0x05 – Write Single Coil
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::writeCoil(uint16_t address, bool value) {
    uint8_t pdu[5] = {
        MB_FC_WRITE_COIL,
        (uint8_t)((address >> 8) & 0xFF),
        (uint8_t)( address       & 0xFF),
        (uint8_t)(value ? 0xFF : 0x00),
        0x00
    };
    uint8_t resp[16]; uint8_t rLen;
    return _transaction(pdu, 5, resp, rLen);
}

// ---------------------------------------------------------------------------
// FC 0x0F – Write Multiple Coils
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::writeMultipleCoils(uint16_t startAddr,
                                                    uint8_t  count,
                                                    const uint8_t* values) {
    uint8_t byteCount = (count + 7) / 8;
    uint8_t pdu[6 + byteCount];

    pdu[0] = MB_FC_WRITE_MULTI_COILS;
    pdu[1] = (startAddr >> 8) & 0xFF;
    pdu[2] =  startAddr       & 0xFF;
    pdu[3] = 0x00;
    pdu[4] = count;
    pdu[5] = byteCount;

    memset(pdu + 6, 0, byteCount);
    for (uint8_t i = 0; i < count; i++) {
        if (values[i]) pdu[6 + i / 8] |= (1 << (i % 8));
    }

    uint8_t resp[16]; uint8_t rLen;
    return _transaction(pdu, 6 + byteCount, resp, rLen);
}

// ---------------------------------------------------------------------------
// FC 0x01 – Read Coils
// ---------------------------------------------------------------------------

int16_t DTR004_ModbusTCP::readCoils(uint16_t startAddr, uint8_t count, uint8_t* buf) {
    uint8_t pdu[5] = {
        MB_FC_READ_COILS,
        (uint8_t)((startAddr >> 8) & 0xFF),
        (uint8_t)( startAddr       & 0xFF),
        0x00, count
    };
    uint8_t resp[32]; uint8_t rLen;
    DTR004::Error e = _transaction(pdu, 5, resp, rLen);
    if (e != DTR004::Error::NONE) return -1;

    uint8_t byteCount = resp[8];
    memcpy(buf, resp + 9, byteCount);
    return byteCount;
}

// ---------------------------------------------------------------------------
// FC 0x02 – Read Discrete Inputs
// ---------------------------------------------------------------------------

int16_t DTR004_ModbusTCP::readDiscreteInputs(uint16_t startAddr, uint8_t count, uint8_t* buf) {
    uint8_t pdu[5] = {
        MB_FC_READ_DI,
        (uint8_t)((startAddr >> 8) & 0xFF),
        (uint8_t)( startAddr       & 0xFF),
        0x00, count
    };
    uint8_t resp[32]; uint8_t rLen;
    DTR004::Error e = _transaction(pdu, 5, resp, rLen);
    if (e != DTR004::Error::NONE) return -1;

    uint8_t byteCount = resp[8];
    memcpy(buf, resp + 9, byteCount);
    return byteCount;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

const char* DTR004_ModbusTCP::errorToString(DTR004::Error err) {
    switch (err) {
        case DTR004::Error::NONE:              return "OK";
        case DTR004::Error::UNREACHABLE:       return "Gateway Unreachable";
        case DTR004::Error::TIMEOUT:           return "Gateway Timeout";
        case DTR004::Error::PARSE_FAILED:      return "Response Parse Error";
        case DTR004::Error::INVALID_CHANNEL:   return "Invalid Channel";
        case DTR004::Error::BUSY:              return "SDK Busy";
        case DTR004::Error::WIFI_DISCONNECTED: return "WiFi Disconnected";
        case DTR004::Error::DEVICE_NOT_FOUND:  return "RS485 Sensor Not Found on Bus";
        case DTR004::Error::MB_ILLEGAL_FUNC:   return "Modbus: Illegal Function (slave)";
        case DTR004::Error::MB_ILLEGAL_VALUE:  return "Modbus: Illegal Data Value (slave)";
        case DTR004::Error::MB_SLAVE_FAILURE:  return "Modbus: Slave Device Failure";
        case DTR004::Error::MB_EXCEPTION:      return "Modbus: Exception (see getLastExceptionCode)";
        default:                               return "Unknown Error";
    }
}
