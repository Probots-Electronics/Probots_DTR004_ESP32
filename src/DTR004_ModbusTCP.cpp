#include "DTR004_ModbusTCP.h"

// MBAP header layout (7 bytes):
//   [0-1] Transaction ID (big-endian)
//   [2-3] Protocol ID (always 0x0000)
//   [4-5] Remaining length = Unit ID (1) + PDU bytes (big-endian)
//   [6]   Unit ID
// PDU starts at byte 7.

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

DTR004_ModbusTCP::DTR004_ModbusTCP(const char* ip, uint16_t port, uint8_t unitId)
    : _ip(ip), _port(port), _unitId(unitId), _txId(0),
      _lastError(DTR004::Error::NONE), _lastExceptionCode(0)
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
                                               uint8_t* resp,       uint8_t& respLen) {
    if (!_ensureConnected()) {
        _lastError = DTR004::Error::UNREACHABLE;
        return _lastError;
    }

    // Build and send frame
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

    // Modbus exception: FC byte has bit-7 set, exception code is in byte 8.
    // The DT-R004 gateway sends exception 0x02 when the RS485 slave does not
    // respond (no device on bus). We map this to DEVICE_NOT_FOUND so callers
    // can distinguish "sensor missing" from real transport errors.
    // Critically: we do NOT stop the TCP socket here — an exception is a valid
    // response. The persistent TCP connection to the gateway stays open.
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

    // Pack individual coil values (1/0) into bytes, LSB first
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
// High-level relay / input API
// ---------------------------------------------------------------------------

DTR004::Error DTR004_ModbusTCP::setRelay(DTR004::Channel ch, DTR004::RelayState state) {
    if (ch < DTR004::Channel::CH1 || ch > DTR004::Channel::CH4) {
        return DTR004::Error::INVALID_CHANNEL;
    }
    uint16_t reg = DTR004_REG_RELAY_BASE + ((uint8_t)ch - 1);
    return writeSingleRegister(reg, (state == DTR004::RelayState::ON) ? 0x0001u : 0x0000u);
}

DTR004::Error DTR004_ModbusTCP::setAllRelays(uint8_t bitmask) {
    // Register 0x0002: high byte = update mask (which relays to change),
    //                  low byte  = new relay states (bit0=relay1, 1=ON)
    // For a 4-channel board update all four → mask = 0x0F
    uint16_t value = (uint16_t)(0x0Fu << 8) | (bitmask & 0x0Fu);
    return writeSingleRegister(DTR004_REG_WRITE_RELAY, value);
}

uint8_t DTR004_ModbusTCP::readRelayStates(DTR004::Error* err) {
    uint16_t reg = 0;
    int16_t r = readHoldingRegisters(DTR004_REG_RELAY_STATUS, 1, &reg);
    if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
    return (r < 0) ? 0 : (uint8_t)(reg & 0x0Fu);
}

uint8_t DTR004_ModbusTCP::readInputStates(DTR004::Error* err) {
    uint16_t reg = 0;
    int16_t r = readHoldingRegisters(DTR004_REG_INPUT_STATUS, 1, &reg);
    if (err) *err = (r < 0) ? _lastError : DTR004::Error::NONE;
    return (r < 0) ? 0 : (uint8_t)(reg & 0x0Fu);
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
// Diagnostics
// ---------------------------------------------------------------------------

const char* DTR004_ModbusTCP::errorToString(DTR004::Error err) {
    switch (err) {
        case DTR004::Error::NONE:             return "OK";
        case DTR004::Error::UNREACHABLE:      return "Gateway Unreachable";
        case DTR004::Error::TIMEOUT:          return "Gateway Timeout";
        case DTR004::Error::PARSE_FAILED:     return "Response Parse Error";
        case DTR004::Error::DEVICE_NOT_FOUND: return "RS485 Sensor Not Found on Bus";
        case DTR004::Error::MB_ILLEGAL_FUNC:  return "Modbus: Illegal Function (slave)";
        case DTR004::Error::MB_ILLEGAL_VALUE: return "Modbus: Illegal Data Value (slave)";
        case DTR004::Error::MB_SLAVE_FAILURE: return "Modbus: Slave Device Failure";
        case DTR004::Error::MB_EXCEPTION:     return "Modbus: Exception (see getLastExceptionCode)";
        default:                              return "Unknown Error";
    }
}
