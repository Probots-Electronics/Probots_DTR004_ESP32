/*
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Probots Electronics (probots.co.in)
 */

#include "Probots_DTR004_ESP32.h"

// ---------------------------------------------------------------------------
// Construction
// ---------------------------------------------------------------------------

Probots_DTR004_ESP32::Probots_DTR004_ESP32(const char* ip, uint16_t port, uint8_t relayCount)
    : _ip(ip), _port(port), _relayCount(relayCount),
      _asyncState(DTR004::AsyncState::IDLE),
      _pendingOp(DTR004::OpType::NONE),
      _asyncDeadline(0), _lastByteMs(0),
      _pendingCh(0),
      _pendingRelayState(DTR004::RelayState::OFF),
      _relayCb(nullptr), _inputCb(nullptr),
      _heartbeatEnabled(false),
      _heartbeatInterval(DTR004_HEARTBEAT_MS),
      _lastHeartbeat(0), _lastConnState(false),
      _connCb(nullptr),
      _successCount(0), _failureCount(0),
      _lastError(DTR004::Error::NONE)
{}

// ---------------------------------------------------------------------------
// Synchronous API
// ---------------------------------------------------------------------------

DTR004::Error Probots_DTR004_ESP32::setRelay(DTR004::Channel ch, DTR004::RelayState state) {
    uint8_t n = (uint8_t)ch;
    if (n < 1 || n > _relayCount) return DTR004::Error::INVALID_CHANNEL;
    String body;
    return _httpGet(_relayPath(n, (uint8_t)state), body);
}

DTR004::Error Probots_DTR004_ESP32::setRelayBitmask(uint32_t bitmask) {
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

uint32_t Probots_DTR004_ESP32::getInputStates(DTR004::Error* err) {
    String body;
    DTR004::Error e = _httpGet("/input.cgi", body);
    if (err) *err = e;
    return (e == DTR004::Error::NONE) ? _parseBitmask(body, _relayCount) : 0;
}

uint32_t Probots_DTR004_ESP32::getRelayStates(DTR004::Error* err) {
    String body;
    DTR004::Error e = _httpGet("/status.cgi", body);
    if (err) *err = e;
    return (e == DTR004::Error::NONE) ? _parseBitmask(body, _relayCount) : 0;
}

// ---------------------------------------------------------------------------
// Async API
// ---------------------------------------------------------------------------

bool Probots_DTR004_ESP32::setRelayAsync(DTR004::Channel ch, DTR004::RelayState state,
                                          DTR004::RelayCallback cb) {
    if (_asyncState != DTR004::AsyncState::IDLE) return false;
    uint8_t n = (uint8_t)ch;
    if (n < 1 || n > _relayCount) {
        if (cb) cb({n, state, DTR004::Error::INVALID_CHANNEL});
        return false;
    }
    _pendingOp         = DTR004::OpType::SET_RELAY;
    _pendingCh         = n;
    _pendingRelayState = state;
    _pendingPath       = _relayPath(n, (uint8_t)state);
    _relayCb           = cb;
    _rxBuf             = "";
    _asyncDeadline     = millis() + DTR004_TIMEOUT_MS;
    _asyncState        = DTR004::AsyncState::CONNECTING;
    return true;
}

bool Probots_DTR004_ESP32::getInputsAsync(DTR004::InputCallback cb) {
    if (_asyncState != DTR004::AsyncState::IDLE) return false;
    _pendingOp     = DTR004::OpType::GET_INPUTS;
    _pendingPath   = "/input.cgi";
    _inputCb       = cb;
    _rxBuf         = "";
    _asyncDeadline = millis() + DTR004_TIMEOUT_MS;
    _asyncState    = DTR004::AsyncState::CONNECTING;
    return true;
}

void Probots_DTR004_ESP32::update() {
    _smStep();
    if (_heartbeatEnabled) _tickHeartbeat();
}

// ---------------------------------------------------------------------------
// Internal state machine
// ---------------------------------------------------------------------------

void Probots_DTR004_ESP32::_smStep() {
    if (_asyncState == DTR004::AsyncState::IDLE) return;

    if (millis() > _asyncDeadline) {
        _client.stop();
        _finishAsync(DTR004::Error::TIMEOUT);
        return;
    }

    switch (_asyncState) {

        case DTR004::AsyncState::CONNECTING:
            if (WiFi.status() != WL_CONNECTED) {
                _finishAsync(DTR004::Error::WIFI_DISCONNECTED);
                return;
            }
            if (_client.connect(_ip, _port)) {
                _client.print(String("GET ") + _pendingPath + " HTTP/1.0\r\n"
                              "Host: " + _ip + "\r\n\r\n");
                _client.flush();
                _lastByteMs = millis();
                _asyncState = DTR004::AsyncState::RECEIVING;
            } else {
                _finishAsync(DTR004::Error::UNREACHABLE);
            }
            break;

        case DTR004::AsyncState::RECEIVING:
            while (_client.available()) {
                _rxBuf += (char)_client.read();
                _lastByteMs = millis();
            }
            // Declare done when server closes connection OR no new data for 300 ms
            // after at least one byte — the DT-R HTTP server often holds TCP open.
            {
                bool serverClosed  = !_client.connected();
                bool idleAfterData = (_rxBuf.length() > 0) &&
                                     (millis() - _lastByteMs > 300);
                if (serverClosed || idleAfterData) {
                    _client.stop();

                    int bodyStart = _rxBuf.indexOf("\r\n\r\n");
                    if (bodyStart == -1) {
                        _finishAsync(DTR004::Error::PARSE_FAILED);
                        return;
                    }
                    String body = _rxBuf.substring(bodyStart + 4);
                    body.trim();

                    if (_pendingOp == DTR004::OpType::GET_INPUTS && _inputCb) {
                        _inputCb({_parseBitmask(body, _relayCount), DTR004::Error::NONE});
                    } else if (_pendingOp == DTR004::OpType::SET_RELAY && _relayCb) {
                        _relayCb({_pendingCh, _pendingRelayState, DTR004::Error::NONE});
                    }
                    _successCount++;
                    _lastError  = DTR004::Error::NONE;
                    _asyncState = DTR004::AsyncState::IDLE;
                    _pendingOp  = DTR004::OpType::NONE;
                }
            }
            break;

        default:
            _asyncState = DTR004::AsyncState::IDLE;
            break;
    }
}

void Probots_DTR004_ESP32::_finishAsync(DTR004::Error err) {
    _lastError = err;
    _failureCount++;
    if (_pendingOp == DTR004::OpType::GET_INPUTS && _inputCb) {
        _inputCb({0, err});
    } else if (_pendingOp == DTR004::OpType::SET_RELAY && _relayCb) {
        _relayCb({_pendingCh, _pendingRelayState, err});
    }
    _asyncState = DTR004::AsyncState::IDLE;
    _pendingOp  = DTR004::OpType::NONE;
}

// ---------------------------------------------------------------------------
// Network Resiliency
// ---------------------------------------------------------------------------

void Probots_DTR004_ESP32::enableHeartbeat(uint32_t intervalMs) {
    _heartbeatEnabled  = true;
    _heartbeatInterval = intervalMs;
    _lastHeartbeat     = millis();
}

void Probots_DTR004_ESP32::disableHeartbeat() {
    _heartbeatEnabled = false;
}

void Probots_DTR004_ESP32::onConnectionChange(DTR004::ConnectionCallback cb) {
    _connCb = cb;
}

bool Probots_DTR004_ESP32::isReachable() {
    WiFiClient probe;
    probe.setTimeout(DTR004_CONNECT_TIMEOUT);
    bool ok = probe.connect(_ip, _port);
    if (ok) probe.stop();
    return ok;
}

void Probots_DTR004_ESP32::_tickHeartbeat() {
    if (millis() - _lastHeartbeat < _heartbeatInterval) return;
    _lastHeartbeat = millis();

    bool alive = (WiFi.status() == WL_CONNECTED) && isReachable();
    if (alive != _lastConnState && _connCb) {
        _connCb(alive);
    }
    _lastConnState = alive;
}

// ---------------------------------------------------------------------------
// Diagnostics
// ---------------------------------------------------------------------------

const char* Probots_DTR004_ESP32::errorToString(DTR004::Error err) {
    switch (err) {
        case DTR004::Error::NONE:              return "OK";
        case DTR004::Error::UNREACHABLE:       return "Device Unreachable";
        case DTR004::Error::TIMEOUT:           return "Timeout";
        case DTR004::Error::PARSE_FAILED:      return "Response Parse Failed";
        case DTR004::Error::INVALID_CHANNEL:   return "Invalid Channel";
        case DTR004::Error::BUSY:              return "SDK Busy";
        case DTR004::Error::WIFI_DISCONNECTED: return "WiFi Disconnected";
        case DTR004::Error::DEVICE_NOT_FOUND:  return "RS485 Sensor Not Found on Bus";
        case DTR004::Error::MB_ILLEGAL_FUNC:   return "Modbus: Illegal Function";
        case DTR004::Error::MB_ILLEGAL_VALUE:  return "Modbus: Illegal Data Value";
        case DTR004::Error::MB_SLAVE_FAILURE:  return "Modbus: Slave Device Failure";
        case DTR004::Error::MB_EXCEPTION:      return "Modbus: Exception (see getLastExceptionCode)";
        default:                               return "Unknown Error";
    }
}

// ---------------------------------------------------------------------------
// Private helpers
// ---------------------------------------------------------------------------

DTR004::Error Probots_DTR004_ESP32::_httpGet(const String& path, String& bodyOut) {
    if (WiFi.status() != WL_CONNECTED) {
        _lastError = DTR004::Error::WIFI_DISCONNECTED;
        _failureCount++;
        return _lastError;
    }

    WiFiClient client;
    client.setTimeout(DTR004_TIMEOUT_MS / 1000);

    if (!client.connect(_ip, _port)) {
        _lastError = DTR004::Error::UNREACHABLE;
        _failureCount++;
        return _lastError;
    }

    client.print(String("GET ") + path + " HTTP/1.1\r\n"
                 "Host: " + _ip + "\r\n"
                 "Connection: close\r\n\r\n");

    String response;
    unsigned long deadline = millis() + DTR004_TIMEOUT_MS;
    while (client.connected() && millis() < deadline) {
        while (client.available()) {
            response += (char)client.read();
        }
    }
    client.stop();

    int sep = response.indexOf("\r\n\r\n");
    if (sep == -1) {
        _lastError = DTR004::Error::PARSE_FAILED;
        _failureCount++;
        return _lastError;
    }

    bodyOut = response.substring(sep + 4);
    bodyOut.trim();
    _lastError = DTR004::Error::NONE;
    _successCount++;
    return DTR004::Error::NONE;
}

// Parses several response formats the DT-R firmware emits:
//   "on,off,on,off"   comma-separated words  (count tokens)
//   "1,0,1,0"         comma-separated digits
//   "1010"            packed binary string
//   {"in1":1,"in2":0} minimal JSON subset
uint32_t Probots_DTR004_ESP32::_parseBitmask(const String& body, uint8_t count) {
    uint32_t mask = 0;

    // JSON-style: scan for "inN":X or "relayN":X patterns
    if (body.indexOf('{') != -1) {
        for (uint8_t i = 1; i <= count; i++) {
            String key1 = "\"in"    + String(i) + "\":";
            String key2 = "\"relay" + String(i) + "\":";
            int idx = body.indexOf(key1);
            if (idx == -1) idx = body.indexOf(key2);
            if (idx == -1) continue;
            // The value character follows the colon — use key1.length() as offset
            // regardless of which key matched (both have the same suffix ":").
            char val = body.charAt(idx + key1.length());
            if (val == '1' || val == 't') mask |= (1UL << (i - 1));
        }
        return mask;
    }

    // Comma-separated or packed string
    String s = body;
    s.toLowerCase();
    s.replace(" ", "");

    if (s.indexOf(',') == -1 && s.length() >= (unsigned)count) {
        // Packed string: "1010..." or "onoffonoff..." — read one char per channel
        for (uint8_t i = 0; i < count; i++) {
            char c = s.charAt(i);
            if (c == '1' || c == 'o') mask |= (1UL << i); // 'o' for "on"
        }
        return mask;
    }

    // Comma-delimited tokens
    uint8_t ch = 0;
    int start = 0;
    while (ch < count) {
        int comma = s.indexOf(',', start);
        String token = (comma != -1) ? s.substring(start, comma) : s.substring(start);
        token.trim();
        if (token == "1" || token == "on" || token == "true") {
            mask |= (1UL << ch);
        }
        ch++;
        if (comma == -1) break;
        start = comma + 1;
    }
    return mask;
}

String Probots_DTR004_ESP32::_relayPath(uint8_t ch, uint8_t state) const {
    return String("/relay_cgi.cgi?type=0&relay=") + ch + "&on=" + state + "&time=0&pwd=";
}
