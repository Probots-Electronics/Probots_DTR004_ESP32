/*
 * Probots_DTR004_ESP32 — ESP32 driver for Dingtian DT-R series relay modules.
 *
 * Compatible with ALL DT-R board variants sold at probots.co.in:
 *   DT-R002 (2 ch)  DT-R004 (4 ch)  DT-R008 (8 ch)
 *   DT-R016 (16 ch) DT-R032 (32 ch)
 *
 * Key features:
 *   - Non-blocking async API via cooperative state machine (call update() in loop)
 *   - uint32_t bitmask I/O — supports up to 32 relay and input channels
 *   - Network resiliency: configurable heartbeat + connection-change callback
 *   - Rich error codes and per-instance diagnostic counters
 *   - Companion DTR004_ModbusTCP class for Modbus TCP protocol access
 *
 * Product page : https://probots.co.in
 * Support      : support@probots.co.in
 * License      : MIT — see LICENSE file
 *
 * SPDX-License-Identifier: MIT
 * Copyright (c) 2024 Probots Electronics (probots.co.in)
 */

#ifndef PROBOTS_DTR004_ESP32_H
#define PROBOTS_DTR004_ESP32_H

#include <Arduino.h>
#include <WiFi.h>
#include "DTR004_Types.h"

#define DTR004_SDK_VERSION      "2.1.0"
#define DTR004_DEFAULT_IP       "192.168.1.100"
#define DTR004_DEFAULT_PORT     80
#define DTR004_TIMEOUT_MS       3000UL
#define DTR004_HEARTBEAT_MS     30000UL
#define DTR004_CONNECT_TIMEOUT  500     // ms for async connect probe

class Probots_DTR004_ESP32 {
public:
    // -----------------------------------------------------------------------
    // Construction
    // -----------------------------------------------------------------------

    /**
     * @param ip         DT-R module IP address
     * @param port       HTTP port (default 80)
     * @param relayCount Number of relay channels on your board (2 / 4 / 8 / 16 / 32).
     *                   Determines valid channel range and bitmask width.
     *                   Boards: DT-R002=2, DT-R004=4, DT-R008=8, DT-R016=16, DT-R032=32
     */
    explicit Probots_DTR004_ESP32(const char* ip         = DTR004_DEFAULT_IP,
                                   uint16_t   port       = DTR004_DEFAULT_PORT,
                                   uint8_t    relayCount = 4);

    // -----------------------------------------------------------------------
    // Synchronous (blocking) API
    //   Suitable for setup() or simple sketches. Each call opens a new TCP
    //   connection, sends the request, waits for the response, and returns.
    // -----------------------------------------------------------------------

    /** Set one relay. Returns Error::NONE on success. */
    DTR004::Error setRelay(DTR004::Channel ch, DTR004::RelayState state);

    /**
     * Set relays from a bitmask (bit 0 = CH1, bit N-1 = CHN).
     * Sends sequential HTTP requests per channel — use Modbus TCP for
     * atomic multi-relay writes.
     */
    DTR004::Error setRelayBitmask(uint32_t bitmask);

    /**
     * Read the live state of all digital inputs.
     * Returns bitmask (bit 0 = DI1, bit N-1 = DIN), 0 on error.
     * Optionally writes the error code to *err.
     */
    uint32_t getInputStates(DTR004::Error* err = nullptr);

    /**
     * Read the reported relay states from the module's /status.cgi endpoint.
     * Returns bitmask (bit 0 = Relay1, bit N-1 = RelayN), 0 on error.
     */
    uint32_t getRelayStates(DTR004::Error* err = nullptr);

    // -----------------------------------------------------------------------
    // Asynchronous (non-blocking) API
    //   Initiate operations with setRelayAsync / getInputsAsync.
    //   Call update() every loop() iteration to advance the internal state
    //   machine. Completion fires the supplied callback from update().
    //   Only one async operation may be in-flight at a time; subsequent calls
    //   while busy return false without queuing.
    // -----------------------------------------------------------------------

    /**
     * Initiate a non-blocking relay write.
     * Returns false immediately if another async op is running or channel is invalid.
     * cb fires once complete (may be nullptr).
     */
    bool setRelayAsync(DTR004::Channel    ch,
                       DTR004::RelayState state,
                       DTR004::RelayCallback cb = nullptr);

    /** Initiate a non-blocking digital input read. */
    bool getInputsAsync(DTR004::InputCallback cb = nullptr);

    /** Drive the async state machine. Must be called every loop() iteration. */
    void update();

    bool isBusy() const { return _asyncState != DTR004::AsyncState::IDLE; }

    // -----------------------------------------------------------------------
    // Network Resiliency
    // -----------------------------------------------------------------------

    /**
     * Enable periodic reachability probes. When the connection state changes,
     * the registered ConnectionCallback fires.
     */
    void enableHeartbeat(uint32_t intervalMs = DTR004_HEARTBEAT_MS);
    void disableHeartbeat();

    /** Register a callback that fires when the module goes online/offline. */
    void onConnectionChange(DTR004::ConnectionCallback cb);

    /** Blocking TCP probe — returns true if the module is reachable. */
    bool isReachable();

    // -----------------------------------------------------------------------
    // Diagnostics
    // -----------------------------------------------------------------------
    uint32_t        getSuccessCount()  const { return _successCount; }
    uint32_t        getFailureCount()  const { return _failureCount; }
    DTR004::Error   getLastError()     const { return _lastError; }
    static const char* errorToString(DTR004::Error err);

private:
    const char* _ip;
    uint16_t    _port;
    uint8_t     _relayCount;

    DTR004::Error _httpGet(const String& path, String& bodyOut);

    // Parses "on,off,on,off" / "1,0,1,0" / "1010" / JSON → bitmask for N channels
    static uint32_t _parseBitmask(const String& body, uint8_t count);

    String _relayPath(uint8_t ch, uint8_t state) const;

    // ---- Async state machine ----
    DTR004::AsyncState  _asyncState;
    DTR004::OpType      _pendingOp;
    String              _pendingPath;
    String              _rxBuf;
    unsigned long       _asyncDeadline;
    unsigned long       _lastByteMs;

    uint8_t                 _pendingCh;
    DTR004::RelayState      _pendingRelayState;
    DTR004::RelayCallback   _relayCb;
    DTR004::InputCallback   _inputCb;

    WiFiClient _client;

    void _smStep();
    void _finishAsync(DTR004::Error err);

    // ---- Heartbeat ----
    bool          _heartbeatEnabled;
    uint32_t      _heartbeatInterval;
    unsigned long _lastHeartbeat;
    bool          _lastConnState;

    DTR004::ConnectionCallback _connCb;

    void _tickHeartbeat();

    // ---- Counters ----
    uint32_t      _successCount;
    uint32_t      _failureCount;
    DTR004::Error _lastError;
};

#endif // PROBOTS_DTR004_ESP32_H
