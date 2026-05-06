#ifndef PROBOTS_DTR004_ESP32_H
#define PROBOTS_DTR004_ESP32_H

/*
 * Probots_DTR004_ESP32 SDK  v2.0.0
 * Professional ESP32 driver for the DT-R004 4-Channel WiFi/Ethernet Relay Module.
 *
 * Key capabilities over v1:
 *   - Non-blocking async API via cooperative state machine (call update() in loop)
 *   - uint8_t bitmask returns for relay and digital-input states
 *   - Network resiliency: configurable heartbeat + connection-change callback
 *   - Rich error codes and per-instance diagnostics counters
 *   - Companion DTR004_ModbusTCP class for Modbus TCP protocol access
 */

#include <Arduino.h>
#include <WiFi.h>
#include "DTR004_Types.h"

#define DTR004_SDK_VERSION      "2.0.0"
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
    explicit Probots_DTR004_ESP32(const char* ip   = DTR004_DEFAULT_IP,
                                   uint16_t   port  = DTR004_DEFAULT_PORT);

    // -----------------------------------------------------------------------
    // Synchronous (blocking) API
    //   Suitable for setup() or simple sketches. Each call opens a new TCP
    //   connection, sends the request, waits for the response, and returns.
    // -----------------------------------------------------------------------

    /** Set one relay. Returns Error::NONE on success. */
    DTR004::Error setRelay(DTR004::Channel ch, DTR004::RelayState state);

    /**
     * Set all four relays in one bitmask (bit 0 = CH1 … bit 3 = CH4).
     * Sends four sequential HTTP requests; prefer Modbus TCP for atomic writes.
     */
    DTR004::Error setRelayBitmask(uint8_t bitmask);

    /**
     * Read the live state of all four digital inputs.
     * Returns a bitmask (bit 0 = DI1 … bit 3 = DI4), 0 on error.
     * Optionally writes the error code to *err.
     */
    uint8_t getInputStates(DTR004::Error* err = nullptr);

    /**
     * Read the reported relay states from the module's /status.cgi endpoint.
     * Returns a bitmask (bit 0 = Relay1 … bit 3 = Relay4), 0 on error.
     */
    uint8_t getRelayStates(DTR004::Error* err = nullptr);

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
     * Returns false immediately if another async op is running.
     * cb fires once complete (may be nullptr).
     */
    bool setRelayAsync(DTR004::Channel     ch,
                       DTR004::RelayState  state,
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
    // Config
    const char* _ip;
    uint16_t    _port;

    // Synchronous helper — opens connection, sends GET, strips headers.
    DTR004::Error _httpGet(const String& path, String& bodyOut);

    // Response parser: converts "on,off,on,off" / "1010" / JSON → bitmask
    static uint8_t _parseBitmask(const String& body);

    // Path builders
    String _relayPath(uint8_t ch, uint8_t state) const;

    // ---- Async state machine ----
    DTR004::AsyncState  _asyncState;
    DTR004::OpType      _pendingOp;
    String              _pendingPath;
    String              _rxBuf;
    unsigned long       _asyncDeadline;
    unsigned long       _lastByteMs;    // tracks last received byte for idle-detection

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

#endif
