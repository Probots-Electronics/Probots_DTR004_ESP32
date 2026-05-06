/*
 * 04_NetworkResiliency — Heartbeat + auto-reconnect
 *
 * Demonstrates:
 *  - Heartbeat probe that detects module going offline/online
 *  - ConnectionCallback for application-level alerts or failsafe logic
 *  - WiFi auto-reconnect (ESP32 handles this natively; we guard with status check)
 *  - Async input polling combined with heartbeat — no blocking anywhere in loop()
 *
 * Industrial use-case:
 *  A PLC-style controller that must keep running even if the relay module
 *  power-cycles, and must alert the operator (via Serial / MQTT / buzzer)
 *  the moment connectivity is lost or restored.
 */

#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

const char* WIFI_SSID = "dtrelay65905";
const char* WIFI_PASS = "dtpassword";

Probots_DTR004_ESP32 relay("192.168.7.1");

bool moduleOnline = false;

// --- Connection state change handler ---
void onConnectionChange(bool connected) {
    moduleOnline = connected;
    if (connected) {
        Serial.println("[ALERT] DT-R004 ONLINE  — restoring safe relay states.");
        // Restore a known-safe state after reconnection
        relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::OFF);
        relay.setRelay(DTR004::Channel::CH2, DTR004::RelayState::OFF);
        relay.setRelay(DTR004::Channel::CH3, DTR004::RelayState::OFF);
        relay.setRelay(DTR004::Channel::CH4, DTR004::RelayState::OFF);
    } else {
        Serial.println("[ALERT] DT-R004 OFFLINE — entering failsafe mode.");
        // Local GPIO failsafe could go here (e.g. activate a buzzer pin)
    }
}

// --- Async input poll callback ---
void onInputs(DTR004::InputEvent evt) {
    if (evt.err != DTR004::Error::NONE) return;
    Serial.printf("[POLL ] Inputs: DI1=%d DI2=%d DI3=%d DI4=%d\n",
                  DTR004::isBitSet(evt.bitmask, 1),
                  DTR004::isBitSet(evt.bitmask, 2),
                  DTR004::isBitSet(evt.bitmask, 3),
                  DTR004::isBitSet(evt.bitmask, 4));

    // Example: DI1 triggers Relay 2 (interlocked logic)
    if (DTR004::isBitSet(evt.bitmask, 1)) {
        relay.setRelayAsync(DTR004::Channel::CH2, DTR004::RelayState::ON);
    }
}

unsigned long lastPoll = 0;

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.println("\nConnected: " + WiFi.localIP().toString());

    // Register connection change callback first
    relay.onConnectionChange(onConnectionChange);

    // Heartbeat every 10 seconds — fires onConnectionChange on state change.
    // Tune to your application's fault-detection latency requirement.
    relay.enableHeartbeat(10000);

    // Immediately check initial state
    moduleOnline = relay.isReachable();
    Serial.printf("Initial module state: %s\n", moduleOnline ? "ONLINE" : "OFFLINE");
}

void loop() {
    // update() drives: async state machine + heartbeat ticker
    relay.update();

    // WiFi guard — ESP32 will reconnect automatically, but we skip ops while down
    if (WiFi.status() != WL_CONNECTED) {
        Serial.println("WiFi lost — waiting for reconnect...");
        delay(2000);
        return;
    }

    // Non-blocking input poll every 2 seconds (only when module is online)
    if (moduleOnline && !relay.isBusy() && millis() - lastPoll > 2000) {
        relay.getInputsAsync(onInputs);
        lastPoll = millis();
    }

    // Diagnostics every 30 seconds
    static unsigned long lastStats = 0;
    if (millis() - lastStats > 30000) {
        Serial.printf("[STATS] OK=%lu  Fail=%lu  LastErr=%s\n",
                      relay.getSuccessCount(),
                      relay.getFailureCount(),
                      Probots_DTR004_ESP32::errorToString(relay.getLastError()));
        lastStats = millis();
    }
}
