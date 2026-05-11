/*
 * 02_AsyncNonBlocking — Cooperative state-machine API
 *
 * Demonstrates:
 *  - Non-blocking relay writes and input reads
 *  - Callback-driven completion handling
 *  - loop() stays free for other real-time work (sensor sampling, display, etc.)
 *
 * Key rule: call relay.update() every loop() iteration.
 *           Only one async op may be in-flight; check isBusy() before queuing.
 *
 * Compatible with all DT-R boards from probots.co.in (pass relay count to constructor).
 * Buy at: https://probots.co.in
 */

#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

const char* WIFI_SSID = "dtrelay65905";
const char* WIFI_PASS = "dtpassword";

Probots_DTR004_ESP32 relay("192.168.7.1");

// --- Callbacks ---
void onRelayDone(DTR004::RelayEvent evt) {
    if (evt.err == DTR004::Error::NONE) {
        Serial.printf("[CB] Relay %d → %s\n",
                      evt.channel,
                      evt.state == DTR004::RelayState::ON ? "ON" : "OFF");
    } else {
        Serial.printf("[CB] Relay error: %s\n",
                      Probots_DTR004_ESP32::errorToString(evt.err));
    }
}

void onInputsDone(DTR004::InputEvent evt) {
    if (evt.err == DTR004::Error::NONE) {
        Serial.printf("[CB] Inputs bitmask: 0x%02X  (DI1=%d DI2=%d DI3=%d DI4=%d)\n",
                      evt.bitmask,
                      DTR004::isBitSet(evt.bitmask, 1),
                      DTR004::isBitSet(evt.bitmask, 2),
                      DTR004::isBitSet(evt.bitmask, 3),
                      DTR004::isBitSet(evt.bitmask, 4));
    } else {
        Serial.printf("[CB] Input error: %s\n",
                      Probots_DTR004_ESP32::errorToString(evt.err));
    }
}

// --- Periodic task timing ---
unsigned long lastRelayToggle = 0;
unsigned long lastInputPoll   = 0;
bool          relayOn         = false;

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.println("\nConnected: " + WiFi.localIP().toString());
}

void loop() {
    // Must be first — advances the async state machine
    relay.update();

    // Toggle Relay 1 every 5 seconds (non-blocking)
    if (millis() - lastRelayToggle > 5000 && !relay.isBusy()) {
        relayOn = !relayOn;
        DTR004::RelayState s = relayOn ? DTR004::RelayState::ON : DTR004::RelayState::OFF;
        relay.setRelayAsync(DTR004::Channel::CH1, s, onRelayDone);
        lastRelayToggle = millis();
    }

    // Poll inputs every 2 seconds (non-blocking)
    if (millis() - lastInputPoll > 2000 && !relay.isBusy()) {
        relay.getInputsAsync(onInputsDone);
        lastInputPoll = millis();
    }

    // Other real-time tasks can go here — this loop is never blocked waiting
    // for a network response.
}
