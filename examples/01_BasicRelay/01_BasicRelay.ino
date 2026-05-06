/*
 * 01_BasicRelay — Synchronous API (drop-in replacement for v1 library)
 *
 * Demonstrates:
 *  - Synchronous relay writes with error checking
 *  - Bitmask-based input reads (uint8_t, not String)
 *  - Channel enum for type-safe channel selection
 *
 * Wiring / Network:
 *  ESP32 connects to the DT-R004's own WiFi AP ("dtrelay…" network)
 *  or the DT-R004 is on the same LAN as the ESP32.
 */

#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

const char* WIFI_SSID = "dtrelay65905";
const char* WIFI_PASS = "dtpassword";

// Point to the module's IP. Change if you configured a static IP.
Probots_DTR004_ESP32 relay("192.168.7.1");

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.println("\nConnected. IP: " + WiFi.localIP().toString());

    // --- Relay control with error checking ---
    DTR004::Error err = relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::ON);
    if (err == DTR004::Error::NONE) {
        Serial.println("Relay 1 ON");
    } else {
        Serial.printf("Error: %s\n", Probots_DTR004_ESP32::errorToString(err));
    }

    delay(2000);
    relay.setRelay(DTR004::Channel::CH1, DTR004::RelayState::OFF);
    Serial.println("Relay 1 OFF");

    // --- Set all relays via bitmask: CH1 + CH3 ON, CH2 + CH4 OFF ---
    relay.setRelayBitmask(0b0101);
    Serial.println("Bitmask write: CH1 and CH3 ON");
}

void loop() {
    DTR004::Error err;
    uint8_t inputs = relay.getInputStates(&err);

    if (err == DTR004::Error::NONE) {
        Serial.printf("Digital Inputs: DI1=%d  DI2=%d  DI3=%d  DI4=%d\n",
                      DTR004::isBitSet(inputs, 1),
                      DTR004::isBitSet(inputs, 2),
                      DTR004::isBitSet(inputs, 3),
                      DTR004::isBitSet(inputs, 4));
    } else {
        Serial.printf("Read error: %s\n", Probots_DTR004_ESP32::errorToString(err));
    }

    Serial.printf("Stats — OK: %lu  Fail: %lu\n",
                  relay.getSuccessCount(), relay.getFailureCount());
    delay(3000);
}
