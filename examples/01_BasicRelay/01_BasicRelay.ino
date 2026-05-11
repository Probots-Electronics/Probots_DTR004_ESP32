/*
 * 01_BasicRelay — Synchronous API
 *
 * Demonstrates:
 *  - Synchronous relay writes with error checking
 *  - uint32_t bitmask input reads (bit 0 = DI1, bit N-1 = DIN)
 *  - Channel enum for type-safe channel selection (CH1 … CH32)
 *
 * Hardware:
 *  Works with any DT-R series board from probots.co.in:
 *  DT-R002 / DT-R004 / DT-R008 / DT-R016 / DT-R032
 *  Pass your board's channel count as the third constructor argument.
 *
 * Network:
 *  Connect the ESP32 to the DT-R module's own WiFi AP ("dtrelay…")
 *  or put both on the same LAN.
 *
 * Buy at: https://probots.co.in
 */

#include <WiFi.h>
#include "Probots_DTR004_ESP32.h"

const char* WIFI_SSID = "dtrelay65905";
const char* WIFI_PASS = "dtpassword";

// Point to the module's IP. Third arg = relay count (default 4 for DT-R004).
// For an 8-channel DT-R008: Probots_DTR004_ESP32 relay("192.168.7.1", 80, 8);
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
        // isBitSet works for all board sizes: ch is 1-based (1=DI1, N=DIN)
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
