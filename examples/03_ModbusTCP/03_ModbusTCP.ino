/*
 * 03_ModbusTCP — Control DT-R series relays via Modbus TCP
 *
 * The DT-R module is a Modbus TCP server. Unit ID 0xFF addresses its own
 * relays and digital inputs via holding registers (FC 0x03/0x06).
 * Unit IDs 1–247 are forwarded to the RS485 bus as Modbus RTU pass-through.
 *
 * Register map (Dingtian SDK programming manual):
 *   0x0000  FC 0x03 → relay count (reads actual channel count of your board)
 *   0x0001  FC 0x03 → relay bitmask (bit0=relay1)
 *   0x0002  FC 0x06 → write-all: high-byte=update mask, low-byte=new states (≤8 ch)
 *   0x000A  FC 0x03 → input bitmask (bit0=input1)
 *   0x0036+N FC 0x06 → relay N individual write (0=OFF, 1=ON)
 *
 * Web UI prerequisite:
 *   Relay Connect → TCP Server → Modbus-TCP, Local Port 502
 *
 * Compatible with all DT-R boards from probots.co.in:
 *   DT-R002 / DT-R004 / DT-R008 / DT-R016 / DT-R032
 *   Pass your board's relay count as the fourth constructor argument.
 *
 * Buy at: https://probots.co.in
 */

#include <WiFi.h>
#include "DTR004_ModbusTCP.h"

const char* WIFI_SSID = "dtrelay65905";
const char* WIFI_PASS = "dtpassword";

// Unit ID 0xFF = DT-R board's own relays. Fourth arg = relay count (default 4).
// For an 8-channel DT-R008: DTR004_ModbusTCP relay("192.168.7.1", 502, 0xFF, 8);
DTR004_ModbusTCP relay("192.168.7.1");

void setup() {
    Serial.begin(115200);
    delay(500);

    WiFi.begin(WIFI_SSID, WIFI_PASS);
    Serial.print("Connecting to WiFi");
    while (WiFi.status() != WL_CONNECTED) { delay(500); Serial.print('.'); }
    Serial.println("\nConnected: " + WiFi.localIP().toString());

    if (!relay.connect()) {
        Serial.println("Modbus TCP connection failed. Check IP / port 502.");
        while (true) delay(1000);
    }
    Serial.println("Modbus TCP connected (unit ID 0xFF = DT-R004 local).\n");

    // ── 1. Atomic write: set all four relays at once in one round-trip ──────
    // setAllRelays writes register 0x0002: mask=0x0F (update all 4), states=bitmask
    Serial.println("setAllRelays(0b0101) — relay 1 ON, 2 OFF, 3 ON, 4 OFF");
    DTR004::Error err = relay.setAllRelays(0b0101);
    Serial.printf("  → %s\n\n", DTR004_ModbusTCP::errorToString(err));
    delay(1000);

    // ── 2. Individual relay write (FC 0x06, register 0x0036+N) ─────────────
    Serial.println("setRelay(CH2, ON)");
    err = relay.setRelay(DTR004::Channel::CH2, DTR004::RelayState::ON);
    Serial.printf("  → %s\n\n", DTR004_ModbusTCP::errorToString(err));
    delay(1000);

    // ── 3. Read relay bitmask (FC 0x03, register 0x0001) ───────────────────
    DTR004::Error relErr;
    uint8_t states = relay.readRelayStates(&relErr);
    if (relErr == DTR004::Error::NONE) {
        Serial.printf("Relay states: R1=%d R2=%d R3=%d R4=%d  (bitmask=0x%02X)\n\n",
                      DTR004::isBitSet(states, 1), DTR004::isBitSet(states, 2),
                      DTR004::isBitSet(states, 3), DTR004::isBitSet(states, 4),
                      states);
    } else {
        Serial.printf("readRelayStates error: %s\n\n", DTR004_ModbusTCP::errorToString(relErr));
    }

    // ── 4. Raw register access: read relay count ────────────────────────────
    uint16_t countReg = 0;
    int16_t n = relay.readHoldingRegisters(DTR004_REG_RELAY_COUNT, 1, &countReg);
    if (n > 0) Serial.printf("Relay count (reg 0x0000): %u\n\n", countReg);

    // ── 5. All relays OFF ───────────────────────────────────────────────────
    delay(2000);
    Serial.println("setAllRelays(0x00) — all OFF");
    relay.setAllRelays(0x00);
}

void loop() {
    if (!relay.isConnected()) {
        Serial.println("Reconnecting...");
        if (!relay.connect()) { delay(5000); return; }
    }

    DTR004::Error err;
    uint8_t inputs = relay.readInputStates(&err);

    if (err == DTR004::Error::NONE) {
        Serial.printf("DI: DI1=%d DI2=%d DI3=%d DI4=%d  (bitmask=0x%02X)\n",
                      DTR004::isBitSet(inputs, 1), DTR004::isBitSet(inputs, 2),
                      DTR004::isBitSet(inputs, 3), DTR004::isBitSet(inputs, 4),
                      inputs);
    } else {
        Serial.printf("readInputStates error: %s\n", DTR004_ModbusTCP::errorToString(err));
    }

    delay(1000);
}
