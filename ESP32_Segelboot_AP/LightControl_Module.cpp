// ======================================================================
//  Modul: LightControl_Module.cpp
//  Aufgabe:
//     - Ansteuerung und Synchronisation der Lichtrelais (A–D).
//     - Direkte Steuerung per Webinterface oder lokaler Logik.
// ======================================================================

#include "LightControl_Module.h"
#include "Sensor_Data.h"
#include "Config.h"

extern SensorData sensorData;


// ----------------------------------------------------------
// Initialisierung der Relaissteuerung
// ----------------------------------------------------------
void setupLightControl() {
  pinMode(RELAY_A_PIN, OUTPUT);
  pinMode(RELAY_B_PIN, OUTPUT);
  pinMode(RELAY_C_PIN, OUTPUT);
  pinMode(RELAY_D_PIN, OUTPUT);

  // Relais: HIGH = AUS, LOW = EIN
  digitalWrite(RELAY_A_PIN, HIGH);
  digitalWrite(RELAY_B_PIN, HIGH);
  digitalWrite(RELAY_C_PIN, HIGH);
  digitalWrite(RELAY_D_PIN, HIGH);

  if (DEBUG_MODE) {
    Serial.println("💡 LightControl_Module: Relais initialisiert.");
  }
}

// ----------------------------------------------------------
// setLightState()
// ----------------------------------------------------------
// Wird vom Webserver bei Klick im Browser aufgerufen.
// Beispiel: /setLight?device=positionslicht&state=on
// ----------------------------------------------------------
void setLightState(const String& device, const String& state) {
  bool turnOn = (state == "on");
  int level = turnOn ? LOW : HIGH;  // LOW = Relais EIN

  if (device == "toplicht") {
    digitalWrite(RELAY_A_PIN, level);
  } else if (device == "positionslicht") {
    digitalWrite(RELAY_B_PIN, level);
  } else if (device == "lichtds") {
    digitalWrite(RELAY_C_PIN, level);
  } else if (device == "lichtcockpit") {
    digitalWrite(RELAY_D_PIN, level);
  } else {
    if (DEBUG_MODE) Serial.println("⚠️ Unbekanntes Gerät: " + device);
    return;
  }

  if (DEBUG_MODE) {
    Serial.printf("🔧 Lichtsteuerung: %s -> %s\n", device.c_str(), turnOn ? "EIN" : "AUS");
  }
}

// ----------------------------------------------------------
// syncRelaysIfChanged()
// ----------------------------------------------------------
// Prüft, ob sich die Relay-Zustände im sensorData geändert haben.
// Nur bei Änderung wird das physische Relais geschaltet.
// ----------------------------------------------------------
void syncRelaysIfChanged() {
  static int lastRelayA = -1;
  static int lastRelayB = -1;
  static int lastRelayC = -1;
  static int lastRelayD = -1;

  // Relay A
  if (sensorData.relay_a != lastRelayA) {
    digitalWrite(RELAY_A_PIN, (sensorData.relay_a == 1) ? HIGH : LOW);
    lastRelayA = sensorData.relay_a;
  }

  // Relay B
  if (sensorData.relay_b != lastRelayB) {
    digitalWrite(RELAY_B_PIN, (sensorData.relay_b == 1) ? HIGH : LOW);
    lastRelayB = sensorData.relay_b;
  }

  // Relay C
  if (sensorData.relay_c != lastRelayC) {
    digitalWrite(RELAY_C_PIN, (sensorData.relay_c == 1) ? HIGH : LOW);
    lastRelayC = sensorData.relay_c;
  }

  // Relay D
  if (sensorData.relay_d != lastRelayD) {
    digitalWrite(RELAY_D_PIN, (sensorData.relay_d == 1) ? HIGH : LOW);
    lastRelayD = sensorData.relay_d;
  }
}
