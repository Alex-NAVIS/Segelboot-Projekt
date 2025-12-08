// ==========================================================
// MagDeclination.cpp
// ----------------------------------------------------------
// Modul zur Berechnung der magnetischen Missweisung basierend
// auf GPS-Position und Datum.
// Verwendet die Bibliothek WMM_Tinier von David Armstrong.
// ==========================================================

#include "Mag_Dec.h"
#include "Sensor_Data.h"
#include <WMM_Tinier.h>   // stellt die Klasse WMM_Tinier bereit

// ----------------------------------------------------------
// Lokale Instanz der WMM-Tinier-Klasse
// ----------------------------------------------------------
static WMM_Tinier wmm;   // das Modell-Objekt

// ----------------------------------------------------------
// setupMagDeclination()
// ----------------------------------------------------------
// Initialisiert das Modell (ruft begin() der Library auf).
// Wird einmalig beim Start ausgeführt.
// ----------------------------------------------------------
void setupMagDeclination() {
  if (DEBUG_MODE) Serial.println(F("[WMM] Initialisiere WMM_Tinier..."));
  if (!wmm.begin()) {
    Serial.println(F("[WMM] Fehler: WMM-Modell konnte nicht initialisiert werden."));
  } else if (DEBUG_MODE) {
    Serial.println(F("[WMM] WMM_Tinier erfolgreich initialisiert."));
  }
}

//Missweisung berechnen bei neuem tag
void missweisung_berechnen(){
  
  // ==========================================================
  // Prüfung & ggf. Neuberechnung der magnetischen Missweisung
  // ----------------------------------------------------------
  // - Führt die Berechnung nur 1× pro Kalendertag aus
  // - Voraussetzung: gültige GPS-Koordinaten + Datum vorhanden
  // - Entlastet den ESP32 (kein unnötiges Neuberechnen)
  // ==========================================================
  static int lastDeclinationYear = 0;
  static int lastDeclinationMonth = 0;
  static int lastDeclinationDay = 0;

  if (!isnan(sensorData.gps_lat) && !isnan(sensorData.gps_lon) && sensorData.gps_jahr > 0) {
    bool newDay =
      (sensorData.gps_jahr != lastDeclinationYear) || (sensorData.gps_monat != lastDeclinationMonth) || (sensorData.gps_tag != lastDeclinationDay);

    if (newDay) {
      // --- Missweisung berechnen ---
      calculateMagDeclination();

      // Aktuelles Datum merken → verhindert doppelte Berechnungen am selben Tag
      lastDeclinationYear = sensorData.gps_jahr;
      lastDeclinationMonth = sensorData.gps_monat;
      lastDeclinationDay = sensorData.gps_tag;

      if (DEBUG_MODE) {
        Serial.print(F("[MAG] Missweisung aktualisiert am "));
        Serial.print(sensorData.gps_tag);
        Serial.print('.');
        Serial.print(sensorData.gps_monat);
        Serial.print('.');
        Serial.println(sensorData.gps_jahr);
      }
    }
  }
}

// ----------------------------------------------------------
// calculateMagDeclination()
// ----------------------------------------------------------
// Führt die eigentliche Berechnung der Missweisung durch,
// sobald gültige GPS-Daten vorliegen. Das Ergebnis wird in
// sensorData.missweisung (°) gespeichert.
// ----------------------------------------------------------
void calculateMagDeclination() {
  // Überprüfen, ob GPS-Daten gültig sind
  if (isnan(sensorData.gps_lat) || isnan(sensorData.gps_lon) || sensorData.gps_jahr == 0)
    return;  // keine Berechnung möglich

  // Optional: nur einmal täglich berechnen
  static int lastCalcDay = -1;
  if (sensorData.gps_tag == lastCalcDay) return;
  lastCalcDay = sensorData.gps_tag;

  // Parameter vorbereiten
  float lat = sensorData.gps_lat;
  float lon = sensorData.gps_lon;
  uint8_t year  = sensorData.gps_jahr - 2000;  // WMM_Tinier nutzt Jahr ab 2000 (z. B. 25 für 2025)
  uint8_t month = sensorData.gps_monat;
  uint8_t day   = sensorData.gps_tag;
   if (DEBUG_MODE) {
    Serial.print(F("[WMM] Missweisung Startwerte: "));
    Serial.println(lat);
    Serial.println(lon);
    Serial.println(year);
    Serial.println(month);
    Serial.println(day);

   }

  // Berechnung mit der Bibliotheksfunktion
  float decl = wmm.magneticDeclination(lat, lon, year, month, day);

  // Ergebnis speichern
  sensorData.missweisung = decl;

  // Debug-Ausgabe
  if (DEBUG_MODE) {
    Serial.print(F("[WMM] Missweisung berechnet: "));
    Serial.print(decl, 2);
    Serial.println(F("°"));
  }
}
