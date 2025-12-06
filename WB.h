// ----------------------------------------------------------
// WB.h  (Wind-Berechnung)
// ----------------------------------------------------------
// Berechnet den "wahren Wind" aus gemessenem Wind (relativ zum Boot)
// und Bootsgeschwindigkeit / Kurs über Grund.
//
// Voraussetzungen:
//   - Zugriff auf globale Struktur `sensorData` (Sensor_Data.h)
//   - Aufruf z. B. im Loop mit 1 Hz
// ----------------------------------------------------------

#ifndef WB_H
#define WB_H

#include "Sensor_Data.h"

// ----------------------------------------------------------
// Funktionsprototypen
// ----------------------------------------------------------

// Führt die Berechnung des wahren Windes durch.
// Nutzt Daten aus der globalen Struktur `sensorData`:
//   - winddir_gemessen (° relativ zum Bug, FROM)
//   - windspeed_gemessen (kn)
//   - kompass (° 0–360, Heading)
//   - gps_kurs (°, Kurs über Grund)
//   - gps_speed (kn, Geschwindigkeit über Grund)
//
// Ergebnis wird in sensorData.winddir_berechnet (° FROM relativ zum Bug)
// und sensorData.windspeed_berechnet (kn) gespeichert.
//
void berechneWahrenWind();

#endif  // WB_H
