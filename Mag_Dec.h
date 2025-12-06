// ==========================================================
// MagDeclination.h
// ----------------------------------------------------------
// Modul zur Berechnung der magnetischen Missweisung
// (Abweichung zwischen magnetischem und geographischem Nordpol)
//
// Zweck:
//   - Berechnet die lokale Missweisung anhand der GPS-Position
//     (Breite, Länge) und des aktuellen Datums.
//   - Speichert das Ergebnis in `sensorData.missweisung`.
//   - Wird nur ausgeführt, wenn gültige GPS-Daten vorliegen.
//   - Eine Neuberechnung pro Tag genügt (Bootsposition ändert
//     sich nur langsam).
//
// Verwendung:
//   setupMagDeclination();     → Initialisiert ggf. Parameter
//   calculateMagDeclination(); → Führt die eigentliche Berechnung aus
//
// Aktuell:
//   - Platzhalter-Implementierung (lineare Näherung oder Dummywert)
//   - Erweiterbar auf echtes WMM-Modell (z. B. WMM_Tinier)
// ==========================================================

#pragma once
#include <Arduino.h>
#include "Sensor_Data.h"
#include "Config.h"

// ----------------------------------------------------------
// Funktionsprototypen
// ----------------------------------------------------------

// Initialisierung (einmalig im Setup)
void setupMagDeclination();

// Berechnung der magnetischen Missweisung (wird bei gültigem GPS-Datum ausgeführt)
void calculateMagDeclination();

// Missweisung starten nach abfrage des Tages
void missweisung_berechnen();