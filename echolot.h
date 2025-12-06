
// ==========================================================
// Echolot Sensor zur Wassertiefenbestimmung
// ==========================================================

#pragma once
#include <Arduino.h>

#include "Sensor_Data.h"  // globale Variablen des Bootes
#include "Config.h"       // Pin und Einstellungen


// Setup / Initialisierung
void setup_echolot();

// Sensor auslesen (Werte in sensorData.Echolot schreiben)
void echolot_lesen();