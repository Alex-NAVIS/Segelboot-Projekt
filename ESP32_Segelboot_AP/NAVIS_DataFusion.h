// ======================================================================
// Datei: NAVIS_DataFusion.h
// Beschreibung:
// Zentrale Datenfusion für externe Sensorquellen.
//
// Dieses Modul sammelt Rohdaten aus:
// - NMEA2000
// - NMEA0183
// - Netzwerk (UDP/TCP)
//
// Die Daten werden in Ringspeichern abgelegt,
// bewertet und anschließend in die globale
// extern_sensorData Struktur übernommen.
// ======================================================================

#pragma once

#include <Arduino.h>

// ======================================================================
// QUELLEN ID
// ======================================================================
#define NAVIS_SOURCE_INTERNAL    0
#define NAVIS_SOURCE_NMEA2000    1
#define NAVIS_SOURCE_NMEA0183    2
#define NAVIS_SOURCE_NETWORK     3
#define NAVIS_SOURCE_SIMULATION  4

// ======================================================================
// Quellen-ID
// ======================================================================
enum DataSource {
    SRC_NMEA2000 = 0,
    SRC_NMEA0183 = 1,
    SRC_NETWORK  = 2
};

// ======================================================================
// Öffentliche Funktionen
// ======================================================================

// Initialisierung
void NAVIS_DataFusion_setup();

// Zyklische Auswertung
void NAVIS_DataFusion_update();

// ----------------------------------------------------------------------
// GPS
// ----------------------------------------------------------------------
void NAVIS_pushGPS(
    double lat,
    double lon,
    double speed,
    double course,
    int sats,
    float hdop,
    uint8_t source
);

// ----------------------------------------------------------------------
// WIND
// ----------------------------------------------------------------------
void NAVIS_pushWind(
    double angle,
    double speed,
    uint8_t source
);

// ----------------------------------------------------------------------
// DEPTH
// ----------------------------------------------------------------------
void NAVIS_pushDepth(
    double depth,
    uint8_t source
);