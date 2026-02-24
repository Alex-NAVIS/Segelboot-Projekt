#pragma once

#include <Arduino.h>
#include "Config.h"

// =================== Konfiguration ===================

// Pfad zur Stationsdatei
#define TIDE_STATION_FILE "/settings/stations.json"

// Maximale Anzahl der erfassten Stationen
#define MAX_TIDE_STATIONS 16

// Suchparameter (vom Request gesetzt)
extern double tideQueryLat;
extern double tideQueryLon;
extern double tideQueryRadiusSM;

// Ergebnisstruktur
struct TideStation {
  String name;
  double lat;
  double lon;
  double distanceSM;
};

// Ergebnisarray
extern TideStation tideStations[MAX_TIDE_STATIONS];
extern uint8_t tideStationCount;

// API
bool findTideStations();
