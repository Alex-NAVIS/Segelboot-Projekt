#pragma once
#include <Arduino.h>

#define TIDE_POINTS 96  // 24h * 4 (15min)

// =================== Zustände ===================
enum TideState {
  TIDE_IDLE = 0,
  TIDE_FIND_STATIONS = 1,
  TIDE_CALC_CURVE_1 = 2,
  TIDE_CALC_CURVE_2 = 3,
  TIDE_CALC_CURVE_3 = 4,
  TIDE_READY_TO_SEND_1 = 5,
  TIDE_READY_TO_SEND_2 = 6,
  TIDE_READY_TO_SEND_3 = 7
};

// =================== Steuer-Variablen ===================
extern volatile int tideState;

// =================== Anfrage ===================
extern double tideQueryLat;
extern double tideQueryLon;
extern void* tideRequest;

// =================== Station-Infos ===================
extern double tideStationLat[3];
extern double tideStationLon[3];
extern double tideStationDist[3];
extern double tideStationBearing[3];

// =================== Tide-Kurven (cm) ===================
extern int16_t tideCurve[3][TIDE_POINTS];

// =================== Funktionen ===================
void tide_reset();                  // 🔄 alles auf Anfang
bool find_stations();               // Liest CSV, füllt tideStationLat/Lon/Dist/Bearing
void berechne_tide_kurve(int idx);  // Berechnet tideCurve[idx] für Station idx
