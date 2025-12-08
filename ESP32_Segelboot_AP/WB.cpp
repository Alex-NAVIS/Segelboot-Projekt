// ----------------------------------------------------------
// WB.cpp  (Wind-Berechnung)
// ----------------------------------------------------------
// Berechnet den "wahren Wind" aus gemessenem Wind (relativ zum Boot)
// und Bootsgeschwindigkeit / Kurs über Grund.
//
// Abhängigkeiten:
//   - Sensor_Data.h   (enthält Definition von `sensorData`)
//   - WB.h            (enthält Funktionsdeklaration)
//
// Autor: Alex & NAVIS
// ----------------------------------------------------------

#include "WB.h"         // Enthält Funktionsprototyp & Sensor_Data.h
#include <cmath>        // Für sin, cos, atan2, fmod, sqrt

// ----------------------------------------------------------
// Lokale Hilfsfunktionen (nur in dieser Datei sichtbar)
// ----------------------------------------------------------
static double deg2rad(double d) {
  return d * M_PI / 180.0;
}

static double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

static double norm360(double a) {
  a = fmod(a, 360.0);
  if (a < 0) a += 360.0;
  return a;
}

// ----------------------------------------------------------
// Funktion: berechneWahrenWind()
// ----------------------------------------------------------
// Berechnet den "wahren Wind" (True Wind) aus:
//   - gemessenem Wind (relativ zum Boot, FROM),
//   - Kompassausrichtung (Heading),
//   - GPS-Kurs (über Grund) und
//   - Bootsgeschwindigkeit.
//
// Ergebnis wird in sensorData.winddir_berechnet (° FROM relativ zum Bug)
// und sensorData.windspeed_berechnet (kn) gespeichert.
// ----------------------------------------------------------
void berechneWahrenWind() {
  // Apparent (gemessen) -- FROM relativ zum Bug (0° = Bug)
  double app_from_boat = sensorData.winddir_gemessen;  // °, FROM
  double app_speed = sensorData.windspeed_gemessen;    // kn

  // Bootsausrichtung & Geschwindigkeit (TO-Direction)
  double compass = norm360(sensorData.kompass);        // ° (Heading, 0–360)
  double boat_speed = sensorData.gps_speed;            // kn
  double boat_to_geo = norm360(sensorData.gps_kurs);   // Kurs über Grund (TO), °

  // Apparent FROM in Erdkoordinaten (Geo)
  double app_from_geo = norm360(app_from_boat + compass);  // FROM GEO °
  double app_to_geo = norm360(app_from_geo + 180.0);       // TO GEO °

  // Vektorkomponenten (TO-Richtung): Ost (u) und Nord (v)
  double app_to_rad = deg2rad(app_to_geo);
  double u_app = app_speed * sin(app_to_rad);  // East-Komponente
  double v_app = app_speed * cos(app_to_rad);  // North-Komponente

  double boat_to_rad = deg2rad(boat_to_geo);
  double u_boat = boat_speed * sin(boat_to_rad);
  double v_boat = boat_speed * cos(boat_to_rad);

  // True Wind (TO) = Apparent (TO) + Boat (TO)
  double u_true = u_app + u_boat;
  double v_true = v_app + v_boat;

  double true_speed = sqrt(u_true * u_true + v_true * v_true);  // kn
  double true_to_rad = atan2(u_true, v_true);                   // rad
  double true_to_geo = norm360(rad2deg(true_to_rad));
  double true_from_geo = norm360(true_to_geo + 180.0);

  // Ergebnis relativ zum Boot (0° = Bug)
  double true_from_boat = norm360(true_from_geo - compass);

  // Werte speichern
  sensorData.winddir_berechnet = true_from_boat;  // FROM relativ zum Bug (0–360)
  sensorData.windspeed_berechnet = true_speed;    // kn
}
