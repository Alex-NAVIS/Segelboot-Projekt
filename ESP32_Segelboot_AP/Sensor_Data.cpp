// ==========================================================
// SensorData.cpp
// ----------------------------------------------------------
// Definition und Initialisierung der globalen SensorData-Struktur.
// Diese Datei stellt realistische Startwerte bereit, damit
// die Weboberfläche beim Start sofort sinnvolle Daten anzeigen kann.
// ==========================================================

#include "Sensor_Data.h"

// ----------------------------------------------------------
// Globale Instanz von SensorData
// ----------------------------------------------------------
// Alle Module können über 'extern SensorData sensorData' darauf zugreifen.
// ----------------------------------------------------------

SensorData sensorData = {
  // ------------------------------------------------------
  // Lage- / Bewegungssensor (MPU9250)
  // ------------------------------------------------------
  .roll = 22.5,      // leicht nach Steuerbord geneigt (°)
  .pitch = -15.2,    // Bug leicht abgesenkt (°)
  .kompass = 145.2,  // Boot zeigt nach Ost (°)

  // ------------------------------------------------------
  // Windsensoren (AS5600 & Encoder)
  // ------------------------------------------------------
  .winddir_gemessen = 270.0,    // Wind kommt leicht schräg von vorne (°)
  .windspeed_gemessen = 12.4,   // gemessene Windgeschwindigkeit (kn/h)
  .winddir_berechnet = 115.0,   // berechnete wahre Windrichtung (°)
  .windspeed_berechnet = 10.8,  // berechnete wahre Windgeschwindigkeit (kn/h)

  // ------------------------------------------------------
  // GPS-Daten (TinyGPS++)
  // ------------------------------------------------------
  .gps_lat = 54.321654,  // Beispiel: Nähe Kiel, Deutschland
  .gps_lon = 10.123987,
  .gps_speed = 8.6,   // 8.6 km/h = ca. 4.6 Knoten
  .gps_kurs = 210.5,  // Kurs nach Osten (°)
  .gps_sats = 0,     // GPS Satanzahl
  .gps_hdop = 99.9,  // gps hdop Fehlerwert

  // ------------------------------------------------------
  // GPS-Datum & Zeit (NMEA)
  // ------------------------------------------------------
  .gps_jahr = 2025,
  .gps_monat = 10,
  .gps_tag = 19,
  .gps_stunde = 14,
  .gps_minute = 37,
  .gps_sekunde = 45,

  // ------------------------------------------------------
  // Magnetische Missweisung (WMM_Tinier)
  // ------------------------------------------------------
  .missweisung = 3.2,  // +3.2° = östliche Deklination (z.B. Norddeutschland)

  // ------------------------------------------------------
  // Echolot (in Metern)
  // ------------------------------------------------------
  .Echolot = 2.8,  // aktuelle Tiefe unter Kiel (m)

  // --------------------------------------------------------
  // Lichtsteuerung Relay A B C D
  // --------------------------------------------------------
  .relay_a = 1,
  .relay_b = 1,
  .relay_c = 1,
  .relay_d = 1,

  //Mastdata Online offline timeout
  .mast_online = 0
};


// ----------------------------------------------------------
// Globale Instanz für Autopilot-Daten
// ----------------------------------------------------------
AutopilotData pinnenautopilotData = {
  // --------------------------------------------------------
  // PID-Regler bei glatter See
  // --------------------------------------------------------
  .P_welle_1 = 1.2,
  .I_welle_1 = 0.8,
  .D_welle_1 = 0.3,

  // --------------------------------------------------------
  // PID-Regler bei mäßiger See
  // --------------------------------------------------------
  .P_welle_2 = 1.6,
  .I_welle_2 = 0.9,
  .D_welle_2 = 0.4,

  // --------------------------------------------------------
  // PID-Regler bei rauer See
  // --------------------------------------------------------
  .P_welle_3 = 2.0,
  .I_welle_3 = 1.1,
  .D_welle_3 = 0.5,

  // --------------------------------------------------------
  // Inverter der Pinnenbewegung
  // --------------------------------------------------------
  .pinne_invertieren = 0,

  // --------------------------------------------------------
  // Autopilot-Zielkoordinaten (Test: Kieler Förde)
  // --------------------------------------------------------
  .autopilot_lat = 54.327000,
  .autopilot_lon = 10.145000
};

int status_autopilot = 0;
int rotary_offset = 0;
int PID_wahl = 1; // oder Startwert

// ----------------------------------------------------------
// Autopilot-Modus & Motor
// ----------------------------------------------------------
bool motor_manuel_einfahren = false;
bool motor_manuel_ausfahren = false;

volatile long motor_position = 0;
volatile int motor_richtung = 0;

bool end_links_aktiv = false;
bool end_rechts_aktiv = false;

bool motor_aktiv = false;
bool motor_hauptschalter = false;
uint32_t stepThreshold = 100;