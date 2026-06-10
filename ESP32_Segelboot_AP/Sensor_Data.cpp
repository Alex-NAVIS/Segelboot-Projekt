//// ======================================================================
// SensorData.cpp
// -----------------------------------------------------------------------
// Bezeichnung: Globale Variablen Initialisierung
// Erklärung:   Hier werden die globalen Instanzen der Datenstrukturen
//              erstellt und mit Startwerten belegt. Diese Werte dienen
//              als "Fallback" oder Test-Setup direkt nach dem Bootvorgang,
//              bevor die Sensoren ihre ersten Echtzeitdaten liefern
//              oder Einstellungen aus dem Speicher geladen werden.
// ======================================================================

#include "Sensor_Data.h"

// Initialisierung der Sensordaten mit Beispielwerten (z.B. Kieler Förde)
SensorData sensorData = {
  .roll = 22.5,
  .pitch = -15.2,
  .kompass = 145.2,
  .winddir_gemessen = 270.0,
  .windspeed_gemessen = 12.4,
  .winddir_berechnet = 115.0,
  .windspeed_berechnet = 10.8,
  .gps_lat = 54.321654,
  .gps_lon = 10.123987,
  .gps_predict_lat = 54.321654,
  .gps_predict_lon = 10.123987,
  .gps_speed = 8.6,
  .gps_kurs = 210.5,
  .gps_sats = 0,
  .gps_hdop = 99.9,
  .gps_jahr = 2025,
  .gps_monat = 10,
  .gps_tag = 19,
  .gps_stunde = 14,
  .gps_minute = 37,
  .gps_sekunde = 45,
  .missweisung = 3.2,
  .Echolot = 2.8,
  .relay_a = 1,  // AUS
  .relay_b = 1,  // AUS
  .relay_c = 1,  // AUS
  .relay_d = 1,  // AUS
  .mast_online = 0,
  .sm_counter = 0.0,  // Seemeileen Zähler
  .track_index = 0,   // Trackspeicher
  .track_filled = false
};

// Initialisierung der externen Sensordaten mit Beispielwerten (z.B. Kieler Förde)
extern_SensorData extern_sensorData = {
  // GPS-Daten Extern
  .gps_lat = 54.321654,
  .gps_lon = 10.123987,
  .gps_speed = 8.6,
  .gps_kurs = 210.5,
  .gps_sats = 0,
  .gps_hdop = 99.9,

  // GPS-Datum & Zeit Extern
  .gps_jahr = 2025,
  .gps_monat = 10,
  .gps_tag = 19,
  .gps_stunde = 14,
  .gps_minute = 37,
  .gps_sekunde = 45,

  // Windsensoren Extern
  .winddir_gemessen = 270.0,
  .windspeed_gemessen = 12.4,

  // Echolot Extern
  .Echolot = 2.8
};

// Initialisierung des Autopiloten (Standard: Standby / Modus 0)
AutopilotData pinnenautopilotData = {
  .autopilot_lat = 54.327000,
  .autopilot_lon = 10.145000,
  .modus_autopilot = 0,
  .rotary_offset = 0,
  .motor_manuel_einfahren = false,
  .motor_manuel_ausfahren = false,
  .modus_counter = 0
};

// Initialisierung des Alarmsystems (Standard: Alle Alarme deaktiviert)
AlarmData alarmData = {
  .alarm = 0,
  .en_speed_max = false,
  .speedMax = 10.0,
  .en_speed_min = false,
  .speedMin = 2.0,
  .en_echolot = false,
  .echolotMin = 4,
  .en_wind_max = false,
  .windMax = 25.0,
  .en_wind_min = false,
  .windMin = 5.0,
  .en_wind_dir = false,
  .windDir = 30.0,
  .windDir_temp = 0.0,
  .en_course_dev = false,
  .courseDev = 15.0,
  .courseDev_temp = 0.0,
  .en_anchor = false,
  .anchorRadius = 30.0,
  .anchorkette = 20.0,
  .anker_lat = 0.0,
  .anker_lon = 0.0,
  .en_roll = false,
  .roll = 25.0,
  .en_pitch = false,
  .pitch = 25.0
};
