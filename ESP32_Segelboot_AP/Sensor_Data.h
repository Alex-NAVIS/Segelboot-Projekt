// ==========================================================
// SensorData.h
// ----------------------------------------------------------
// Globale Struktur für alle Sensordaten.
// Diese Struktur wird von allen Modulen gemeinsam genutzt.
// Alle Module lesen/schreiben über "sensorData".
// ----------------------------------------------------------
// Erweiterung:
//  - GPS-Datum und -Zeit
//  - Magnetische Missweisung (Berechnung mit WMM_Tinier)
// ==========================================================

#pragma once
#include <Arduino.h>
#include <math.h>  // für fmod()

// ======================================================================
// Bezeichnung: SensorData
// Erklärung:   Zentrale Sammelstruktur für sämtliche Sensordaten des
//              Schiffssystems. Beinhaltet die Lage- und Bewegungsdaten
//              (IMU), verarbeitete Winddaten, GPS-Positions- und
//              Zeitinformationen, magnetische Korrekturwerte sowie
//              Zustände der Peripherie (Echolot, Relais, Mast-Status).
// ======================================================================
struct SensorData {
  // Lage- / Bewegungssensor (MPU9250)
  double roll;     // Rollwinkel (Krängung) des Bootes (°)
  double pitch;    // Nickwinkel (Stampfen) des Bootes (°)
  double kompass;  // Magnetischer Steuerkurs (° 0–360)

  // Windsensoren (AS5600 & Encoder)
  double winddir_gemessen;     // Scheinbarer Windwinkel (relativ zum Bug) (°)
  double windspeed_gemessen;   // Scheinbare Windgeschwindigkeit (kn/h)
  double winddir_berechnet;    // Wahrer Windwinkel (True Wind) (°)
  double windspeed_berechnet;  // Wahre Windgeschwindigkeit (True Wind) (kn/h)

  // GPS-Daten (TinyGPS++)
  double gps_lat;    // Geografische Breite (WGS84)
  double gps_lon;    // Geografische Länge (WGS84)
  double gps_speed;  // SOG (Speed Over Ground) in kn/h
  double gps_kurs;   // COG (Course Over Ground) (°)
  int gps_sats;      // Anzahl empfangener Satelliten
  float gps_hdop;    // Horizontale Genauigkeit (Präzision)

  // GPS-Datum & Zeit (aus NMEA)
  int gps_jahr;     // Aktuelles Kalenderjahr
  int gps_monat;    // Aktueller Monat (1–12)
  int gps_tag;      // Aktueller Tag (1–31)
  int gps_stunde;   // UTC Stunde (0–23)
  int gps_minute;   // UTC Minute (0–59)
  int gps_sekunde;  // UTC Sekunde (0–59)

  // Magnetische Missweisung (WMM_Tinier)
  double missweisung;  // Ortsabhängige magnetische Deklination (°)

  // Echolot
  double Echolot;  // Wassertiefe unter dem Geber (m)

  // Lichtsteuerung Relais A-D
  int relay_a;  // Schaltzustand Relais A (0=AN, 1=AUS)
  int relay_b;  // Schaltzustand Relais B (0=AN, 1=AUS)
  int relay_c;  // Schaltzustand Relais C (0=AN, 1=AUS)
  int relay_d;  // Schaltzustand Relais D (0=AN, 1=AUS)

  // Mast-Kommunikationsstatus
  int mast_online;  // Netzwerkstatus Mast-Einheit (0=off, 1=on, 2=timeout)

  double sm_counter; // SM Zähler für Wegstrecke
};

// Globale Instanz zur Bereitstellung der Sensordaten im System
extern SensorData sensorData;



// ======================================================================
// Bezeichnung: AutopilotData
// Erklärung:   Zentrale Datenstruktur für die Autopilot-Steuerung.
//              Speichert die Zielkoordinaten, den aktuellen Betriebsmodus,
//              Benutzereingaben für den Kurs-Offset sowie Statusflags
//              für die manuelle Aktuator-Steuerung (Motor).
// ======================================================================
struct AutopilotData {
  double autopilot_lat;         // Ziel-Breitengrad (WGS84)
  double autopilot_lon;         // Ziel-Längengrad (WGS84)
  int modus_autopilot;          // Aktueller Steuerungsmodus (z.B. Standby, Track, Wind)
  int rotary_offset;            // Kurskorrektur über Inkrementalgeber (+/- 45°)
  bool motor_manuel_einfahren;  // Manueller Befehl: Linearantrieb einziehen
  bool motor_manuel_ausfahren;  // Manueller Befehl: Linearantrieb ausfahren
  uint32_t modus_counter;       // Sequenzzähler zur Validierung von Benutzerinteraktionen
};

// Globale Instanz zur Steuerung des Pinnenautopilots
extern AutopilotData pinnenautopilotData;


// ======================================================================
// Bezeichnung: AlarmData
// Erklärung:   Struktur zur Speicherung der Alarm-Konfiguration.
//              Enthält Aktivierungsschalter (Booleans) und die
//              zugehörigen Schwellenwerte (Floats) für die Überwachung
//              von Geschwindigkeit, Winddaten, Kursabweichungen,
//              Ankerwache sowie die Schiffslage (Roll & Pitch).
// ======================================================================
struct AlarmData {
  int alarm;             // Alarmausgelöst true false
  bool en_speed_max;     // Max. Geschwindigkeit aktiviert
  float speedMax;        // Schwellenwert Max. Geschwindigkeit
  bool en_speed_min;     // Min. Geschwindigkeit aktiviert
  float speedMin;        // Schwellenwert Min. Geschwindigkeit
  bool en_echolot;       // Echolot Alarm aktiv
  float echolotMin;      // minimale Wassertiefe
  bool en_wind_max;      // Max. Windgeschwindigkeit aktiviert
  float windMax;         // Schwellenwert Max. Windgeschwindigkeit
  bool en_wind_min;      // Min. Windgeschwindigkeit aktiviert
  float windMin;         // Schwellenwert Min. Windgeschwindigkeit
  bool en_wind_dir;      // Windrichtungswarnung aktiviert
  float windDir;         // Toleranzbereich Windrichtung
  float windDir_temp;    // Windrichtung zur Überwachung
  bool en_course_dev;    // Kursabweichung (XTE) aktiviert
  float courseDev;       // Max. Kursabweichung in Grad/Metern
  float courseDev_temp;  // Steuerkurs zur Überwachung
  bool en_anchor;        // Ankerwache aktiviert
  float anchorRadius;    // Radius für Ankerwache
  float anchorkette;     // Länge der Ankerkette
  double anker_lat;      // Ankerpunkt Lat
  double anker_lon;      // Ankerpunkt Lon
  bool en_roll;          // Krängungswarnung (Roll) aktiviert
  float roll;            // Max. Winkel Roll
  bool en_pitch;         // Stampfwarnung (Pitch) aktiviert
  float pitch;           // Max. Winkel Pitch
};

// Globale Instanz für den Zugriff im gesamten Projekt
extern AlarmData alarmData;
