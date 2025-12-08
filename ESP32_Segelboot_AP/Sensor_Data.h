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

// ----------------------------------------------------------
// Struktur: SensorData
// ----------------------------------------------------------
// Enthält alle gemessenen und berechneten Werte des Systems.
// Alle Werte sind global über "sensorData" zugänglich.
// ----------------------------------------------------------
struct SensorData {
  // --------------------------------------------------------
  // Lage- / Bewegungssensor (MPU9250)
  // --------------------------------------------------------
  double roll;     // Rollwinkel des Bootes (°)
  double pitch;    // Nickwinkel des Bootes (°)
  double kompass;  // Magnetischer Kurs (° 0–360)

  // --------------------------------------------------------
  // Windsensoren (AS5600 & Encoder)
  // --------------------------------------------------------
  double winddir_gemessen;     // Momentane Windrichtung relativ zum Boot (°)
  double windspeed_gemessen;   // Momentane Windgeschwindigkeit (kn/h)
  double winddir_berechnet;    // Berechnete wahre Windrichtung (°)
  double windspeed_berechnet;  // Berechnete wahre Windgeschwindigkeit (kn/h)

  // --------------------------------------------------------
  // GPS-Daten (TinyGPS++)
  // --------------------------------------------------------
  double gps_lat;    // Geografische Breite (°)
  double gps_lon;    // Geografische Länge (°)
  double gps_speed;  // Geschwindigkeit über Grund (kn/h)
  double gps_kurs;   // Kurs über Grund (°)
  int gps_sats;      // GPS Satanzahl
  float gps_hdop;    // gps hdop Fehlerwert

  // --------------------------------------------------------
  // GPS-Datum & Zeit (aus NMEA)
  // --------------------------------------------------------
  int gps_jahr;     // Jahr (z. B. 2025)
  int gps_monat;    // Monat (1–12)
  int gps_tag;      // Tag (1–31)
  int gps_stunde;   // Stunde (0–23)
  int gps_minute;   // Minute (0–59)
  int gps_sekunde;  // Sekunde (0–59)

  // --------------------------------------------------------
  // Magnetische Missweisung (Berechnet aus WMM_Tinier)
  // --------------------------------------------------------
  double missweisung;  // Deklination: Differenz zwischen magnetisch & geografisch Nord (°)
                       // Positiv = östlich, negativ = westlich

  // --------------------------------------------------------
  // Echolot gemessen mit Sensor im Rumpf
  // --------------------------------------------------------
  double Echolot;  // in Meter

  // --------------------------------------------------------
  // Lichtsteuerung Relay A B C D
  // --------------------------------------------------------
  int relay_a;  // 0 on 1 off
  int relay_b;  // 0 on 1 off
  int relay_c;  // 0 on 1 off
  int relay_d;  // 0 on 1 off

  //Mastdata Online offline timeout
  int mast_online;  // 0 = offline 1 = online 2 = timeout
};

// ----------------------------------------------------------
// Globale Instanz, von allen Modulen verwendbar
// ----------------------------------------------------------
extern SensorData sensorData;



// -----------------------------------------------------------------------------------
// Globale Instanz, von allen Modulen verwendbar Autopilot Pinnenautopilot Variablen
// -----------------------------------------------------------------------------------

//PID Werte für den Autopilot Pinnenautopilot für glatte; mäßig und raue See
struct AutopilotData {
  // --------------------------------------------------------
  // PID Regler bei glatter See
  // --------------------------------------------------------
  double P_welle_1;
  double I_welle_1;
  double D_welle_1;
  // --------------------------------------------------------
  // PID Regler bei mäßiger See
  // --------------------------------------------------------
  double P_welle_2;
  double I_welle_2;
  double D_welle_2;
  // --------------------------------------------------------
  // PID Regler bei rauer See
  // --------------------------------------------------------
  double P_welle_3;
  double I_welle_3;
  double D_welle_3;

  //Inverter der Pinnenbewegung
  int pinne_invertieren;

  //Autopilot Zielkoordinaten
  double autopilot_lat;  // Geografische Breite (°) Ziel Autopilot
  double autopilot_lon;  // Geografische Länge (°) Ziel Autopilot
};

// ----------------------------------------------------------
// Globale Instanz, von allen Modulen verwendbar
// ----------------------------------------------------------
extern AutopilotData pinnenautopilotData;

// ==========================================================
// Status Autopilot
// 0 = keine Funktion / Suche WLAN
// 1 = WLAN verbunden
// 2 = Regelung ON Kompassmodus
// 3 = GPS-Kursmodus
// 4 = Windsteuerung
// 5 = Fehler im AP
// ==========================================================
extern int status_autopilot;
// ==========================================================
// Rotary Encoder
// ==========================================================
extern int rotary_offset;  // Sollkurs Offset ±45°

// ==========================================================
// Display-Modus für Rotary Encoder PID Control
// ==========================================================
// 1 = kleine Wellen
// 2 = mäßige Wellen
// 3 = große Wellen
extern int PID_wahl;


// ==========================================================
// Soll Steuerkurs (vom Autopilot gehalten)
// ==========================================================
extern double soll_kurs;  // aktuell eingestellter Sollkurs

// ==========================================================
// Schrittmotor / NEMA23 Steuerung
// Motorsteuerung
// ==========================================================
extern bool motor_manuel_einfahren;  // manuelle Steuerung EINFAHREN
extern bool motor_manuel_ausfahren;  // manuelle Steuerung AUSFAHREN

extern volatile int motor_richtung;  // Richtung (0 / 1)
extern bool motor_hauptschalter;     // Motorstatus aktiv / inaktiv

extern bool motor_aktiv;  // Motorstatus aktiv / inaktiv

extern volatile long motor_position;  // aktuelle Position in Schritten

extern bool end_links_aktiv;   // linker Endschalter
extern bool end_rechts_aktiv;  // rechter Endschalter

extern uint32_t stepThreshold;  // Schrittfrequenz-Teiler für ISR
