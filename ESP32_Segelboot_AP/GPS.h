// ==========================================================
// GPS_Nema.h
// ----------------------------------------------------------
// Modul zur Auswertung eines NMEA-GPS-Empfängers (z. B. u-blox, NEO-6M).
// Liest die NMEA-Daten über UART (Serial1) und schreibt die Werte
// in die globale Struktur "sensorData".
//
// Dieses Header-File deklariert die Funktionen für:
//   - setupGPS():   Initialisierung des GPS-Empfängers
//   - readGPS():    Regelmäßiges Einlesen und Parsen der NMEA-Daten
//
// ----------------------------------------------------------
// Anschlusspins (ESP32):
//   GPS_RX_PIN (GPIO 16) ← TX des GPS-Moduls  → Empfangsleitung ESP32
//   GPS_TX_PIN (GPIO 17) → RX des GPS-Moduls  → optional, nur für Senden an GPS nötig
//   Baudrate: GPS_BAUDRATE (Standard: 9600)
//
// Stromversorgung: 3.3V (oder 5V je nach GPS-Modul) + GND
//
// ASCII-Schaltplan (vereinfachte Übersicht):
//
//        +----------------------+
//        |      GPS-Modul       |
//        |   (z. B. NEO-6M)     |
//        |                      |
//        |  TX ---------------> GPIO16 (GPS_RX_PIN)
//        |  RX <--------------- GPIO17 (GPS_TX_PIN, optional)
//        |  VCC --------------> 3.3V
//        |  GND --------------> GND
//        +----------------------+
//
// ----------------------------------------------------------
// Dieses Modul nutzt die Library "TinyGPSPlus" zum Parsen der NMEA-Daten.
// ==========================================================

#pragma once
#include <Arduino.h>
#include "Config.h"       // Enthält Pinbelegung und Baudrate für GPS
#include "Sensor_Data.h"  // Zugriff auf die globale Sensorstruktur
#include <TinyGPSPlus.h>  // GPS-NMEA-Parser-Bibliothek
#include <time.h>
#include <sys/time.h>

// ----------------------------------------------------------
// Funktionsprototypen
// ----------------------------------------------------------

// setupGPS():
//   - Wird einmalig im setup() des Hauptprogramms aufgerufen.
//   - Initialisiert die serielle Schnittstelle (Serial1 oder definierter GPS_SERIAL_PORT)
//     mit den Pins (GPS_RX_PIN, GPS_TX_PIN) und der Baudrate aus Config.h.
//   - Aktiviert optional Debug-Ausgaben, wenn DEBUG_MODE = true.
//   - Bereitet das GPS-Modul für das kontinuierliche Empfangen von NMEA-Daten vor.
void setupGPS();

// readGPS():
//   - Wird regelmäßig im loop() des Hauptprogramms aufgerufen.
//   - Liest alle verfügbaren Bytes vom GPS über die serielle Schnittstelle.
//   - Übergibt die NMEA-Daten an TinyGPSPlus zur Dekodierung.
//   - Aktualisiert die globalen Sensorwerte (sensorData) für:
//       - Breite & Länge (gps_lat, gps_lon)
//       - Geschwindigkeit (gps_speed)
//       - Kurs (gps_kurs)
//   - Ruft updateLocalTime() auf, wenn gültige Datum- & Zeitinformationen vorliegen,
//     um die lokale Zeit basierend auf der GPS-Position zu berechnen.
void readGPS();

// updateLocalTime(...):
//   - Berechnet aus UTC-Datum und -Uhrzeit die lokale Zeit anhand der GPS-Koordinaten.
//   - Berücksichtigt Zeitzonenverschiebung nach Längengrad.
//   - Prüft auf Mitteleuropäische Sommerzeit (DST) und passt Stunden ggf. an.
//   - Trägt die berechnete lokale Zeit in sensorData ein:
//       - gps_jahr, gps_monat, gps_tag
//       - gps_stunde, gps_minute, gps_sekunde
void updateLocalTime(int utcYear, int utcMonth, int utcDay, int utcHour, int utcMinute, int utcSecond);

// isDST(...):
//   - Prüft, ob ein bestimmtes Datum in der Mitteleuropäischen Sommerzeit liegt.
//   - Liefert true zurück, wenn Sommerzeit gilt, sonst false.
//   - Wird von updateLocalTime() verwendet, um Sommerzeitkorrektur zu berücksichtigen.
bool isDST(int year, int month, int day);

// setzt die Systemzeit (UTC) aus GPS-Daten
void setSystemTimeFromGPS(int year, int month, int day, int hour, int minute, int second);

// gps_ahrs():
//   - Berechnet interpolierte GPS-Koordinaten basierend auf den im Ringpuffer
//     gespeicherten letzten GPS-Werten.
//   - Prüft automatisch auf Ausreißer (z. B. fehlerhafte Positionsdaten) und ersetzt
//     diese durch Mittelwerte.
//   - Korrigiert Sprünge im Kurswinkel (0°/360°) für stabile Dead-Reckoning-Berechnungen.
//   - Nutzt die Geschwindigkeit (gps_speed) und den Kurs (gps_kurs) der letzten Punkte,
//     um neue Positionen zwischen GPS-Updates zu schätzen (z. B. 20 Hz).
//   - Berechnet gleitende Mittelwerte über die letzten 3–5 interpolierten Punkte,
//     um die Positionswerte weiter zu stabilisieren.
//   - Aktualisiert die globale Sensorstruktur sensorData mit:
//       - gps_lat, gps_lon: interpolierte Position
//   - Kann zyklisch im Hauptprogramm (loop) aufgerufen werden, um zwischen GPS-Empfängen
//     flüssige Positionsdaten zu liefern.
void gps_ahrs();