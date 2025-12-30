// ==========================================================
// Tide Map – Gezeitenberechnung aus Stations-Tiles
// (CSV-Tiles auf SD-Karte, 0.5° Raster)
// ==========================================================

#pragma once
#include <Arduino.h>
#include <SD.h>
#include <time.h>
#include <vector>
#include <map>

// ==========================================================
// Konfiguration
// ==========================================================

// Ordner mit Tide-Tiles auf der SD-Karte
#define TIDE_TILE_FOLDER "/tide"

// Tile-Größe in Grad (muss zum Python-Skript passen!)
#define TIDE_TILE_SIZE_DEG 0.5

// Maximale Anzahl berücksichtigter Stationen
#define TIDE_MAX_NEAREST_STATIONS 8

// Suchparameter für Extremwertsuche
#define TIDE_SEARCH_STEP_SECONDS 300  // 5 Minuten Raster
#define TIDE_SEARCH_WINDOW_HOURS 36   // +/- 36h Suchfenster

// ==========================================================
// Öffentliche Ergebniswerte (globale Bootsdaten)
// ==========================================================

// Nächstes Hochwasser
extern time_t tide_next_high_time;
extern double tide_next_high_height;

// Nächstes Niedrigwasser
extern time_t tide_next_low_time;
extern double tide_next_low_height;

// Aktueller Wasserstand
extern double tide_height_now;

// Qualitätsmetrik der Berechnung (0 .. 100)
extern int tide_quality;

// ==========================================================
// Status der Tide-Berechnung
// ==========================================================
enum TideStatus {
  TIDE_OK = 0,
  TIDE_NO_TILES,          // Keine Stationsdaten in Umgebung
  TIDE_NO_CONSTITUENTS    // Stationsdaten vorhanden, aber nicht interpolierbar
};

// Letzter Status der Berechnung
extern TideStatus tide_status;

// ==========================================================
// API
// ==========================================================

// Initialisierung (z. B. SD-Check, interne Tabellen)
void tide_map_init();

// Hauptabfrage:
//  lat / lon  : aktuelle Bootsposition
//  now        : aktuelle Zeit (Unix time_t, UTC)
//  Rückgabe   : true = gültige Tidewerte berechnet
bool tide_query(double lat, double lon, time_t now);
