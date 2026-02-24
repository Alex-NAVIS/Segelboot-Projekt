#include "tide.h"
#include <ArduinoJson.h>
#include <LittleFS.h>
#include <math.h>

#define EARTH_RADIUS_NM 3440.065

double tideQueryLat = 0.0;
double tideQueryLon = 0.0;
double tideQueryRadiusSM = 100.0;

TideStation tideStations[MAX_TIDE_STATIONS];
uint8_t tideStationCount = 0;

// Hilfsfunktionen
static inline double deg2rad(double deg) {
  return deg * M_PI / 180.0;
}

static double distanceNM(double lat1, double lon1, double lat2, double lon2) {
  double dLat = deg2rad(lat2 - lat1);
  double dLon = deg2rad(lon2 - lon1);

  double a =
    sin(dLat / 2) * sin(dLat / 2) + cos(deg2rad(lat1)) * cos(deg2rad(lat2)) * sin(dLon / 2) * sin(dLon / 2);

  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return EARTH_RADIUS_NM * c;
}

// Sortierfunktion (nach Distanz)
static void sortStationsByDistance() {
  for (uint8_t i = 0; i < tideStationCount - 1; i++) {
    for (uint8_t j = i + 1; j < tideStationCount; j++) {
      if (tideStations[j].distanceSM < tideStations[i].distanceSM) {
        TideStation tmp = tideStations[i];
        tideStations[i] = tideStations[j];
        tideStations[j] = tmp;
      }
    }
  }
}

// Zentrale Suchfunktion
bool findTideStations() {
  tideStationCount = 0;

  if (!LittleFS.exists(TIDE_STATION_FILE)) {
    if (DEBUG_MODE_Tile) Serial.println("[TIDE] stations.json nicht gefunden");
    return false;
  }

  File file = LittleFS.open(TIDE_STATION_FILE, "r");
  if (!file) {
    if (DEBUG_MODE_Tile) Serial.println("[TIDE] stations.json konnte nicht geöffnet werden");
    return false;
  }

  StaticJsonDocument<32768> doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();
  if (err) {
    if (DEBUG_MODE_Tile) Serial.println("[TIDE] JSON Fehler");
    return false;
  }

  JsonObject root = doc.as<JsonObject>();
  for (JsonPair kv : root) {
    const char* name = kv.key().c_str();
    double lat = kv.value()[0];
    double lon = kv.value()[1];
    double dist = distanceNM(
      tideQueryLat,
      tideQueryLon,
      lat,
      lon);
    if (dist <= tideQueryRadiusSM) {
      if (tideStationCount < MAX_TIDE_STATIONS) {
        tideStations[tideStationCount++] = {
          String(name),
          lat,
          lon,
          dist
        };
      }
    }
  }

  if (tideStationCount == 0) {
    if (DEBUG_MODE_Tile) Serial.println("[TIDE] Keine Stationen im Radius");
    return false;
  }

  sortStationsByDistance();
  if (DEBUG_MODE_Tile) Serial.printf("[TIDE] %d Stationen gefunden (nächste: %.1f sm)\n", tideStationCount, tideStations[0].distanceSM);

  return true;
}
