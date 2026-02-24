#include "ConfigStorage.h"

// Pfad für die Konfigurationsdatei auf LittleFS
// Datei-Pfade
const char* FILE_AUTOPILOT = "/settings/autopilot_config.json";
const char* FILE_SYSTEM = "/settings/system_config.json";

// ----------------------------------------------------------
// Initialisierung: mounten + laden
// ----------------------------------------------------------
void ConfigStorage_begin() {
  if (!LittleFS.begin()) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Mounten von LittleFS!");
    return;
  }

  if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("✅ LittleFS erfolgreich gemountet.");

  // beide Konfigurationsbereiche laden
  ConfigStorage_loadSystem();
  ConfigStorage_loadAutopilot();
}

// ==========================================================
// AUTOPILOT SPEICHERN / LADEN (nur Basis-PID)
// ==========================================================
void ConfigStorage_saveAutopilot() {
  File file = LittleFS.open(FILE_AUTOPILOT, "w");
  if (!file) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Öffnen Autopilot-Config!");
    return;
  }
  StaticJsonDocument<512> doc;
  doc["pinne_invertieren"] = pinnenautopilotData.pinne_invertieren;
  doc["P_base"] = pinnenautopilotData.P_base;
  doc["I_base"] = pinnenautopilotData.I_base;
  doc["D_base"] = pinnenautopilotData.D_base;
  doc["lat"] = pinnenautopilotData.autopilot_lat;
  doc["lon"] = pinnenautopilotData.autopilot_lon;
  serializeJson(doc, file);
  file.close();
  if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("💾 Autopilot-Basis-PID gespeichert.");
}

void ConfigStorage_loadAutopilot() {
  if (!LittleFS.exists(FILE_AUTOPILOT)) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("⚠️ Keine Autopilot-Konfiguration gefunden.");
    return;
  }

  File file = LittleFS.open(FILE_AUTOPILOT, "r");
  if (!file) return;

  StaticJsonDocument<512> doc;

  if (deserializeJson(doc, file)) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Laden Autopilot JSON.");
    file.close();
    return;
  }
  file.close();
  pinnenautopilotData.pinne_invertieren = doc["pinne_invertieren"] | 0;
  pinnenautopilotData.P_base = doc["P_base"] | 0.0;
  pinnenautopilotData.I_base = doc["I_base"] | 0.0;
  pinnenautopilotData.D_base = doc["D_base"] | 0.0;
  pinnenautopilotData.autopilot_lat = doc["lat"] | 0.0;
  pinnenautopilotData.autopilot_lon = doc["lon"] | 0.0;

  if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("📥 Autopilot-Basis-PID geladen.");
}


// ==========================================================
// SYSTEM SPEICHERN / LADEN
// ==========================================================

// ----------------------------------------------------------
// SYSTEM speichern
// ----------------------------------------------------------
void ConfigStorage_saveSystem() {
  File file = LittleFS.open(FILE_SYSTEM, "w");
  if (!file) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Öffnen der Datei zum Speichern (System)!");
    return;
  }

  StaticJsonDocument<1024> doc;

  // Strings (Arduino String)
  doc["ap_ssid"] = AP_SSID;
  doc["ap_pass"] = AP_PASSWORD;
  // numerische Einstellungen
  doc["log_interval_sec"] = LOG_UPDATE_INTERVAL_SEKUNDEN;
  doc["gps_hz"] = GPS_UPDATE_HZ;
  doc["gps_ahrs_hz"] = GPS_AHRS_UPDATE_HZ;
  doc["mpu_hz"] = MPU_UPDATE_HZ;
  doc["winddir_hz"] = WINDDIR_UPDATE_HZ;
  doc["windspeed_hz"] = WINDSPEED_UPDATE_HZ;
  doc["licht_hz"] = LICHT_UPDATE_HZ;
  doc["sd_hz"] = SD_UPDATE_HZ;
  doc["sensor_update_hz"] = SENSOR_UPDATE_HZ;
  doc["use_decl_compass"] = USE_DECLINATION_FOR_COMPASS;
  doc["use_decl_wind"] = USE_DECLINATION_FOR_WIND;
  // Echolot / Boot
  doc["wassertiefe_einbau"] = Wassertiefe_Einbau;
  doc["tiefgang_boot"] = Tiefgang_Boot;
  // Wasser-Parameter
  doc["wassertemp"] = Wassertemperatur;
  doc["salz"] = Salzgehalt;

  if (serializeJson(doc, file) == 0) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Schreiben der System-Konfiguration!");
  } else {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("💾 System-Konfiguration gespeichert.");
  }
  file.close();
}

// ----------------------------------------------------------
// SYSTEM laden
// ----------------------------------------------------------
void ConfigStorage_loadSystem() {
  if (!LittleFS.exists(FILE_SYSTEM)) {
    Serial.println("⚠️ Keine System-Konfiguration gefunden.");
    return;
  }

  File file = LittleFS.open(FILE_SYSTEM, "r");
  if (!file) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("❌ Fehler beim Öffnen der System-Datei!");
    return;
  }

  StaticJsonDocument<1024> doc;
  DeserializationError err = deserializeJson(doc, file);
  file.close();

  if (err) {
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.print("❌ Fehler beim Parsen der System-Konfiguration: ");
    if (DEBUG_MODE_CONFIGSTORAGE) Serial.println(err.c_str());
    return;
  }

  // Strings
  AP_SSID = doc["ap_ssid"] | AP_SSID;
  AP_PASSWORD = doc["ap_pass"] | AP_PASSWORD;

  // Zahlen / Raten
  LOG_UPDATE_INTERVAL_SEKUNDEN = doc["log_interval_sec"] | LOG_UPDATE_INTERVAL_SEKUNDEN;

  GPS_UPDATE_HZ = doc["gps_hz"] | GPS_UPDATE_HZ;
  GPS_AHRS_UPDATE_HZ = doc["gps_ahrs_hz"] | GPS_AHRS_UPDATE_HZ;
  MPU_UPDATE_HZ = doc["mpu_hz"] | MPU_UPDATE_HZ;
  WINDDIR_UPDATE_HZ = doc["winddir_hz"] | WINDDIR_UPDATE_HZ;
  WINDSPEED_UPDATE_HZ = doc["windspeed_hz"] | WINDSPEED_UPDATE_HZ;
  LICHT_UPDATE_HZ = doc["licht_hz"] | LICHT_UPDATE_HZ;
  SD_UPDATE_HZ = doc["sd_hz"] | SD_UPDATE_HZ;
  SENSOR_UPDATE_HZ = doc["sensor_update_hz"] | SENSOR_UPDATE_HZ;

  USE_DECLINATION_FOR_COMPASS = doc["use_decl_compass"] | USE_DECLINATION_FOR_COMPASS;
  USE_DECLINATION_FOR_WIND = doc["use_decl_wind"] | USE_DECLINATION_FOR_WIND;

  // Echolot / Boot
  Wassertiefe_Einbau = doc["wassertiefe_einbau"] | Wassertiefe_Einbau;
  Tiefgang_Boot = doc["tiefgang_boot"] | Tiefgang_Boot;

  // Wasser-Parameter
  Wassertemperatur = doc["wassertemp"] | Wassertemperatur;
  Salzgehalt = doc["salz"] | Salzgehalt;

  // nach dem Laden neu berechnen
  calc_schallgeschwindigkeit();
  recalc_intervals();

  if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("📥 System-Konfiguration geladen.");
}