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
  // IMU Kalibrierung
  doc["roll_offset"] = roll_offset;
  doc["pitch_offset"] = pitch_offset;
  // Datenquellen-Matrix
  doc["ext_gps_can"] = extern_gps_CAN;
  doc["ext_gps_rs"] = extern_gps_RS;
  doc["ext_gps_udp_tcp"] = extern_gps_UDP_tcp;
  doc["ext_wind_can"] = extern_wind_CAN;
  doc["ext_wind_rs"] = extern_wind_RS;
  doc["ext_wind_udp_tcp"] = extern_wind_UDP_tcp;
  doc["ext_depth_can"] = extern_echolot_CAN;
  doc["ext_depth_rs"] = extern_echolot_RS;
  doc["ext_depth_udp_tcp"] = extern_echolot_UDP_tcp;

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

  // IMU Kalibrierung
  roll_offset = doc["roll_offset"] | roll_offset;
  pitch_offset = doc["pitch_offset"] | pitch_offset;

  // Laden der Matrix-Werte mit sicherem Fallback auf den bestehenden RAM-Zustand
  extern_gps_CAN = doc["ext_gps_can"] | extern_gps_CAN;
  extern_gps_RS = doc["ext_gps_rs"] | extern_gps_RS;
  extern_gps_UDP_tcp = doc["ext_gps_udp_tcp"] | extern_gps_UDP_tcp;
  extern_wind_CAN = doc["ext_wind_can"] | extern_wind_CAN;
  extern_wind_RS = doc["ext_wind_rs"] | extern_wind_RS;
  extern_wind_UDP_tcp = doc["ext_wind_udp_tcp"] | extern_wind_UDP_tcp;
  extern_echolot_CAN = doc["ext_depth_can"] | extern_echolot_CAN;
  extern_echolot_RS = doc["ext_depth_rs"] | extern_echolot_RS;
  extern_echolot_UDP_tcp = doc["ext_depth_udp_tcp"] | extern_echolot_UDP_tcp;

  // nach dem Laden neu berechnen
  calc_schallgeschwindigkeit();
  recalc_intervals();

  if (DEBUG_MODE_CONFIGSTORAGE) Serial.println("📥 System-Konfiguration geladen.");
}