#include <Arduino.h>  // für String, bool etc.
#include <cstdint>    // für uint32_t
#include "Config.h"


// ==========================================================
// 0. Funktionen
// ==========================================================
// kleine Hilfsfunktion: Schallgeschwindigkeit berechnen (gleich wie in echolot)
void calc_schallgeschwindigkeit() {
  // UNESCO-ähnliche Näherung (T in °C, S in PSU)
  Schallgeschwindigkeit_Wasser = 1449.2
                                 + 4.6 * Wassertemperatur
                                 - 0.055 * Wassertemperatur * Wassertemperatur
                                 + 0.00029 * Wassertemperatur * Wassertemperatur * Wassertemperatur
                                 + (1.34 - 0.01 * Wassertemperatur) * (Salzgehalt - 35.0);
}

// Neu berechnen der Intervall-Millis (wenn Hz geändert wurden)
void recalc_intervals() {
  GPS_UPDATE_INTERVAL_MS = 1000.0f / GPS_UPDATE_HZ;
  GPS_AHRS_UPDATE_INTERVAL_MS = 1000.0f / GPS_AHRS_UPDATE_HZ;
  MPU_UPDATE_INTERVAL_MS = 1000.0f / MPU_UPDATE_HZ;
  WINDDIR_UPDATE_INTERVAL_MS = 1000.0f / WINDDIR_UPDATE_HZ;
  WINDSPEED_UPDATE_INTERVAL_MS = 1000.0f / WINDSPEED_UPDATE_HZ;
  LICHTSCHALTER_UPDATE_INTERVAL_MS = 1000.0f / LICHT_UPDATE_HZ;
  LOG_UPDATE_INTERVAL_MS = 1000.0f * 60 * SD_UPDATE_HZ;
  SENSOR_UPDATE_INTERVAL_MS = 1000.0f / SENSOR_UPDATE_HZ;
}
// ==========================================================
// 2. WLAN Access Point Einstellungen
// ==========================================================
String AP_SSID = "Mein_Boot";
String AP_PASSWORD = "12345678";

// ==========================================================
// 5. SD-Karte Logging Intervall
// ==========================================================
uint32_t LOG_UPDATE_INTERVAL_SEKUNDEN = 60 * 10;

// ==========================================================
// 7. Sensor-Timings (Hz)
// ==========================================================
float GPS_UPDATE_HZ = 1.0f;
float GPS_AHRS_UPDATE_HZ = 5.0f;
float MPU_UPDATE_HZ = 15.0f;
float WINDDIR_UPDATE_HZ = 2.0f;
float WINDSPEED_UPDATE_HZ = 1.0f;
float LICHT_UPDATE_HZ = 3.0f;
float SD_UPDATE_HZ = 1.0f;
float SENSOR_UPDATE_HZ = 20.0f;

// ==========================================================
// 8. Berechnete Intervalle (ms)
// ==========================================================
float GPS_UPDATE_INTERVAL_MS = 1000.0f / GPS_UPDATE_HZ;
float GPS_AHRS_UPDATE_INTERVAL_MS = 1000.0f / GPS_AHRS_UPDATE_HZ;
float MPU_UPDATE_INTERVAL_MS = 1000.0f / MPU_UPDATE_HZ;
float WINDDIR_UPDATE_INTERVAL_MS = 1000.0f / WINDDIR_UPDATE_HZ;
float WINDSPEED_UPDATE_INTERVAL_MS = 1000.0f / WINDSPEED_UPDATE_HZ;
float LICHTSCHALTER_UPDATE_INTERVAL_MS = 1000.0f / LICHT_UPDATE_HZ;
float LOG_UPDATE_INTERVAL_MS = 1000.0f * 60 * SD_UPDATE_HZ;
float SENSOR_UPDATE_INTERVAL_MS = 1000.0f / SENSOR_UPDATE_HZ;

// ==========================================================
// 9. Navigation / Missweisung
// ==========================================================
bool USE_DECLINATION_FOR_COMPASS = true;
bool USE_DECLINATION_FOR_WIND = true;

// =========================
// 11. Echolot / Boot
// =========================
double Wassertiefe_Einbau = 0.45;  // Meter: Sensoroberkante bis Kiel
double Tiefgang_Boot = 1.15;       // Meter: Bootstiefgang

// =========================
// Schallgeschwindigkeit
// =========================
double Wassertemperatur = 20.0;                // °C
double Salzgehalt = 35.0;                      // ‰
double Schallgeschwindigkeit_Wasser = 1520.0;  // initial, wird berechnet