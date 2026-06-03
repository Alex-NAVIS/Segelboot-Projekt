// ==========================================================
// Config.h – zentrale Systemkonfiguration
// ----------------------------------------------------------
// Alle globalen Einstellungen für das BootSensorController-System
// ==========================================================
#pragma once

#include <Arduino.h>  // für String, uint8_t etc.
#include <cstdint>    // für uint32_t

// ==========================================================
// 0. Funktionen in Einstellungen
// ==========================================================
// Schallgeschwindigkeit berechnen
void calc_schallgeschwindigkeit();
void recalc_intervals();

// ==========================================================
// 1. Debug-Einstellungen
// ==========================================================
#define DEBUG_MODE false
#define DEBUG_MODE_CONFIGSTORAGE false
#define DEBUG_MODE_IMU false
#define DEBUG_MODE_SD true
#define DEBUG_MODE_SERVER false
#define DEBUG_MODE_GPS false
#define DEBUG_MODE_Tile false
#define DEBUG_MODE_Autopilot false
#define DEBUG_MODE_CANBUS false
#define DEBUG_MODE_RS false
#define DEBUG_MODE_NET true


// ==========================================================
// 2. WLAN Access Point Einstellungen
// ==========================================================
extern String AP_SSID;      // UMBAU ALS VARIABLE ZUM SPEICHERN
extern String AP_PASSWORD;  // UMBAU ALS VARIABLE ZUM SPEICHERN


// ==========================================================
// 3. GPS-Konfiguration (UART)
// ==========================================================
#define GPS_SERIAL_PORT Serial1
#define GPS_BAUDRATE 9600

// GPS Pins
#define GPS_RX_PIN 16  // GPS → ESP32
#define GPS_TX_PIN 17  // ESP32 → GPS (selten genutzt)

//GPS AHRS System Vektor Filter
// Vorhersage/Filter-Einstellungen (in Sekunden)
#define GPS_PREDICT_S 1.0f         // wie viele Sekunden in die Zukunft prognostizieren
#define GPS_MIN_VALID_SPEED 0.13f  // kn — unterhalb: als "stillstehend" behandeln
#define GPS_HDOP_MAX_WEIGHT 20.0f  // HDOP höheren als das geben sehr kleinen weight

// GPS AHRS true Stabilisierung false reine GPS Daten
#define GPS_AHRS_SYSTEM true

#define LOG_INTERVAL_TICKS 60     // Anzahl GPS Updates
#define LOG_MIN_DISTANCE_SM 0.04  // Mindestdistanz in sm (einstellbar)≈ 75 m

// ==========================================================
// 4. I2C Pins
// ==========================================================
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9


// ==========================================================
// 5. SPI / SD-Karte Konfiguration
// ==========================================================
// SD per SPI
#define SD_CS_PIN 10
#define SPI_MOSI 11
#define SPI_MISO 13
#define SPI_SCK 12

// LOG Intervall in Sekunden (10 Minuten)
extern uint32_t LOG_UPDATE_INTERVAL_SEKUNDEN;


// ==========================================================
// 6. GPIO-Pinbelegung – Sensoren, Aktoren, Hardware
// ==========================================================
// Relais
#define RELAY_A_PIN 19  // Toplicht
#define RELAY_B_PIN 20  // Positionslicht
#define RELAY_C_PIN 21  // Licht DS
#define RELAY_D_PIN 47  // Cockpitlicht

// Rotary Encoder (Windgeschwindigkeit)
#define ENCODER_PIN 14

// Stepper Motor (NEMA)
#define STEPPER_STEP_PIN 35
#define STEPPER_DIR_PIN 36

// Endschalter
#define END_LEFT_PIN 37
#define END_RIGHT_PIN 38


// ==========================================================
// 7. Sensor-Timings (Hz)
// ==========================================================
extern float GPS_UPDATE_HZ;
extern float GPS_AHRS_UPDATE_HZ;
extern float MPU_UPDATE_HZ;
extern float WINDDIR_UPDATE_HZ;
extern float WINDSPEED_UPDATE_HZ;
extern float LICHT_UPDATE_HZ;
extern float SD_UPDATE_HZ;
extern float SENSOR_UPDATE_HZ;


// ==========================================================
// 8. Berechnete Intervalle (ms)
// ==========================================================
extern float GPS_UPDATE_INTERVAL_MS;
extern float GPS_AHRS_UPDATE_INTERVAL_MS;
extern float MPU_UPDATE_INTERVAL_MS;
extern float WINDDIR_UPDATE_INTERVAL_MS;
extern float WINDSPEED_UPDATE_INTERVAL_MS;
extern float LICHTSCHALTER_UPDATE_INTERVAL_MS;
extern float LOG_UPDATE_INTERVAL_MS;
extern float SENSOR_UPDATE_INTERVAL_MS;
#define MASTCHECK_UPDATE_INTERVAL_MS 1000
#define TRACK_INTERVAL_MS 300000UL   // 5 Minuten

// ==========================================================
// 9. Navigation / Missweisung
// ==========================================================
extern bool USE_DECLINATION_FOR_COMPASS;
extern bool USE_DECLINATION_FOR_WIND;

// ==========================================================
// 10. Webserver / Maps
// ==========================================================
#define USE_SD_TILES true  // false = LittleFS, true = SD-Karte

// ==========================================================
// 11. Echolot
// ==========================================================
// Pin für Ultraschallsensor
#define PIN_ECHOLOTT_TRIGGER 42
#define PIN_ECHOLOTT_ECHO 40

// Sensor-/Bootwerte (anpassbar über Settings)
extern double Wassertiefe_Einbau;  // Abstand Sensoroberkante zum Kiel / Schwert
extern double Tiefgang_Boot;       // Bootstiefgang

// Einstellungen zur Berechnung der Schallgeschwindigkeit
extern double Wassertemperatur;  // °C, z.B. 20.0
extern double Salzgehalt;        // PSU / ‰, z.B. 35.0

// Berechnete Schallgeschwindigkeit Wasser (m/s)
extern double Schallgeschwindigkeit_Wasser;

// ==========================================================
// 12. Kalibrierung Gyro und Mag
// ==========================================================
extern bool gyro_start_cal;
extern bool mag_start_cal;
extern float roll_offset;
extern float pitch_offset;

// ==========================================================
// 13. Alarmzustand und Pin für Piper
// ==========================================================
#define BUZZER_PIN 41  // Digital Ausgang für Piezo

#define ALARM_STABLE_MS 2000

#define HYST_ECHOLOT 0.2f
#define HYST_SPEED 0.3f
#define HYST_WIND 0.5f
#define HYST_ANGLE 2.0f
#define HYST_ATT 1.0f

#define ALARM_NONE 0
#define ALARM_SPEED_MAX 1
#define ALARM_SPEED_MIN 2
#define ALARM_WIND_MAX 3
#define ALARM_WIND_MIN 4
#define ALARM_WIND_DIR 5
#define ALARM_COURSE_DEV 6
#define ALARM_ANCHOR 7
#define ALARM_ROLL 8
#define ALARM_PITCH 9
#define ALARM_ECHOLOT 10

#define ALARMTIME 1000

// ==========================================================
// 14. Externe Datenquellen
// ==========================================================
extern bool extern_gps_CAN;
extern bool extern_gps_RS;
extern bool extern_gps_UDP_tcp;
extern bool extern_wind_CAN;
extern bool extern_wind_RS;
extern bool extern_wind_UDP_tcp;
extern bool extern_echolot_CAN;
extern bool extern_echolot_RS;
extern bool extern_echolot_UDP_tcp;

// Hardware-Konfiguration für den ESP32 CAN-Controller (Tx/Rx Pins)
#define ESP32_CAN_TX_PIN GPIO_NUM_5
#define ESP32_CAN_RX_PIN GPIO_NUM_4
#define TIMEOUT_GPS     5000
#define TIMEOUT_WIND    5000
#define TIMEOUT_DEPTH   5000

#define PIN_RS422_RX  16  // Passe diesen Pin an dein ESP32-Board an
// ==========================================================
// ENDE
// ==========================================================
