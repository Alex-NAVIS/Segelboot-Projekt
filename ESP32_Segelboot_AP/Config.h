// ==========================================================
// Config.h – zentrale Systemkonfiguration
// ----------------------------------------------------------
// Alle globalen Einstellungen für das BootSensorController-System
// ==========================================================


#pragma once

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
#define DEBUG_MODE_SD false
#define DEBUG_MODE_SERVER false
#define DEBUG_MODE_GPS false
#define DEBUG_MODE_Tile true


// ==========================================================
// 2. WLAN Access Point Einstellungen
// ==========================================================
extern String AP_SSID;       // UMBAU ALS VARIABLE ZUM SPEICHERN
extern String AP_PASSWORD;   // UMBAU ALS VARIABLE ZUM SPEICHERN


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
#define GPS_PREDICT_S        1.0f   // wie viele Sekunden in die Zukunft prognostizieren
#define GPS_MIN_VALID_SPEED  0.13f  // kn — unterhalb: als "stillstehend" behandeln
#define GPS_HDOP_MAX_WEIGHT  20.0f  // HDOP höheren als das geben sehr kleinen weight

// GPS AHRS true Stabilisierung false reine GPS Daten
#define GPS_AHRS_SYSTEM true

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
#define STEPPER_STEP_PIN 18
#define STEPPER_DIR_PIN 19  // ⚠️ doppelt mit RELAY_A_PIN? prüfen!

// Endschalter
#define END_LEFT_PIN 27
#define END_RIGHT_PIN 14  // ⚠️ doppelt mit ENCODER_PIN → unbedingt klären!


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

// ==========================================================
// ENDE
// ==========================================================
