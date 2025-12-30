

#include "Config.h"               // Einstellungen Pin Belegung globale Einstellungen usw.
#include "ConfigStorage.h"        // Einstellungen Pin Belegung globale Einstellungen usw.
#include "WebServer_Module.h"     // Schnittstelle zu allen Webseiten Kartenplotter Horizont Kompass index.html usw.
#include "Sensor_Data.h"          // Variablen zu allen Sensordaten
#include "GPS.h"                  // GPS Sensor Empfänger Koordinate GPS Kurs & Speed
#include "Mag_Dec.h"              // Berechnung der Missweisung Kompass
#include "IMU.h"                  // I2C MPU9250 Roll Üitch Kompass
#include "LightControl_Module.h"  //Relays für Licht
#include "WB.h"                   //Relays für Licht
#include "NAVIS_SD.h"             // SD Card Module für Karten
#include "echolot.h"              // SD Card Module für Karten

// ----------------------------------------------------------
// Zeitvariablen für das millis()-basierte Sensor-Timing
// ----------------------------------------------------------
// Diese Variablen speichern den letzten Auslesezeitpunkt
// jedes Sensors. Dadurch kann der ESP32 jeden Sensor mit
// unterschiedlicher Frequenz abfragen – ganz ohne delay().
unsigned long lastGPSRead = 0;
int gpsreadcount = 300;
unsigned long lastgpsahrs = 0;
unsigned long lastMPURead = 0;
unsigned long lastWindSpeedRead = 0;
unsigned long lastMastupdate = 0;
unsigned long lastLichtRead = 0;
unsigned long lastsdwrite = 0;
int write_SD_log = 450;
unsigned long lastsensor = 0;

void setup() {
  Serial.begin(115200);   // Serial Port einrichten
  ConfigStorage_begin();  // Einrichtung des Little File System
  setupWiFiAP();          // WLAN Access Point starten
  setupWebServer();       // Webserver starten + Routen einrichten
  setupWebSocket();       // initialisiert ausschließlich WebSocket
  setupGPS();             // Initialisiert den GPS-Empfänger (UART, Pins 16/17)
  setupMagDeclination();  // Berechnung der Missweisung anhand der GPS Koordinate 1 mal am Tag
  setup_imu();            // Initialisiert den MPU9250 (I²C: SDA=21, SCL=22)
  setupLightControl();    // Relays für Licht ansteuern
  setup_sd();             // Zugriff auf die SD Karte einrichten
  setup_echolot();        // Echolot einrichten

  // Optionale Schutzfunktion:
  // Verhindert, dass alle Sensoren direkt nach dem Booten gleichzeitig
  // getriggert werden (reduziert Stromspitzen und I²C-Konflikte).
  lastGPSRead = millis();
  lastgpsahrs = millis() + 5;
  lastMPURead = millis() + 10;
  lastMastupdate = millis() + 20;
  lastWindSpeedRead = millis() + 30;
  lastLichtRead = millis() + 40;
  lastsensor = millis() + 50;

  if (DEBUG_MODE) Serial.println(F("=== Initialisierung abgeschlossen ==="));
}

// loop()
// ----------------------------------------------------------
// Der Haupt-Loop läuft endlos. Er ist vollständig nicht-blockierend.
// Jeder Sensor wird mit einem eigenen Zeitintervall (in ms) abgefragt,
// wie in Config.h definiert. Der Webserver läuft dabei parallel.
//
// Übersicht Sensorzyklen:
//   GPS:           0.24 Hz   (alle ~4,17 Sekunden)
//   MPU9250:      10.00 Hz   (alle 100 ms)
//   AS5600:        2.00 Hz   (alle 500 ms)
//   WindEncoder:   1.00 Hz   (alle 1 Sekunde)
// ==========================================================
void loop() {
  unsigned long currentMillis = millis();

  // --- GPS: 1 Hz (alle 1000 ms) ---
  if (currentMillis - lastGPSRead >= GPS_UPDATE_INTERVAL_MS) {
    lastGPSRead = currentMillis;
    readGPS();
    //Missweisung berechnen
    gpsreadcount = gpsreadcount + 1;
    if (gpsreadcount >= 600) {
      missweisung_berechnen();
      gpsreadcount = 0;
    }
  }

  // --- IMU ICM 20948 9 Achsen ACC GYRO MAG ---
  if (currentMillis - lastMPURead >= MPU_UPDATE_INTERVAL_MS) {
    lastMPURead = currentMillis;
    read_imu();
  }

  // --- Windgeschwindigkeit (Encoder): 1 Hz (alle 1 Sekunde) ---
  // Berechnung des waren Wind 1hz
  if (currentMillis - lastWindSpeedRead >= WINDSPEED_UPDATE_INTERVAL_MS) {
    lastWindSpeedRead = currentMillis;
    berechneWahrenWind();
  }


  //Sicherheitscheck für verbundenen Mastseensor.
  if (currentMillis - lastMastupdate >= MASTCHECK_UPDATE_INTERVAL_MS) {
    lastMastupdate = currentMillis;
    checkMastOnlineStatus();
  }

  //Server Lichtsteuerung 3 Hz (alle 333 ms) ---
  if (currentMillis - lastLichtRead >= LICHTSCHALTER_UPDATE_INTERVAL_MS) {
    lastLichtRead = currentMillis;
    syncRelaysIfChanged();  //Abfrage Relay lichtsteuerung
  }

  //Server Log schreiben alle 10 Minuten ---
  if (currentMillis - lastsdwrite >= LOG_UPDATE_INTERVAL_MS) {
    lastsdwrite = currentMillis;
    write_SD_log = write_SD_log + 1;
    if (write_SD_log >= LOG_UPDATE_INTERVAL_SEKUNDEN) {
      write_SD_log = 0;
      write_log();
    }
  }

  // Sensordaten liefern
  // Aufruf Sensordaten no polling Konstant 1000ms senden
  if (currentMillis - lastsensor >= SENSOR_UPDATE_INTERVAL_MS) {
    lastsensor = currentMillis;
    wsBroadcastSensorData();
  }

  //Kalibrierung Gyro und Mag
  if (gyro_start_cal == true) {
    gyro_start_cal = false;
    calibrate_gyro();
  }
  if (mag_start_cal == true) {
    mag_start_cal = false;
    calibrate_magnetometer(30);
  }
}
