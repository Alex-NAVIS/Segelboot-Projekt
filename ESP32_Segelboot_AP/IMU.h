/*
  IMU.h – Schnittstelle für ICM-20948

  Diese Header-Datei definiert die öffentlichen Funktionen der IMU:
  - setup_imu(): Initialisierung + Laden der Kalibrierungsdaten
  - read_imu(): Rohdaten lesen, Roll/Pitch/Heading berechnen
  - calibrate_mpu(): Gyroskop-Kalibrierung starten
  - load_calibration(): Gyroskop-Kalibrierungsdaten aus LittleFS laden
  - calibrate_magnetometer(): Magnetometer-Kalibrierung starten (z.B. 30 Sekunden drehen)
  - load_mag_calibration(): Magnetometer-Kalibrierungsdaten aus LittleFS laden

  Feature Flags:
  - MAG_ENABLED: Aktiviert das Magnetometer (Heading-Berechnung)

  Die Header-Datei enthält keine internen Variablen oder Sensorlogik,
  sondern nur die Schnittstelle für andere Module.
*/


#ifndef IMU_H
#define IMU_H

#include <Arduino.h>
#include <MadgwickAHRS.h>
#include <Wire.h>
#include <ArduinoJson.h>
#include <LittleFS.h>
#include "ICM_20948.h"
#include "Config.h"
#include "Sensor_Data.h"  // Zugriff auf die globale Sensorstruktur


// --- Skalierungswerte ---
extern float MAG_SCALE; // µT pro LSB

// --- veränderbare Hard-Iron Offsets ---
extern float mag_offset_x;
extern float mag_offset_y;
extern float mag_offset_z;

// --- veränderbare Gyro Offsets ---
extern float gyro_offset_x;
extern float gyro_offset_y;
extern float gyro_offset_z;


// --- Funktionen ---
void setup_imu(TwoWire &wirePort = Wire);
void read_imu();
void calibrate_gyro();
void calibrate_magnetometer(int durationSeconds);

#endif

