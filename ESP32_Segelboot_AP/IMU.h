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

// Pins für I2C ESP32-S3
#define I2C_SDA_PIN 8
#define I2C_SCL_PIN 9


// Forward Declaration
void setup_imu();
void read_imu();
void calibrate_gyro();
void calibrate_magnetometer(int durationSeconds);

float tilt_compensated_heading_from_mag(float mx, float my, float mz, float roll, float pitch);
#endif

