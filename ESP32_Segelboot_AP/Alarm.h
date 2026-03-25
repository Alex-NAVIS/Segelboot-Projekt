// ==========================================================
// Alarm.h – Alarmüberwachungssystem
// ==========================================================
#pragma once

#include <Arduino.h>
#include "Config.h"
#include "Sensor_Data.h"

// ==========================================================
// Setup & zentrale Steuerung
// ==========================================================

// Initialisiert den Piezo-Buzzer
void setup_alarm(bool testSignal);

// Schaltet den Buzzer
void alarm_trigger(bool state);

// Führt alle Alarmprüfungen aus
void check_alarm(void);

// ==========================================================
// Einzel-Alarmprüfungen
// ==========================================================

// Echolot
bool check_echolot(void);

// Geschwindigkeit
bool check_speed_max(void);
bool check_speed_min(void);

// Windgeschwindigkeit (LIVE)
bool check_wind_max(void);
bool check_wind_min(void);

// Windrichtung (gegen Referenz)
bool check_wind_dir(void);

// Kursabweichung (gegen Referenzkurs)
bool check_course_dev(void);

// Ankerwache (separat, keine Priorität)
bool check_anchor(void);

// Lage
bool check_roll(void);
bool check_pitch(void);
