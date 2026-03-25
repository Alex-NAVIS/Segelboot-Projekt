#pragma once

#ifndef CONFIG_STORAGE_H
#define CONFIG_STORAGE_H

#include <Arduino.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "Sensor_Data.h"
#include "Config.h"

// --- Little File System einrichten ---
void ConfigStorage_begin();

// --- Boot System Einstellungen---
void ConfigStorage_saveSystem();
void ConfigStorage_loadSystem();

#endif // CONFIG_STORAGE_H
