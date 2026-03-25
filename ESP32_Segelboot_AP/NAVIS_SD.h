#pragma once
#include <SPI.h>
#include <SD.h>


// Basisordner für Logs
#define LOG_BASE "/LOGS"

// Interface
void setup_sd();
void write_log();
void write_event_log(String type, String text, String dir, String wind, String waves);
void update_gps_tracks();

bool sd_file_exists(const String &path);
File sd_open_file(const String &path, const char* mode);