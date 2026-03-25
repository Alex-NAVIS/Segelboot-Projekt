#ifndef WEBSERVER_MODULE_H
#define WEBSERVER_MODULE_H

#include <Arduino.h>
#include <WiFi.h>
#include <ESPAsyncWebServer.h>
#include <AsyncWebSocket.h>
#include <AsyncTCP.h>
#include <LittleFS.h>

#include "NAVIS_SD.h"
#include "Sensor_Data.h"
#include "Config.h"
#include "ConfigStorage.h"
#include "LightControl_Module.h"
#include "tide.h"

// ----------------------------------------------------------------------
// Zugriff auf den Webserver und WebSocket
// ----------------------------------------------------------------------
extern AsyncWebServer server;
extern AsyncWebSocket ws;

// ----------------------------------------------------------------------
// Setup Access Point + Webserver + WebSocket
// ----------------------------------------------------------------------
void setupWiFiAP();
void setupWebServer();
void setupWebSocket();  // falls du die separate setup-Funktion nutzen willst

// ----------------------------------------------------------------------
// Broadcast: Sendet kontinuierlich alle Sensorwerte per WebSocket
// ----------------------------------------------------------------------
void wsBroadcastSensorData();

// ----------------------------------------------------------------------
// WebSocket Handler für eingehende Nachrichten
// ----------------------------------------------------------------------
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len);

// ----------------------------------------------------------------------
// HTTP-Hilfsfunktionen (diese bleiben sinnvoll)
// ----------------------------------------------------------------------
void handleFile(AsyncWebServerRequest *request, const char *path);                                           // PID-Konfiguration setzen
void handleSetTarget(AsyncWebServerRequest *request);                                                        // Zielpunkt Autopilot setzen
void handleTide(AsyncWebServerRequest *request);                                                             // Abfrage zu einer Koordinate zum Tide Zeitpunkt Hochwasser Niedrigwasser
void handleMastBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);  // Mast Sensor GPS Wind Sensordaten
void handleAlarm(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);     // Alarm Einstellungen
void handleTiles(AsyncWebServerRequest *request);                                                            // PNG-Kacheln liefern
void handleTideCurve(AsyncWebServerRequest *request);                                                        //Tile Zeiten abfragen und zurück geben
void handleWetterIndex(AsyncWebServerRequest *request);                                                      // Liste alle Wetterdaten im System auf
void handleWetterRequest(AsyncWebServerRequest *request);                                                    // Sendet Wetter Wind Datei
void closeConnection();

// ----------------------------------------------------------------------
// Abfrage vom Main loop
// ----------------------------------------------------------------------
void checkMastOnlineStatus();

// ----------------------------------------------------------------------
// Sendet die Kurve idx an die vorher gespeicherte Anfrage
// ----------------------------------------------------------------------
void send_tide_curve(int idx);
void wsSendTideCurve(int idx);

// ----------------------------------------------------------------------
// FS File System abfragen
// ----------------------------------------------------------------------
void listDir(fs::FS &fs, String dirname, String &json);
#endif  // WEBSERVER_MODULE_H
