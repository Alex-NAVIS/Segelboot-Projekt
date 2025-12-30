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
void handleFile(AsyncWebServerRequest *request, const char *path);
void handleSetConfig(AsyncWebServerRequest *request);                                                        // PID-Konfiguration setzen
void handleSetTarget(AsyncWebServerRequest *request);                                                        // Zielpunkt Autopilot setzen
void handleTide(AsyncWebServerRequest *request);                                                             // Abfrage zu einer Koordinate zum Tide Zeitpunkt Hochwasser Niedrigwasser
void handleMastBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total);  // Mast Sensor GPS Wind Sensordaten
void handleTiles(AsyncWebServerRequest *request);                                                            // PNG-Kacheln liefern                                                  //Tile Zeiten abfragen und zurück geben
void closeConnection();

// ----------------------------------------------------------------------
// Abfrage vom Main loop
// ----------------------------------------------------------------------
void checkMastOnlineStatus();

// ----------------------------------------------------------------------
// Fehlerabfrage bei Tiles anfrage, zurücksetzen der Anfragezahl
// ----------------------------------------------------------------------
void checkTileTransferWatchdog();

#endif  // WEBSERVER_MODULE_H
