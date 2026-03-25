#include "WebServer_Module.h"
#include <ArduinoJson.h>

// Tracking Mast sensor
unsigned long lastReceiveTime = 0;
int noReceiveCounter = 0;
SemaphoreHandle_t sdMutex;

// ======================================================================
// Globale Instanzen
// ======================================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");


// ======================================================================
// Bezeichnung: setupWiFiAP
// Erklärung:   Initialisiert einen lokalen WLAN-Access-Point (AP).
//              Erstellt einen Mutex für den sicheren SD-Zugriff, setzt
//              den WiFi-Modus und startet das Funknetzwerk mit
//              reduzierter Sendeleistung sowie optionalem Debug-Logging.
// ======================================================================
void setupWiFiAP() {
  sdMutex = xSemaphoreCreateMutex();
  WiFi.mode(WIFI_AP);
  bool success = WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_11dBm);  // reduzierte Sendeleistung

  if (success) {
    if (DEBUG_MODE_SERVER) Serial.println("✅ Access Point gestartet!");
    if (DEBUG_MODE_SERVER) Serial.print("SSID: ");
    if (DEBUG_MODE_SERVER) Serial.println(AP_SSID);
    if (DEBUG_MODE_SERVER) Serial.print("IP-Adresse: ");
    if (DEBUG_MODE_SERVER) Serial.println(WiFi.softAPIP());
  } else {
    if (DEBUG_MODE_SERVER) Serial.println("❌ Fehler beim Starten des Access Points!");
  }
}

// ======================================================================
// Bezeichnung: onWsEvent
// Erklärung:   Zentraler Event-Handler für die WebSocket-Kommunikation.
//              Verwaltet den gesamten Lebenszyklus von Client-Verbindungen:
//              Registriert neue Verbindungen (Connect), erkennt Trennungen
//              (Disconnect) und leitet eingehende Datenpakete zur
//              Verarbeitung an die Logik-Funktion handleWebSocketMessage weiter.
// ======================================================================
void onWsEvent(AsyncWebSocket *server, AsyncWebSocketClient *client,
               AwsEventType type, void *arg, uint8_t *data, size_t len) {
  switch (type) {
    case WS_EVT_CONNECT:
      if (DEBUG_MODE_SERVER) Serial.printf("WebSocket: Client verbunden, ID %u\n", client->id());
      break;

    case WS_EVT_DISCONNECT:
      if (DEBUG_MODE_SERVER) Serial.printf("WebSocket: Client getrennt, ID %u\n", client->id());
      break;

    case WS_EVT_DATA:
      if (DEBUG_MODE_SERVER) Serial.println("WebSocket: Daten empfangen");
      handleWebSocketMessage(arg, data, len);
      break;
    default:
      break;
  }
}

// ======================================================================
// Bezeichnung: setupWebSocket
// Erklärung:   Konfiguriert die WebSocket-Schnittstelle, indem der zentrale
//              Event-Handler (onWsEvent) registriert und die WebSocket-
//              Instanz fest in den asynchronen Webserver eingebunden wird.
// ======================================================================
void setupWebSocket() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
}

// ======================================================================
// Bezeichnung: handleGetSystem
// Erklärung:   HTTP-GET-Handler, der die aktuellen Systemeinstellungen,
//              Update-Frequenzen (Hz) und Sensorkonfigurationen in einem
//              JSON-Objekt bündelt. Die Daten werden serialisiert und an
//              das Web-Interface gesendet, um dort die Formularfelder
//              mit den aktuellen Live-Werten zu füllen.
// ======================================================================
void handleGetSystem(AsyncWebServerRequest *request) {
  StaticJsonDocument<512> doc;
  doc["ap_ssid"] = AP_SSID;
  doc["ap_password"] = AP_PASSWORD;
  doc["gps_hz"] = GPS_UPDATE_HZ;
  doc["ahrs_hz"] = GPS_AHRS_UPDATE_HZ;
  doc["mpu_hz"] = MPU_UPDATE_HZ;
  doc["wind_hz"] = WINDDIR_UPDATE_HZ;  // HTML erwartet wind_hz
  doc["sd_hz"] = SD_UPDATE_HZ;
  doc["sensor_update_hz"] = SENSOR_UPDATE_HZ;
  doc["use_decl_compass"] = USE_DECLINATION_FOR_COMPASS;
  doc["use_decl_wind"] = USE_DECLINATION_FOR_WIND;
  doc["wassertiefe_einbau"] = Wassertiefe_Einbau;  // HTML Feldname angepasst
  doc["tiefgang_boot"] = Tiefgang_Boot;            // HTML Feldname angepasst
  doc["wassertemperatur"] = Wassertemperatur;      // HTML Feldname angepasst
  doc["waterSalinity"] = Salzgehalt;               // HTML Feldname angepasst
  String json;
  serializeJson(doc, json);
  request->send(200, "application/json", json);
}

// ======================================================================
// Bezeichnung: handleSetSystem
// Erklärung:   Verarbeitet eingehende HTTP-Parameter, um Systemeinstellungen
//              wie WLAN-Daten, Update-Raten und Sensorkonfigurationen zu
//              aktualisieren. Berechnet nach der Übernahme die Schall-
//              geschwindigkeit neu und speichert die Werte persistent.
// ======================================================================
void handleSetSystem(AsyncWebServerRequest *request) {
  if (request->hasParam("ap_ssid")) AP_SSID = request->getParam("ap_ssid")->value();
  if (request->hasParam("ap_password")) AP_PASSWORD = request->getParam("ap_password")->value();
  if (request->hasParam("gps_hz")) GPS_UPDATE_HZ = request->getParam("gps_hz")->value().toFloat();
  if (request->hasParam("ahrs_hz")) GPS_AHRS_UPDATE_HZ = request->getParam("ahrs_hz")->value().toFloat();
  if (request->hasParam("mpu_hz")) MPU_UPDATE_HZ = request->getParam("mpu_hz")->value().toFloat();
  if (request->hasParam("wind_hz")) WINDDIR_UPDATE_HZ = request->getParam("wind_hz")->value().toFloat();
  if (request->hasParam("sd_hz")) SD_UPDATE_HZ = request->getParam("sd_hz")->value().toFloat();
  if (request->hasParam("sensor_update_hz")) SENSOR_UPDATE_HZ = request->getParam("sensor_update_hz")->value().toFloat();
  if (request->hasParam("use_decl_compass")) USE_DECLINATION_FOR_COMPASS = request->getParam("use_decl_compass")->value().toInt() != 0;
  if (request->hasParam("use_decl_wind")) USE_DECLINATION_FOR_WIND = request->getParam("use_decl_wind")->value().toInt() != 0;
  if (request->hasParam("wassertiefe_einbau")) Wassertiefe_Einbau = request->getParam("wassertiefe_einbau")->value().toFloat();
  if (request->hasParam("tiefgang_boot")) Tiefgang_Boot = request->getParam("tiefgang_boot")->value().toFloat();
  if (request->hasParam("wassertemperatur")) Wassertemperatur = request->getParam("wassertemperatur")->value().toFloat();
  if (request->hasParam("waterSalinity")) Salzgehalt = request->getParam("waterSalinity")->value().toFloat();
  calc_schallgeschwindigkeit();
  request->send(200, "text/plain", "✔ Systemkonfiguration gespeichert");
  ConfigStorage_saveSystem();
}

// ======================================================================
// Bezeichnung: handleFile
// Erklärung:   Prüft die Existenz einer angeforderten Datei im LittleFS-
//              Dateisystem und liefert diese aus. Falls die Datei nicht
//              existiert, wird eine 404-Fehlermeldung gesendet. Standard-
//              mäßig wird der Content-Type "text/html" verwendet.
// ======================================================================
void handleFile(AsyncWebServerRequest *request, const char *path) {
  if (!LittleFS.exists(path)) {
    request->send(404, "text/plain", String(path) + " nicht gefunden");
    return;
  }
  request->send(LittleFS, path, "text/html");
}

// ======================================================================
// Bezeichnung: handleTiles
// Erklärung:   Liefert Karten-Kacheln (Tiles) von der SD-Karte aus. Die
//              Funktion nutzt einen Mutex-Semaphore zur Thread-Sicherheit,
//              prüft die Verfügbarkeit via USE_SD_TILES und setzt
//              spezifische HTTP-Header für effizientes Browser-Caching.
// ======================================================================
void handleTiles(AsyncWebServerRequest *request) {
  String path = request->url();
  if (!USE_SD_TILES) {
    request->send(404, "text/plain", "Tiles nicht verfügbar");
    return;
  }
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(250)) != pdTRUE) {
    request->send(503, "text/plain", "SD busy");
    return;
  }
  if (!sd_file_exists(path)) {
    xSemaphoreGive(sdMutex);
    request->send(404, "text/plain", "Tile nicht gefunden");
    return;
  }
  AsyncWebServerResponse *response = request->beginResponse(SD, path, "image/png", false);
  response->addHeader("Cache-Control", "public, max-age=604800, immutable");
  response->addHeader("Connection", "close");
  request->send(response);
  xSemaphoreGive(sdMutex);
}

// ======================================================================
// Bezeichnung: handleTideStation
// Erklärung:   Extrahiert den Dateinamen aus der Anfrage-URL, um Gezeiten-
//              daten (CSV) aus einem spezifischen SD-Verzeichnis ("/tide/")
//              auszuliefern. Die Funktion implementiert Mutex-geschützten
//              SD-Zugriff, umfangreiches Debug-Logging und deaktiviert
//              das Client-Caching für stets aktuelle Daten.
// ======================================================================
void handleTideStation(AsyncWebServerRequest *request) {
  String path = request->url();
  path.replace("/tide_station/", "");
  String fullPath = "/tide/" + path;
  if (DEBUG_MODE_SERVER) {
    Serial.print("[TideStation] Request URL: ");
    Serial.println(request->url());
    Serial.print("[TideStation] Datei-Name extrahiert: ");
    Serial.println(path);
    Serial.print("[TideStation] Voller Pfad auf SD: ");
    Serial.println(fullPath);
  }
  if (!sd_file_exists(fullPath)) {
    if (DEBUG_MODE_SERVER) Serial.println("[TideStation] Datei NICHT gefunden, sende 404");
    request->send(404, "text/plain", "Datei nicht gefunden");
    return;
  }
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(50)) != pdTRUE) {
    if (DEBUG_MODE_SERVER) Serial.println("[TideStation] SD busy, sende 503");
    request->send(503, "text/plain", "SD busy");
    return;
  }
  if (DEBUG_MODE_SERVER) Serial.println("[TideStation] Datei existiert, sende Response");
  AsyncWebServerResponse *response = request->beginResponse(SD, fullPath, "text/csv", false);
  response->addHeader("Cache-Control", "no-cache");
  request->send(response);
  xSemaphoreGive(sdMutex);
  if (DEBUG_MODE_SERVER) Serial.println("[TideStation] Datei gesendet, Mutex freigegeben");
}

// ======================================================================
// Bezeichnung: wsBroadcastSensorData
// Erklärung:   Erstellt ein JSON-Paket mit sämtlichen aktuellen Sensorwerten
//              (Lage, Wind, GPS, Zeit, Echolot und Relais-Status) und
//              sendet dieses per WebSocket-Broadcast an alle verbundenen
//              Clients. Nutzt einen statischen Buffer zur Speicheroptimierung
//              und Vermeidung von Heap-Fragmentierung.
// ======================================================================
// ============================================================
// WEBSOCKET: BROADCAST ALLER SENSOR- & SYSTEMDATEN
// ============================================================
void wsBroadcastSensorData() {
  static char buffer[896];                                              // Statischer Puffer für das JSON-Paket
  StaticJsonDocument<896> doc;                                          // JSON-Dokument zur Datenstrukturierung
  doc["roll"] = sensorData.roll;                                        // Neigung um die Längsachse
  doc["pitch"] = sensorData.pitch;                                      // Neigung um die Querachse
  doc["kompass"] = sensorData.kompass;                                  // Heading/Kurs des Bootes
  doc["winddir_gemessen"] = sensorData.winddir_gemessen;                // Apparent Wind Direction
  doc["windspeed_gemessen"] = sensorData.windspeed_gemessen;            // Apparent Wind Speed
  doc["winddir_berechnet"] = sensorData.winddir_berechnet;              // True Wind Direction
  doc["windspeed_berechnet"] = sensorData.windspeed_berechnet;          // True Wind Speed
  doc["gps_lat"] = sensorData.gps_lat;                                  // Aktueller Breitengrad
  doc["gps_lon"] = sensorData.gps_lon;                                  // Aktueller Längengrad
  doc["gps_speed"] = sensorData.gps_speed;                              // Geschwindigkeit über Grund
  doc["gps_kurs"] = sensorData.gps_kurs;                                // Kurs über Grund
  doc["sm"] = sensorData.sm_counter;                                    // Seemeilen Zähler
  doc["gps_jahr"] = sensorData.gps_jahr;                                // Datum: Jahr
  doc["gps_monat"] = sensorData.gps_monat;                              // Datum: Monat
  doc["gps_tag"] = sensorData.gps_tag;                                  // Datum: Tag
  doc["gps_stunde"] = sensorData.gps_stunde;                            // Zeit: Stunde (UTC)
  doc["gps_minute"] = sensorData.gps_minute;                            // Zeit: Minute (UTC)
  doc["gps_sekunde"] = sensorData.gps_sekunde;                          // Zeit: Sekunde (UTC)
  doc["missweisung"] = sensorData.missweisung;                          // Magnetische Variation vor Ort
  doc["Echolot"] = sensorData.Echolot;                                  // Wassertiefe unter Kiel
  doc["relay_a"] = sensorData.relay_a;                                  // Status Relais A
  doc["relay_b"] = sensorData.relay_b;                                  // Status Relais B
  doc["relay_c"] = sensorData.relay_c;                                  // Status Relais C
  doc["relay_d"] = sensorData.relay_d;                                  // Status Relais D
  doc["mast_sensor"] = sensorData.mast_online;                          // Status der Masteinheit
  doc["alarm_status"] = alarmData.alarm;                                // Alarm ausgeben
  JsonObject ap = doc.createNestedObject("autopilot");                  // Verschachteltes JSON-Objekt erstellen
  ap["mode"] = pinnenautopilotData.modus_autopilot;                     // Aktueller Betriebsmodus
  ap["offset"] = pinnenautopilotData.rotary_offset;                     // Kurs-Offset (Drehregler)
  ap["modus_counter"] = pinnenautopilotData.modus_counter;              // Interner Modus-Zähler
  int pinneState = 0;                                                   // Bestimmung des manuellen Pinnen-Status (0=Stop, 1=In, 2=Out)
  if (pinnenautopilotData.motor_manuel_einfahren) pinneState = 1;       // einfahren
  else if (pinnenautopilotData.motor_manuel_ausfahren) pinneState = 2;  //ausfahren
  ap["pinne"] = pinneState;                                             // Status der manuellen Bewegung
  ap["target_lat"] = pinnenautopilotData.autopilot_lat;                 // Navigationsziel Breitengrad
  ap["target_lon"] = pinnenautopilotData.autopilot_lon;                 // Navigationsziel Längengrad
  // Daten paketieren und an alle WebSocket-Clients senden
  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  ws.textAll(buffer, len);
}

// ======================================================================
// Bezeichnung: handleWebSocketMessage
// Erklärung:   Verarbeitet eingehende WebSocket-Textnachrichten im JSON-Format.
//              Validiert die Datenintegrität (Frame-Check) und steuert bei
//              "setRelay"-Befehlen die logischen Zustände der Relais-Variablen
//              in sensorData. Nach der Wertänderung wird ein Broadcast
//              ausgelöst, um alle verbundenen UIs synchron zu halten.
// ======================================================================
void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) return;
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, data);
  if (err) {
    if (DEBUG_MODE_SERVER) Serial.println("WS JSON Parse Error");
    return;
  }
  const char *type = doc["type"];
  if (!type) return;
  if (strcmp(type, "setRelay") == 0) {
    const char *relayId = doc["relay"];
    const char *state = doc["state"];
    bool turnOn = (strcmp(state, "on") == 0);
    if (strcmp(relayId, "relay_a") == 0) sensorData.relay_a = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_b") == 0) sensorData.relay_b = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_c") == 0) sensorData.relay_c = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_d") == 0) sensorData.relay_d = turnOn ? 0 : 1;
    wsBroadcastSensorData();
  }
}

// ======================================================================
// Bezeichnung: handleMastBody
// Erklärung:   Verarbeitet eingehende POST-Daten des Mast-Sensors im JSON-
//              Format. Die Funktion validiert die Vollständigkeit des
//              Datenpakets, prüft alle erforderlichen GPS- und Wind-
//              Parameter auf Existenz sowie Gültigkeit und aktualisiert
//              bei Erfolg die sensorData-Struktur sowie den Online-Status
//              des Mast-Moduls.
// ======================================================================
void handleMastBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  if (index + len != total) return;
  StaticJsonDocument<512> doc;
  DeserializationError error = deserializeJson(doc, data, total);
  if (error) {
    if (DEBUG_MODE_SERVER) {
      Serial.print("❌ JSON Fehler: ");
      Serial.println(error.c_str());
    }
    request->send(400, "text/plain", "❌ Ungültiges JSON");
    return;
  }
  const char *keys[] = {
    "winddir_gemessen", "windspeed_gemessen", "gps_lat", "gps_lon",
    "gps_speed", "gps_kurs", "gps_sats", "gps_hdop",
    "gps_jahr", "gps_monat", "gps_tag", "gps_stunde", "gps_minute", "gps_sekunde"
  };
  for (const char *k : keys) {
    if (!doc.containsKey(k)) {
      request->send(400, "text/plain", String("❌ Wert fehlt: ") + k);
      return;
    }
  }
  if (doc["gps_lat"].isNull() || doc["gps_lon"].isNull()) {
    request->send(400, "text/plain", "❌ Ungültige GPS-Koordinaten");
    return;
  }
  sensorData.winddir_gemessen = doc["winddir_gemessen"];
  sensorData.windspeed_gemessen = doc["windspeed_gemessen"];
  sensorData.gps_lat = doc["gps_lat"];
  sensorData.gps_lon = doc["gps_lon"];
  sensorData.gps_speed = doc["gps_speed"];
  sensorData.gps_kurs = doc["gps_kurs"];
  sensorData.gps_sats = doc["gps_sats"];
  sensorData.gps_hdop = doc["gps_hdop"];
  sensorData.gps_jahr = doc["gps_jahr"];
  sensorData.gps_monat = doc["gps_monat"];
  sensorData.gps_tag = doc["gps_tag"];
  sensorData.gps_stunde = doc["gps_stunde"];
  sensorData.gps_minute = doc["gps_minute"];
  sensorData.gps_sekunde = doc["gps_sekunde"];
  lastReceiveTime = millis();
  noReceiveCounter = 0;
  sensorData.mast_online = 1;
  request->send(200, "text/plain", "OK");
}

// ======================================================================
// Bezeichnung: checkMastOnlineStatus
// Erklärung:   Überwacht zyklisch die Verbindung zum Mast-Sensor. Bleiben
//              neue Datenpakete über einen definierten Zeitraum aus, wird
//              ein Zähler inkrementiert. Nach fünf aufeinanderfolgenden
//              Fehlintervallen (ca. 5 Sekunden) wird der Status des
//              Sensors explizit auf "Offline" (Wert 2) gesetzt.
// ======================================================================
void checkMastOnlineStatus() {
  unsigned long now = millis();
  if (sensorData.mast_online > 0) {
    if (now - lastReceiveTime > 1000) {
      noReceiveCounter++;
      if (noReceiveCounter >= 5) {
        sensorData.mast_online = 2;
      }
    }
  }
}

// ======================================================================
// Bezeichnung: handleGetFSFile
// Erklärung:   Liefert eine spezifische Konfigurations- oder Datendatei
//              aus dem LittleFS-Dateisystem im JSON-Format aus. Die
//              Funktion erzwingt durch den "no-store" Header, dass der
//              Browser die Datei nicht zwischenspeichert, um stets
//              die aktuellsten Dateiinhalte zu garantieren.
// ======================================================================
void handleGetFSFile(AsyncWebServerRequest *request, const char *path) {
  if (!LittleFS.exists(path)) {
    request->send(404, "application/json", "{\"error\":\"file not found\"}");
    return;
  }
  AsyncWebServerResponse *response = request->beginResponse(LittleFS, path, "application/json");
  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

// ======================================================================
// Bezeichnung: handleAutopilotData
// Erklärung:   Verarbeitet Steuerbefehle für den Autopiloten und die
//              Manuelle Pinnensteuerung. Die Funktion setzt den Betriebs-
//              modus, steuert das manuelle Ein- und Ausfahren des Motors
//              und ermöglicht die Kalibrierung des Ruderwinkels durch
//              Anpassung des Rotary-Offsets.
// ======================================================================
void handleAutopilotData(AsyncWebServerRequest *request) {
  // ===============================
  // Manual Pinne (HAT PRIORITÄT)
  // ===============================
  if (request->hasParam("pinne")) {
    int p = request->getParam("pinne")->value().toInt();
    // 🔴 Regel 1: Manual Override stoppt Autopilot
    if (p == 1 || p == 2) {
      pinnenautopilotData.modus_autopilot = 0;  // STOP
    }
    // 🔒 harte gegenseitige Verriegelung
    pinnenautopilotData.motor_manuel_einfahren = false;
    pinnenautopilotData.motor_manuel_ausfahren = false;
    if (p == 1) {
      pinnenautopilotData.motor_manuel_einfahren = true;
    } else if (p == 2) {
      pinnenautopilotData.motor_manuel_ausfahren = true;
    }
    // p==0 ⇒ STOP
  }

  // ===============================
  // Autopilot-Modus
  // ===============================
  if (request->hasParam("modus")) {
    int newMode = request->getParam("modus")->value().toInt();
    // 🟢 Regel 2: Mode setzt System sauber zurück
    pinnenautopilotData.modus_autopilot = newMode;
    // Manual stoppen
    pinnenautopilotData.motor_manuel_einfahren = false;
    pinnenautopilotData.motor_manuel_ausfahren = false;
    // Offset zurücksetzen
    pinnenautopilotData.rotary_offset = 0;
  }
  // ===============================
  // Offset (nur wenn NICHT durch Mode überschrieben)
  // ===============================
  if (request->hasParam("winkel")) {
    pinnenautopilotData.rotary_offset = request->getParam("winkel")->value().toInt();
  }
  pinnenautopilotData.modus_counter++;  // Counter immer hochzählen, auch wenn Mode gleich bleibt
  request->send(200, "text/plain", "✔ Autopilot-Daten aktualisiert");
}

// ======================================================================
// Bezeichnung: handleAutopilotStatus
// Erklärung:   Gibt den aktuellen Status des Autopiloten im JSON-Format
//              aus. Übermittelt den Betriebsmodus, den Bewegungsstatus
//              der Pinne (Stopp, Einfahren oder Ausfahren) sowie den
//              aktuell gesetzten Ruderwinkel-Offset (Rotary-Offset)
//              an das Web-Interface.
// ======================================================================
void handleAutopilotStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<256> doc;
  // ===============================
  // Modus
  // ===============================
  doc["mode"] = pinnenautopilotData.modus_autopilot;
  // ===============================
  // Pinne-Status (0=STOP,1=EIN,2=AUS)
  // ===============================
  int pinneState = 0;
  if (pinnenautopilotData.motor_manuel_einfahren) {
    pinneState = 1;
  } else if (pinnenautopilotData.motor_manuel_ausfahren) {
    pinneState = 2;
  }
  doc["pinne"] = pinneState;
  // ===============================
  // Offset
  // ===============================
  doc["offset"] = pinnenautopilotData.rotary_offset;
  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

// ======================================================================
// Bezeichnung: handleTideCurve
// Erklärung:   Verarbeitet Geodaten-Anfragen (Latitude, Longitude, Radius),
//              um nahegelegene Gezeitenstationen zu finden. Die Funktion
//              validiert die Eingabeparameter, führt eine Umkreissuche
//              durch und sendet die Ergebnisse (Name, Position, Distanz)
//              als JSON-Array via Stream-Response direkt an den Client,
//              um den Arbeitsspeicher zu schonen.
// ======================================================================
void handleTideCurve(AsyncWebServerRequest *request) {
  if (!request->hasParam("lat") || !request->hasParam("lon") || !request->hasParam("radius")) {
    request->send(400, "text/plain", "missing lat/lon/radius");
    return;
  }
  tideQueryLat = request->getParam("lat")->value().toDouble();
  tideQueryLon = request->getParam("lon")->value().toDouble();
  tideQueryRadiusSM = request->getParam("radius")->value().toDouble();
  if (tideQueryRadiusSM < 1.0) tideQueryRadiusSM = 1.0;
  if (!findTideStations()) {
    request->send(200, "application/json", "[]");
    return;
  }
  AsyncResponseStream *response = request->beginResponseStream("application/json");
  StaticJsonDocument<4096> doc;
  JsonArray arr = doc.to<JsonArray>();
  for (uint8_t i = 0; i < tideStationCount; i++) {
    JsonObject s = arr.createNestedObject();
    s["name"] = tideStations[i].name;
    s["lat"] = tideStations[i].lat;
    s["lon"] = tideStations[i].lon;
    s["dist"] = tideStations[i].distanceSM;
  }
  serializeJson(arr, *response);
  request->send(response);
}

// ======================================================================
// Bezeichnung: handleAutopilotTarget
// Erklärung:   Empfängt und speichert die geografischen Zielkoordinaten
//              (Breiten- und Längengrad) für die GPS-Navigation des
//              Autopiloten. Die Funktion validiert das Vorhandensein der
//              Parameter und übernimmt die Werte als Double-Präzision
//              in die globale Autopilot-Datenstruktur.
// ======================================================================
void handleAutopilotTarget(AsyncWebServerRequest *request) {
  if (!request->hasParam("lat") || !request->hasParam("lon")) {
    request->send(400, "text/plain", "❌ Fehlende Parameter lat/lon");
    return;
  }
  pinnenautopilotData.autopilot_lat = request->getParam("lat")->value().toDouble();
  pinnenautopilotData.autopilot_lon = request->getParam("lon")->value().toDouble();
  request->send(200, "text/plain", "✅ Zielkoordinaten gesetzt");
}

// ======================================================================
// Bezeichnung: handleAlarm
// Erklärung:   Verarbeitet eingehende Alarm-Konfigurationsdaten im
//              JSON-Format. Die Funktion parst den Request-Body,
//              extrahiert Schwellenwerte für Geschwindigkeit, Wind,
//              Kursabweichung und Schiffslage (Roll/Pitch) und
//              aktualisiert damit die globale alarmData-Struktur.
// ======================================================================
void handleAlarm(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  StaticJsonDocument<512> doc;
  DeserializationError err = deserializeJson(doc, data, len);
  if (err) {
    request->send(400, "application/json", "{\"status\":\"error\",\"msg\":\"json parse error\"}");
    return;
  }
  // ===== Werte übernehmen =====
  alarmData.en_speed_max = doc["en_speed_max"] | false;
  alarmData.speedMax = doc["speedMax"] | 0.0;
  alarmData.en_speed_min = doc["en_speed_min"] | false;
  alarmData.speedMin = doc["speedMin"] | 0.0;
  alarmData.en_echolot = doc["en_echolot"] | false;
  alarmData.echolotMin = doc["echolotMin"] | 0.0;
  alarmData.en_wind_max = doc["en_wind_max"] | false;
  alarmData.windMax = doc["windMax"] | 0.0;
  alarmData.en_wind_min = doc["en_wind_min"] | false;
  alarmData.windMin = doc["windMin"] | 0.0;
  alarmData.en_wind_dir = doc["en_wind_dir"] | false;
  alarmData.windDir = doc["windDir"] | 0.0;
  alarmData.en_course_dev = doc["en_course_dev"] | false;
  alarmData.courseDev = doc["courseDev"] | 0.0;
  alarmData.en_anchor = doc["en_anchor"] | false;
  alarmData.anchorRadius = doc["anchorRadius"] | 0.0;
  alarmData.anchorkette = doc["anchorChain"] | 0.0;
  alarmData.en_roll = doc["en_roll"] | false;
  alarmData.roll = doc["roll"] | 0.0;
  alarmData.en_pitch = doc["en_pitch"] | false;
  alarmData.pitch = doc["pitch"] | 0.0;
  if (DEBUG_MODE_SERVER) Serial.println("Alarmdaten aktualisiert");
  request->send(200, "application/json", "{\"status\":\"ok\"}");
  // Zur überwachung aktuelle Werte Windrichtung, Steuerkurs und koordinate Anker aktuelle Position und Werte
  alarmData.windDir_temp = sensorData.winddir_gemessen;
  alarmData.courseDev_temp = sensorData.kompass;
  alarmData.anker_lat = sensorData.gps_lat;
  alarmData.anker_lon = sensorData.gps_lon;
}

// ======================================================================
// Bezeichnung: listDir
// Erklärung:   Durchläuft rekursiv ein Dateisystem (LittleFS oder SD) und
//              erstellt einen vollständigen Verzeichnisbaum im JSON-Format.
//              Die Funktion erfasst für jedes Element den absoluten Pfad,
//              die Dateigröße sowie den Typ (Datei oder Ordner) und fügt
//              diese strukturiert an einen String-Buffer an.
// ======================================================================
void listDir(fs::FS &fs, String dirname, String &json) {
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) return;
  File file = root.openNextFile();
  while (file) {
    if (json != "[") json += ",";
    json += "{\"name\":\"" + String(file.path()) + "\",";
    json += "\"size\":" + String(file.size()) + ",";
    json += "\"type\":\"" + String(file.isDirectory() ? "dir" : "file") + "\"}";
    if (file.isDirectory()) {
      listDir(fs, file.path(), json);
    }
    file = root.openNextFile();
  }
}

// ======================================================================
// Bezeichnung: handleWetterRequest
// Erklärung: Verarbeitet HTTP-Anfragen für spezifische Wetter-Dateien.
// Prüft die Existenz der angeforderten Datei im LittleFS,
// ermittelt den passenden Content-Type (JSON oder Text)
// und sendet die Datei an den Client zurück.
// ======================================================================
void handleWetterRequest(AsyncWebServerRequest *request) {
  String path = request->url();  // z.B. /Wetter/wind1.json
  Serial.println("Wetter Request: " + path);
  if (!LittleFS.exists(path)) {
    request->send(404, "text/plain", "Datei nicht gefunden");
    return;
  }
  // Content-Type bestimmen (optional aber sauber)
  String contentType = "application/json";
  if (path.endsWith(".json")) contentType = "application/json";
  else if (path.endsWith(".txt")) contentType = "text/plain";
  request->send(LittleFS, path, contentType);
}

// ======================================================================
// Bezeichnung: handleWetterIndex
// Erklärung: Erstellt eine dynamische Liste aller verfügbaren Wetter-
// Konfigurationen. Scannt das Verzeichnis "/Wetter" nach
// JSON-Dateien und gibt deren Namen und Labels als
// strukturiertes JSON-Array an den Webserver zurück.
// ======================================================================
void handleWetterIndex(AsyncWebServerRequest *request) {
  Serial.println("Wetter Index angefragt");
  String json = "[";
  File root = LittleFS.open("/Wetter");
  File file = root.openNextFile();
  bool first = true;
  while (file) {
    String name = file.name();
    if (name.endsWith(".json")) {
      // Dateiname extrahieren (ohne Pfad)
      int slash = name.lastIndexOf('/');
      if (slash >= 0) name = name.substring(slash + 1);
      if (!first) json += ",";
      first = false;
      json += "{";
      json += "\"file\":\"" + name + "\",";  // Dateiname
      json += "\"label\":\"" + name + "\"";  // Label identisch
      json += "}";
    }
    file = root.openNextFile();
  }
  json += "]";
  request->send(200, "application/json", json);
}

// ======================================================================
// Bezeichnung: setupWebServer
// Erklärung:   Zentrale Konfigurationsinstanz des asynchronen Webservers.
//              Implementiert eine REST-API durch Mapping von LittleFS-Dateien,
//              Steuerungs-Endpunkte für Autopilot und Gezeiten, sowie
//              komplexe Handler für Datei-Uploads, Verzeichnis-Listing und
//              Sensor-Kalibrierung. Beinhaltet zudem das Statik-Datei-
//              Serving (HTML/JS/Leaflet) und die WebSocket-Registrierung.
// ======================================================================
void setupWebServer() {
  // 1. Dateisystem-API (Mapping über ein Array spart Handler)
  struct FileMapping {
    const char *url;
    const char *path;
  };
  FileMapping configs[] = {
    { "/fs/system_config", "/settings/system_config.json" },
    { "/fs/autopilot_config", "/settings/autopilot_config.json" },
    { "/fs/imu_mag", "/settings/IMU_mag.json" },
    { "/fs/imu_gyro", "/settings/IMU_gyro.json" }
  };
  for (auto const &cfg : configs) {
    server.on(cfg.url, HTTP_GET, [cfg](AsyncWebServerRequest *r) {
      handleGetFSFile(r, cfg.path);
    });
  }

  // 2. Steuerungen & Autopilot
  server.on("/autopilot_data", HTTP_GET, handleAutopilotData);
  server.on("/autopilot_status", HTTP_GET, handleAutopilotStatus);
  server.on("/autopilot", HTTP_GET, handleAutopilotTarget);

  // 3. Alarm Systeme
  server.on(
    "/saveAlarms", HTTP_POST, [](AsyncWebServerRequest *request) {}, NULL, handleAlarm);
  server.on("/getAlarms", HTTP_GET, [](AsyncWebServerRequest *request) {
    // JSON-Dokument erstellen (klein, statisch für ESP32)
    StaticJsonDocument<512> doc;
    doc["en_speed_max"] = alarmData.en_speed_max;
    doc["speedMax"] = alarmData.speedMax;
    doc["en_speed_min"] = alarmData.en_speed_min;
    doc["speedMin"] = alarmData.speedMin;
    doc["en_echolot"] = alarmData.en_echolot;
    doc["echolotMin"] = alarmData.echolotMin;
    doc["en_wind_max"] = alarmData.en_wind_max;
    doc["windMax"] = alarmData.windMax;
    doc["en_wind_min"] = alarmData.en_wind_min;
    doc["windMin"] = alarmData.windMin;
    doc["en_wind_dir"] = alarmData.en_wind_dir;
    doc["windDir"] = alarmData.windDir;
    doc["en_course_dev"] = alarmData.en_course_dev;
    doc["courseDev"] = alarmData.courseDev;
    doc["en_anchor"] = alarmData.en_anchor;
    doc["anchorRadius"] = alarmData.anchorRadius;
    doc["anchorChain"] = alarmData.anchorkette;
    doc["en_roll"] = alarmData.en_roll;
    doc["roll"] = alarmData.roll;
    doc["en_pitch"] = alarmData.en_pitch;
    doc["pitch"] = alarmData.pitch;
    String json;
    serializeJson(doc, json);
    request->send(200, "application/json", json);
  });


  // 4. Wetter Wellen
  server.on("/Wetter/index.json", HTTP_GET, handleWetterIndex);
  server.on("/Wetter/*", HTTP_GET, handleWetterRequest);


  // 5. Logbuch Einträge
  server.on("/logbook", HTTP_GET, [](AsyncWebServerRequest *request) {
    String type = "";
    String text = "";
    String dir = "";
    String wind = "";
    String waves = "";
    if (request->hasParam("type")) type = request->getParam("type")->value();
    if (request->hasParam("text")) text = request->getParam("text")->value();
    if (request->hasParam("dir")) dir = request->getParam("dir")->value();
    if (request->hasParam("wind")) wind = request->getParam("wind")->value();
    if (request->hasParam("waves")) waves = request->getParam("waves")->value();
    write_event_log(type, text, dir, wind, waves);
    request->send(200, "text/plain", "OK");
  });

  // 6. Gezeiten-API
  server.on("/tide_curve", HTTP_GET, handleTideCurve);
  server.on("/tide_station/*", HTTP_GET, handleTideStation);

  // 7. System & Sensoren
  server.on("/getSystem", HTTP_GET, handleGetSystem);
  server.on(
    "/mastdata", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL, handleMastBody);
  server.on("/setSystem", HTTP_GET, handleSetSystem);
  server.on("/tiles/*", HTTP_GET, handleTiles);
  server.on("/resetLog", HTTP_GET, [](AsyncWebServerRequest *request) {
    sensorData.sm_counter = 0.0;
    request->send(200, "text/plain", "OK");
  });

  // 8. Kompakte Kalibrierung
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (!request->hasParam("type")) return request->send(400, "text/plain", "Parameter fehlt");
    String type = request->getParam("type")->value();
    if (type == "compass") {
      mag_start_cal = true;
      request->send(200, "text/plain", "Kompass OK");
    } else if (type == "gyro") {
      gyro_start_cal = true;
      request->send(200, "text/plain", "Gyro OK");
    } else {
      request->send(400, "text/plain", "Unbekannter Typ");
    }
  });

  // 9. Datei-Upload Endpunkt (LittleFS) mit Speicherprüfung
  server.on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      request->send(request->_tempObject != nullptr ? 507 : 200, "text/plain", request->_tempObject != nullptr ? "Fehler" : "Erfolg");
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0 && request->contentLength() > (LittleFS.totalBytes() - LittleFS.usedBytes())) {
        request->_tempObject = (void *)1;
        return;
      }
      if (request->_tempObject == nullptr) {
        File file = LittleFS.open("/" + filename, index == 0 ? "w" : "a");
        if (file) {
          if (file.write(data, len) != len) request->_tempObject = (void *)1;
          file.close();
        } else {
          request->_tempObject = (void *)1;
        }
      }
    });

  // 10. Dateien des FS auflisten (Rekursiv)
  server.on("/list_files", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";
    listDir(LittleFS, "/", json);
    if (json != "[") json += ",";
    json += "{\"fs_total\":" + String(LittleFS.totalBytes()) + ",\"fs_used\":" + String(LittleFS.usedBytes()) + "}]";
    request->send(200, "application/json", json);
  });

  // 11. WebSocket & Statische Dateien
  server.addHandler(&ws);
  server.serveStatic("/leaflet", LittleFS, "/leaflet/");
  server.serveStatic("/js", LittleFS, "/js/");
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.begin();
  if (DEBUG_MODE_SERVER) Serial.println("✅ Server optimiert gestartet");
}
