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
// Access Point starten
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
// WebSocket Event – EIN Handler für ALLE Events
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

//WebSocket initialisieren
void setupWebSocket() {
  ws.onEvent(onWsEvent);
  server.addHandler(&ws);
}

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

  doc["wassertemperatur"] = Wassertemperatur;  // HTML Feldname angepasst
  doc["waterSalinity"] = Salzgehalt;           // HTML Feldname angepasst

  String json;
  serializeJson(doc, json);

  request->send(200, "application/json", json);
}


void handleSetSystem(AsyncWebServerRequest *request) {
  if (request->hasParam("ap_ssid")) AP_SSID = request->getParam("ap_ssid")->value();
  if (request->hasParam("ap_password")) AP_PASSWORD = request->getParam("ap_password")->value();

  if (request->hasParam("gps_hz")) GPS_UPDATE_HZ = request->getParam("gps_hz")->value().toFloat();
  if (request->hasParam("ahrs_hz")) GPS_AHRS_UPDATE_HZ = request->getParam("ahrs_hz")->value().toFloat();
  if (request->hasParam("mpu_hz")) MPU_UPDATE_HZ = request->getParam("mpu_hz")->value().toFloat();
  if (request->hasParam("wind_hz")) WINDDIR_UPDATE_HZ = request->getParam("wind_hz")->value().toFloat();
  if (request->hasParam("sd_hz")) SD_UPDATE_HZ = request->getParam("sd_hz")->value().toFloat();
  if (request->hasParam("sensor_update_hz")) SENSOR_UPDATE_HZ = request->getParam("sensor_update_hz")->value().toFloat();  // <— NEU


  if (request->hasParam("use_decl_compass")) USE_DECLINATION_FOR_COMPASS = request->getParam("use_decl_compass")->value().toInt() != 0;
  if (request->hasParam("use_decl_wind")) USE_DECLINATION_FOR_WIND = request->getParam("use_decl_wind")->value().toInt() != 0;

  if (request->hasParam("wassertiefe_einbau")) Wassertiefe_Einbau = request->getParam("wassertiefe_einbau")->value().toFloat();
  if (request->hasParam("tiefgang_boot")) Tiefgang_Boot = request->getParam("tiefgang_boot")->value().toFloat();

  if (request->hasParam("wassertemperatur")) Wassertemperatur = request->getParam("wassertemperatur")->value().toFloat();
  if (request->hasParam("waterSalinity")) Salzgehalt = request->getParam("waterSalinity")->value().toFloat();

  // Berechne Schallgeschwindigkeit erneut
  calc_schallgeschwindigkeit();

  request->send(200, "text/plain", "✔ Systemkonfiguration gespeichert");
  ConfigStorage_saveSystem();
}


// ======================================================================
// Dateien aus LittleFS ausliefern
// ======================================================================
void handleFile(AsyncWebServerRequest *request, const char *path) {
  if (!LittleFS.exists(path)) {
    request->send(404, "text/plain", String(path) + " nicht gefunden");
    return;
  }
  request->send(LittleFS, path, "text/html");
}

// ======================================================================
// tiles aus SD Karte ausliefern
// ======================================================================
void handleTiles(AsyncWebServerRequest *request) {
  String path = request->url();
  if (!USE_SD_TILES) {
    request->send(404, "text/plain", "Tiles nicht verfügbar");
    return;
  }
  // 🔧 NEU: SD-Zugriffe serialisieren
  if (xSemaphoreTake(sdMutex, pdMS_TO_TICKS(250)) != pdTRUE) {
    request->send(503, "text/plain", "SD busy");
    return;
  }
  if (!sd_file_exists(path)) {
    xSemaphoreGive(sdMutex);
    request->send(404, "text/plain", "Tile nicht gefunden");
    return;
  }
  AsyncWebServerResponse *response =
    request->beginResponse(SD, path, "image/png", false);
  // 🔧 NEU: stabile HTTP-Header
  response->addHeader("Cache-Control", "public, max-age=604800, immutable");
  response->addHeader("Connection", "close");
  request->send(response);
  xSemaphoreGive(sdMutex);
}

void handleTideStation(AsyncWebServerRequest *request) {
  String path = request->url();
  path.replace("/tide_station/", "");
  String fullPath = "/tide/" + path;

  if (DEBUG_MODE_SERVER) Serial.print("[TideStation] Request URL: ");
  if (DEBUG_MODE_SERVER) Serial.println(request->url());
  if (DEBUG_MODE_SERVER) Serial.print("[TideStation] Datei-Name extrahiert: ");
  if (DEBUG_MODE_SERVER) Serial.println(path);
  if (DEBUG_MODE_SERVER) Serial.print("[TideStation] Voller Pfad auf SD: ");
  if (DEBUG_MODE_SERVER) Serial.println(fullPath);

  // Prüfen, ob Datei existiert
  if (!sd_file_exists(fullPath)) {
    if (DEBUG_MODE_SERVER) Serial.println("[TideStation] Datei NICHT gefunden, sende 404");
    request->send(404, "text/plain", "Datei nicht gefunden");
    return;
  }

  // Zugriff auf SD absichern
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

void wsBroadcastSensorData() {
  static char buffer[768];  // fester, stackfreier Buffer
  StaticJsonDocument<768> doc;

  doc["roll"] = sensorData.roll;
  doc["pitch"] = sensorData.pitch;
  doc["kompass"] = sensorData.kompass;
  doc["winddir_gemessen"] = sensorData.winddir_gemessen;
  doc["windspeed_gemessen"] = sensorData.windspeed_gemessen;
  doc["winddir_berechnet"] = sensorData.winddir_berechnet;
  doc["windspeed_berechnet"] = sensorData.windspeed_berechnet;
  doc["gps_lat"] = sensorData.gps_lat;
  doc["gps_lon"] = sensorData.gps_lon;
  doc["gps_speed"] = sensorData.gps_speed;
  doc["gps_kurs"] = sensorData.gps_kurs;
  doc["gps_jahr"] = sensorData.gps_jahr;
  doc["gps_monat"] = sensorData.gps_monat;
  doc["gps_tag"] = sensorData.gps_tag;
  doc["gps_stunde"] = sensorData.gps_stunde;
  doc["gps_minute"] = sensorData.gps_minute;
  doc["gps_sekunde"] = sensorData.gps_sekunde;
  doc["missweisung"] = sensorData.missweisung;
  doc["Echolot"] = sensorData.Echolot;
  doc["relay_a"] = sensorData.relay_a;
  doc["relay_b"] = sensorData.relay_b;
  doc["relay_c"] = sensorData.relay_c;
  doc["relay_d"] = sensorData.relay_d;
  doc["mast_sensor"] = sensorData.mast_online;

  size_t len = serializeJson(doc, buffer, sizeof(buffer));
  ws.textAll(buffer, len);  // zero-copy WebSocket send
}


void handleWebSocketMessage(void *arg, uint8_t *data, size_t len) {
  AwsFrameInfo *info = (AwsFrameInfo *)arg;
  if (!info->final || info->index != 0 || info->len != len || info->opcode != WS_TEXT) {
    return;
  }
  StaticJsonDocument<200> doc;
  DeserializationError err = deserializeJson(doc, data);
  if (err) {
    if (DEBUG_MODE_SERVER) Serial.println("WS JSON Parse Error");
    return;
  }
  const char *type = doc["type"];
  if (!type) return;

  // =======================================================
  // 1) RELAY-SETZUNG (WebSocket) – KEINE Hardware-Schaltung
  // =======================================================
  if (strcmp(type, "setRelay") == 0) {
    const char *relayId = doc["relay"];
    const char *state = doc["state"];

    bool turnOn = strcmp(state, "on") == 0;

    // ========== Nur Variablen setzen ==========
    if (strcmp(relayId, "relay_a") == 0) sensorData.relay_a = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_b") == 0) sensorData.relay_b = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_c") == 0) sensorData.relay_c = turnOn ? 0 : 1;
    else if (strcmp(relayId, "relay_d") == 0) sensorData.relay_d = turnOn ? 0 : 1;

    // UI & alle Clients aktualisieren
    wsBroadcastSensorData();
  }
}

// ======================================================================
// Mast-Sensor Daten
// ======================================================================
void handleMastBody(AsyncWebServerRequest *request, uint8_t *data, size_t len, size_t index, size_t total) {
  // Wir verarbeiten nur vollständige JSON-Pakete
  if (index == 0) {
    // optional: Start eines neuen Pakets
  }
  if (index + len != total) {
    // noch nicht komplett
    return;
  }
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
  // Pflichtfelder prüfen
  const char *keys[] = {
    "winddir_gemessen", "windspeed_gemessen",
    "gps_lat", "gps_lon",
    "gps_speed", "gps_kurs",
    "gps_sats", "gps_hdop",
    "gps_jahr", "gps_monat", "gps_tag",
    "gps_stunde", "gps_minute", "gps_sekunde"
  };
  for (const char *k : keys) {
    if (!doc.containsKey(k)) {
      request->send(400, "text/plain", String("❌ Wert fehlt: ") + k);
      return;
    }
  }
  // **GPS validieren**
  if (doc["gps_lat"].isNull() || doc["gps_lon"].isNull()) {
    request->send(400, "text/plain", "❌ Ungültige GPS-Koordinaten");
    return;
  }
  // In sensorData übernehmen:
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
  // Mast online setzen
  lastReceiveTime = millis();
  noReceiveCounter = 0;
  sensorData.mast_online = 1;
  request->send(200, "text/plain", "OK");
}

// ======================================================================
// Zyklische Offline-Erkennung in loop() (alle 1000 ms)
// ======================================================================
void checkMastOnlineStatus() {
  unsigned long now = millis();

  // Offline-Überprüfung nur, wenn Mast schon mindestens einmal online war
  if (sensorData.mast_online > 0) {
    if (now - lastReceiveTime > 1000) {  // 1 Sekunde ohne neue Daten
      noReceiveCounter++;

      if (noReceiveCounter >= 5) {
        sensorData.mast_online = 2;  // Offline
      }
    }
  }
}

// ======================================================================
// File.system Datei anzeigen
// ======================================================================
void handleGetFSFile(AsyncWebServerRequest *request, const char *path) {
  if (!LittleFS.exists(path)) {
    request->send(404, "application/json",
                  "{\"error\":\"file not found\"}");
    return;
  }

  AsyncWebServerResponse *response =
    request->beginResponse(LittleFS, path, "application/json");

  response->addHeader("Cache-Control", "no-store");
  request->send(response);
}

// ======================================================================
// PID-Konfiguration (nur Basis-PID)
// ======================================================================
void handleSetConfig(AsyncWebServerRequest *request) {
  // Prüfen, ob alle Basis-PID-Parameter vorhanden sind
  if (!request->hasParam("invert") || !request->hasParam("P_base") || !request->hasParam("I_base") || !request->hasParam("D_base")) {
    request->send(400, "text/plain", "❌ Fehlende Parameter");
    return;
  }

  // Pinnenrichtung
  pinnenautopilotData.pinne_invertieren = request->getParam("invert")->value().toInt();

  // Basis-PID Werte
  pinnenautopilotData.P_base = request->getParam("P_base")->value().toFloat();
  pinnenautopilotData.I_base = request->getParam("I_base")->value().toFloat();
  pinnenautopilotData.D_base = request->getParam("D_base")->value().toFloat();

  // Speichern im LittleFS
  ConfigStorage_saveAutopilot();

  request->send(200, "text/plain", "✅ Konfiguration gespeichert");
}

// ======================================================================
// Autopilot Einstellungen empfangen senden
// ======================================================================
void handleAutopilotData(AsyncWebServerRequest *request) {
  if (request->hasParam("modus")) {
    status_autopilot = request->getParam("modus")->value().toInt();
  }
  if (request->hasParam("pinne")) {
    int p = request->getParam("pinne")->value().toInt();
    motor_manuel_einfahren = (p == 1);
    motor_manuel_ausfahren = (p == 2);
  }
  if (request->hasParam("winkel")) {
    rotary_offset = request->getParam("winkel")->value().toInt();
  }
  request->send(200, "text/plain", "✔ Autopilot-Daten aktualisiert");
}


// ======================================================================
// Autopilot Einstellungen senden
// ======================================================================
void handleAutopilotStatus(AsyncWebServerRequest *request) {
  StaticJsonDocument<256> doc;

  doc["mode"] = status_autopilot;  // entspricht JS: mode
  doc["pinne"] = 0;                // Standard Stop
  if (motor_manuel_einfahren) doc["pinne"] = 1;
  else if (motor_manuel_ausfahren) doc["pinne"] = 2;

  doc["offset"] = rotary_offset;  // entspricht JS: offset/winkel

  String out;
  serializeJson(doc, out);
  request->send(200, "application/json", out);
}

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

  // Stream direkt an den Client senden spart RAM-Kopien
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
// Autopilot Ziel-Koordinaten setzen
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
// Ordnerbaum abfragen und das JSON-Format aufbaut
// ======================================================================
void listDir(fs::FS &fs, String dirname, String &json) {
  File root = fs.open(dirname);
  if (!root || !root.isDirectory()) return;

  File file = root.openNextFile();
  while (file) {
    if (json != "[") json += ",";

    // JSON-Objekt für die aktuelle Datei/den Ordner
    json += "{\"name\":\"" + String(file.path()) + "\",";  // Voller Pfad
    json += "\"size\":" + String(file.size()) + ",";
    json += "\"type\":\"" + String(file.isDirectory() ? "dir" : "file") + "\"}";

    // Wenn es ein Ordner ist, rufe die Funktion rekursiv auf
    if (file.isDirectory()) {
      listDir(fs, file.path(), json);
    }
    file = root.openNextFile();
  }
}

void setupWebServer() {
  // 1. Dateisystem-API (Mapping über ein Array spart 4 separate Handler)
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
  server.on("/setConfig", HTTP_GET, handleSetConfig);
  server.on("/autopilot_data", HTTP_GET, handleAutopilotData);
  server.on("/autopilot_status", HTTP_GET, handleAutopilotStatus);
  server.on("/autopilot", HTTP_GET, handleAutopilotTarget);

  // 3. Gezeiten-API (Hier war der Fehler - Logik wieder inline)
  server.on("/tide_curve", HTTP_GET, handleTideCurve);
  server.on("/tide_station/*", HTTP_GET, handleTideStation);

  // 4. System & Sensoren
  server.on("/getSystem", HTTP_GET, handleGetSystem);
  server.on("/setSystem", HTTP_GET, handleSetSystem);
  server.on(
    "/mastdata", HTTP_POST, [](AsyncWebServerRequest *r) {}, NULL, handleMastBody);
  server.on("/tiles/*", HTTP_GET, handleTiles);

  // 5. Kompakte Kalibrierung
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
  // 6. Datei-Upload Endpunkt (LittleFS)
  server.on(
    "/upload", HTTP_POST,
    [](AsyncWebServerRequest *request) {
      if (request->_tempObject != nullptr) {
        request->send(507, "text/plain", "Fehler: Speicher voll oder Schreibfehler");
      } else {
        request->send(200, "text/plain", "Upload erfolgreich");
      }
    },
    [](AsyncWebServerRequest *request, String filename, size_t index, uint8_t *data, size_t len, bool final) {
      if (index == 0) {
        size_t freeSpace = LittleFS.totalBytes() - LittleFS.usedBytes();
        if (request->contentLength() > freeSpace) {
          request->_tempObject = (void *)1;
          return;
        }
      }
      if (request->_tempObject == nullptr) {
        String path = "/" + filename;
        File file = LittleFS.open(path, index == 0 ? "w" : "a");
        if (file) {
          if (file.write(data, len) != len) request->_tempObject = (void *)1;
          file.close();
        } else {
          request->_tempObject = (void *)1;
        }
      }
    });
  // 7. Dateien des FS auflisten
  server.on("/list_files", HTTP_GET, [](AsyncWebServerRequest *request) {
    String json = "[";

    // Rekursives Auslesen starten
    listDir(LittleFS, "/", json);

    // Speicherinfo hinzufügen (als letztes Element)
    size_t total = LittleFS.totalBytes();
    size_t used = LittleFS.usedBytes();
    if (json != "[") json += ",";
    json += "{\"fs_total\":" + String(total) + ",\"fs_used\":" + String(used) + "}";

    json += "]";
    request->send(200, "application/json", json);
  });
  // 8 WebSocket Handler registrieren (WICHTIG!)
  // "ws" muss deine globale AsyncWebSocket Instanz sein
  server.addHandler(&ws);
  // 8. Statische HTML-Seiten & Verzeichnisse kompakt verwalten
  // Nutze serveStatic für ganze Ordner und fange / ab
  server.serveStatic("/leaflet", LittleFS, "/leaflet/");
  server.serveStatic("/js", LittleFS, "/js/");
  server.serveStatic("/", LittleFS, "/").setDefaultFile("index.html");

  server.begin();
  if (DEBUG_MODE_SERVER) Serial.println("✅ Server optimiert gestartet");
}
