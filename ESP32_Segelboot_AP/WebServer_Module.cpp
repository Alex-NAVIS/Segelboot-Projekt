#include "WebServer_Module.h"
#include <ArduinoJson.h>

// Tracking Mast sensor
unsigned long lastReceiveTime = 0;
int noReceiveCounter = 0;

// ======================================================================
// Globale Instanzen
// ======================================================================
AsyncWebServer server(80);
AsyncWebSocket ws("/ws");

// ======================================================================
// Access Point starten
// ======================================================================
void setupWiFiAP() {
  WiFi.mode(WIFI_AP);
  bool success = WiFi.softAP(AP_SSID, AP_PASSWORD);
  WiFi.setTxPower(WIFI_POWER_5dBm);  // reduzierte Sendeleistung

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

  if (!sd_file_exists(path)) {
    request->send(404, "text/plain", "Tile nicht gefunden");
    return;
  }

  AsyncWebServerResponse *response = request->beginResponse(SD, path, "image/png", false);

  response->addHeader("Cache-Control", "public, max-age=86400");
  request->send(response);
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
// Autopilot Ziel-Koordinate
// ======================================================================
void handleSetTarget(AsyncWebServerRequest *request) {
  if (!request->hasParam("lat") || !request->hasParam("lon")) {
    request->send(400, "text/plain", "❌ Fehlende Parameter");
    return;
  }
  pinnenautopilotData.autopilot_lat = request->getParam("lat")->value().toDouble();
  pinnenautopilotData.autopilot_lon = request->getParam("lon")->value().toDouble();
  request->send(200, "text/plain", "OK");
}

// ======================================================================
// Tide Werte abfragen zu Koordinaten
// ======================================================================
void handleTide(AsyncWebServerRequest *request) {
  Serial.println("[TIDE] Anfrage eingegangen");

  // lat/lon prüfen
  if (!request->hasParam("lat") || !request->hasParam("lon")) {
    request->send(400, "application/json",
                  "{\"status\":\"error\",\"message\":\"missing lat/lon\"}");
    return;
  }

  double lat = request->getParam("lat")->value().toDouble();
  double lon = request->getParam("lon")->value().toDouble();

  time_t now = time(nullptr);
  Serial.print("[TIDE] time(): "); Serial.println(now);

  // Tide-Berechnung durchführen
  if (!tide_query(lat, lon, now)) {
    // Fehler oder keine Daten
    if (tide_status == TIDE_NO_TILES) {
      request->send(200, "application/json",
                    "{\"status\":\"no_data\",\"message\":\"Keine Tidedaten in dieser Region.\"}");
    } else if (tide_status == TIDE_NO_CONSTITUENTS) {
      request->send(200, "application/json",
                    "{\"status\":\"flat\",\"message\":\"Tide vernachlässigbar (z. B. Ostsee).\"}");
    } else {
      request->send(500, "application/json",
                    "{\"status\":\"error\",\"message\":\"Interner Tidefehler.\"}");
    }
    Serial.println("[TIDE] tide_query() fehlgeschlagen");
    Serial.print("[TIDE] lat="); Serial.println(lat, 6);
    Serial.print("[TIDE] lon="); Serial.println(lon, 6);
    return;
  }

  // Tide erfolgreich berechnet
  char json[512];
  snprintf(json, sizeof(json),
           "{"
           "\"status\":\"ok\","
           "\"now\":%.2f,"
           "\"high\":{"
           "\"time\":%ld,"
           "\"height\":%.2f"
           "},"
           "\"low\":{"
           "\"time\":%ld,"
           "\"height\":%.2f"
           "},"
           "\"quality\":%d"
           "}",
           tide_height_now,
           (long)tide_next_high_time,
           tide_next_high_height,
           (long)tide_next_low_time,
           tide_next_low_height,
           tide_quality);

  request->send(200, "application/json", json);
  Serial.println("[TIDE] Tide berechnet");
}



// ======================================================================
// Mast-Sensor Daten
// ======================================================================
void handleMastBody(AsyncWebServerRequest *request,
                    uint8_t *data, size_t len,
                    size_t index, size_t total) {

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
// PID-Konfiguration
// ======================================================================
void handleSetConfig(AsyncWebServerRequest *request) {
  if (!request->hasParam("invert") || !request->hasParam("p1")) {
    request->send(400, "text/plain", "❌ Fehlende Parameter");
    return;
  }
  pinnenautopilotData.pinne_invertieren = request->getParam("invert")->value().toInt();
  pinnenautopilotData.P_welle_1 = request->getParam("p1")->value().toFloat();
  pinnenautopilotData.I_welle_1 = request->getParam("i1")->value().toFloat();
  pinnenautopilotData.D_welle_1 = request->getParam("d1")->value().toFloat();
  pinnenautopilotData.P_welle_2 = request->getParam("p2")->value().toFloat();
  pinnenautopilotData.I_welle_2 = request->getParam("i2")->value().toFloat();
  pinnenautopilotData.D_welle_2 = request->getParam("d2")->value().toFloat();
  pinnenautopilotData.P_welle_3 = request->getParam("p3")->value().toFloat();
  pinnenautopilotData.I_welle_3 = request->getParam("i3")->value().toFloat();
  pinnenautopilotData.D_welle_3 = request->getParam("d3")->value().toFloat();

  // speichere die Autopiloteinstellungen in der Config Storage im little FileSystem
  ConfigStorage_saveAutopilot();

  request->send(200, "text/plain", "✅ Konfiguration gespeichert");
}


// ======================================================================
// Setup Webserver + WebSocket
// ======================================================================
void setupWebServer() {
  // Statische Seiten
  server.on("/", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/index.html");
  });
  server.on("/index.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/index.html");
  });
  server.on("/karte.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/karte.html");
  });
  server.on("/kompass.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/kompass.html");
  });
  server.on("/horizont.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/horizont.html");
  });
  server.on("/settings.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/settings.html");
  });
  server.on("/autopilot.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/autopilot.html");
  });
  server.on("/system_settings.html", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleFile(r, "/system_settings.html");
  });

  // Leaflet
  server.serveStatic("/leaflet", LittleFS, "/leaflet/");

  // JSON & Steuerungen
  server.on("/setConfig", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleSetConfig(r);
  });
  server.on("/autopilot", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleSetTarget(r);
  });
  server.on("/tide", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleTide(r);
  });
  // ===========================
  //  SYSTEM CONFIG API
  // ===========================
  server.on("/getSystem", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleGetSystem(r);
  });

  server.on("/setSystem", HTTP_GET, [](AsyncWebServerRequest *r) {
    handleSetSystem(r);
  });
  // Mast Sensor Daten Empfang
  server.on(
    "/mastdata", HTTP_POST,
    [](AsyncWebServerRequest *request) {},
    NULL,
    handleMastBody);

  // Debug-Handler für alle /tiles/... Anfragen
  server.on("/tiles/*", HTTP_GET, [](AsyncWebServerRequest *request) {
    handleTiles(request);
  });

  // Kalibrierungs-Endpunkt
  server.on("/calibrate", HTTP_GET, [](AsyncWebServerRequest *request) {
    if (request->hasParam("type")) {
      String type = request->getParam("type")->value();

      if (type == "compass") {
        mag_start_cal = true;
        request->send(200, "text/plain", "Kompass Kalibrierung gestartet");
      } else if (type == "gyro") {
        gyro_start_cal = true;
        request->send(200, "text/plain", "Gyro Kalibrierung gestartet");
      } else {
        request->send(400, "text/plain", "Unbekannter Typ");
      }
    } else {
      request->send(400, "text/plain", "Parameter fehlt");
    }
  });

  server.begin();
  if (DEBUG_MODE_SERVER) Serial.println("✅ Async Webserver + WebSocket gestartet");
}
