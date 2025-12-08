#include "NAVIS_SD.h"
#include "Sensor_Data.h"
#include "Config.h"
#include <Arduino.h>

// ----------------------------------------------------------
// Status der SD-Karte
// ----------------------------------------------------------
static bool sd_available = false;

// Basisordner
const char* LOG_BASE_PATH = "/LOGS";

// ----------------------------------------------------------
// SD initialisieren (im Setup aufrufen)
// ----------------------------------------------------------
void setup_sd() {
  // SPI manuell initialisieren
  SPI.begin(SPI_SCK, SPI_MISO, SPI_MOSI, SD_CS_PIN);

  // SD initialisieren
  if (!SD.begin(SD_CS_PIN, SPI)) {
    if (DEBUG_MODE_SD) Serial.println("❌ Fehler: SD-Karte konnte nicht initialisiert werden!");
    sd_available = false;
    return;
  }

  sd_available = true;
  if (DEBUG_MODE_SD) Serial.println("✅ SD-Karte erfolgreich erkannt!");
}

// ----------------------------------------------------------
// Prüfen, ob SD verfügbar, ggf. neu initialisieren
// ----------------------------------------------------------
static bool checkSD() {
  if (!SD.begin(SD_CS_PIN, SPI)) {
    if (sd_available) Serial.println(F("[LOG] SD-Karte entfernt oder Fehler!"));
    sd_available = false;
    return false;
  }

  if (!sd_available) {
    if (DEBUG_MODE_SD) Serial.println(F("[LOG] SD-Karte wieder verfügbar."));
    sd_available = true;
  }

  // Basisordner prüfen
  if (!SD.exists(LOG_BASE_PATH)) {
    if (!SD.mkdir(LOG_BASE_PATH)) {
      if (DEBUG_MODE_SD) Serial.println(F("[LOG] Fehler: Basisordner konnte nicht erstellt werden!"));
      return false;
    }
  }

  return true;
}

// ----------------------------------------------------------
// Tagesordner erstellen / zurückgeben
// ----------------------------------------------------------
static String getLogFolder() {
  char folder[64];
  sprintf(folder, "%s/%04d_%02d_%02d", LOG_BASE_PATH,
          sensorData.gps_jahr, sensorData.gps_monat, sensorData.gps_tag);

  if (!SD.exists(folder)) {
    if (!SD.mkdir(folder)) {
      if (DEBUG_MODE_SD) Serial.println(F("[LOG] Fehler: Tagesordner konnte nicht erstellt werden!"));
      return String("");
    }
  }

  return String(folder);
}

// ----------------------------------------------------------
// Dateinamen
// ----------------------------------------------------------
static String getCSVFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/log_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".csv";
}

static String getGPXFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/track_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".gpx";
}

static String getKMLFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/track_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".kml";
}

// ----------------------------------------------------------
// CSV-Log schreiben
// ----------------------------------------------------------
void write_log() {
  if (!checkSD()) return;

  String csvFile = getCSVFile();
  if (csvFile.length() == 0) return;

  bool newFile = !SD.exists(csvFile);

  File file = SD.open(csvFile, FILE_APPEND);
  if (!file) {
    if (DEBUG_MODE_SD) Serial.println(F("[LOG] Fehler: CSV-Datei nicht öffnungsbereit!"));
    sd_available = false;
    return;
  }

  if (newFile) {
    file.println(F("Datum;Zeit;Latitude;Longitude;Satelliten;HDOP;Speed;Kurs;Kompass;"
                   "WindDir_Gemessen;WindSpeed_Gemessen;WindDir_Berechnet;WindSpeed_Berechnet;"
                   "Relay_A;Relay_B;Relay_C;Relay_D;Status_Autopilot;Rotary_Offset;PID_Wahl;Echolot"));
  }

  char buf[256];
  sprintf(buf, "%04d-%02d-%02d;%02d:%02d:%02d;%.6f;%.6f;%d;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%.2f;%d;%d;%d;%d;%d;%d;%d;%.2f",
          sensorData.gps_jahr, sensorData.gps_monat, sensorData.gps_tag,
          sensorData.gps_stunde, sensorData.gps_minute, sensorData.gps_sekunde,
          sensorData.gps_lat, sensorData.gps_lon,
          sensorData.gps_sats, sensorData.gps_hdop,
          sensorData.gps_speed, sensorData.gps_kurs, sensorData.kompass,
          sensorData.winddir_gemessen, sensorData.windspeed_gemessen,
          sensorData.winddir_berechnet, sensorData.windspeed_berechnet,
          sensorData.relay_a, sensorData.relay_b, sensorData.relay_c, sensorData.relay_d,
          status_autopilot, rotary_offset, PID_wahl,
          sensorData.Echolot);
  file.println(buf);
  file.close();

  if (DEBUG_MODE_SD) Serial.println(F("[LOG] CSV-Eintrag geschrieben."));
}

// ----------------------------------------------------------
// GPX/KML aktualisieren
// ----------------------------------------------------------
void update_gps_tracks() {
  if (!checkSD()) return;

  File root = SD.open(LOG_BASE);
  if (!root) {
    if (DEBUG_MODE_SD) Serial.println(F("[LOG] Fehler: LOGS-Ordner nicht zugänglich!"));
    return;
  }

  File entry;
  while ((entry = root.openNextFile())) {
    if (!entry.isDirectory()) {
      entry.close();
      continue;
    }

    String folderName = entry.name();
    entry.close();

    // Tagesordner und Dateinamen
    String dateStr = folderName.substring(folderName.lastIndexOf('/') + 1);
    String csvFile = folderName + "/log_" + dateStr + ".csv";
    String gpxFile = folderName + "/track_" + dateStr + ".gpx";
    String kmlFile = folderName + "/track_" + dateStr + ".kml";

    // Überspringen, wenn GPX/KML bereits existieren
    if (SD.exists(gpxFile) && SD.exists(kmlFile)) continue;

    if (!SD.exists(csvFile)) {
      if (DEBUG_MODE_SD) Serial.print(F("[LOG] Keine CSV gefunden in "));
      if (DEBUG_MODE_SD) Serial.println(folderName);
      continue;
    }

    File fCsv = SD.open(csvFile);
    if (!fCsv) {
      if (DEBUG_MODE_SD) Serial.print(F("[LOG] CSV nicht lesbar: "));
      if (DEBUG_MODE_SD) Serial.println(csvFile);
      continue;
    }

    // GPX-Datei erstellen
    File fGpx = SD.open(gpxFile, FILE_WRITE);
    if (!fGpx) {
      if (DEBUG_MODE_SD) Serial.print(F("[LOG] GPX nicht öffnungsbereit: "));
      if (DEBUG_MODE_SD) Serial.println(gpxFile);
      fCsv.close();
      continue;
    }
    fGpx.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    fGpx.println(F("<gpx version=\"1.1\" creator=\"Bootsystem\">"));
    fGpx.println(F("<trk><name>Boot Track</name><trkseg>"));

    // KML-Datei erstellen
    File fKml = SD.open(kmlFile, FILE_WRITE);
    if (!fKml) {
      if (DEBUG_MODE_SD) Serial.print(F("[LOG] KML nicht öffnungsbereit: "));
      if (DEBUG_MODE_SD) Serial.println(kmlFile);
      fGpx.close();
      fCsv.close();
      continue;
    }
    fKml.println(F("<?xml version=\"1.0\" encoding=\"UTF-8\"?>"));
    fKml.println(F("<kml xmlns=\"http://www.opengis.net/kml/2.2\">"));
    fKml.println(F("<Document><name>Boot Track</name><Placemark><LineString><coordinates>"));

    bool firstLine = true;
    while (fCsv.available()) {
      String line = fCsv.readStringUntil('\n');
      line.trim();
      if (line.length() == 0) continue;
      if (firstLine) {
        firstLine = false;
        continue;
      }  // Header überspringen

      // CSV-Felder splitten: Datum;Zeit;Latitude;Longitude;...
      int y, m, d, H, M, S;
      double lat, lon;
      int parsed = sscanf(line.c_str(),
                          "%d-%d-%d;%d:%d:%d;%lf;%lf",
                          &y, &m, &d, &H, &M, &S, &lat, &lon);
      if (parsed < 8) {
        if (DEBUG_MODE_SD) Serial.print(F("[LOG] Ungültige CSV-Zeile: "));
        if (DEBUG_MODE_SD) Serial.println(line);
        continue;
      }

      fGpx.printf("<trkpt lat=\"%.6f\" lon=\"%.6f\"><time>%04d-%02d-%02dT%02d:%02d:%02dZ</time></trkpt>\n",
                  lat, lon, y, m, d, H, M, S);
      fKml.printf("%.6f,%.6f,0\n", lon, lat);
    }

    fGpx.println(F("</trkseg></trk></gpx>"));
    fGpx.close();

    fKml.println(F("</coordinates></LineString></Placemark></Document></kml>"));
    fKml.close();

    fCsv.close();
    Serial.print(F("[LOG] GPX/KML aus CSV erstellt für "));
    if (DEBUG_MODE_SD) Serial.println(folderName);
  }

  root.close();
}

// ----------------------------------------------------------
// Datei prüfen (für Webserver / Tiles)
// ----------------------------------------------------------
bool sd_file_exists(const String& path) {
    if (DEBUG_MODE_SD) Serial.print("SD Karte Datei: ");
    if (DEBUG_MODE_SD) Serial.println(path);
  //if (!checkSD()) return false;
  return SD.exists(path);
}

// ----------------------------------------------------------
// Datei öffnen (für Webserver / Tiles)
// ----------------------------------------------------------
File sd_open_file(const String& path, const char* mode) {
  if (!checkSD()) return File();
  return SD.open(path, mode);
}
