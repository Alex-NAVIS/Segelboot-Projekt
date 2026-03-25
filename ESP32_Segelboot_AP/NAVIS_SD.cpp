// ======================================================================
// Modul: NAVIS_SD
// Erklärung:   SD-Karten-Logging und Dateiverwaltung für das NAVIS-System.
//              Das Modul übernimmt:
//
//              - Initialisierung der SD-Karte
//              - Überwachung der Kartenverfügbarkeit
//              - Tagesordner-Verwaltung
//              - CSV-Datenlogging der Sensordaten
//              - Generierung von GPX- und KML-Tracks aus CSV
//              - Datei-Zugriff für Webserver/Tiles
//
//              Besonderheiten:
//              - automatische Reinitialisierung bei Kartenfehlern
//              - tägliche Log-Struktur
//              - robuste Fehlerbehandlung
// ======================================================================

#include "NAVIS_SD.h"
#include "Sensor_Data.h"
#include "Config.h"
#include <Arduino.h>

// ----------------------------------------------------------
// Status der SD-Karte
// ----------------------------------------------------------

// interner Verfügbarkeitsstatus der SD-Karte
static bool sd_available = false;

// Basisordner für alle Logs
const char* LOG_BASE_PATH = "/LOGS";


// ======================================================================
// Funktion: setup_sd
// Erklärung:   Initialisiert SPI und SD-Karte.
//              Muss einmal im Setup aufgerufen werden.
// ======================================================================
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


// ======================================================================
// Funktion: checkSD
// Erklärung:   Prüft, ob die SD-Karte verfügbar ist und versucht bei
//              Bedarf eine Reinitialisierung. Stellt außerdem sicher,
//              dass der Basisordner existiert.
// Rückgabe:    true = SD einsatzbereit
// ======================================================================
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


// ======================================================================
// Funktion: getLogFolder
// Erklärung:   Ermittelt den Tagesordner basierend auf dem GPS-Datum.
//              Legt den Ordner bei Bedarf automatisch an.
// Rückgabe:    Pfad des Tagesordners oder leerer String bei Fehler
// ======================================================================
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


// ======================================================================
// Funktion: getCSVFile
// Erklärung:   Erzeugt den vollständigen Pfad zur Tages-CSV-Datei.
// ======================================================================
static String getCSVFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/log_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".csv";
}


// ======================================================================
// Funktion: getGPXFile
// Erklärung:   Erzeugt den vollständigen Pfad zur Tages-GPX-Datei.
// ======================================================================
static String getGPXFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/track_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".gpx";
}


// ======================================================================
// Funktion: getKMLFile
// Erklärung:   Erzeugt den vollständigen Pfad zur Tages-KML-Datei.
// ======================================================================
static String getKMLFile() {
  String folder = getLogFolder();
  if (folder.length() == 0) return String("");
  return folder + "/track_" + String(sensorData.gps_jahr) + "_" + String(sensorData.gps_monat) + "_" + String(sensorData.gps_tag) + ".kml";
}


// ======================================================================
// Funktion: write_log
// Erklärung:   Schreibt einen Datensatz der aktuellen Sensordaten in
//              die Tages-CSV-Datei. Erstellt die Datei inklusive
//              Kopfzeile automatisch bei Bedarf.
// ======================================================================
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

  // Kopfzeile (20 Spalten)
  if (newFile) {
    file.println(F("Datum;Zeit;Latitude;Longitude;Satelliten;HDOP;Speed;Kurs;Kompass;"
                   "WindDir_Gemessen;WindSpeed_Gemessen;WindDir_Berechnet;WindSpeed_Berechnet;"
                   "Relay_A;Relay_B;Relay_C;Relay_D;Status_Autopilot;Rotary_Offset;Echolot"));
  }

  char buf[256];

  sprintf(buf, "%04d-%02d-%02d;%02d:%02d:%02d;%.6f;%.6f;%d;%.2f;%.2f;%.2f;%.1f;"
                "%.1f;%.2f;%.1f;%.2f;"
                "%d;%d;%d;%d;%d;%d;%.2f",
          sensorData.gps_jahr, sensorData.gps_monat, sensorData.gps_tag,
          sensorData.gps_stunde, sensorData.gps_minute, sensorData.gps_sekunde,
          sensorData.gps_lat, sensorData.gps_lon,
          sensorData.gps_sats, sensorData.gps_hdop,
          sensorData.gps_speed, sensorData.gps_kurs, (float)sensorData.kompass,
          sensorData.winddir_gemessen, sensorData.windspeed_gemessen,
          sensorData.winddir_berechnet, sensorData.windspeed_berechnet,
          sensorData.relay_a, sensorData.relay_b, sensorData.relay_c, sensorData.relay_d,
          pinnenautopilotData.modus_autopilot, pinnenautopilotData.rotary_offset,
          sensorData.Echolot);

  file.println(buf);
  file.close();

  if (DEBUG_MODE_SD) Serial.println(F("[LOG] CSV-Eintrag geschrieben."));
}


void write_event_log(String type, String text, String dir, String wind, String waves)
{
  if (!checkSD()) return;

  String folder = getLogFolder();
  if (folder.length() == 0) return;

  String filePath = folder + "/events_" +
                    String(sensorData.gps_jahr) + "_" +
                    String(sensorData.gps_monat) + "_" +
                    String(sensorData.gps_tag) + ".csv";

  bool newFile = !SD.exists(filePath);

  File file = SD.open(filePath, FILE_APPEND);
  if (!file)
  {
    if (DEBUG_MODE_SD) Serial.println("[LOG] Event-Datei nicht öffnungsbereit!");
    return;
  }

  if (newFile)
  {
    file.println("Datum;Zeit;Typ;Text;WindDir;WindSpeed;Wellen");
  }

  char buf[256];

  sprintf(buf,"%04d-%02d-%02d;%02d:%02d:%02d;%s;%s;%s;%s;%s",
          sensorData.gps_jahr,
          sensorData.gps_monat,
          sensorData.gps_tag,
          sensorData.gps_stunde,
          sensorData.gps_minute,
          sensorData.gps_sekunde,
          type.c_str(),
          text.c_str(),
          dir.c_str(),
          wind.c_str(),
          waves.c_str());

  file.println(buf);
  file.close();

  if (DEBUG_MODE_SD)
  {
    Serial.print("[EVENT] ");
    Serial.println(buf);
  }
}


// ======================================================================
// Funktion: update_gps_tracks
// Erklärung:   Durchsucht alle Tagesordner und erzeugt aus vorhandenen
//              CSV-Dateien automatisch GPX- und KML-Trackdateien,
//              sofern diese noch nicht existieren.
// ======================================================================
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

    String dateStr = folderName.substring(folderName.lastIndexOf('/') + 1);
    String csvFile = folderName + "/log_" + dateStr + ".csv";
    String gpxFile = folderName + "/track_" + dateStr + ".gpx";
    String kmlFile = folderName + "/track_" + dateStr + ".kml";

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

    // --- GPX erstellen ---
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

    // --- KML erstellen ---
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
      }

      int y, m, d, H, M, S;
      double lat, lon;

      int parsed = sscanf(line.c_str(),
                          "%d-%d-%d;%d:%d:%d;%lf;%lf",
                          &y, &m, &d, &H, &M, &S, &lat, &lon);

      if (parsed < 8) continue;

      fGpx.printf("<trkpt lat=\"%.6f\" lon=\"%.6f\"><time>%04d-%02d-%02dT%02d:%02d:%02dZ</time></trkpt>\n",
                  lat, lon, y, m, d, H, M, S);

      fKml.printf("%.6f,%.6f,0\n", lon, lat);
    }

    fGpx.println(F("</trkseg></trk></gpx>"));
    fGpx.close();

    fKml.println(F("</coordinates></LineString></Placemark></Document></kml>"));
    fKml.close();

    fCsv.close();
  }

  root.close();
}


// ======================================================================
// Funktion: sd_file_exists
// Erklärung:   Prüft, ob eine Datei auf der SD-Karte existiert.
//              Wird z. B. vom Webserver verwendet.
// ======================================================================
bool sd_file_exists(const String& path) {
  if (!checkSD()) return false;
  return SD.exists(path);
}


// ======================================================================
// Funktion: sd_open_file
// Erklärung:   Öffnet eine Datei auf der SD-Karte im gewünschten Modus.
//              Wird primär vom Webserver genutzt.
// ======================================================================
File sd_open_file(const String& path, const char* mode) {
  if (!checkSD()) return File();
  return SD.open(path, mode);
}
