/**
 * ============================================================================
 * MODUL-BESCHREIBUNG: NAVIS NMEA2000 Gateway & Parser
 * ============================================================================
 * Dieses Modul dient dem passiven Einlesen (Listen-Only) von NMEA2000-Busdaten
 * auf einem ESP32. Es filtert spezifische PGNs für GPS, Wind und Echolot (Tiefe).
 * 
 * DATENSTEUERUNG ÜBER EXTERNE FLAGS (Aktivierung / Deaktivierung):
 * Das Modul nutzt globale Kontrollvariablen (Flags), um den Empfang und die 
 * Verarbeitung für einzelne Sensortypen dynamisch zu steuern:
 * - `extern_gps_CAN`: Aktiviert/Deaktiviert das Parsen von GPS-Position, 
 *   Kurs (COG), Geschwindigkeit (SOG) und der UTC-Systemzeit.
 * - `extern_wind_CAN`: Aktiviert/Deaktiviert das Parsen von Windgeschwindigkeit 
 *   und Windwinkel (scheinbarer Wind).
 * - `extern_echolot_CAN`: Aktiviert/Deaktiviert das Parsen der Wassertiefe.
 * 
 * Ist ein Flag auf 'false' gesetzt, werden eingehende Telegramme des Typs 
 * sofort ignoriert, und die Timeout-Überwachung für diesen Sensor ausgesetzt.
 * 
 * Hauptfunktionen:
 * 1. Initialisierung des CAN-Busses auf dedizierten GPIO-Pins.
 * 2. Filtern und Parsen der empfangenen NMEA2000-Telegramme.
 * 3. Konvertierung von Rohdaten (z.B. Bogenmaß/Metrisch zu Grad/Knoten).
 * 4. Speicherung in globalen Datenstrukturen für andere Programmteile.
 * 5. Überwachung von Timeouts: Bleiben Sensordaten für eine definierte Zeit
 *    aus, werden die globalen Werte automatisch auf sichere Default-/Fehlerwerte
 *    zurückgesetzt und eine Warnung ausgegeben.
 * ============================================================================
 */

#include "NMEA2000_Module.h"
#include <NMEA2000_esp32.h>
#include <N2kMessages.h>
#include "config.h"
#include "Sensor_Data.h"


// Zentrales NMEA2000-Objekt für die ESP32-Hardware-Bibliothek
tNMEA2000_esp32 NMEA2000;

// Whitelist-Filter: Nur diese PGNs werden vom Bus verarbeitet (Systemzeit, GPS, COG/SOG, Tiefe, Wind)
const unsigned long ReceiveMessages[] PROGMEM = { 126992L, 129025L, 129026L, 128267L, 130306L, 0 };

// Struktur zur Speicherung der Zeitstempel (millis) des letzten erfolgreichen Datenempfangs
struct SensorEventState { uint32_t gps_last = 0, wind_last = 0, depth_last = 0; };
static SensorEventState sensorEvents; // Instanz für die Timeout-Überwachung

// Vorwärtsdeklarationen der Parser- und Überwachungsfunktionen
void HandleGPS(const tN2kMsg &N2kMsg);
void HandleWind(const tN2kMsg &N2kMsg);
void HandleDepth(const tN2kMsg &N2kMsg);
void updateSensorTimeouts();

// Inline-Hilfsfunktionen: Aktualisieren den Zeitstempel bei gültigem Datenempfang
inline void eventGPS() { sensorEvents.gps_last = millis(); }
inline void eventWind() { sensorEvents.wind_last = millis(); }
inline void eventDepth() { sensorEvents.depth_last = millis(); }

// Hauptverteiler (Dispatcher): Reicht jedes empfangene Telegramm an die Spezial-Parser weiter
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) { HandleGPS(N2kMsg); HandleWind(N2kMsg); HandleDepth(N2kMsg); }

/**
 * Richtet das NMEA2000-Modul ein, setzt Produkt-/Geräteinfos, 
 * aktiviert den passiven Modus, registriert den Handler und startet den CAN-Bus.
 */
void setup_nmea2000() {
  Serial.println(F("Starte NMEA2000..."));
  NMEA2000.SetProductInformation("00000001", 100, "NAVIS NMEA2000", "1.0.0.0", "1.0.0.0");
  NMEA2000.SetDeviceInformation(1, 130, 25, 2046);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenOnly); // Passiver Modus (keine eigenen Sendungen)
  NMEA2000.ExtendReceiveMessages(ReceiveMessages); // PGN-Filter anwenden
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg); // Callback-Funktion registrieren
  NMEA2000.Open(); // CAN-Schnittstelle aktivieren
  Serial.println(F("NMEA2000 aktiv"));
}

/**
 * Muss zyklisch in der Hauptschleife aufgerufen werden. 
 * Verarbeitet anstehende Bus-Nachrichten und prüft Sensor-Timeouts.
 */
void loop_nmea2000() { NMEA2000.ParseMessages(); updateSensorTimeouts(); }

/**
 * Parser für GPS-Daten (PGN 129025: Position, PGN 129026: COG/SOG, PGN 126992: Zeit)
 * Konvertiert Rohwerte in Grad und Knoten und speichert sie global ab.
 */
void HandleGPS(const tN2kMsg &N2kMsg) {
  if (!extern_gps_CAN) return; // Abbruch, wenn GPS-Empfang via CAN deaktiviert ist
  if (N2kMsg.PGN == 129025L) { // Breitengrad / Längengrad
    double lat, lon;
    if (ParseN2kPGN129025(N2kMsg, lat, lon)) {
      extern_sensorData.gps_lat = lat; extern_sensorData.gps_lon = lon; eventGPS();
      Serial.print(F("[N2K] GPS: ")); Serial.print(lat, 6); Serial.print(F(" / ")); Serial.println(lon, 6);
    }
  } else if (N2kMsg.PGN == 129026L) { // Kurs über Grund (COG) / Geschwindigkeit über Grund (SOG)
    unsigned char SID; tN2kHeadingReference ref; double cog, sog;
    if (ParseN2kPGN129026(N2kMsg, SID, ref, cog, sog)) {
      extern_sensorData.gps_kurs = RadToDeg(cog); extern_sensorData.gps_speed = msToKnots(sog); eventGPS();
      Serial.print(F("[N2K] COG: ")); Serial.print(extern_sensorData.gps_kurs); Serial.print(F("  SOG: ")); Serial.println(extern_sensorData.gps_speed);
    }
  } else if (N2kMsg.PGN == 126992L) { // Systemzeit / Datum (UTC)
    unsigned char SID; uint16_t days; double sec; tN2kTimeSource ts;
    if (ParseN2kPGN126992(N2kMsg, SID, days, sec, ts)) {
      extern_sensorData.gps_tag = days; extern_sensorData.gps_sekunde = sec; eventGPS();
      Serial.println(F("[N2K] GPS Zeit empfangen"));
    }
  }
}

/**
 * Parser für Winddaten (PGN 130306). Filtert nach relativem Wind (Apparent),
 * konvertiert Winkel in Grad sowie Geschwindigkeit in Knoten.
 */
void HandleWind(const tN2kMsg &N2kMsg) {
  if (!extern_wind_CAN) return; // Abbruch, wenn Wind-Empfang via CAN deaktiviert ist
  if (N2kMsg.PGN == 130306L) {
    unsigned char SID; double WindSpeed, WindAngle; tN2kWindReference WindReference;
    if (ParseN2kPGN130306(N2kMsg, SID, WindSpeed, WindAngle, WindReference)) {
      if (WindReference == N2kWind_Apparent) { // Nur scheinbaren Wind verarbeiten
        extern_sensorData.winddir_gemessen = RadToDeg(WindAngle); extern_sensorData.windspeed_gemessen = msToKnots(WindSpeed); eventWind();
        Serial.print(F("[N2K] Wind: ")); Serial.print(extern_sensorData.windspeed_gemessen); Serial.print(F(" kn  ")); Serial.println(extern_sensorData.winddir_gemessen);
      }
    }
  }
}

/**
 * Parser für Echolot/Wassertiefe (PGN 128267). 
 * Berechnet die Gesamttiefe aus gemessener Tiefe und dem Sensor-Offset.
 */
void HandleDepth(const tN2kMsg &N2kMsg) {
  if (!extern_echolot_CAN) return; // Abbruch, wenn Echolot-Empfang via CAN deaktiviert ist
  if (N2kMsg.PGN == 128267L) {
    unsigned char SID; double Depth, Offset, Range;
    if (ParseN2kPGN128267(N2kMsg, SID, Depth, Offset, Range)) {
      extern_sensorData.Echolot = Depth + Offset; eventDepth(); // Tiefe inklusive Geber-Offset (z.B. Kiel/Wasserlinie)
      Serial.print(F("[N2K] Depth: ")); Serial.println(extern_sensorData.Echolot);
    }
  }
}

/**
 * Prüft fortlaufend, ob die Zeitspanne seit dem letzten Signal eines Sensors 
 * den definierten Maximalwert (Timeout) überschritten hat. Wenn ja, werden
 * die globalen Variablen genullt bzw. auf definierte Fehlerwerte gesetzt.
 */
void updateSensorTimeouts() {
  uint32_t now = millis();
  
  // GPS-Timeout-Überprüfung
  if (extern_gps_CAN && sensorEvents.gps_last != 0 && (now - sensorEvents.gps_last > TIMEOUT_GPS)) {
    extern_sensorData.gps_lat = 0.0; extern_sensorData.gps_lon = 0.0; extern_sensorData.gps_speed = 0.0; extern_sensorData.gps_kurs = 0.0;
    extern_sensorData.gps_sats = 0; extern_sensorData.gps_hdop = 99.9; sensorEvents.gps_last = 0;
    Serial.println(F("[WARN] GPS Timeout"));
  }
  
  // Wind-Timeout-Überprüfung
  if (extern_wind_CAN && sensorEvents.wind_last != 0 && (now - sensorEvents.wind_last > TIMEOUT_WIND)) {
    extern_sensorData.winddir_gemessen = -999.0; extern_sensorData.windspeed_gemessen = -1.0; sensorEvents.wind_last = 0;
    Serial.println(F("[WARN] Wind Timeout"));
  }
  
  // Echolot-Timeout-Überprüfung
  if (extern_echolot_CAN && sensorEvents.depth_last != 0 && (now - sensorEvents.depth_last > TIMEOUT_DEPTH)) {
    extern_sensorData.Echolot = -1.0; sensorEvents.depth_last = 0;
    Serial.println(F("[WARN] Depth Timeout"));
  }
}
