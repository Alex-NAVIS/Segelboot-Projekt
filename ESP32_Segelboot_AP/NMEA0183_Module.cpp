/**
 * ============================================================================
 * MODUL-BESCHREIBUNG: NAVIS NMEA0183 Gateway & Parser (RS-422)
 * ============================================================================
 * Dieses Modul dient dem passiven Einlesen (Listen-Only) von NMEA0183-Daten
 * über eine serielle RS-422 Schnittstelle auf einem ESP32.
 * 
 * DATENSTEUERUNG ÜBER EXTERNE FLAGS (Aktivierung / Deaktivierung):
 * Das Modul nutzt globale Kontrollvariablen (Flags), um den Empfang und die 
 * Verarbeitung für einzelne Sensortypen dynamisch zu steuern:
 * - `extern_gps_RS`: Aktiviert/Deaktiviert das Parsen von GPS-Position, 
 *   Kurs (COG), Geschwindigkeit (SOG), Satellitenanzahl und HDOP.
 * - `extern_wind_RS`: Aktiviert/Deaktiviert das Parsen von Windgeschwindigkeit 
 *   und Windwinkel (scheinbarer Wind).
 * - `extern_echolot_RS`: Aktiviert/Deaktiviert das Parsen der Wassertiefe.
 * 
 * Ist ein Flag auf 'false' gesetzt, werden eingehende Telegramme des Typs 
 * sofort ignoriert, und die Timeout-Überwachung für diesen Sensor ausgesetzt.
 * 
 * Hauptfunktionen:
 * 1. Initialisierung der HardwareSerial-Schnittstelle auf definierten Pins.
 * 2. Filtern und Parsen der empfangenen NMEA0183-Telegramme via MicroNMEA.
 * 3. Manueller, zeigerbasierter Fallback-Parser für Wind- und Tiefendaten.
 * 4. Speicherung in globalen Datenstrukturen für andere Programmteile.
 * 5. Überwachung von Timeouts: Bleiben Sensordaten für eine definierte Zeit
 *    aus, werden die globalen Werte automatisch auf sichere Default-/Fehlerwerte
 *    zurückgesetzt und eine Warnung (falls DEBUG_MODE_RS aktiv) ausgegeben.
 * ============================================================================
 */

#include "NMEA0183_Module.h"
#include <MicroNMEA.h>
#include "config.h"
#include "Sensor_Data.h"

// Zentrales Serielles Objekt (Port 2) für die RS-422 Hardware-Schnittstelle
HardwareSerial SerialNMEA0183(2);

// Interner Puffer für den MicroNMEA Parser (Sätze sind laut Standard max. 82 Zeichen lang)
char nmeaBuffer[85];
MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer));

// Struktur zur Speicherung der Zeitstempel (millis) des letzten erfolgreichen Datenempfangs
struct SensorEventState0183 { uint32_t gps_last = 0, wind_last = 0, depth_last = 0; };
static SensorEventState0183 sensorEvents0183; // Instanz für die Timeout-Überwachung

// Vorwärtsdeklarationen der Parser- und Überwachungsfunktionen
void processCustomSentences(const char* sentence);
void updateSensorTimeouts0183();

// Inline-Hilfsfunktionen: Aktualisieren den Zeitstempel bei gültigem Datenempfang
inline void eventGPS0183()   { sensorEvents0183.gps_last = millis(); }
inline void eventWind0183()  { sensorEvents0183.wind_last = millis(); }
inline void eventDepth0183() { sensorEvents0183.depth_last = millis(); }

/**
 * Richtet das NMEA0183-Modul ein, setzt die Baudrate für RS-422,
 * deaktiviert den TX-Kanal für den passiven Modus und startet die Schnittstelle.
 */
void setup_nmea0183() {
  if (DEBUG_MODE_RS) Serial.println(F("Starte NMEA0183 (RS-422)..."));
  // 4800 Baud Standard, TX auf -1 für Listen-Only (passiv)
  SerialNMEA0183.begin(4800, SERIAL_8N1, PIN_RS422_RX, -1);
  if (DEBUG_MODE_RS) Serial.println(F("NMEA0183 aktiv"));
}

/**
 * Muss zyklisch in der Hauptschleife aufgerufen werden. 
 * Verarbeitet anstehende serielle Zeichen und prüft Sensor-Timeouts.
 */
void loop_nmea0183() {
  while (SerialNMEA0183.available()) {
    char c = SerialNMEA0183.read();

    // MicroNMEA verarbeitet Standard-GPS-Sätze (RMC, GGA, GLL etc.) automatisch intern
    if (nmea.process(c)) {
      const char* currentSentence = nmea.getSentence();

      // 1. GPS verarbeiten, falls Flag aktiv und Datensatz valide ist
      if (extern_gps_RS && nmea.isValid()) {
        extern_sensorData.gps_lat = nmea.getLatitude() / 1000000.0;
        extern_sensorData.gps_lon = nmea.getLongitude() / 1000000.0;
        extern_sensorData.gps_speed = nmea.getSpeed() / 1000.0;  // Konvertierung in Knoten
        extern_sensorData.gps_kurs = nmea.getCourse() / 1000.0;  // Konvertierung in Grad
        extern_sensorData.gps_sats = nmea.getNumSatellites();
        extern_sensorData.gps_hdop = nmea.getHDOP() / 10.0;
        eventGPS0183();
        
        if (DEBUG_MODE_RS) {
          Serial.print(F("[0183] GPS: ")); Serial.print(extern_sensorData.gps_lat, 6);
          Serial.print(F(" / ")); Serial.println(extern_sensorData.gps_lon, 6);
        }
      }

      // 2. Nicht-GPS Datensätze (Wind, Tiefe) über den Custom-Parser verarbeiten
      processCustomSentences(currentSentence);

      // Parser-Zustand für den nächsten Satz vollständig zurücksetzen
      nmea.clear();
    }
  }
  updateSensorTimeouts0183();
}

/**
 * Robuster, zeigerbasierter Parser für Wind (MWV) und Tiefe (DBT/DPT).
 * Filtert herstellerspezifische Datensätze und konvertiert Rohwerte.
 */
void processCustomSentences(const char* sentence) {
  if (sentence == nullptr || strlen(sentence) < 6) return;

  // Prüfe die dreistelligen Satz-Identifier ab Zeichen 3 (z.B. $IIMWV -> "MWV")
  const char* talker = &sentence[3];

  // --- WIND PARSING ($--MWV) ---
  if (extern_wind_RS && strncmp(talker, "MWV", 3) == 0) {
    // Format: $--MWV,x.x,R,y.y,N,A*hh (Winkel, Relativ, Speed, Einheit, Status)
    double angle = 0.0, speed = 0.0;
    char status = 'V', unit = 'N';

    // Sicheres Extrahieren der Variablen per sscanf
    if (sscanf(sentence, "$%*5c,%lf,R,%lf,%c,%c", &angle, &speed, &unit, &status) >= 3) {
      if (status == 'A') {  // 'A' = Datensatz ist gültig (Valid)
        extern_sensorData.winddir_gemessen = angle;
        if (unit == 'M') {                                         
          extern_sensorData.windspeed_gemessen = speed * 1.94384;  // m/s in Knoten umrechnen
        } else {
          extern_sensorData.windspeed_gemessen = speed;  // Standard ist bereits Knoten ('N')
        }
        eventWind0183();
        
        if (DEBUG_MODE_RS) {
          Serial.print(F("[0183] Wind: ")); Serial.print(extern_sensorData.windspeed_gemessen);
          Serial.println(F(" kn"));
        }
      }
    }
  }

  // --- TIEFEN PARSING ($--DBT) ---
  else if (extern_echolot_RS && strncmp(talker, "DBT", 3) == 0) {
    // Format: $--DBT,x.x,f,y.y,M,z.z,F*hh (Meter-Wert steht vor dem 'M')
    double depthMeters = 0.0;
    int commas = 0;
    const char* p = sentence;
    
    // Iteriere durch die Zeichen, um den exakten Wert vor dem 'M' (3. Komma) zu finden
    while (*p) {
      if (*p == ',') {
        commas++;
        if (commas == 3) {  // Nach dem dritten Komma folgt der numerische Meter-Wert
          depthMeters = atof(p + 1);
          break;
        }
      }
      p++;
    }

    if (depthMeters > 0.0) {
      extern_sensorData.Echolot = depthMeters;
      eventDepth0183();
      
      if (DEBUG_MODE_RS) {
        Serial.print(F("[0183] Tiefe: ")); Serial.print(extern_sensorData.Echolot);
        Serial.println(F(" m"));
      }
    }
  }
}

/**
 * Prüft fortlaufend, ob die Zeitspanne seit dem letzten Signal eines Sensors 
 * den definierten Maximalwert (Timeout) überschritten hat. Wenn ja, werden
 * die globalen Variablen genullt bzw. auf definierte Fehlerwerte gesetzt.
 */
void updateSensorTimeouts0183() {
  uint32_t now = millis();

  // GPS-Timeout-Überprüfung
  if (extern_gps_RS && sensorEvents0183.gps_last != 0 && (now - sensorEvents0183.gps_last > TIMEOUT_GPS)) {
    extern_sensorData.gps_lat = 0.0; extern_sensorData.gps_lon = 0.0; extern_sensorData.gps_speed = 0.0; extern_sensorData.gps_kurs = 0.0;
    extern_sensorData.gps_sats = 0; extern_sensorData.gps_hdop = 99.9; sensorEvents0183.gps_last = 0;
    if (DEBUG_MODE_RS) Serial.println(F("[WARN] RS-422 GPS Timeout"));
  }

  // Wind-Timeout-Überprüfung
  if (extern_wind_RS && sensorEvents0183.wind_last != 0 && (now - sensorEvents0183.wind_last > TIMEOUT_WIND)) {
    extern_sensorData.winddir_gemessen = -999.0; extern_sensorData.windspeed_gemessen = -1.0; sensorEvents0183.wind_last = 0;
    if (DEBUG_MODE_RS) Serial.println(F("[WARN] RS-422 Wind Timeout"));
  }

  // Echolot-Timeout-Überprüfung
  if (extern_echolot_RS && sensorEvents0183.depth_last != 0 && (now - sensorEvents0183.depth_last > TIMEOUT_DEPTH)) {
    extern_sensorData.Echolot = -1.0; sensorEvents0183.depth_last = 0;
    if (DEBUG_MODE_RS) Serial.println(F("[WARN] RS-422 Depth Timeout"));
  }
}
