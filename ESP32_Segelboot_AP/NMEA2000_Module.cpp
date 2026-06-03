/**
 * ============================================================================
 * MODUL: NAVIS NMEA2000 Gateway & Parser (CAN-Bus)
 * ============================================================================
 * NMEA2000_Module.cpp
 * Implementierung des NMEA2000-Parsers über den ESP32-internen CAN-Controller.
 * 
 * 
 * Das Modul klinkt sich passiv (ListenOnly) in das NMEA2000-Netzwerk ein. Es filtert 
 * spezifische PGNs (Parameter Group Numbers) für GPS, Wind und Tiefe, konvertiert 
 * die SI-Einheiten des CAN-Busses in maritime Navigationseinheiten und übergibt 
 * diese an die Datenfusion. Zudem überwacht es den Ausfall von Sensordaten per Timeout.
 * 
 * ARCHITEKTUR-REGELN:
 * - Datenübergabe ausschließlich via `NAVIS_push...` an NAVIS_DataFusion.
 * - Keine direkten Schreibzugriffe auf die globale Struktur `extern_sensorData`.
 * - Konsistente Steuerung aller Ausgaben über das Flag `DEBUG_MODE_CANBUS`.
 * ============================================================================

 Hardware zum testen: https://de.aliexpress.com/item/1005012175863956.html

 */
#include "config.h"              // 1. Zuerst die Konfiguration laden

#include <NMEA2000_esp32_twai.h>  // 2. Die neue TWAI-Bibliothek laden
#include <N2kMessages.h>

#include "NMEA2000_Module.h"     // 3. Erst danach die eigenen Projekt-Header
#include "Sensor_Data.h"
#include "NAVIS_DataFusion.h"

/// Instanz des ESP32-spezifischen NMEA2000-Objekts (nutzt den internen TWAI/CAN-Controller)
NMEA2000_esp32_twai NMEA2000;

/**
 * Array der zu filternden PGNs (Parameter Group Numbers).
 * 
 * Enthält alle PGN-Nummern, auf die der CAN-Hardware-Filter reagieren soll:
 * - 126992L: System Time (Systemzeit)
 * - 129025L: Position, Rapid Update (Schnelles Positions-Update)
 * - 129026L: COG & SOG, Rapid Update (Kurs/Geschwindigkeit über Grund)
 * - 128267L: Water Depth (Wassertiefe)
 * - 130306L: Wind Data (Windgeschwindigkeit und -winkel)
 */
const unsigned long ReceiveMessages[] PROGMEM = {
  126992L,
  129025L,
  129026L,
  128267L,
  130306L,
  0  ///< Array-Terminator (wichtig für die Bibliothek)
};

/**
 * Struktur zur Verwaltung der Zeitstempel für die Timeout-Überwachung.
 * Speichert den jeweils letzten Empfangszeitpunkt (`millis()`) pro Sensortyp.
 */
struct SensorEventState {
  uint32_t gps_last = 0;    ///< Letzter Empfang von GPS-Daten [ms]
  uint32_t wind_last = 0;   ///< Letzter Empfang von Winddaten [ms]
  uint32_t depth_last = 0;  ///< Letzter Empfang von Tiefendaten [ms]
};

static SensorEventState sensorEvents;

// Vorwärtsdeklarationen interner Funktionen
void HandleGPS(const tN2kMsg &N2kMsg);
void HandleWind(const tN2kMsg &N2kMsg);
void HandleDepth(const tN2kMsg &N2kMsg);
void updateSensorTimeouts();

// Inline-Hilfsfunktionen zur Aktualisierung der Empfangs-Zeitstempel
inline void eventGPS() {
  sensorEvents.gps_last = millis();
}
inline void eventWind() {
  sensorEvents.wind_last = millis();
}
inline void eventDepth() {
  sensorEvents.depth_last = millis();
}

/**
 * Zentraler Nachrichten-Verteiler (Dispatcher) der NMEA2000-Bibliothek.
 * 
 * N2kMsg Referenz auf die empfangene NMEA2000-Nachrichtenstruktur.
 * Leitet jede eingehende CAN-Bus-Nachricht sequentiell an die spezialisierten 
 * Sub-Parser (GPS, Wind, Tiefe) weiter.
 */
void HandleNMEA2000Msg(const tN2kMsg &N2kMsg) {
  HandleGPS(N2kMsg);
  HandleWind(N2kMsg);
  HandleDepth(N2kMsg);
}

/**
 * Initialisiert den CAN-Bus und konfiguriert das NMEA2000-Geräteprofil.
 * 
 * Setzt Produkt- und Geräteinformationen für den Bus auf, schaltet den Controller 
 * in den sicheren passiven Modus (ListenOnly), verknüpft die PGN-Filterliste und 
 * öffnet den Kommunikationskanal.
 */
void setup_nmea2000() {
  if (DEBUG_MODE_CANBUS) Serial.println(F("Starte NMEA2000..."));

  // Produkt-Metadaten für Netzwerk-Scans festlegen
  NMEA2000.SetProductInformation("00000001", 100, "NAVIS NMEA2000", "1.0.0.0", "1.0.0.0");

  // Geräteklasse definieren (Klasse 1 = Navigation, Funktion 130 = Echolot/Logge/Navigationsgeräte)
  NMEA2000.SetDeviceInformation(1, 130, 25, 2046);

  // Sicherheit: Nur lauschen, um aktive Störungen oder Kollisionen auf dem Bus auszuschließen
  NMEA2000.SetMode(tNMEA2000::N2km_ListenOnly);

  // Aktiviert die PGN-Filterung direkt auf dem CAN-Controller
  NMEA2000.ExtendReceiveMessages(ReceiveMessages);

  // Callback für den Nachrichtenempfang registrieren
  NMEA2000.SetMsgHandler(HandleNMEA2000Msg);

  // CAN-Hardware öffnen
  NMEA2000.Open();

  if (DEBUG_MODE_CANBUS) Serial.println(F("NMEA2000 aktiv"));
}

/**
 * Zyklische Verarbeitung der CAN-Bus-Schnittstelle.
 * 
 * Muss zwingend hochfrequent in der Hauptschleife (loop) aufgerufen werden.
 * Holt anstehende Frames aus dem CAN-Hardware-FIFO ab und stößt die 
 * Schnittstellen-Timeouts an.
 */
void loop_nmea2000() {
  NMEA2000.ParseMessages();
  updateSensorTimeouts();
}

/**
 * Parst GPS-relevante PGNs (129025 und 129026).
 * 
 * N2kMsg Referenz auf die NMEA2000-Nachricht.
 * 
 * Da NMEA2000 Positionsdaten (PGN 129025) und Bewegungsdaten (PGN 129026) getrennt 
 * überträgt, puffert die Funktion die Koordinaten statisch zwischen. Sobald die 
 * Bewegungsdaten (COG/SOG) eintreffen, wird der Gesamtdatensatz an die Datenfusion 
 * gepusht. Die mathematischen Konvertierungen (Bogenmaß/Rad -> Grad und m/s -> Knoten) 
 * erfolgen on-the-fly.
 */
void HandleGPS(const tN2kMsg &N2kMsg) {
  if (!extern_gps_CAN) return;

  static double lat = 0.0;
  static double lon = 0.0;

  // 1. Schritt: Position extrahieren (Rapid Update)
  if (N2kMsg.PGN == 129025L) {
    if (ParseN2kPGN129025(N2kMsg, lat, lon)) {
      eventGPS();
    }
  }

  // 2. Schritt: Kurs & Geschwindigkeit extrahieren und Komplettdatensatz pushen
  else if (N2kMsg.PGN == 129026L) {

    unsigned char SID;
    tN2kHeadingReference ref;
    double cog, sog;

    if (ParseN2kPGN129026(N2kMsg, SID, ref, cog, sog)) {

      NAVIS_pushGPS(
        lat,
        lon,
        msToKnots(sog),  // m/s in Knoten umrechnen
        RadToDeg(cog),   // Radian/Bogenmaß in Grad umrechnen
        0,               // Satellitenanzahl (Nicht in PGN 129026 enthalten)
        0.0,             // HDOP (Nicht in PGN 129026 enthalten)
        NAVIS_SOURCE_NMEA2000);

      eventGPS();

      if (DEBUG_MODE_CANBUS) {
        Serial.print(F("[N2K] GPS "));
        Serial.print(lat, 6);
        Serial.print(F(" "));
        Serial.println(lon, 6);
      }
    }
  }
}

/**
 * Parst die Winddaten-PGN (130306).
 * 
 * N2kMsg Referenz auf die NMEA2000-Nachricht.
 * Extrahiert Windgeschwindigkeit und Windwinkel. Akzeptiert ausschließlich 
 * scheinbaren Wind (`N2kWind_Apparent`), rechnet die Werte in Knoten und 
 * Grad um und leitet sie weiter.
 */
void HandleWind(const tN2kMsg &N2kMsg) {
  if (!extern_wind_CAN) return;

  if (N2kMsg.PGN == 130306L) {

    unsigned char SID;
    double WindSpeed, WindAngle;
    tN2kWindReference WindReference;

    if (ParseN2kPGN130306(
          N2kMsg,
          SID,
          WindSpeed,
          WindAngle,
          WindReference)) {

      // Nur scheinbaren Wind verarbeiten
      if (WindReference == N2kWind_Apparent) {

        NAVIS_pushWind(
          RadToDeg(WindAngle),   // Radian -> Grad
          msToKnots(WindSpeed),  // m/s -> Knoten
          NAVIS_SOURCE_NMEA2000);

        eventWind();
      }
    }
  }
}

/**
 * Parst die Echolot/Wassertiefen-PGN (128267).
 * 
 * N2kMsg Referenz auf die NMEA2000-Nachricht.
 * Extrahiert die reine Tiefe und rechnet den Geber-Offset (z.B. Tiefe ab 
 * Wasserlinie oder ab Kiel) direkt mit ein, bevor der Wert übergeben wird.
 */
void HandleDepth(const tN2kMsg &N2kMsg) {
  if (!extern_echolot_CAN) return;

  if (N2kMsg.PGN == 128267L) {

    unsigned char SID;
    double Depth, Offset, Range;

    if (ParseN2kPGN128267(
          N2kMsg,
          SID,
          Depth,
          Offset,
          Range)) {

      // Übergabe der berechneten Tiefe (Rohwert + Offset)
      NAVIS_pushDepth(
        Depth + Offset,
        NAVIS_SOURCE_NMEA2000);

      eventDepth();
    }
  }
}

/**
 * Überprüft die Einhaltung der maximal zulässigen Sensor-Timeouts.
 * 
 * Vergleicht die Differenz zwischen der aktuellen Systemzeit (`now`) und 
 * dem letzten Paket-Zeitstempel. Wird ein in der `config.h` definierter 
 * Schwellenwert (`TIMEOUT_...`) überschritten, wird der Zeitstempel genullt 
 * und eine Warnmeldung ausgegeben (gesteuert via DEBUG_MODE_CANBUS).
 */
void updateSensorTimeouts() {

  uint32_t now = millis();

  // GPS-Timeout Prüfung
  if (extern_gps_CAN && sensorEvents.gps_last != 0 && (now - sensorEvents.gps_last > TIMEOUT_GPS)) {
    sensorEvents.gps_last = 0;
    if (DEBUG_MODE_CANBUS) Serial.println(F("[WARN] N2K GPS Timeout"));
  }

  // Wind-Timeout Prüfung
  if (extern_wind_CAN && sensorEvents.wind_last != 0 && (now - sensorEvents.wind_last > TIMEOUT_WIND)) {
    sensorEvents.wind_last = 0;
    if (DEBUG_MODE_CANBUS) Serial.println(F("[WARN] N2K Wind Timeout"));
  }

  // Wassertiefen-Timeout Prüfung
  if (extern_echolot_CAN && sensorEvents.depth_last != 0 && (now - sensorEvents.depth_last > TIMEOUT_DEPTH)) {
    sensorEvents.depth_last = 0;
    if (DEBUG_MODE_CANBUS) Serial.println(F("[WARN] N2K Depth Timeout"));
  }
}
