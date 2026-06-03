/**
 * ============================================================================
 * MODUL: NAVIS Netzwerk Parser (UDP + TCP)
 * ============================================================================
 * @file Network_Parser_Module.cpp
 * @brief Implementierung des asynchronen NMEA0183-Parsers über UDP und TCP.
 * 
 * @details
 * Das Modul stellt Netzwerkdienste bereit, um maritime Sensordaten (GPS, Wind, 
 * Tiefe) im NMEA0183-Format zu empfangen. Es verarbeitet standardisierte und 
 * spezifische Datensätze und leitet die extrahierten Werte an die zentrale 
 * Datenfusion weiter.
 * 
 * ARCHITEKTUR-REGELN:
 * - Datenübergabe ausschließlich via `NAVIS_push...` an NAVIS_DataFusion.
 * - Keine direkten Schreibzugriffe auf die globale Struktur `extern_sensorData`.
 * - Dynamische Filterung über globale Konfigurations-Flags.
 * ============================================================================
 */

#include "Network_Parser_Module.h"
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <MicroNMEA.h>
#include "config.h"
#include "Sensor_Data.h"
#include "NAVIS_DataFusion.h"

/// Standard-Port für NMEA0183-Datenströme über IP (RFC 2734 / Industriestandard)
static const uint16_t NMEA_PORT = 10110;

static WiFiUDP udpReceiver;               ///< UDP-Instanz für Broadcast/Multicast-Empfang
static WiFiServer tcpServer(NMEA_PORT);   ///< TCP-Server zur Bereitstellung einer dedizierten Verbindung
static WiFiClient tcpClient;               ///< Aktiver TCP-Client (Single-Client-Prinzip)

static char netBuffer[90];                ///< Puffer für die Rohdaten-Zeile (NMEA max. 82 Zeichen + Reserve)
static MicroNMEA netNmea(netBuffer, sizeof(netBuffer)); ///< Parser-Instanz der MicroNMEA-Bibliothek

// Interne Funktionsdeklaration für Custom-Sentences (Wind/Tiefe)
void processNetworkCustomSentences(const char* sentence);

/**
 * @brief Initialisiert die Netzwerkschnittstellen für den NMEA-Empfang.
 * 
 * Startet den UDP-Empfänger und den TCP-Server auf dem definierten Port (10110).
 * Aktiviert zudem die Option 'setNoDelay', um Paket-Verzögerungen im TCP-Stack 
 * zu minimieren (wichtig für Echtzeit-Sensordaten).
 */
void setup_network_parser() {
  if (DEBUG_MODE_NET) {
    Serial.printf("[NET] Starte Parser Port %u\n", NMEA_PORT);
  }

  udpReceiver.begin(NMEA_PORT);

  tcpServer.begin();
  tcpServer.setNoDelay(true);

  if (DEBUG_MODE_NET) {
    Serial.println(F("[NET] UDP + TCP aktiv"));
  }
}

/**
 * @brief Verarbeitet den eingehenden Byte-Stream einer aktiven Schnittstelle.
 * 
 * @param stream Referenz auf den aktiven Datenstrom (z.B. UDP-Paket oder TCP-Client).
 * 
 * @details
 * Liest verfügbare Bytes sequentiell ein. Sobald ein vollständiger und valider 
 * NMEA-Standarddatensatz erkannt wird, werden die GPS-Daten (Position, Speed, 
 * Course, Satelliten, HDOP) extrahiert, skaliert und an die Datenfusion übergeben.
 * Anschließend wird die Verarbeitung herstellerspezifischer Sätze angestoßen.
 */
void parseNetworkStream(Stream& stream) {
  while (stream.available()) {

    char c = stream.read();

    if (netNmea.process(c)) {

      // ==========================================================
      // GPS-Standarddatensätze extrahieren
      // ==========================================================
      if (extern_gps_UDP_tcp && netNmea.isValid()) {

        // Konvertierung der MicroNMEA-Integer-Werte in physikalische Einheiten
        NAVIS_pushGPS(
          netNmea.getLatitude() / 1000000.0,   // Micro-Grad -> Grad
          netNmea.getLongitude() / 1000000.0,  // Micro-Grad -> Grad
          netNmea.getSpeed() / 1000.0,         // Milli-Knoten -> Knoten
          netNmea.getCourse() / 1000.0,        // Milli-Grad -> Grad
          netNmea.getNumSatellites(),          // Anzahl Satelliten
          netNmea.getHDOP() / 10.0,            // HDOP-Faktor
          NAVIS_SOURCE_NETWORK                 // Datenherkunft
        );

        if (DEBUG_MODE_NET) {
          Serial.print(F("[NET] GPS "));
          Serial.print(netNmea.getLatitude() / 1000000.0, 6);
          Serial.print(F(" / "));
          Serial.println(netNmea.getLongitude() / 1000000.0, 6);
        }
      }

      // Übergabe des Rohsatzes an den Custom-Parser (für MWV, DBT etc.)
      processNetworkCustomSentences(netNmea.getSentence());

      // Parser-Zustand für den nächsten Satz zurücksetzen
      netNmea.clear();
    }
  }
}

/**
 * @brief Zyklische Abfrage (Polling) der Netzwerk-Schnittstellen.
 * 
 * @note Muss periodisch in der Hauptschleife (loop) aufgerufen werden.
 * 
 * @details
 * 1. UDP-Prüfung: Falls ein Paket bereitsteht, wird es direkt geparst.
 * 2. TCP-Verwaltung: Neue Verbindungsanfragen werden akzeptiert. Bestehende 
 *    Verbindungen werden überschrieben (Es wird immer nur der neueste Client bedient).
 * 3. TCP-Daten: Liest Daten vom verbundenen Client, sofern dieser aktiv ist.
 */
void loop_network_parser() {

  // ==========================================================
  // UDP-Verarbeitung
  // ==========================================================
  int packetSize = udpReceiver.parsePacket();

  if (packetSize > 0) {
    parseNetworkStream(udpReceiver);
  }

  // ==========================================================
  // TCP-Verarbeitung (Verbindungs-Management & Daten-Streaming)
  // ==========================================================
  if (tcpServer.hasClient()) {

    // Bestehenden Client trennen, um Ressourcen für den neuen Client freizugeben
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop();
    }

    // Neuen Client akzeptieren
    tcpClient = tcpServer.available();

    if (DEBUG_MODE_NET) {
      Serial.println(F("[TCP] Client verbunden"));
    }
  }

  // Daten vom aktiven TCP-Client streamen
  if (tcpClient && tcpClient.connected()) {
    parseNetworkStream(tcpClient);
  }
}

/**
 * @brief Filtert und parst herstellerspezifische oder nicht-standardisierte NMEA-Sätze.
 * 
 * @param sentence Zeiger auf den null-terminierten NMEA-String.
 * 
 * @details
 * Nutzt `sscanf`, um gezielt Datenstrukturen aus vordefinierten Datensätzen zu extrahieren:
 * - **MWV (Wind):** Extrahiert Winkel, Geschwindigkeit und Einheit. Rechnet m/s bei Bedarf in Knoten um.
 * - **DBT (Tiefe):** Extrahiert die Wassertiefe unter Kiel (bevorzugt metrische Werte).
 * 
 * Die Daten werden nach erfolgreicher Validierung direkt an die Datenfusion übergeben.
 */
void processNetworkCustomSentences(const char* sentence) {
  // Sicherheits- und Validierungsprüfungen vor dem Parsen
  if (!sentence) return;
  if (sentence[0] != '$') return;
  if (strlen(sentence) < 10) return;

  // Zeiger auf das 3. Zeichen nach '$' setzen, um den NMEA-Datensatz-Typ zu bestimmen
  const char* talker = &sentence[3];

  // ==========================================================
  // WIND DATENSATZ: MWV (Wind Speed and Angle)
  // ==========================================================
  if (extern_wind_UDP_tcp && strncmp(talker, "MWV", 3) == 0) {

    double angle = 0.0;
    double speed = 0.0;
    char unit = 'N';    // 'N' = Knoten, 'M' = Meter/Sekunde
    char status = 'V';  // 'A' = Gültig (Valid), 'V' = Ungültig (Void)

    // Format-Parsing: $--MWV,x.x,R,x.x,M,A*hh
    if (sscanf(sentence, "$%*5c,%lf,R,%lf,%c,%c",
      &angle,
      &speed,
      &unit,
      &status) == 4) {

      // Daten nur übernehmen, wenn der Datensatz als valide markiert ist
      if (status == 'A') {

        // Umrechnung von Meter pro Sekunde (M) in Knoten (Nautische Einheit)
        if (unit == 'M') speed *= 1.94384;

        NAVIS_pushWind(
          angle,
          speed,
          NAVIS_SOURCE_NETWORK
        );

        if (DEBUG_MODE_NET) {
          Serial.print(F("[NET] Wind "));
          Serial.print(speed);
          Serial.print(F(" kn "));
          Serial.println(angle);
        }
      }
    }
  }

  // ==========================================================
  // TIEFEN DATENSATZ: DBT (Depth Below Transducer)
  // ==========================================================
  else if (extern_echolot_UDP_tcp && strncmp(talker, "DBT", 3) == 0) {

    double depthFeet = 0.0;
    double depthMeters = 0.0;
    double depthFathoms = 0.0;

    // Format-Parsing: $--DBT,x.x,f,x.x,M,x.x,F*hh
    if (sscanf(sentence,
      "$%*5c,%lf,f,%lf,M,%lf,F",
      &depthFeet,
      &depthMeters,
      &depthFathoms) >= 2) { // Mindestens Feet und Meter müssen vorhanden sein

      // Validierung auf plausible Tiefenwerte
      if (depthMeters > 0.0) {

        NAVIS_pushDepth(
          depthMeters,
          NAVIS_SOURCE_NETWORK
        );

        if (DEBUG_MODE_NET) {
          Serial.print(F("[NET] Depth "));
          Serial.print(depthMeters);
          Serial.println(F(" m"));
        }
      }
    }
  }
}
