/**
 * ============================================================================
 * MODUL: NAVIS NMEA0183 Gateway & Parser (RS-422)
 * ============================================================================
 * @file NMEA0183_Module.cpp
 * @brief Implementierung des seriellen NMEA0183-Parsers über eine RS-422 Schnittstelle.
 * 
 * @details
 * Das Modul liest passiv serielle Datenströme von physischen Sensoren (GPS, Wind, 
 * Echolot) ein, die über den RS-422 Standard übertragen werden. Die Rohdaten 
 * werden extrahiert, validiert und an die zentrale Datenfusion übergeben.
 * 
 * ARCHITEKTUR-REGELN:
 * - Datenübergabe ausschließlich via `NAVIS_push...` an NAVIS_DataFusion.
 * - Keine direkten Schreibzugriffe auf die globale Struktur `extern_sensorData`.
 * - Dynamische Filterung über globale Konfigurations-Flags (`extern_..._RS`).
 * ============================================================================

 Hardware: https://de.aliexpress.com/item/1005006770104535.html?src=google&src=google&albch=shopping&acnt=272-267-0231&isdl=y&slnk=&plac=&mtctp=&albbt=Google_7_shopping&aff_platform=google&aff_short_key=UneMJZVf&gclsrc=aw.ds&albagn=888888&ds_e_adid=726717691648&ds_e_matchtype=search&ds_e_device=c&ds_e_network=g&ds_e_product_group_id=296303633664&ds_e_product_id=de1005006770104535&ds_e_product_merchant_id=5341171192&ds_e_product_country=DE&ds_e_product_language=de&ds_e_product_channel=online&ds_e_product_store_id=&ds_url_v=2&albcp=22054759359&albag=172828878419&isSmbAutoCall=false&needSmbHouyi=false&gad_source=1&gad_campaignid=22054759359&gbraid=0AAAAAoukdWMR8m6Gf8BlywkvRnsXBHPP-&gclid=Cj0KCQjwof_QBhCgARIsADaMzOeWseD2UnvGrUYz6zq-B45FadW4poAKu9fC-GJrNRHHp80WS6WUYMMaAhRKEALw_wcB
 */

#include "NMEA0183_Module.h"
#include <MicroNMEA.h>
#include "config.h"
#include "Sensor_Data.h"
#include "NAVIS_DataFusion.h"

/// Nutzt die Hardware-Schnittstelle UART2 des ESP32 für den RS-422 Empfang
HardwareSerial SerialNMEA0183(2);

static char nmeaBuffer[85]; ///< Puffer für die serielle Rohdaten-Zeile (NMEA Standard bis zu 82 Zeichen)
static MicroNMEA nmea(nmeaBuffer, sizeof(nmeaBuffer)); ///< Parser-Instanz der MicroNMEA-Bibliothek

// Interne Funktionsdeklaration für Custom-Sentences (Wind/Tiefe)
void processCustomSentences(const char* sentence);

/**
 * @brief Initialisiert die serielle Hardware-Schnittstelle für den NMEA-Empfang.
 * 
 * Konfiguriert UART2 auf die marine Standard-Baudrate von 4800 Baud mit der 
 * Konfiguration 8N1. Der TX-Pin wird auf -1 gesetzt, da das Modul rein passiv 
 * (nur RX) operiert.
 */
void setup_nmea0183() {
  if (DEBUG_MODE_RS) Serial.println(F("Starte NMEA0183 (RS422)..."));
  
  // Initialisierung: 4800 Baud, 8 Datenbits, keine Parität, 1 Stoppbit
  SerialNMEA0183.begin(4800, SERIAL_8N1, PIN_RS422_RX, -1);
  
  if (DEBUG_MODE_RS) Serial.println(F("NMEA0183 active"));
}

/**
 * @brief Zyklische Abfrage (Polling) des seriellen Hardware-Puffers.
 * 
 * @note Muss periodisch in der Hauptschleife (loop) aufgerufen werden.
 * 
 * @details
 * Liest eingehende Bytes aus dem UART-Hardwarepuffer, solange Daten bereitstehen. 
 * Sobald ein vollständiger Standard-Datensatz erkannt wird, extrahiert das Modul 
 * bei gesetztem Flag `extern_gps_RS` die GPS-Informationen, skaliert sie und pusht 
 * sie mit der Herkunft `NAVIS_SOURCE_NMEA0183` an die Datenfusion. 
 * Danach erfolgt der Aufruf des Custom-Parsers für Wind- und Tiefendaten.
 */
void loop_nmea0183() {
  while (SerialNMEA0183.available()) {
    char c = SerialNMEA0183.read();

    // Byte in den MicroNMEA-Parser einspeisen
    if (nmea.process(c)) {
      const char* currentSentence = nmea.getSentence();

      // ==========================================================
      // GPS-Standarddatensätze extrahieren
      // ==========================================================
      if (extern_gps_RS && nmea.isValid()) {

        // Konvertierung der MicroNMEA-Integer-Werte in physikalische Einheiten
        NAVIS_pushGPS(
          nmea.getLatitude() / 1000000.0,   // Micro-Grad -> Grad
          nmea.getLongitude() / 1000000.0,  // Micro-Grad -> Grad
          nmea.getSpeed() / 1000.0,         // Milli-Knoten -> Knoten
          nmea.getCourse() / 1000.0,        // Milli-Grad -> Grad
          nmea.getNumSatellites(),          // Anzahl Satelliten
          nmea.getHDOP() / 10.0,            // HDOP-Faktor
          NAVIS_SOURCE_NMEA0183             // Datenherkunft (Physisches RS-422)
        );

        if (DEBUG_MODE_RS) {
          Serial.print(F("[0183] GPS "));
          Serial.print(nmea.getLatitude() / 1000000.0, 6);
          Serial.print(F(" / "));
          Serial.println(nmea.getLongitude() / 1000000.0, 6);
        }
      }

      // ==========================================================
      // WIND / DEPTH (Herstellerspezifisches/Erweitertes Parsing)
      // ==========================================================
      processCustomSentences(currentSentence);

      // Parser-Zustand für den nächsten Satz zurücksetzen
      nmea.clear();
    }
  }
}

/**
 * @brief Filtert und parst herstellerspezifische oder nicht-standardisierte NMEA-Sätze.
 * 
 * @param sentence Zeiger auf den null-terminierten NMEA-String.
 * 
 * @details
 * Verarbeitet gezielt zwei Datensatztypen aus dem seriellen Stream:
 * - **MWV (Wind):** Extrahiert Winkel und Windgeschwindigkeit per `sscanf`. Konvertiert m/s bei Bedarf in Knoten.
 * - **DBT (Tiefe):** Extrahiert die Wassertiefe unter Kiel. Nutzt einen schnellen Komma-Zähler-Algorithmus 
 *   statt `sscanf`, um direkt auf das metrische Feld (3. Feld) zuzugreifen.
 * 
 * Alle validierten Daten werden mit der Kennung `NAVIS_SOURCE_NMEA0183` an die Datenfusion übergeben.
 */
void processCustomSentences(const char* sentence) {
  // Sicherheits- und Validierungsprüfungen vor dem Parsen
  if (!sentence) return;
  if (strlen(sentence) < 6) return;

  // Zeiger auf das 3. Zeichen nach '$' setzen, um den NMEA-Datensatz-Typ zu bestimmen
  const char* talker = &sentence[3];

  // ==========================================================
  // WIND DATENSATZ: MWV (Wind Speed and Angle)
  // ==========================================================
  if (extern_wind_RS && strncmp(talker, "MWV", 3) == 0) {

    double angle = 0.0;
    double speed = 0.0;
    char unit = 'N';    // 'N' = Knoten, 'M' = Meter/Sekunde
    char status = 'V';  // 'A' = Gültig (Valid), 'V' = Ungültig (Void)

    // Format-Parsing: $--MWV,x.x,R,x.x,M,A*hh
    if (sscanf(sentence, "$%*5c,%lf,R,%lf,%c,%c",
      &angle,
      &speed,
      &unit,
      &status) >= 3) {

      // Daten nur übernehmen, wenn der Datensatz als valide markiert ist
      if (status == 'A') {

        // Umrechnung von Meter pro Sekunde (M) in Knoten (Nautische Einheit)
        if (unit == 'M') speed *= 1.94384;

        NAVIS_pushWind(
          angle,
          speed,
          NAVIS_SOURCE_NMEA0183
        );

        if (DEBUG_MODE_RS) {
          Serial.print(F("[0183] Wind "));
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
  else if (extern_echolot_RS && strncmp(talker, "DBT", 3) == 0) {

    double depthMeters = 0.0;
    int commas = 0;
    const char* p = sentence;

    // Manueller, speichereffizienter Parser für das metrische Feld (nach dem 3. Komma)
    // Format: $--DBT,x.x,f,[HIER]x.x,M,x.x,F*hh
    while (*p) {
      if (*p == ',') {
        commas++;
        if (commas == 3) {
          depthMeters = atof(p + 1); // Extrahiert den numerischen Wert direkt nach dem 3. Komma
          break;
        }
      }
      p++;
    }

    // Validierung auf plausible Tiefenwerte
    if (depthMeters > 0.0) {

      NAVIS_pushDepth(
        depthMeters,
        NAVIS_SOURCE_NMEA0183
      );

      if (DEBUG_MODE_RS) {
        Serial.print(F("[0183] Depth "));
        Serial.print(depthMeters);
        Serial.println(F(" m"));
      }
    }
  }
}
