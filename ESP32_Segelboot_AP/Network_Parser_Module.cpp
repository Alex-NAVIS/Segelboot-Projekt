/**
 * ============================================================================
 * MODUL-BESCHREIBUNG: NAVIS Netzwerk NMEA0183 Kombi-Server (UDP & TCP)
 * ============================================================================
 * Dieses Modul ermöglicht das GLEICHZEITIGE passive Einlesen von NMEA0183-Daten
 * über UDP-Broadcasts sowie über feste TCP-Client-Verbindungen auf Port 10110.
 * Es läuft vollkompatibel im Hintergrund deines bestehenden WLAN-Access-Points.
 * 
 * DATENSTEUERUNG ÜBER EXTERNE FLAGS (Aktivierung / Deaktivierung):
 * Das Modul nutzt deine globalen Kontrollvariablen (Flags):
 * - `extern_gps_UDP_tcp`: Aktiviert/Deaktiviert das Parsen von GPS-Netzwerkdaten.
 * - `extern_wind_UDP_tcp`: Aktiviert/Deaktiviert das Parsen von Wind-Netzwerkdaten.
 * - `extern_echolot_UDP_tcp`: Aktiviert/Deaktiviert das Parsen von Tiefen-Netzwerkdaten.
 * ============================================================================
 */

#include "Network_Parser_Module.h"
#include <WiFiUdp.h>
#include <WiFiServer.h>
#include <WiFiClient.h>
#include <MicroNMEA.h>
#include "config.h"
#include "Sensor_Data.h"

// Definition des maritimen Standardports
static const uint16_t NMEA_PORT = 10110;

// Instanzen für beide Netzwerk-Protokolle
static WiFiUDP udpReceiver;
static WiFiServer tcpServer(NMEA_PORT);
static WiFiClient tcpClient;

// Interner Puffer für den MicroNMEA Netzwerk-Parser
static char netBuffer[90];
static MicroNMEA netNmea(netBuffer, sizeof(netBuffer));

// Struktur zur Speicherung der Zeitstempel (millis) für die Timeout-Überwachung
struct SensorEventStateNET { 
  uint32_t gps_last = 0, wind_last = 0, depth_last = 0; 
};
static SensorEventStateNET sensorEventsNET;

// Vorwärtsdeklarationen
void processNetworkCustomSentences(const char* sentence);
void updateSensorTimeoutsNET();

inline void eventGPSNET()   { sensorEventsNET.gps_last = millis(); }
inline void eventWindNET()  { sensorEventsNET.wind_last = millis(); }
inline void eventDepthNET() { sensorEventsNET.depth_last = millis(); }

/**
 * Initialisiert sowohl den UDP-Listener als auch den TCP-Server parallel
 * auf dem Port 10110. Knüpft nahtlos an das bestehende WLAN an.
 */
void setup_network_parser() {
  if (DEBUG_MODE_NET) Serial.printf("[NET] Initialisiere Kombi-Parser auf Port %u...\n", NMEA_PORT);
  
  // 1. UDP-Marktschreier starten
  udpReceiver.begin(NMEA_PORT);
  
  // 2. TCP-Postbote starten
  tcpServer.begin();
  tcpServer.setNoDelay(true);

  if (DEBUG_MODE_NET) Serial.println(F("[NET] UDP & TCP Netzwerk-Eingänge aktiv"));
}

/**
 * Hauptschleife: Prüft parallel, ob Daten via UDP reinkommen ODER ob 
 * ein Client Daten über eine stehende TCP-Verbindung schickt.
 */
void loop_network_parser() {
  
  // ==========================================
  // WEG 1: UDP-DATEN EINLESEN (Marktschreier)
  // ==========================================
  int packetSize = udpReceiver.parsePacket();
  if (packetSize > 0) {
    while (udpReceiver.available()) {
      char c = udpReceiver.read();
      if (netNmea.process(c)) {
        // GPS & Custom-Sätze verarbeiten
        if (extern_gps_UDP_tcp && netNmea.isValid()) {
          extern_sensorData.gps_lat = netNmea.getLatitude() / 1000000.0;
          extern_sensorData.gps_lon = netNmea.getLongitude() / 1000000.0;
          extern_sensorData.gps_speed = netNmea.getSpeed() / 1000.0;
          extern_sensorData.gps_kurs = netNmea.getCourse() / 1000.0;
          extern_sensorData.gps_sats = netNmea.getNumSatellites();
          extern_sensorData.gps_hdop = netNmea.getHDOP() / 10.0;
          eventGPSNET();
        }
        processNetworkCustomSentences(netNmea.getSentence());
        netNmea.clear();
      }
    }
  }

  // ==========================================
  // WEG 2: TCP-VERBINDUNG VERWALTEN & LESEN
  // ==========================================
  if (tcpServer.hasClient()) {
    if (tcpClient && tcpClient.connected()) {
      tcpClient.stop(); // Alten Client trennen, wenn ein neuer anklopft
    }
    tcpClient = tcpServer.available();
    if (DEBUG_MODE_NET) Serial.println(F("[TCP] Neuer Datensender verbunden"));
  }

  // Wenn der TCP-Client verbunden ist und Daten sendet
  if (tcpClient && tcpClient.connected() && tcpClient.available()) {
    while (tcpClient.available()) {
      char c = tcpClient.read();
      if (netNmea.process(c)) {
        if (extern_gps_UDP_tcp && netNmea.isValid()) {
          extern_sensorData.gps_lat = netNmea.getLatitude() / 1000000.0;
          extern_sensorData.gps_lon = netNmea.getLongitude() / 1000000.0;
          extern_sensorData.gps_speed = netNmea.getSpeed() / 1000.0;
          extern_sensorData.gps_kurs = netNmea.getCourse() / 1000.0;
          extern_sensorData.gps_sats = netNmea.getNumSatellites();
          extern_sensorData.gps_hdop = netNmea.getHDOP() / 10.0;
          eventGPSNET();
        }
        processNetworkCustomSentences(netNmea.getSentence());
        netNmea.clear();
      }
    }
  }

  // Zyklische Timeout-Prüfung für die Netzwerkdaten
  updateSensorTimeoutsNET();
}

/**
 * Abgesicherter sscanf-Parser gegen korrupte Datenfragmente
 */
void processNetworkCustomSentences(const char* sentence) {
  if (sentence == nullptr || sentence[0] != '$') return;
  if (strlen(sentence) < 10) return; 

  const char* talker = &sentence[3];

  // Wind-Parsing ($--MWV)
  if (extern_wind_UDP_tcp && strncmp(talker, "MWV", 3) == 0) {
    double angle = 0.0, speed = 0.0;
    char status = 'V', unit = 'N';

    if (sscanf(sentence, "$%*5c,%lf,R,%lf,%c,%c", &angle, &speed, &unit, &status) == 4) {
      if (status == 'A') { 
        extern_sensorData.winddir_gemessen = angle;
        extern_sensorData.windspeed_gemessen = (unit == 'M') ? (speed * 1.94384) : speed;
        eventWindNET();
      }
    }
  }

  // Tiefen-Parsing ($--DBT)
  else if (extern_echolot_UDP_tcp && strncmp(talker, "DBT", 3) == 0) {
    double depthFeet = 0.0, depthMeters = 0.0, depthFathoms = 0.0;
    
    if (sscanf(sentence, "$%*5c,%lf,f,%lf,M,%lf,F", &depthFeet, &depthMeters, &depthFathoms) >= 2) {
      if (depthMeters > 0.0) {
        extern_sensorData.Echolot = depthMeters;
        eventDepthNET();
      }
    }
  }
}

/**
 * Einheitliche Timeout-Überwachung für alle Netzwerkeingänge (UDP & TCP)
 */
void updateSensorTimeoutsNET() {
  uint32_t now = millis();

  if (extern_gps_UDP_tcp && sensorEventsNET.gps_last != 0 && (now - sensorEventsNET.gps_last > TIMEOUT_GPS)) {
    extern_sensorData.gps_lat = 0.0; extern_sensorData.gps_lon = 0.0; extern_sensorData.gps_speed = 0.0; extern_sensorData.gps_kurs = 0.0;
    extern_sensorData.gps_sats = 0; extern_sensorData.gps_hdop = 99.9; sensorEventsNET.gps_last = 0;
    if (DEBUG_MODE_NET) Serial.println(F("[WARN] Netzwerk GPS Timeout"));
  }

  if (extern_wind_UDP_tcp && sensorEventsNET.wind_last != 0 && (now - sensorEventsNET.wind_last > TIMEOUT_WIND)) {
    extern_sensorData.winddir_gemessen = -999.0; extern_sensorData.windspeed_gemessen = -1.0; sensorEventsNET.wind_last = 0;
    if (DEBUG_MODE_NET) Serial.println(F("[WARN] Netzwerk Wind Timeout"));
  }

  if (extern_echolot_UDP_tcp && sensorEventsNET.depth_last != 0 && (now - sensorEventsNET.depth_last > TIMEOUT_DEPTH)) {
    extern_sensorData.Echolot = -1.0; sensorEventsNET.depth_last = 0;
    if (DEBUG_MODE_NET) Serial.println(F("[WARN] Netzwerk Depth Timeout"));
  }
}
