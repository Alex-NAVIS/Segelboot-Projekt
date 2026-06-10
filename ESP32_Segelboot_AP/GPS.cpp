// ==========================================================
// GPS_Nema.cpp
// ---------------------------------------------------------
// Dieses Modul verarbeitet NMEA-Daten eines GPS-Empfängers.
// Es nutzt die TinyGPSPlus-Library, um Positions-, Geschwindigkeits-
// und Kursinformationen zu extrahieren.
// Die Daten werden in die globale Struktur "sensorData" geschrieben.
//
// Hinweis:
// - Dieses Modul führt keine eigene Zeitsteuerung durch.
// - Das Hauptprogramm bestimmt die Aufrufhäufigkeit von readGPS().
// ==========================================================

#include "GPS.h"  // Header-Datei mit Funktionsprototypen und Includes

// ---------------------------------------------------------
// Lokale TinyGPSPlus-Instanz
// ---------------------------------------------------------
// Diese Instanz übernimmt die Dekodierung der NMEA-Daten
// in strukturierte Informationen (Latitude, Longitude, Speed, Course)
TinyGPSPlus gps;


//GPS-Puffer-Struktur
//Speichert die letzten GPS-Daten inkl. Zeitstempel.
struct GPS_Point {
  double lat;
  double lon;
  float speed;         // Knoten
  float kurs;          // Grad
  float hdop;          // HDOP
  unsigned long time;  // millis()
};

#define GPS_BUFFER_SIZE 10
static GPS_Point gpsBuffer[GPS_BUFFER_SIZE];
static int gpsWriteIndex = 0;
static int logTickCounter = 0;

// sm Zähler Strecke
static double lastCheckLat = 0.0;
static double lastCheckLon = 0.0;
static bool logInitialized = false;
#define GPS_MIN_SATS 5
// ==========================================================
// setupGPS()
// ---------------------------------------------------------
// Initialisierung der seriellen Schnittstelle für GPS.
// Aufgerufen einmalig im setup() des Hauptprogramms.
// ---------------------------------------------------------
// Aufgaben:
// - Startet die serielle Schnittstelle mit Pins und Baudrate aus Config.h
// - Optional: Debug-Ausgabe der Initialisierung
// - Bereitet das GPS-Modul auf das kontinuierliche Senden von NMEA-Daten vor
// ---------------------------------------------------------
void setupGPS() {
  if (DEBUG_MODE) Serial.println(F("[GPS] Initialisierung..."));

  // Serielle Schnittstelle starten (GPS_SERIAL_PORT = z.B. Serial1)
  GPS_SERIAL_PORT.begin(GPS_BAUDRATE, SERIAL_8N1, GPS_RX_PIN, GPS_TX_PIN);
  GPS_SERIAL_PORT.setRxBufferSize(1024);
  if (DEBUG_MODE) {
    Serial.print(F("[GPS] Port gestartet mit Baudrate: "));
    Serial.println(GPS_BAUDRATE);
    Serial.print(F("[GPS] RX-Pin: "));
    Serial.println(GPS_RX_PIN);
    Serial.print(F("[GPS] TX-Pin: "));
    Serial.println(GPS_TX_PIN);
  }
}


// ==========================================================
// readGPS()
// ---------------------------------------------------------
// Zyklisch im loop() des Hauptprogramms aufgerufen.
// Liest eingehende NMEA-Daten und übergibt sie an TinyGPSPlus.
// Aktualisiert sensorData mit neuen Positions- und Bewegungsdaten.
// ---------------------------------------------------------
void readGPS() {
  // --- Alle verfügbaren Bytes vom GPS auslesen und dekodieren ---
  while (GPS_SERIAL_PORT.available() > 0) {
    gps.encode(GPS_SERIAL_PORT.read());
  }

  // --- Wenn neue Positionsdaten vorhanden sind ---
  if (gps.location.isUpdated()) {
    //GPS Ringspeicher für AHRS System
    GPS_Point &p = gpsBuffer[gpsWriteIndex];
    p.lat = gps.location.lat();
    p.lon = gps.location.lng();
    p.speed = gps.speed.knots();
    p.kurs = gps.course.deg();
    p.time = millis();
    // HDOP erfassen
    if (gps.hdop.isValid()) {
      p.hdop = gps.hdop.hdop();  // liefert Float-Wert in Standardeinheit
    } else {
      p.hdop = 99.9f;  // ungültig, sehr schlechter Wert
    }
    gpsWriteIndex = (gpsWriteIndex + 1) % GPS_BUFFER_SIZE;  // Ringpuffer

    if (GPS_AHRS_SYSTEM) {
      gps_ahrs();
    } else {
      sensorData.gps_lat = gps.location.lat();   // Breitengrad
      sensorData.gps_lon = gps.location.lng();   // Längengrad
      sensorData.gps_speed = gps.speed.knots();  // Geschwindigkeit in kn/h
      sensorData.gps_kurs = gps.course.deg();    // Kurs über Grund (Grad)
    }

    // --- Optional: Debug-Ausgabe ---
    if (DEBUG_MODE_GPS) {
      Serial.println(F("[GPS] Neue Daten empfangen:"));
      Serial.print(F("  Lat: "));
      Serial.println(sensorData.gps_lat, 6);
      Serial.print(F("  Lon: "));
      Serial.println(sensorData.gps_lon, 6);
      Serial.print(F("  Speed (km/h): "));
      Serial.println(sensorData.gps_speed, 2);
      Serial.print(F("  Kurs (°): "));
      Serial.println(sensorData.gps_kurs, 2);
    }
  }

  // --- Nur wenn gültiges Datum & Zeit vorhanden ---
  if (gps.date.isValid() && gps.time.isValid()) {
    // Berechne lokale Zeit basierend auf GPS-Koordinaten
    updateLocalTime(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());

    //Systemzeit auf Zeitzone 0
    time_t now = time(nullptr);
    if (now < 1600000000) {  // Systemzeit noch ungültig
      setSystemTimeFromGPS(gps.date.year(), gps.date.month(), gps.date.day(), gps.time.hour(), gps.time.minute(), gps.time.second());
    }

    // --- Debug-Ausgabe der Zeit ---
    if (DEBUG_MODE) {
      Serial.print(F("  Datum: "));
      Serial.print(sensorData.gps_tag);
      Serial.print(F("."));
      Serial.print(sensorData.gps_monat);
      Serial.print(F("."));
      Serial.println(sensorData.gps_jahr);
      Serial.print(F("  Zeit (lokal): "));
      Serial.print(sensorData.gps_stunde);
      Serial.print(F(":"));
      Serial.print(sensorData.gps_minute);
      Serial.print(F(":"));
      Serial.println(sensorData.gps_sekunde);
    }
  }
}

// ==========================================================
// deg2rad()
// ---------------------------------------------------------
// Wandelt einen Winkel von Grad in das Bogenmaß um
// - Multipliziert den Wert mit (PI / 180)
// - Rückgabewert: Winkel in Radianten (double)
// ==========================================================
static double deg2rad(double d) {
  return d * M_PI / 180.0;
}

// ==========================================================
// rad2deg()
// ---------------------------------------------------------
// Wandelt einen Winkel vom Bogenmaß in Grad um
// - Multipliziert den Wert mit (180 / PI)
// - Rückgabewert: Winkel in Grad (double)
// ==========================================================
static double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// ==========================================================
// distance_nm()
// ---------------------------------------------------------
// Berechnet die Distanz zwischen zwei Koordinaten in Seemeilen
// - Nutzt Mittelbreitenverfahren zur Längengrad-Skalierung
// - Rückgabewert: Distanz in Seemeilen (double)
// ==========================================================
static double distance_nm(double lat1, double lon1, double lat2, double lon2) {
  // Mittelwert-Lat für Längengrad-Skalierung
  double latm = deg2rad((lat1 + lat2) * 0.5);
  double dlat = lat2 - lat1;                  // degrees
  double dlon = lon2 - lon1;                  // degrees
  double dlats_nm = dlat * 60.0;              // nm
  double dlons_nm = dlon * 60.0 * cos(latm);  // nm (korrigiert)
  return sqrt(dlats_nm * dlats_nm + dlons_nm * dlons_nm);
}

// ==========================================================
// isGPSValid()
// ---------------------------------------------------------
// Prüft, ob ein ausreichend stabiles GPS-Signal vorliegt
// - Vergleicht aktuelle Satellitenanzahl mit Schwellwert
// - Rückgabewert: true = Signal OK, false = Signal zu schwach
// ==========================================================
bool isGPSValid() {
  return (sensorData.gps_sats >= GPS_MIN_SATS);
}

// ==========================================================
// updateDistanceLogTick()
// ---------------------------------------------------------
// Aktualisiert den Distanzzähler basierend auf GPS-Daten
// - Filtert ungültige Signale und nutzt Interval-Ticks
// - Addiert Bewegung erst ab einem Mindest-Schwellwert (m)
// ==========================================================
void updateDistanceLogTick(double lat, double lon) {
  // ❌ KEIN gültiges GPS → sofort raus
  if (!isGPSValid()) {
    logInitialized = false;  // wichtig!
    return;
  }
  logTickCounter++;
  // Nur alle X Ticks auswerten
  if (logTickCounter < LOG_INTERVAL_TICKS) return;
  logTickCounter = 0;

  // Initialisierung
  if (!logInitialized) {
    lastCheckLat = lat;
    lastCheckLon = lon;
    logInitialized = true;
    return;
  }
  // Distanz berechnen
  double d = distance_nm(lastCheckLat, lastCheckLon, lat, lon);
  // Nur zählen wenn größer als Schwellwert
  if (d >= LOG_MIN_DISTANCE_SM) {
    // in sm umrechnen
    sensorData.sm_counter += d / 1852.0;
    // neuen Referenzpunkt setzen
    lastCheckLat = lat;
    lastCheckLon = lon;
  }
}

// ==========================================================
// isDST()
// ---------------------------------------------------------
// Prüft, ob ein Datum in Mitteleuropa Sommerzeit (DST) liegt
// - Letzter Sonntag im März bis letzter Sonntag im Oktober
// - Rückgabewert: true = Sommerzeit, false = Normalzeit
// ==========================================================
bool isDST(int year, int month, int day) {
  int d1 = 31 - (5 * year / 4 + 4) % 7;  // Letzter Sonntag März
  int d2 = 31 - (5 * year / 4 + 1) % 7;  // Letzter Sonntag Oktober

  if ((month > 3 && month < 10)) return true;  // April–September
  if (month == 3 && day >= d1) return true;    // März ab letztem Sonntag
  if (month == 10 && day < d2) return true;    // Oktober vor letztem Sonntag
  return false;
}


// ==========================================================
// updateLocalTime()
// ---------------------------------------------------------
// Berechnet die lokale Zeit aus UTC + Zeitzonenoffset.
// - Zeitzonenoffset wird aus Längengrad berechnet (15° = 1 Stunde)
// - Sommerzeit wird berücksichtigt
// - Speichert berechnete lokale Zeit in sensorData
// ==========================================================
void updateLocalTime(int utcYear, int utcMonth, int utcDay,
                     int utcHour, int utcMinute, int utcSecond) {
  // Zeitzonenoffset nach Längengrad (Stunden)
  int tzOffset = int(round(sensorData.gps_lon / 15.0));

  // Sommerzeit prüfen
  if (isDST(utcYear, utcMonth, utcDay)) tzOffset += 1;

  int hour = utcHour + tzOffset;
  int minute = utcMinute;
  int second = utcSecond;

  // Überlaufkorrektur für Sekunden/Minuten/Stunden
  if (second >= 60) {
    second -= 60;
    minute += 1;
  }
  if (minute >= 60) {
    minute -= 60;
    hour += 1;
  }
  if (hour >= 24) { hour -= 24; }

  // Berechnete lokale Zeit in sensorData eintragen
  sensorData.gps_jahr = utcYear;
  sensorData.gps_monat = utcMonth;
  sensorData.gps_tag = utcDay;
  sensorData.gps_stunde = hour;
  sensorData.gps_minute = minute;
  sensorData.gps_sekunde = second;
}

//Setze die Systemzeit
void setSystemTimeFromGPS(int year, int month, int day, int hour, int minute, int second) {
  struct tm tm = {};
  tm.tm_year = year - 1900;
  tm.tm_mon = month - 1;
  tm.tm_mday = day;
  tm.tm_hour = hour;
  tm.tm_min = minute;
  tm.tm_sec = second;
  tm.tm_isdst = 0;  // GPS = UTC
  time_t t = mktime(&tm);
  if (t < 1600000000) return;  // Schutz gegen Müllzeit (optional)
  struct timeval tv;
  tv.tv_sec = t;
  tv.tv_usec = 0;
  settimeofday(&tv, nullptr);
}

//-----------------------------------------------------------------------------------------------
// ------------------------------ AHRS System für GPS (NAVIS v1.0) ------------------------------
// --------------- GPS AHRS – NAVIS Production Candidate v1.0 (Release Candidate) ---------------
// --------------- Zertifizierte Version für Testfahrten und Logdaten-Aufzeichnung --------------
//-----------------------------------------------------------------------------------------------

// ----------------------------------------------------------------------------------------------
// GPS AHRS – Geschwindigkeitsadaptive Vektorglättung & getrennte Koppel-Prädiktion
//   - Datenquelle: Liest Rohdaten aus gpsBuffer[] (Struktur: GPS_Point, Größe: GPS_BUFFER_SIZE)
//   - Datensenke: Schreibt direkt in sensorData (Echtzeit-Fixes, SOG, COG und Prädiktion)
//
// Funktionsweise & Signalverarbeitung:
//   1. Chronologische Härtung: Sammelt alle gültigen Fixes und sortiert sie via Insertion-Sort
//      strikt nach Zeitstempel. Das eliminiert Speicherlücken und asynchrone Puffer-Schreibfehler.
//   2. Vektorisierung: Nutzt Positionsdifferenzen (Lat/Lon) als primäre Bewegungsquelle. Falls
//      Punkte unvollständig sind, erfolgt ein robuster, Pol-abgesicherter Fallback auf COG/SOG.
//   3. Gewichtung & Dämpfung: Kombiniert eine lineare HDOP-Filterung (gegen ungenaue Fixes) mit
//      einer moderaten Recency-Zeitgewichtung (0.7-1.0), um Wellenschlag (Gieren) zu dämpfen.
//   4. Geschwindigkeitsadaptive Vektormischung:
//      - < 2.0 kn (Hafen/Flaute): Extrem kursstabil (80% Langzeit- / 20% Kurzzeitvektor)
//      - > 6.0 kn (Segeln/Gleiten): Reaktionsfreudig (50% Langzeit- / 50% Kurzzeitvektor)
//      - Dazwischen erfolgt eine präzise lineare Interpolation des Mischfaktors (Alpha).
//   5. Sprungfreier Kurs-Tiefpass (COG): Glättet den Ausgabekurs über ein adaptives Delta-Filter.
//      Der Dämpfungsfaktor (course_alpha) atmet geschwindigkeitsabhängig mit ([0.15 ... 0.50]),
//      fängt Phasenübergänge (359° -> 1°) fehlerfrei ab und unterdrückt Standrauschen im Hafen.
//   6. Striktes Architektur-Splitting:
//      - Real-Positions-Erhalt: sensorData.gps_lat/lon speichert die ungefilterte, echte GPS-
//        Wahrheitsquelle der letzten Messung, um Lag-Fehler bei engen Hafenmanövern zu vermeiden.
//      - Koppelnavigation: Berechnet eine separate, Antimeridian- und Pol-abgesicherte Zukunft-
//        Vorschau (predict_s Sekunden) exklusiv in sensorData.gps_predict_lat/lon für Autopiloten.
// ----------------------------------------------------------------------------------------------
void gps_ahrs() {
  int validIdx[GPS_BUFFER_SIZE];
  int validCount = 0;

  // 1. Alle verfügbaren Punkte mit gültigem Zeitstempel einsammeln
  for (int i = 0; i < GPS_BUFFER_SIZE; ++i) {
    if (gpsBuffer[i].time != 0) {
      validIdx[validCount] = i;
      validCount++;
    }
  }

  // Für stabile Vektorketten (mindestens zwei Segmente) brauchen wir 3 Punkte
  if (validCount < 3) return; 

  // 2. Explizite chronologische Sortierung nach Zeitstempel (Garantie gegen Puffer-Lücken)
  // Einfacher Insertion-Sort, da GPS_BUFFER_SIZE sehr klein ist (ressourcenschonend für ESP32)
  for (int i = 1; i < validCount; i++) {
    int keyIdx = validIdx[i];
    unsigned long keyTime = gpsBuffer[keyIdx].time;
    int j = i - 1;
    while (j >= 0 && gpsBuffer[validIdx[j]].time > keyTime) {
      validIdx[j + 1] = validIdx[j];
      j--;
    }
    validIdx[j + 1] = keyIdx;
  }

  // Jüngster gültiger Datensatz ist nach der Sortierung mathematisch sicher am Ende
  int lastIndex = validIdx[validCount - 1];
  GPS_Point last = gpsBuffer[lastIndex];
  if (last.lat == 0.0 && last.lon == 0.0) return;

  double sum_dlat_long = 0.0, sum_dlon_long = 0.0, sum_w_long = 0.0, sum_t_long = 0.0;
  double sum_dlat_short = 0.0, sum_dlon_short = 0.0, sum_w_short = 0.0, sum_t_short = 0.0;

  // Definition des Kurzzeitfensters (Maximal 4 reale Segmente)
  const int SHORT_SEGMENTS = (validCount - 1 < 4) ? (validCount - 1) : 4;

  // Rückkopplungsfreie Geschwindigkeitsbasis direkt aus den Rohdaten
  double current_speed = last.speed;
  if (isnan(current_speed) || current_speed < 0.0) {
    current_speed = sensorData.gps_speed; // Fallback auf das letzte Filterergebnis
  }

  // 3. Iteration über die nun garantiert chronologische Vektorkette
  for (int i = 0; i < validCount - 1; ++i) {
    int idxA = validIdx[i];
    int idxB = validIdx[i + 1];
    
    double dt = double(gpsBuffer[idxB].time - gpsBuffer[idxA].time);
    if (dt <= 0.0 || dt > 15.0) continue; // Zeitsprungfilter

    double dlat_deg = 0.0;
    double dlon_deg = 0.0;

    // Position als Primärquelle
    if (gpsBuffer[idxA].lat != 0.0 && gpsBuffer[idxB].lat != 0.0) {
      dlat_deg = gpsBuffer[idxB].lat - gpsBuffer[idxA].lat;
      dlon_deg = gpsBuffer[idxB].lon - gpsBuffer[idxA].lon;
      
      // Korrektur der Datumsgrenze (Antimeridian-Passage)
      if (dlon_deg > 180.0)       dlon_deg -= 360.0;
      else if (dlon_deg < -180.0) dlon_deg += 360.0;
    } 
    // COG/SOG nur als Fallback
    else if (!isnan(gpsBuffer[idxA].speed) && gpsBuffer[idxA].speed >= 0.0 && !isnan(gpsBuffer[idxA].kurs)) {
      double distance_deg = (gpsBuffer[idxA].speed * dt) / 216000.0; 
      double kurs_rad = gpsBuffer[idxA].kurs * M_PI / 180.0;
      
      double cosLatFallback = cos(gpsBuffer[idxA].lat * M_PI / 180.0);
      if (fabs(cosLatFallback) < 0.01) {
        cosLatFallback = (cosLatFallback >= 0.0) ? 0.01 : -0.01;
      }
      
      dlat_deg = distance_deg * cos(kurs_rad);
      dlon_deg = (distance_deg * sin(kurs_rad)) / cosLatFallback;
    }

    // Ruhige HDOP-Begrenzung (Linear gedämpft statt quadratisch überbetont)
    float hdop = gpsBuffer[idxB].hdop;
    if (hdop <= 0.0f || isnan(hdop)) hdop = 4.0f;
    double w = 1.0 / (double(hdop) + 0.1);
    if (w > 1.0) w = 1.0; 
    if (w < 0.1) w = 0.1;

    // Moderate Zeitgewichtung (0.7 bis 1.0) gegen Wellenschlag-Peaks
    double progress = double(i + 1) / double(validCount);
    double recency_factor = 0.7 + 0.3 * progress; 
    w *= recency_factor;

    // Segment-Aufteilung über die chronologisch jüngsten Segmente
    if (i >= (validCount - 1 - SHORT_SEGMENTS)) {
      sum_dlat_short += dlat_deg * w;
      sum_dlon_short += dlon_deg * w;
      sum_w_short += w;
      sum_t_short += dt * w;
    }

    sum_dlat_long += dlat_deg * w;
    sum_dlon_long += dlon_deg * w;
    sum_w_long += w;
    sum_t_long += dt * w;
  }

  if (sum_w_long <= 0.0 || sum_w_short <= 0.0) return;

  // Umrechnung in "Änderung pro Sekunde"
  double avg_dt_long = sum_t_long / sum_w_long;
  double avg_dt_short = sum_t_short / sum_w_short;
  if (avg_dt_long <= 0.1) avg_dt_long = 1.0;
  if (avg_dt_short <= 0.1) avg_dt_short = 1.0;

  double vec_lat_long = (sum_dlat_long / sum_w_long) / avg_dt_long;
  double vec_lon_long = (sum_dlon_long / sum_w_long) / avg_dt_long;
  
  double vec_lat_short = (sum_dlat_short / sum_w_short) / avg_dt_short;
  double vec_lon_short = (sum_dlon_short / sum_w_short) / avg_dt_short;

  // Adaptive Vektormischung (Alpha) basierend auf der stabilen Roh-Geschwindigkeit
  double alpha;
  if (current_speed < 2.0) {
    alpha = 0.2; 
  } else if (current_speed > 6.0) {
    alpha = 0.5; 
  } else {
    alpha = 0.2 + (current_speed - 2.0) * (0.3 / 4.0); 
  }

  // Vektoren final mischen (Bewegung pro Sekunde)
  double final_vec_lat = alpha * vec_lat_short + (1.0 - alpha) * vec_lat_long;
  double final_vec_lon = alpha * vec_lon_short + (1.0 - alpha) * vec_lon_long;

  // --- ECHTE GPS-POSITION (Wahrheitsquelle für Kartenplotter & AIS) ---
  sensorData.gps_lat = last.lat;
  sensorData.gps_lon = last.lon;

  // --- BERECHNUNG DER PRÄDIKTION (GETRENNT SPEICHERN) ---
  double predict_s = GPS_PREDICT_S; 
  sensorData.gps_predict_lat = last.lat + (final_vec_lat * predict_s);
  sensorData.gps_predict_lon = last.lon + (final_vec_lon * predict_s);

  // Pol-Absicherung für Breiten-Projektionen
  double lat_rad = last.lat * M_PI / 180.0;
  double cosLat = cos(lat_rad);
  if (fabs(cosLat) < 0.01) {
    cosLat = (cosLat >= 0.0) ? 0.01 : -0.01;
  }

  // Antimeridian-Normalisierung für die berechnete Prädiktion
  while (sensorData.gps_predict_lon > 180.0)  sensorData.gps_predict_lon -= 360.0;
  while (sensorData.gps_predict_lon < -180.0) sensorData.gps_predict_lon += 360.0;

  // Pol-Überquerungsschutz für die Prädiktions-Latitude
  if (sensorData.gps_predict_lat > 90.0)   sensorData.gps_predict_lat = 90.0;
  if (sensorData.gps_predict_lat < -90.0)  sensorData.gps_predict_lat = -90.0;

  // SOG (Knoten) ableiten
  double dlon_adj = final_vec_lon * cosLat;
  double dist_deg_per_sec = sqrt(final_vec_lat * final_vec_lat + dlon_adj * dlon_adj);
  double speed_knots = dist_deg_per_sec * 60.0 * 3600.0;

  // COG (Kurs) über Tangens berechnen
  double kurs_rad = atan2(dlon_adj, final_vec_lat);
  double kurs_deg = kurs_rad * 180.0 / M_PI;
  if (kurs_deg < 0.0) kurs_deg += 360.0;

  // Filterung bei sehr geringer Fahrt (Rauschen im Stand blockieren)
  if (speed_knots < GPS_MIN_VALID_SPEED) {
    speed_knots = 0.0;
    kurs_deg = sensorData.gps_kurs; 
  } else {
    // 4. Adaptiver, sprungfreier Kurs-Tiefpass mit defensiver Bereichsabsicherung
    double course_alpha;
    if (speed_knots < 2.0) {
      course_alpha = 0.15; 
    } else if (speed_knots > 6.0) {
      course_alpha = 0.50; 
    } else {
      course_alpha = 0.15 + (speed_knots - 2.0) * (0.35 / 4.0); 
    }

    // Gegen zukünftige Codeänderungen absichern (Garantiebereich [0.15 ... 0.50])
    if (course_alpha < 0.15) course_alpha = 0.15;
    if (course_alpha > 0.50) course_alpha = 0.50;

    double diff = kurs_deg - sensorData.gps_kurs;
    if (diff > 180.0)  diff -= 360.0;
    if (diff < -180.0) diff += 360.0;
    
    kurs_deg = sensorData.gps_kurs + course_alpha * diff;
    if (kurs_deg < 0.0)   kurs_deg += 360.0;
    if (kurs_deg >= 360.0) kurs_deg -= 360.0;
  }

  // Ausgabewerte in das globale System schreiben
  sensorData.gps_speed = speed_knots;
  sensorData.gps_kurs = kurs_deg;
}

// ==========================================================
// getGPSTimestamp()
// ----------------------------------------------------------
// Erzeugt Unix UTC Timestamp aus GPS Datum/Zeit
// ==========================================================
uint32_t getGPSTimestamp() {
  struct tm t;
  t.tm_year = gps.date.year() - 1900;
  t.tm_mon  = gps.date.month() - 1;
  t.tm_mday = gps.date.day();
  t.tm_hour = gps.time.hour();
  t.tm_min  = gps.time.minute();
  t.tm_sec  = gps.time.second();
  t.tm_isdst = 0;
  return (uint32_t)mktime(&t);
}

// ==========================================================
// addTrackPoint()
// ----------------------------------------------------------
// Fügt neuen GPS Punkt in Ringspeicher ein
// ==========================================================
void addTrackPoint() {
  // Nur speichern wenn GPS gültig
  if (!gps.location.isValid()) return;
  int i = sensorData.track_index;
  sensorData.track[i].lat = sensorData.gps_lat;
  sensorData.track[i].lon = sensorData.gps_lon;
  sensorData.track[i].speed = sensorData.gps_speed;
  sensorData.track[i].course = sensorData.gps_kurs;
  sensorData.track[i].timestamp = getGPSTimestamp();
  // Nächster Schreibindex
  sensorData.track_index++;
  // Ringspeicher Überlauf
  if (sensorData.track_index >= SensorData::TRACK_SIZE) {
    sensorData.track_index = 0;
    sensorData.track_filled = true;
  }
}
