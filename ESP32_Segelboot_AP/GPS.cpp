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
// -----------------------------------AHRS System für GPS----------------------------------------
//-----------------------------------------------------------------------------------------------

// ------------------------------------------------------------------
// GPS AHRS – HDOP-gewichtete Vektor-Glättung & kurzzeitige Vorhersage
//   - Liest Daten aus gpsBuffer[] (GPS_Point, GPS_BUFFER_SIZE, gpsWriteIndex)
//   - Schreibt in sensorData.gps_lat, gps_lon, gps_speed, gps_kurs
// Erstellt aus 10 Werten den Langzeitvektor (10s) Erstellt aus den letzten 5 Werten den Kurzzeitvektor (5s) Beide Vektoren werden gewichtet (0.6 / 0.4)
// Bildet finalen Bewegungsvektor, Daraus werden Geschwindigkeit und Kurs berechnet Projektion neue Zielposition berechnet
// Ergebnis → direkt in sensorData.gps_lat/lon/speed/kurs
// ------------------------------------------------------------------

static double deg2rad(double d) {
  return d * M_PI / 180.0;
}
static double rad2deg(double r) {
  return r * 180.0 / M_PI;
}

// Entfernung in Nautical Miles zwischen 2 lat/lon in degrees (approx.)
// Hinweis: 1° Breitengrad ≈ 60 NM
static double distance_nm(double lat1, double lon1, double lat2, double lon2) {
  // Mittelwert-Lat für Längengrad-Skalierung
  double latm = deg2rad((lat1 + lat2) * 0.5);
  double dlat = lat2 - lat1;                  // degrees
  double dlon = lon2 - lon1;                  // degrees
  double dlats_nm = dlat * 60.0;              // nm
  double dlons_nm = dlon * 60.0 * cos(latm);  // nm (korrigiert)
  return sqrt(dlats_nm * dlats_nm + dlons_nm * dlons_nm);
}

// Hauptfunktion: AHRS-basierte Glättung
void gps_ahrs() {
  // Anzahl gültiger Pufferpunkte ermitteln
  int validCount = 0;
  for (int i = 0; i < GPS_BUFFER_SIZE; ++i)
    if (gpsBuffer[i].time != 0) ++validCount;
  if (validCount < 2) return;  // noch zu wenige Daten

  // Letzten gültigen Punkt (aktueller Zeitstempel)
  int lastIndex = (gpsWriteIndex + GPS_BUFFER_SIZE - 1) % GPS_BUFFER_SIZE;
  GPS_Point last = gpsBuffer[lastIndex];
  double lastLat = last.lat;
  double lastLon = last.lon;
  if (lastLat == 0 && lastLon == 0) return;  // ungültig

  // Weighted vector sums (degrees)
  double sum_dlat = 0.0;
  double sum_dlon = 0.0;
  double sum_weight = 0.0;
  double sum_time = 0.0;  // gewichtete Zeitsumme

  // Kurz- und Langzeit Vektoren (separat)
  const int SHORT_WINDOW = min(GPS_BUFFER_SIZE, 5);  // z.B. 5s Fenster
  double sum_dlat_short = 0.0, sum_dlon_short = 0.0, sum_w_short = 0.0, sum_t_short = 0.0;

  // Iteriere über Segmente (von älter zu neuer)
  // Für jedes Segment i->i+1 berechne delta (in deg) basierend auf reported Kurs+Speed oder lat/lon diff.
  for (int k = 0; k < GPS_BUFFER_SIZE; ++k) {
    int idxA = (lastIndex + k) % GPS_BUFFER_SIZE;
    int idxB = (idxA + 1) % GPS_BUFFER_SIZE;
    // sicherstellen, dass beide Punkte gültig und zeitlich aufsteigend sind
    if (gpsBuffer[idxA].time == 0 || gpsBuffer[idxB].time == 0) continue;
    unsigned long tA = gpsBuffer[idxA].time;
    unsigned long tB = gpsBuffer[idxB].time;
    if (tB <= tA) continue;
    double dt = double(tB - tA);  // seconds

    // delta aus Geschwindigkeit+Kurs falls vorhanden, sonst aus differenz lat/lon
    double dlat_deg = 0.0;
    double dlon_deg = 0.0;
    if (!isnan(gpsBuffer[idxA].speed) && gpsBuffer[idxA].speed >= 0.0 && !isnan(gpsBuffer[idxA].kurs)) {
      // Geschwindigkeit (kn) * dt -> distance in degrees: deg = (kn * dt_seconds) / 3600 * (1/60)
      // simpler: distance_deg = (speed_knots * dt_seconds) / 3600.0 / 60.0 * 3600? wait -> use distance_nm = speed_knots * dt_seconds / 3600 * 3600? simpler compute:
      // distance in degrees (deg) = (speed_knots * dt_seconds) / 3600.0 / 60.0?? -> correct formula: 1 degree = 60 NM, speed_knots = NM/hour.
      // distance_hours = dt / 3600.0; distance_nm = speed_knots * distance_hours; distance_deg = distance_nm / 60.0;
      double distance_hours = dt / 3600.0;
      double distance_nm = gpsBuffer[idxA].speed * distance_hours;
      double distance_deg = distance_nm / 60.0;
      double kurs_rad = gpsBuffer[idxA].kurs * M_PI / 180.0;
      dlat_deg = distance_deg * cos(kurs_rad);
      dlon_deg = distance_deg * sin(kurs_rad) / cos(gpsBuffer[idxA].lat * M_PI / 180.0);
    } else {
      // fallback: direkter difference (B - A)
      dlat_deg = gpsBuffer[idxB].lat - gpsBuffer[idxA].lat;
      dlon_deg = gpsBuffer[idxB].lon - gpsBuffer[idxA].lon;
    }

    // Gewicht aus HDOP (niedriger HDOP => höheres Gewicht), aber cap negatives
    float hdop = gpsBuffer[idxB].hdop;
    if (hdop <= 0.0f || isnan(hdop)) hdop = GPS_HDOP_MAX_WEIGHT;              // fallback
    double w = 1.0 / (double(min((float)GPS_HDOP_MAX_WEIGHT, hdop)) + 0.01);  // saturate hdop weight

    // zusätzliche Gewichtung für neuere Punkte (lineare)
    // jüngstes Segment k=0 sollte höhere Gewichtung
    double recency = double(k == 0 ? GPS_BUFFER_SIZE : (GPS_BUFFER_SIZE - k));
    w *= (0.5 + 0.5 * (recency / GPS_BUFFER_SIZE));  // 0.5..1.0

    // Summen
    sum_dlat += dlat_deg * w;
    sum_dlon += dlon_deg * w;
    sum_weight += w;
    sum_time += dt * w;

    // Kurzzeit (> last SHORT_WINDOW seconds) accumulate
    if (k < SHORT_WINDOW) {
      sum_dlat_short += dlat_deg * w;
      sum_dlon_short += dlon_deg * w;
      sum_w_short += w;
      sum_t_short += dt * w;
    }
  }

  if (sum_weight <= 0.0) return;

  // Normierte vectors (degrees)
  double vec_dlat = sum_dlat / sum_weight;
  double vec_dlon = sum_dlon / sum_weight;

  double vec_dlat_short = sum_w_short > 0.0 ? (sum_dlat_short / sum_w_short) : vec_dlat;
  double vec_dlon_short = sum_w_short > 0.0 ? (sum_dlon_short / sum_w_short) : vec_dlon;

  // Kombiniere kurz/long mit adaptivem Faktor basierend auf short-weight reliability
  double shortReliability = (sum_w_short / sum_weight);
  // clamp 0..1
  if (shortReliability < 0.0) shortReliability = 0.0;
  if (shortReliability > 1.0) shortReliability = 1.0;

  // Gewichtung: wenn shortRealiable high -> mehr kurzzeitanteil
  double alpha = 0.6 * shortReliability + 0.2;  // alpha in [0.2..0.8]
  if (alpha < 0.2) alpha = 0.2;
  if (alpha > 0.85) alpha = 0.85;

  double combined_dlat = alpha * vec_dlat_short + (1.0 - alpha) * vec_dlat;
  double combined_dlon = alpha * vec_dlon_short + (1.0 - alpha) * vec_dlon;

  // Vorhersage (predict_s)
  double predict_s = GPS_PREDICT_S;
  // Umrechnung: wir haben degrees-per-segment sums; wir brauchen eine durchschnittliche Sekundendauer:
  double avg_dt = sum_time / sum_weight;
  if (avg_dt <= 0.0) avg_dt = 1.0;

  // Skaliere combined vector vom (per-segment-average) auf predict_s
  // Wir interpretieren combined_dlat/dlon als "expected degrees per avg_dt seconds"
  double scale = predict_s / avg_dt;
  double pred_dlat = combined_dlat * scale;
  double pred_dlon = combined_dlon * scale;

  // Neue prognostizierte Position
  double predLat = lastLat + pred_dlat;
  double predLon = lastLon + pred_dlon;

  // Abstand in degrees (adjust lon by cos(lat))
  double lat_rad = lastLat * M_PI / 180.0;
  double dlon_adj = pred_dlon * cos(lat_rad);  // effective lat-projected lon
  double dist_deg = sqrt(pred_dlat * pred_dlat + dlon_adj * dlon_adj);

  // Geschwindigkeit berechnen (kn)
  // distance_deg -> NM = deg * 60
  // avg_time_seconds ~ predict_s (we used predict_s), so speed_knots = distance_NM / (predict_s/3600)
  double distance_nm = dist_deg * 60.0;
  double hours = predict_s / 3600.0;
  double speed_knots = (hours > 0.0) ? (distance_nm / hours) : 0.0;

  // Kurs berechnen (deg) — atan2(dlon_adj, dlat)
  double kurs_rad = atan2(dlon_adj, pred_dlat);
  double kurs_deg = kurs_rad * 180.0 / M_PI;
  if (kurs_deg < 0.0) kurs_deg += 360.0;

  // Wenn sehr klein speed, behandeln als still (Rauschen)
  if (speed_knots < GPS_MIN_VALID_SPEED) {
    speed_knots = 0.0;
    // optional: kurs behalten oder auf 0 setzen; wir behalten alten Kurs
    // kurs_deg = sensorData.gps_kurs; // falls gewünscht
  }

  // Schreib in sensorData (überschreibt Rohwerte mit gefilterten)
  sensorData.gps_lat = predLat;
  sensorData.gps_lon = predLon;
  sensorData.gps_speed = speed_knots;
  sensorData.gps_kurs = kurs_deg;
}
