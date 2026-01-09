#include "tide.h"
#include <SD.h>
#include <math.h>

#define MAX_DIST_NM 30.0
#define MAX_CONS 32

// =================== Globale Variablen ===================
volatile int tideState = TIDE_IDLE;
double tideQueryLat = 0;
double tideQueryLon = 0;
void* tideRequest = nullptr;

double tideStationLat[3];
double tideStationLon[3];
double tideStationDist[3];
double tideStationBearing[3];

int16_t tideCurve[3][TIDE_POINTS];

// =================== Hilfsfunktionen ===================
static double deg2rad(double d) { return d * M_PI / 180.0; }

static double distance_nm(double lat1, double lon1, double lat2, double lon2) {
    double dlat = deg2rad(lat2 - lat1);
    double dlon = deg2rad(lon2 - lon1);
    double a = sin(dlat/2)*sin(dlat/2) + cos(deg2rad(lat1))*cos(deg2rad(lat2))*sin(dlon/2)*sin(dlon/2);
    double c = 2*atan2(sqrt(a), sqrt(1-a));
    return 3440.065 * c;
}

static double bearing_deg(double lat1, double lon1, double lat2, double lon2) {
    double φ1 = deg2rad(lat1);
    double φ2 = deg2rad(lat2);
    double Δλ = deg2rad(lon2 - lon1);
    double y = sin(Δλ)*cos(φ2);
    double x = cos(φ1)*sin(φ2) - sin(φ1)*cos(φ2)*cos(Δλ);
    return fmod(atan2(y,x)*180.0/M_PI + 360.0, 360.0);
}

// =================== Tide-Frequenzen ===================
struct ConstituentFreq {
    const char* name;
    double omega; // rad/h
};

const ConstituentFreq freqTable[] = {
    {"M2", 28.9841042* M_PI/180.0}, 
    {"S2", 30.0* M_PI/180.0}, 
    {"K1", 15.0410686* M_PI/180.0},
    {"O1", 13.9430356* M_PI/180.0},
    {"N2", 28.4397295* M_PI/180.0},
    {"P1", 14.9589314* M_PI/180.0},
    {"K2", 30.0821372* M_PI/180.0},
    // ggf. weitere hinzufügen
};

// =================== Stationen + Konstituenten ===================
struct Constituent {
    double amp;
    double phase_deg;
    double omega;
};

struct Station {
    double lat;
    double lon;
    double dist;
    Constituent cons[MAX_CONS];
    int cons_count;
} stations[3];

// =================== CSV-Lesen ===================
bool find_stations() {
    char path[48];

    double tile_lat = floor(tideQueryLat * 10.0) / 10.0;
    double tile_lon = floor(tideQueryLon * 10.0) / 10.0;
    tile_lon = fmod(tile_lon + 360.0, 360.0);

    snprintf(path, sizeof(path), "/tide/%.1f_%.1f.csv", tile_lat, tile_lon);

    if(!SD.exists(path)){
        Serial.printf("Tile nicht gefunden: %s\n", path);
        return false;
    }

    File f = SD.open(path);
    if(!f){
        Serial.printf("Tile konnte nicht geöffnet werden: %s\n", path);
        return false;
    }

    f.readStringUntil('\n'); // Header überspringen

    // Top-3 initialisieren
    for(int i=0;i<3;i++){
        stations[i].dist = 1e9;
        stations[i].lat = stations[i].lon = 0;
        stations[i].cons_count = 0;
    }

    while(f.available()){
        String line = f.readStringUntil('\n');
        if(line.length()<10) continue;

        int p1 = line.indexOf(',');
        int p2 = line.indexOf(',', p1+1);
        int p3 = line.indexOf(',', p2+1);
        int p4 = line.indexOf(',', p3+1);
        if(p4 < 0) continue;

        double lat = line.substring(0,p1).toFloat();
        double lon = line.substring(p1+1,p2).toFloat();
        String cons_name = line.substring(p2+1,p3);
        double amp = line.substring(p3+1,p4).toFloat();
        double phase = line.substring(p4+1).toFloat(); // Phase in Grad

        double d = distance_nm(tideQueryLat, tideQueryLon, lat, lon);
        if(d > MAX_DIST_NM) continue;

        // Prüfen auf Duplikate in Top-3
        bool duplicate = false;
        for(int j=0;j<3;j++){
            if(stations[j].dist < 1e9 && stations[j].lat == lat && stations[j].lon == lon){
                duplicate = true;
                break;
            }
        }
        if(duplicate) continue;

        // Einfügen sortiert nach Distanz
        int idx = 0;
        while(idx < 3 && d > stations[idx].dist) idx++;
        if(idx == 3) continue;
        for(int j=2;j>idx;j--) stations[j] = stations[j-1];

        stations[idx].lat = lat;
        stations[idx].lon = lon;
        stations[idx].dist = d;

        // Frequenz aus Tabelle
        double omega = 0;
        for(auto &f : freqTable){
            if(cons_name == f.name){ omega = f.omega; break; }
        }
        if(omega>0 && stations[idx].cons_count<MAX_CONS){
            stations[idx].cons[stations[idx].cons_count].amp = amp;
            stations[idx].cons[stations[idx].cons_count].phase_deg = phase;
            stations[idx].cons[stations[idx].cons_count].omega = omega;
            stations[idx].cons_count++;
        }

        // Früher Abbruch
        if(stations[0].dist < 1.0) break;
        if(stations[0].dist<MAX_DIST_NM && stations[1].dist<MAX_DIST_NM && stations[2].dist<MAX_DIST_NM) break;
    }

    f.close();

    // Globale Variablen füllen
    for(int i=0;i<3;i++){
        tideStationLat[i] = stations[i].lat;
        tideStationLon[i] = stations[i].lon;
        tideStationDist[i] = stations[i].dist;
        tideStationBearing[i] = bearing_deg(tideQueryLat, tideQueryLon, stations[i].lat, stations[i].lon);
    }

    Serial.printf("[TIDE] Nächste Stationen gefunden:\n");
    for(int i=0;i<3;i++){
        Serial.printf(" %d) Lat=%.6f Lon=%.6f Dist=%.2f NM Brg=%.1f°\n",
            i+1, tideStationLat[i], tideStationLon[i], tideStationDist[i], tideStationBearing[i]);
    }

    return true;
}

// =================== Echte Tidekurve berechnen ===================
void berechne_tide_kurve(int idx) {
    for(int i=0;i<TIDE_POINTS;i++){
        double t = i*0.25; // Stunden
        double h = 0.0;
        for(int k=0;k<stations[idx].cons_count;k++){
            Constituent &c = stations[idx].cons[k];
            h += c.amp * cos(c.omega * t - c.phase_deg * M_PI/180.0);
        }
        tideCurve[idx][i] = int16_t(h);
    }
}

// --------------------------------------------------
// Tide komplett zurücksetzen
// --------------------------------------------------
void tide_reset() {
  tideState = TIDE_IDLE;
  tideQueryLat = 0.0;
  tideQueryLon = 0.0;
  tideRequest = nullptr;
  for (int s = 0; s < 3; s++) {
    tideStationDist[s] = 0.0f;
    tideStationBearing[s] = 0.0f;
    stations[s].cons_count = 0;
    for (int i = 0; i < TIDE_POINTS; i++) {
      tideCurve[s][i] = 0;
    }
  }
  Serial.println("[TIDE] State reset");
}
