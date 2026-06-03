// ======================================================================
// Datei: NAVIS_DataFusion.cpp
// Beschreibung:
// Intelligente Datenfusion für GPS, Wind und Echolot.
//
// Features:
// - Ringspeicher (Historisierung der Sensorwerte)
// - Zeitüberwachung (automatisches Altern/Verwerfen veralteter Daten)
// - Mehrheitslogik (Validierung durch gegenseitige Stichproben)
// - Ausreißererkennung (Schutz vor fehlerhaften Sensorsprüngen)
// - Winkelkorrektur 0°/360° (Moduloberechnung und Vektormittelung)
// - Quellenunabhängigkeit (Kombiniert UDP, TCP, RS-422 und CAN)
// - Robuste Mittelwertbildung (Fallback bei fehlender Konsistenz)
//
// Finale Ausgabe nach:
// extern_sensorData
// ======================================================================

#include "NAVIS_DataFusion.h"
#include "Sensor_Data.h"
#include "config.h"
#include <math.h>

// ======================================================================
// Einstellungen
// ======================================================================

#define GPS_BUFFER_SIZE       3
#define WIND_BUFFER_SIZE      3
#define DEPTH_BUFFER_SIZE     3

#define GPS_MAX_AGE_MS        5000
#define WIND_MAX_AGE_MS       5000
#define DEPTH_MAX_AGE_MS      5000

#define GPS_SPEED_DIFF_MAX    5.0
#define GPS_COURSE_DIFF_MAX   20.0
#define GPS_POSITION_DIFF_MAX 0.0010

#define WIND_SPEED_DIFF_MAX   8.0
#define WIND_ANGLE_DIFF_MAX   30.0

#define DEPTH_DIFF_MAX        3.0

// ======================================================================
// Datenstrukturen
// ======================================================================

struct GPSSample {
    double lat;
    double lon;
    double speed;
    double course;
    int sats;
    float hdop;
    uint32_t timestamp;
    uint8_t source;
    bool valid;
};

struct WindSample {
    double angle;
    double speed;
    uint32_t timestamp;
    uint8_t source;
    bool valid;
};

struct DepthSample {
    double depth;
    uint32_t timestamp;
    uint8_t source;
    bool valid;
};

// ======================================================================
// Ringspeicher
// ======================================================================

static GPSSample gpsBuffer[GPS_BUFFER_SIZE];
static WindSample windBuffer[WIND_BUFFER_SIZE];
static DepthSample depthBuffer[DEPTH_BUFFER_SIZE];

static uint8_t gpsIndex = 0;
static uint8_t windIndex = 0;
static uint8_t depthIndex = 0;

// ======================================================================
// Hilfsfunktionen
// ======================================================================

double diffAbs(double a,double b){ return fabs(a-b); }

double angleDiff(double a,double b){
    double d=fmod((a-b+540.0),360.0)-180.0;
    return fabs(d);
}

double normalizeAngle(double a){
    while(a<0) a+=360.0;
    while(a>=360.0) a-=360.0;
    return a;
}

double averageAngle(double *angles,int count){
    double x=0;
    double y=0;

    for(int i=0;i<count;i++){
        double r=angles[i]*DEG_TO_RAD;
        x+=cos(r);
        y+=sin(r);
    }

    return normalizeAngle(atan2(y,x)*RAD_TO_DEG);
}

// ======================================================================
// GPS PUSH
// ======================================================================

void NAVIS_pushGPS(
    double lat,
    double lon,
    double speed,
    double course,
    int sats,
    float hdop,
    uint8_t source
){
    GPSSample &s=gpsBuffer[gpsIndex];

    s.lat=lat;
    s.lon=lon;
    s.speed=speed;
    s.course=normalizeAngle(course);
    s.sats=sats;
    s.hdop=hdop;
    s.timestamp=millis();
    s.source=source;
    s.valid=true;

    gpsIndex++;
    if(gpsIndex>=GPS_BUFFER_SIZE) gpsIndex=0;
}

// ======================================================================
// WIND PUSH
// ======================================================================

void NAVIS_pushWind(
    double angle,
    double speed,
    uint8_t source
){
    WindSample &s=windBuffer[windIndex];

    s.angle=normalizeAngle(angle);
    s.speed=speed;
    s.timestamp=millis();
    s.source=source;
    s.valid=true;

    windIndex++;
    if(windIndex>=WIND_BUFFER_SIZE) windIndex=0;
}

// ======================================================================
// DEPTH PUSH
// ======================================================================

void NAVIS_pushDepth(
    double depth,
    uint8_t source
){
    DepthSample &s=depthBuffer[depthIndex];

    s.depth=depth;
    s.timestamp=millis();
    s.source=source;
    s.valid=true;

    depthIndex++;
    if(depthIndex>=DEPTH_BUFFER_SIZE) depthIndex=0;
}

// ======================================================================
// GPS AUSWERTUNG
// ======================================================================
// ======================================================================
// GPS AUSWERTUNG
// ======================================================================

void evaluateGPS(){
    uint32_t now=millis();

    GPSSample valid[GPS_BUFFER_SIZE];
    int validCount=0;

    // Filtert veraltete oder ungültige Datensätze aus dem Ringspeicher
    for(int i=0;i<GPS_BUFFER_SIZE;i++){
        if(!gpsBuffer[i].valid) continue;
        if((now-gpsBuffer[i].timestamp)>GPS_MAX_AGE_MS) continue;
        valid[validCount++]=gpsBuffer[i];
    }

    // Fallback 1: Keine gültigen Daten vorhanden -> Standardwerte setzen
    if(validCount==0){
        extern_sensorData.gps_lat=0.0;
        extern_sensorData.gps_lon=0.0;
        extern_sensorData.gps_speed=0.0;
        extern_sensorData.gps_kurs=0.0;
        extern_sensorData.gps_sats=0;
        extern_sensorData.gps_hdop=99.9;
        return;
    }

    // Fallback 2: Nur ein Datensatz vorhanden -> Ungefiltert übernehmen
    if(validCount==1){
        extern_sensorData.gps_lat=valid[0].lat;
        extern_sensorData.gps_lon=valid[0].lon;
        extern_sensorData.gps_speed=valid[0].speed;
        extern_sensorData.gps_kurs=valid[0].course;
        extern_sensorData.gps_sats=valid[0].sats;
        extern_sensorData.gps_hdop=valid[0].hdop;
        return;
    }

    int bestIndex=-1;
    int bestScore=0;

    // Mehrheitslogik: Vergleicht Stichproben auf Positions-, Speed- und Kurs-Konsistenz
    for(int i=0;i<validCount;i++){
        int score=0;

        for(int j=0;j<validCount;j++){
            if(i==j) continue;

            bool posOK=
                diffAbs(valid[i].lat,valid[j].lat)<GPS_POSITION_DIFF_MAX &&
                diffAbs(valid[i].lon,valid[j].lon)<GPS_POSITION_DIFF_MAX;

            bool speedOK=
                diffAbs(valid[i].speed,valid[j].speed)<GPS_SPEED_DIFF_MAX;

            bool courseOK=
                angleDiff(valid[i].course,valid[j].course)<GPS_COURSE_DIFF_MAX;

            if(posOK && speedOK && courseOK) score++;
        }

        if(score>bestScore){
            bestScore=score;
            bestIndex=i;
        }
    }

    // Option A: Ein fehlerfreier "Gewinner"-Satz wurde ermittelt
    if(bestIndex>=0 && bestScore>=1){
        GPSSample &s=valid[bestIndex];

        extern_sensorData.gps_lat=s.lat;
        extern_sensorData.gps_lon=s.lon;
        extern_sensorData.gps_speed=s.speed;
        extern_sensorData.gps_kurs=s.course;
        extern_sensorData.gps_sats=s.sats;
        extern_sensorData.gps_hdop=s.hdop;
        return;
    }

    // Option B: Keine eindeutige Konsistenz -> Arithmetische Mittelung
    double lat=0;
    double lon=0;
    double speed=0;
    double hdop=0;
    int sats=0;

    double courseList[GPS_BUFFER_SIZE];

    for(int i=0;i<validCount;i++){
        lat+=valid[i].lat;
        lon+=valid[i].lon;
        speed+=valid[i].speed;
        sats+=valid[i].sats;
        hdop+=valid[i].hdop;
        courseList[i]=valid[i].course;
    }

    extern_sensorData.gps_lat=lat/validCount;
    extern_sensorData.gps_lon=lon/validCount;
    extern_sensorData.gps_speed=speed/validCount;
    extern_sensorData.gps_kurs=averageAngle(courseList,validCount);
    extern_sensorData.gps_sats=sats/validCount;
    extern_sensorData.gps_hdop=hdop/validCount;
}

// ======================================================================
// WIND AUSWERTUNG
// ======================================================================

void evaluateWind(){
    uint32_t now=millis();

    WindSample valid[WIND_BUFFER_SIZE];
    int validCount=0;

    // Filtert veraltete oder ungültige Winddatensätze
    for(int i=0;i<WIND_BUFFER_SIZE;i++){
        if(!windBuffer[i].valid) continue;
        if((now-windBuffer[i].timestamp)>WIND_MAX_AGE_MS) continue;
        valid[validCount++]=windBuffer[i];
    }

    // Fallback 1: Keine gültigen Daten vorhanden -> Fehlerwerte setzen
    if(validCount==0){
        extern_sensorData.winddir_gemessen=-999;
        extern_sensorData.windspeed_gemessen=-1;
        return;
    }

    // Fallback 2: Nur ein Datensatz vorhanden -> Direkt ausgeben
    if(validCount==1){
        extern_sensorData.winddir_gemessen=valid[0].angle;
        extern_sensorData.windspeed_gemessen=valid[0].speed;
        return;
    }

    int bestIndex=-1;
    int bestScore=0;

    // Mehrheitslogik: Prüft Konsistenz von Windwinkel und Windgeschwindigkeit
    for(int i=0;i<validCount;i++){
        int score=0;

        for(int j=0;j<validCount;j++){
            if(i==j) continue;

            bool angleOK=
                angleDiff(valid[i].angle,valid[j].angle)<WIND_ANGLE_DIFF_MAX;

            bool speedOK=
                diffAbs(valid[i].speed,valid[j].speed)<WIND_SPEED_DIFF_MAX;

            if(angleOK && speedOK) score++;
        }

        if(score>bestScore){
            bestScore=score;
            bestIndex=i;
        }
    }

    // Option A: Konsistenter Sensorwert setzt sich durch
    if(bestIndex>=0 && bestScore>=1){
        extern_sensorData.winddir_gemessen=valid[bestIndex].angle;
        extern_sensorData.windspeed_gemessen=valid[bestIndex].speed;
        return;
    }

    // Option B: Diskrepanzen im Speicher -> Trigonometrisches Mittel berechnen
    double speed=0;
    double angleList[WIND_BUFFER_SIZE];

    for(int i=0;i<validCount;i++){
        speed+=valid[i].speed;
        angleList[i]=valid[i].angle;
    }

    extern_sensorData.windspeed_gemessen=speed/validCount;
    extern_sensorData.winddir_gemessen=averageAngle(angleList,validCount);
}

// ======================================================================
// DEPTH AUSWERTUNG
// ======================================================================

void evaluateDepth(){
    uint32_t now=millis();

    DepthSample valid[DEPTH_BUFFER_SIZE];
    int validCount=0;

    // Filtert veraltete oder ungültige Tiefendatensätze
    for(int i=0;i<DEPTH_BUFFER_SIZE;i++){
        if(!depthBuffer[i].valid) continue;
        if((now-depthBuffer[i].timestamp)>DEPTH_MAX_AGE_MS) continue;
        valid[validCount++]=depthBuffer[i];
    }

    // Fallback 1: Keine gültigen Daten vorhanden -> Fehlerwert setzen
    if(validCount==0){
        extern_sensorData.Echolot=-1;
        return;
    }

    // Fallback 2: Nur ein Datensatz vorhanden -> Direkt übernehmen
    if(validCount==1){
        extern_sensorData.Echolot=valid[0].depth;
        return;
    }

    int bestIndex=-1;
    int bestScore=0;

    // Mehrheitslogik: Filterung von Sprüngen und Ausreißern (z.B. Fischschwärme)
    for(int i=0;i<validCount;i++){
        int score=0;

        for(int j=0;j<validCount;j++){
            if(i==j) continue;

            if(diffAbs(valid[i].depth,valid[j].depth)<DEPTH_DIFF_MAX)
                score++;
        }

        if(score>bestScore){
            bestScore=score;
            bestIndex=i;
        }
    }

    // Option A: Valider Tiefenwert ohne unplausible Sprünge ermittelt
    if(bestIndex>=0 && bestScore>=1){
        extern_sensorData.Echolot=valid[bestIndex].depth;
        return;
    }

    // Option B: Signalschwankungen -> Berechneten Mittelwert ausgeben
    double depth=0;

    for(int i=0;i<validCount;i++)
        depth+=valid[i].depth;

    extern_sensorData.Echolot=depth/validCount;
}

// ======================================================================
// SETUP
// ======================================================================

void NAVIS_DataFusion_setup(){
    // Setzt alle Ringspeicher-Slots beim Systemstart auf ungültig
    for(int i=0;i<GPS_BUFFER_SIZE;i++)
        gpsBuffer[i].valid=false;

    for(int i=0;i<WIND_BUFFER_SIZE;i++)
        windBuffer[i].valid=false;

    for(int i=0;i<DEPTH_BUFFER_SIZE;i++)
        depthBuffer[i].valid=false;
}

// ======================================================================
// UPDATE
// ======================================================================

void NAVIS_DataFusion_update(){
    // Zyklischer Aufruf aller Teilauswertungen aus der Hauptschleife
    evaluateGPS();
    evaluateWind();
    evaluateDepth();
}
