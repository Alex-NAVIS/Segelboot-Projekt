// ======================================================================
// Modul: Alarm
// Erklärung:   Zentrale Alarmüberwachung für das NAVIS-System.
//              Dieses Modul wertet kontinuierlich Sensordaten aus und
//              prüft konfigurierbare Grenzwerte für Geschwindigkeit,
//              Wind, Kursabweichung, Ankerdrift sowie Lage (Roll/Pitch).
//
//              Enthält:
//              - Priorisierte Alarmentscheidung
//              - Hysterese zur Flattervermeidung
//              - Stabilitätsfilter (Zeitverzögerung)
//              - Physikalisch verbesserte Ankerkreisberechnung
// ======================================================================

#include "Alarm.h"
#include <math.h>

// ===========================
// interne Zustände
// ===========================

// Zeitpunkt seit dem ein neuer Alarmkandidat anliegt
static unsigned long alarmSince = 0;

// Anker-Mittelpunkt wurde bereits berechnet
static bool anchorCenterLocked = false;

// gespeicherter Mittelpunkt des Ankerkreises
static double anchorCenterLat = 0.0;
static double anchorCenterLon = 0.0;


// ======================================================================
// Funktion: angleDiff
// Erklärung:   Berechnet die kleinste Winkeldifferenz zwischen zwei
//              Winkeln (0…360°). Wird für Kurs- und Windvergleiche
//              verwendet.
// Rückgabe:    Absoluter Winkelunterschied in Grad (0…180)
// ======================================================================
static float angleDiff(float a, float b) {
  float diff = fabs(a - b);
  if (diff > 180.0f) diff = 360.0f - diff;
  return diff;
}


// ======================================================================
// Funktion: distanceMeters
// Erklärung:   Berechnet die Distanz zwischen zwei GPS-Koordinaten
//              mittels Haversine-Formel.
// Rückgabe:    Entfernung in Metern
// ======================================================================
static double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000.0;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  lat1 = radians(lat1);
  lat2 = radians(lat2);
  double a = sin(dLat / 2) * sin(dLat / 2) + cos(lat1) * cos(lat2) * sin(dLon / 2) * sin(dLon / 2);
  double c = 2 * atan2(sqrt(a), sqrt(1 - a));
  return R * c;
}


// ======================================================================
// Funktion: setup_alarm
// Erklärung:   Initialisiert den Summer-Pin und kann optional einen
//              kurzen Testton ausgeben.
// Parameter:   testSignal → wenn true, wird ein kurzer Ton erzeugt
// ======================================================================
void setup_alarm(bool testSignal) {
  pinMode(BUZZER_PIN, OUTPUT);
  digitalWrite(BUZZER_PIN, LOW);

  if (testSignal) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(300);
    digitalWrite(BUZZER_PIN, LOW);
  }
}


// ======================================================================
// Funktion: alarm_trigger
// Erklärung:   Schaltet den Summer entsprechend des Alarmzustands.
// Parameter:   state → true = Summer EIN, false = AUS
// ======================================================================
void alarm_trigger(bool state) {
  digitalWrite(BUZZER_PIN, state ? HIGH : LOW);
}


// ======================================================================
// Funktion: hyst
// Erklärung:   Allgemeiner Hysterese-Wrapper zur Entprellung von
//              Grenzwertentscheidungen.
// Rückgabe:    Neuer stabilisierter Zustand
// ======================================================================
static bool hyst(bool currentState, bool triggerOn, bool triggerOff) {
  if (!currentState && triggerOn) return true;
  if (currentState && !triggerOff) return true;
  return false;
}


// ======================================================================
// Funktion: check_alarm
// Erklärung:   Zentrale Alarm-Auswertung mit Prioritätenlogik.
//              Bestimmt den aktuell gültigen Alarmzustand,
//              wendet Stabilitätsfilter an und steuert den Summer.
// ======================================================================
void check_alarm(void) {
  int newAlarm = ALARM_NONE;

  // ======================================================
  // PRIORITÄTEN (oben = wichtiger)
  // ======================================================
  if (alarmData.en_echolot && check_echolot()) {
    newAlarm = ALARM_ECHOLOT;
  } else if (alarmData.en_speed_max && check_speed_max()) {
    newAlarm = ALARM_SPEED_MAX;
  } else if (alarmData.en_speed_min && check_speed_min()) {
    newAlarm = ALARM_SPEED_MIN;
  } else if (alarmData.en_wind_max && check_wind_max()) {
    newAlarm = ALARM_WIND_MAX;
  } else if (alarmData.en_wind_min && check_wind_min()) {
    newAlarm = ALARM_WIND_MIN;
  } else if (alarmData.en_wind_dir && check_wind_dir()) {
    newAlarm = ALARM_WIND_DIR;
  } else if (alarmData.en_course_dev && check_course_dev()) {
    newAlarm = ALARM_COURSE_DEV;
  } else if (alarmData.en_roll && check_roll()) {
    newAlarm = ALARM_ROLL;
  } else if (alarmData.en_pitch && check_pitch()) {
    newAlarm = ALARM_PITCH;
  }

  // ======================================================
  // ANKER SEPARAT
  // ======================================================
  if (newAlarm == ALARM_NONE && alarmData.en_anchor && check_anchor()) {
    newAlarm = ALARM_ANCHOR;
  }

  // ======================================================
  // Stabilitätsfilter
  // ======================================================
  static int alarmCandidateCode = ALARM_NONE;
  static int alarmActiveCode = ALARM_NONE;

  if (newAlarm != alarmCandidateCode) {
    alarmCandidateCode = newAlarm;
    alarmSince = millis();
  }

  if (alarmActiveCode != alarmCandidateCode) {
    if (millis() - alarmSince > ALARM_STABLE_MS) {
      alarmActiveCode = alarmCandidateCode;
    }
  }

  // Ergebnis übernehmen
  alarmData.alarm = alarmActiveCode;
  alarm_trigger(alarmActiveCode != ALARM_NONE);
}

// ======================================================================
// Funktion: check_echolot
// Erklärung:   Prüft das Echolot auf unterschreiten der minimalen Tiefe
//              mit Hysterese.
// ======================================================================
bool check_echolot(void) {
  static bool state = false;

  if (!state)
    state = sensorData.Echolot < alarmData.echolotMin;
  else
    state = sensorData.Echolot < alarmData.echolotMin + HYST_ECHOLOT;
  return state;
}

// ======================================================================
// Funktion: check_speed_max
// Erklärung:   Prüft Überschreitung der maximalen Geschwindigkeit
//              mit Hysterese.
// ======================================================================
bool check_speed_max(void) {
  static bool state = false;

  if (!state)
    state = sensorData.gps_speed > alarmData.speedMax;
  else
    state = sensorData.gps_speed > (alarmData.speedMax - HYST_SPEED);

  return state;
}


// ======================================================================
// Funktion: check_speed_min
// Erklärung:   Prüft Unterschreitung der minimalen Geschwindigkeit
//              mit Hysterese.
// ======================================================================
bool check_speed_min(void) {
  static bool state = false;

  if (!state)
    state = sensorData.gps_speed < alarmData.speedMin;
  else
    state = sensorData.gps_speed < (alarmData.speedMin + HYST_SPEED);

  return state;
}


// ======================================================================
// Funktion: check_wind_max
// Erklärung:   Prüft Überschreitung der maximalen Windgeschwindigkeit.
// ======================================================================
bool check_wind_max(void) {
  static bool state = false;

  if (!state)
    state = sensorData.windspeed_gemessen > alarmData.windMax;
  else
    state = sensorData.windspeed_gemessen > (alarmData.windMax - HYST_WIND);

  return state;
}


// ======================================================================
// Funktion: check_wind_min
// Erklärung:   Prüft Unterschreitung der minimalen Windgeschwindigkeit.
// ======================================================================
bool check_wind_min(void) {
  static bool state = false;

  if (!state)
    state = sensorData.windspeed_gemessen < alarmData.windMin;
  else
    state = sensorData.windspeed_gemessen < (alarmData.windMin + HYST_WIND);

  return state;
}


// ======================================================================
// Funktion: check_wind_dir
// Erklärung:   Überwacht Abweichung der Windrichtung relativ zur
//              Referenzrichtung.
// ======================================================================
bool check_wind_dir(void) {
  static bool state = false;
  float diff = angleDiff(sensorData.winddir_berechnet, alarmData.windDir_temp);

  if (!state)
    state = diff > alarmData.windDir;
  else
    state = diff > (alarmData.windDir - HYST_ANGLE);

  return state;
}


// ======================================================================
// Funktion: check_course_dev
// Erklärung:   Überwacht Kursabweichung relativ zum Referenzkurs.
// ======================================================================
bool check_course_dev(void) {
  static bool state = false;
  float diff = angleDiff(sensorData.kompass, alarmData.courseDev_temp);

  if (!state)
    state = diff > alarmData.courseDev;
  else
    state = diff > (alarmData.courseDev - HYST_ANGLE);

  return state;
}


// ======================================================================
// Funktion: check_anchor
// Erklärung:   Überwacht das Abtreiben vom berechneten Ankerkreis.
//              Der Mittelpunkt wird einmalig aus Kettenlänge,
//              Wassertiefe und Bootsheading bestimmt und danach
//              eingefroren.
// ======================================================================
bool check_anchor(void) {
  if (!alarmData.en_anchor) {
    anchorCenterLocked = false;
    return false;
  }

  if (!anchorCenterLocked) {
    if (alarmData.anchorkette <= 0.0 || sensorData.Echolot <= 0.0) {
      return false;
    }

    double L = alarmData.anchorkette;
    double D = sensorData.Echolot;

    double horizontal = sqrt(max(0.0, L * L - D * D));

    const double SAG_FACTOR = 0.85;
    horizontal *= SAG_FACTOR;

    double bearing = sensorData.kompass + 180.0;
    if (bearing >= 360.0) bearing -= 360.0;

    const double R = 6371000.0;
    double brng = radians(bearing);
    double lat1 = radians(sensorData.gps_lat);
    double lon1 = radians(sensorData.gps_lon);

    double lat2 = asin(
      sin(lat1) * cos(horizontal / R) + cos(lat1) * sin(horizontal / R) * cos(brng));

    double lon2 = lon1 + atan2(sin(brng) * sin(horizontal / R) * cos(lat1), cos(horizontal / R) - sin(lat1) * sin(lat2));

    anchorCenterLat = degrees(lat2);
    anchorCenterLon = degrees(lon2);
    anchorCenterLocked = true;
  }

  double dist = distanceMeters(
    anchorCenterLat,
    anchorCenterLon,
    sensorData.gps_lat,
    sensorData.gps_lon);

  return dist > alarmData.anchorRadius;
}


// ======================================================================
// Funktion: check_roll
// Erklärung:   Überwacht die maximale Krängung (Roll).
// ======================================================================
bool check_roll(void) {
  static bool state = false;

  if (!state)
    state = fabs(sensorData.roll) > alarmData.roll;
  else
    state = fabs(sensorData.roll) > (alarmData.roll - HYST_ATT);

  return state;
}


// ======================================================================
// Funktion: check_pitch
// Erklärung:   Überwacht die maximale Stampfbewegung (Pitch).
// ======================================================================
bool check_pitch(void) {
  static bool state = false;

  if (!state)
    state = fabs(sensorData.pitch) > alarmData.pitch;
  else
    state = fabs(sensorData.pitch) > (alarmData.pitch - HYST_ATT);

  return state;
}
