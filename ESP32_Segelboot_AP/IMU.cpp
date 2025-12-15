#include "IMU.h"


// --- Definition der Konstante fuer den Dateinamen ---
// -------------------- FS Save/Load --------------------
#define CAL_FILE "/settings/IMU_gyro.json"
#define MAG_CAL_FILE "/settings/IMU_mag.json"

// --- Instanzen ---
static ICM_20948_I2C myICM;
static Madgwick filter;
static bool imu_initialized = false;

// --- Skalierung / Konstanten ---
float MAG_SCALE = 0.15; // µT pro LSB

// --- veränderbare Hard-Iron Offsets ---
float mag_offset_x = 64.0;
float mag_offset_y = 40.0;
float mag_offset_z = -203.0;

// --- veränderbare Gyro Offsets ---
float gyro_offset_x = 0.0;
float gyro_offset_y = 0.0;
float gyro_offset_z = 0.0;


// -------------------- Gyro Calibration Load --------------------
static void load_gyro_cal() {
  // Stellen Sie sicher, dass LittleFS.begin(true) in setup() aufgerufen wurde!

  if(!LittleFS.exists(CAL_FILE)) {
    Serial.println("Info: Gyro-Kalibrierungsdatei nicht gefunden, verwende Standardwerte (0.0).");
    return;
  }
  
  File f = LittleFS.open(CAL_FILE, "r");
  if (!f) {
    Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Lesen.");
    return;
  }
  
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, f); 
  f.close();

  if (error) {
    Serial.print("Fehler beim Deserialisieren der Gyro-Kalibrierung: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Lese Werte aus JSON und weise sie den neuen Variablen zu
  gyro_offset_x = doc["gx"] | 0.0f;
  gyro_offset_y = doc["gy"] | 0.0f;
  gyro_offset_z = doc["gz"] | 0.0f;
  
  Serial.println("Gyroskop Kalibrierung geladen.");
}

// -------------------- Gyro Calibration Save --------------------
static void save_gyro_cal() {
  if(!LittleFS.exists("/settings")) {
    LittleFS.mkdir("/settings");
  }
  
  File f = LittleFS.open(CAL_FILE, "w");
  if (!f) {
    Serial.println("Fehler beim Öffnen der Gyro-Kalibrierungsdatei zum Schreiben.");
    return;
  }

  StaticJsonDocument<128> doc;
  // Schreibe Werte der neuen Variablen in das JSON-Dokument
  doc["gx"] = gyro_offset_x; 
  doc["gy"] = gyro_offset_y; 
  doc["gz"] = gyro_offset_z;
  
  if (serializeJson(doc, f) == 0) {
    Serial.println("Fehler beim Schreiben der Gyro-Kalibrierung in die Datei.");
  }
  f.close();
  Serial.println("Gyroskop Kalibrierung gespeichert.");
}

// -------------------- Gyro Calibration --------------------
void calibrate_gyro() {
  Serial.println("Gyro calibration: keep sensor still...");
  const int N = 200;
  float sumX = 0, sumY = 0, sumZ = 0;
  int valid = 0;
  
  for(int i = 0; i < N; i++) {
    if(myICM.dataReady()) {
      myICM.getAGMT(); // Daten lesen (befüllt die internen Strukturen)
      
      // Die SparkFun Library liefert hier die skalierten DPS (Degrees per Second)
      sumX += myICM.gyrX(); 
      sumY += myICM.gyrY();
      sumZ += myICM.gyrZ();
      valid++;
    } else {
      delay(2); // Gibt der Hardware Zeit, neue Daten bereitzustellen
    }
  }
  
  if(valid == 0) valid = 1; // Division durch Null verhindern

  // Berechne die Offsets (Durchschnitt der Ruhewerte) und weise sie den globalen Variablen zu:
  gyro_offset_x = sumX / valid;
  gyro_offset_y = sumY / valid;
  gyro_offset_z = sumZ / valid;

  save_gyro_cal(); // Ruft Ihre Speicherfunktion auf
  Serial.println("Gyro calibration done");
}

// -------------------- Mag Calibration Load --------------------
static void load_mag_cal() {
  
  if(!LittleFS.exists(MAG_CAL_FILE)) {
    Serial.println("Info: Kalibrierungsdatei nicht gefunden, verwende Standardwerte.");
    return;
  }
  
  File f = LittleFS.open(MAG_CAL_FILE, "r");
  if (!f) {
    Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Lesen.");
    return;
  }
  
  StaticJsonDocument<128> doc;
  DeserializationError error = deserializeJson(doc, f); 
  f.close();

  if (error) {
    Serial.print("Fehler beim Deserialisieren der Kalibrierung: ");
    Serial.println(error.c_str());
    return;
  }
  
  // Lese Werte aus JSON und weise sie den neuen Variablen zu
  mag_offset_x = doc["xOff"] | 0.0f;
  mag_offset_y = doc["yOff"] | 0.0f;
  mag_offset_z = doc["zOff"] | 0.0f;
  
  Serial.println("Magnetometer Kalibrierung geladen.");
}

// -------------------- Mag Calibration Save --------------------
static void save_mag_cal() {
  if(!LittleFS.exists("/settings")) {
    LittleFS.mkdir("/settings");
  }
  
  File f = LittleFS.open(MAG_CAL_FILE, "w");
  if (!f) {
    Serial.println("Fehler beim Öffnen der Kalibrierungsdatei zum Schreiben.");
    return;
  }

  StaticJsonDocument<128> doc;
  // Schreibe Werte der neuen Variablen in das JSON-Dokument
  doc["xOff"] = mag_offset_x; 
  doc["yOff"] = mag_offset_y; 
  doc["zOff"] = mag_offset_z;
  
  if (serializeJson(doc, f) == 0) {
    Serial.println("Fehler beim Schreiben der Kalibrierung in die Datei.");
  }
  f.close();
  Serial.println("Magnetometer Kalibrierung gespeichert.");
}

// -------------------- Mag Calibration --------------------
void calibrate_magnetometer(int durationSeconds) {
  Serial.println("Mag calibration: slowly rotate sensor for duration...");
  unsigned long startMs = millis();
  
  // Initialisiere Min/Max Werte
  float mxmin = 1e6, mxmax = -1e6;
  float mymin = 1e6, mymax = -1e6;
  float mzmin = 1e6, mzmax = -1e6;
  
  while(millis() - startMs < durationSeconds * 1000) {
    if(myICM.dataReady()) {
      myICM.getAGMT(); // Daten lesen (befüllt myICM.agmt Struktur)
      
      // Lesen der Rohdaten aus der myICM.agmt Struktur
      // Die SparkFun Library liefert hier int16_t Rohwerte
      float mx = myICM.agmt.mag.axes.x; 
      float my = myICM.agmt.mag.axes.y; 
      float mz = myICM.agmt.mag.axes.z; 

      if(mx < mxmin) mxmin = mx; if(mx > mxmax) mxmax = mx;
      if(my < mymin) mymin = my; if(my > mymax) mymax = my;
      if(mz < mzmin) mzmin = mz; if(mz > mzmax) mzmax = mz;
    }
    delay(10); // Verhindert blockieren der Loop zu 100%
  }

  // Berechne die Offsets und weise sie den globalen Variablen zu:
  mag_offset_x = (mxmin + mxmax) / 2.0f;
  mag_offset_y = (mymin + mymax) / 2.0f;
  mag_offset_z = (mzmin + mzmax) / 2.0f;

  save_mag_cal(); // Ruft Ihre Speicherfunktion auf
  Serial.println("Mag calibration done");
}

void setup_imu(TwoWire &wirePort) {
    wirePort.begin(8, 9); // SDA, SCL
    wirePort.setClock(400000);
    int ad0_val = 1;       // feste Adresse 0x69

    while (!imu_initialized) {
        myICM.begin(wirePort, ad0_val);
        if (myICM.status == ICM_20948_Stat_Ok) {
            imu_initialized = true;
        } else {
            delay(500);
        }
    }
    filter.begin(100); // 100 Hz Update-Rate
}

void read_imu() {
    if (!imu_initialized) return;
    if (!myICM.dataReady()) return;

    myICM.getAGMT();

    // --- Beschleunigung ---
    float ax = myICM.accX() / 1000.0;
    float ay = myICM.accY() / 1000.0;
    float az = myICM.accZ() / 1000.0;

    // --- Gyroskop ---
    float gx = (myICM.gyrX() - gyro_offset_x) * DEG_TO_RAD;
    float gy = (myICM.gyrY() - gyro_offset_y) * DEG_TO_RAD;
    float gz = (myICM.gyrZ() - gyro_offset_z) * DEG_TO_RAD;

    // --- Magnetometer Rohdaten + Skalierung + Hard-Iron ---
    float mx_raw = (myICM.agmt.mag.axes.x - mag_offset_x) * MAG_SCALE;
    float my_raw = (myICM.agmt.mag.axes.y - mag_offset_y) * MAG_SCALE;
    float mz_raw = (myICM.agmt.mag.axes.z - mag_offset_z) * MAG_SCALE;

    // --- Achsen-Mapping (Axis_Map 2) ---
    float mx_korr = mx_raw;
    float my_korr = -my_raw;
    float mz_korr = -mz_raw;

    // --- Madgwick Filter Update ---
    filter.update(gx, gy, gz, ax, ay, az, mx_korr, my_korr, mz_korr);

    // --- Ergebnisse direkt in sensorData schreiben ---
    sensorData.roll    = filter.getRoll();
    sensorData.pitch   = filter.getPitch();
    sensorData.kompass = -filter.getYaw(); // Kompass-Richtung korrigieren

    // --- Normalisierung Yaw 0..360° ---
    if (sensorData.kompass < 0)   sensorData.kompass += 360.0;
    if (sensorData.kompass > 360) sensorData.kompass -= 360.0;
}


