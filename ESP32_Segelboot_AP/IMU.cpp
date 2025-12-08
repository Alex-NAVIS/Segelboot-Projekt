#include <Wire.h>
#include <LittleFS.h>
#include <ArduinoJson.h>
#include "ICM_20948.h"
#include "IMU.h"
#include <MadgwickAHRS.h>

// -------------------- Konfiguration --------------------
#define IMU_ADDR 0x69
#define IMU_LOOP_HZ 200
#define MAG_HZ 10
#define MAG_INTERVAL (1000 / MAG_HZ)  // ms

ICM_20948_I2C myICM;
Madgwick filter;

// -------------------- Offsets --------------------
static float gyroXoffset=0, gyroYoffset=0, gyroZoffset=0;
static float magXoffset=0, magYoffset=0, magZoffset=0;

// -------------------- NAVIS Koordinaten --------------------
static float ax_t, ay_t, az_t;
static float gx_t, gy_t, gz_t;
static float mx_t, my_t, mz_t;

static unsigned long lastMagMs = 0;

// -------------------- Skalierung --------------------
static const float ACC_SCALE  = 2.0f / 32768.0f;              // g
static const float GYRO_SCALE = 250.0f / 32768.0f * (PI/180); // rad/s
static const float MAG_SCALE  = 0.15f;                        // µT

// -------------------- Low-Level Transform Test-Ergebnisse --------------------
static void transform_to_navis(float raw_ax,float raw_ay,float raw_az,
                               float raw_gx,float raw_gy,float raw_gz,
                               float raw_mx,float raw_my,float raw_mz,
                               float &out_ax,float &out_ay,float &out_az,
                               float &out_gx,float &out_gy,float &out_gz,
                               float &out_mx,float &out_my,float &out_mz)
{
    // --- ACHSEN ZUORDNUNG NACH TESTDATEN ---
    // ACC
    out_ax = raw_ax;  // Pitch
    out_ay = raw_ay;  // Quer
    out_az = raw_az;  // Roll

    // GYRO
    out_gx = raw_gx;  // Pitch
    out_gy = raw_gy;  // Quer
    out_gz = raw_gz;  // Roll

    // MAG
    out_mx = raw_mx;  // Nord
    out_my = raw_my;  // Quer
    out_mz = raw_mz;  // Hoch
}

// -------------------- FS Save/Load --------------------
#define CAL_FILE "/settings/IMU_gyro.json"
#define MAG_CAL_FILE "/settings/IMU_mag.json"

static void load_gyro_cal() {
  if(!LittleFS.exists(CAL_FILE)) return;
  File f=LittleFS.open(CAL_FILE,"r");
  StaticJsonDocument<128> doc;
  deserializeJson(doc,f);
  f.close();
  gyroXoffset=doc["gx"]|0.0f;
  gyroYoffset=doc["gy"]|0.0f;
  gyroZoffset=doc["gz"]|0.0f;
}

static void save_gyro_cal() {
  if(!LittleFS.exists("/settings")) LittleFS.mkdir("/settings");
  File f=LittleFS.open(CAL_FILE,"w");
  StaticJsonDocument<128> doc;
  doc["gx"]=gyroXoffset; doc["gy"]=gyroYoffset; doc["gz"]=gyroZoffset;
  serializeJson(doc,f); f.close();
}

static void load_mag_cal() {
  if(!LittleFS.exists(MAG_CAL_FILE)) return;
  File f=LittleFS.open(MAG_CAL_FILE,"r");
  StaticJsonDocument<128> doc;
  deserializeJson(doc,f); f.close();
  magXoffset=doc["xOff"]|0.0f;
  magYoffset=doc["yOff"]|0.0f;
  magZoffset=doc["zOff"]|0.0f;
}

static void save_mag_cal() {
  if(!LittleFS.exists("/settings")) LittleFS.mkdir("/settings");
  File f=LittleFS.open(MAG_CAL_FILE,"w");
  StaticJsonDocument<128> doc;
  doc["xOff"]=magXoffset; doc["yOff"]=magYoffset; doc["zOff"]=magZoffset;
  serializeJson(doc,f); f.close();
}

// -------------------- Setup IMU --------------------
void setup_imu() {
  Wire.begin();
  if(!LittleFS.begin(true)) Serial.println("LittleFS init failed");
  
  if(myICM.begin(Wire,IMU_ADDR)!=ICM_20948_Stat_Ok) {
    Serial.println("ICM-20948 nicht gefunden!");
    while(1);
  }

  load_gyro_cal();
  load_mag_cal();

  filter.begin(IMU_LOOP_HZ);

  Serial.println("IMU setup done");
}

// -------------------- Gyro Calibration --------------------
void calibrate_gyro() {
  Serial.println("Gyro calibration: keep sensor still...");
  const int N=200;
  float sumX=0,sumY=0,sumZ=0;
  int valid=0;
  for(int i=0;i<N;i++){
    if(myICM.dataReady()){
      myICM.getAGMT();
      sumX+=myICM.gyrX();
      sumY+=myICM.gyrY();
      sumZ+=myICM.gyrZ();
      valid++;
    }else delay(2);
  }
  if(valid==0) valid=1;
  gyroXoffset=sumX/valid;
  gyroYoffset=sumY/valid;
  gyroZoffset=sumZ/valid;
  save_gyro_cal();
  Serial.println("Gyro calibration done");
}

// -------------------- Mag Calibration --------------------
void calibrate_magnetometer(int durationSeconds) {
  Serial.println("Mag calibration: slowly rotate sensor for duration...");
  unsigned long startMs=millis();
  float mxmin=1e6,mxmax=-1e6;
  float mymin=1e6,mymax=-1e6;
  float mzmin=1e6,mzmax=-1e6;
  
  while(millis()-startMs<durationSeconds*1000){
    if(myICM.dataReady()){
      myICM.getAGMT();
      float mx=myICM.magX(), myy=myICM.magY(), mz=myICM.magZ();
      if(mx<mxmin) mxmin=mx; if(mx>mxmax) mxmax=mx;
      if(myy<mymin) mymin=myy; if(myy>mymax) mymax=myy;
      if(mz<mzmin) mzmin=mz; if(mz>mzmax) mzmax=mz;
    }
    delay(10);
  }

  magXoffset=(mxmin+mxmax)/2.0f;
  magYoffset=(mymin+mymax)/2.0f;
  magZoffset=(mzmin+mzmax)/2.0f;
  save_mag_cal();
  Serial.println("Mag calibration done");
}

// -------------------- Read IMU --------------------
void read_imu() {
  if(!myICM.dataReady()) return;
  myICM.getAGMT();

  float raw_ax=myICM.accX();
  float raw_ay=myICM.accY();
  float raw_az=myICM.accZ();

  float raw_gx=myICM.gyrX()-gyroXoffset;
  float raw_gy=myICM.gyrY()-gyroYoffset;
  float raw_gz=myICM.gyrZ()-gyroZoffset;

  float raw_mx=myICM.magX()-magXoffset;
  float raw_my=myICM.magY()-magYoffset;
  float raw_mz=myICM.magZ()-magZoffset;

  transform_to_navis(raw_ax,raw_ay,raw_az,
                     raw_gx,raw_gy,raw_gz,
                     raw_mx,raw_my,raw_mz,
                     ax_t,ay_t,az_t,
                     gx_t,gy_t,gz_t,
                     mx_t,my_t,mz_t);

  // --- Skalieren ---
  ax_t *= ACC_SCALE; ay_t *= ACC_SCALE; az_t *= ACC_SCALE;
  gx_t *= GYRO_SCALE; gy_t *= GYRO_SCALE; gz_t *= GYRO_SCALE;
  mx_t *= MAG_SCALE; my_t *= MAG_SCALE; mz_t *= MAG_SCALE;

  // --- Filterupdate ---
  filter.update(gx_t, gy_t, gz_t, ax_t, ay_t, az_t, mx_t, my_t, mz_t);

  // --- Winkel in Grad ---
  float roll  = filter.getRoll();
  float pitch = filter.getPitch();
  float yaw   = filter.getYaw();
  if(yaw<0) yaw+=360.0f; // Kompass 0-360°

  Serial.printf("ROLL: %.2f PITCH: %.2f YAW: %.2f\n", roll, pitch, yaw);
}
