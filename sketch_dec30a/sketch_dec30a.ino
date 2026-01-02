#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>
#include <Adafruit_MPU6050.h>
#include <Adafruit_Sensor.h>

/* ================= MPU6050 ================= */
Adafruit_MPU6050 mpu;

/* FALL THRESHOLDS */
const float FALL_ACCEL_THRESHOLD = 18.0;
const float FALL_GYRO_THRESHOLD  = 2.0;

/* FALL COOLDOWN */
unsigned long lastFallTime = 0;›
const unsigned long FALL_COOLDOWN_MS = 15000;

/* ================= GPS ================= */
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

/* HOME LOCATION */
const double HOME_LAT = 10.005671;
const double HOME_LON = 76.363100;
const double SAFE_RADIUS = 140.0;

bool wasInsideZone = true;
unsigned long lastGPSCheck = 0;
const unsigned long GPS_INTERVAL_MS = 3000;

double lastDistance = -1.0;

/* ================= WIFI ================= */
const char* WIFI_SSID = "5B23";
const char* WIFI_PASSWORD = "rajagirics";
const char* PREDICT_URL  = "http://192.168.1.6:5000/predict";
const char* GEOFENCE_URL = "http://192.168.1.6:5000/geofence";

/* ================= DISTANCE ================= */
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);
  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2)) *
             sin(dLon/2)*sin(dLon/2);
  return 2 * R * atan2(sqrt(a), sqrt(1 - a));
}

/* ================= SEND FALL TO ML ================= */
void sendFallToML(float accMag, float gyroMag) {
  if (WiFi.status() != WL_CONNECTED) return;

  float accStd  = abs(accMag - 9.8);     // deviation from gravity
  float gyroStd = gyroMag * 0.3;         // approximate variation

  HTTPClient http;
  http.begin(PREDICT_URL);
  http.addHeader("Content-Type", "application/json");

  String payload =
    "{"
    "\"acc_mean\":" + String(accMag, 2) +
    ",\"acc_std\":" + String(accStd, 2) +
    ",\"gyro_mean\":" + String(gyroMag, 2) +
    ",\"gyro_std\":" + String(gyroStd, 2) +
    "}";

  http.POST(payload);
  http.end();

  Serial.println("📡 FALL DATA SENT TO ML");
}

/* ================= SEND GEOFENCE ================= */
void sendGeofenceStatus(const char* status, double distance, double lat, double lon) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(GEOFENCE_URL);
  http.addHeader("Content-Type", "application/json");

  String payload =
    "{"
    "\"status\":\"" + String(status) + "\","
    "\"distance\":" + String(distance, 2) + ","
    "\"lat\":" + String(lat, 6) + ","
    "\"lon\":" + String(lon, 6) +
    "}";

  http.POST(payload);
  http.end();

  Serial.println("📡 Geofence event sent to Flask");
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(2000);

  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");

  Wire.begin(21, 22);
  if (!mpu.begin()) {
    Serial.println("❌ MPU6050 not found");
    while (1) delay(1000);
  }
  Serial.println("✅ MPU6050 Found");

  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);
  Serial.println("🚀 ESP32 system started");
}

/* ================= LOOP ================= */
void loop() {

  sensors_event_t a, g, temp;
  mpu.getEvent(&a, &g, &temp);

  float ax = a.acceleration.x;
  float ay = a.acceleration.y;
  float az = a.acceleration.z;
  float gx = g.gyro.x;
  float gy = g.gyro.y;
  float gz = g.gyro.z;

  float accMag  = sqrt(ax*ax + ay*ay + az*az);
  float gyroMag = sqrt(gx*gx + gy*gy + gz*gz);

  /* --------- ALWAYS PRINT VALUES --------- */
 Serial.print("Acc=");
Serial.print(accMag, 2);
Serial.print(" | Gyro=");
Serial.print(gyroMag, 2);

if (lastDistance >= 0) {
  Serial.print(" | Dist=");
  Serial.print(lastDistance, 2);
  Serial.print(" m");
}

Serial.println("  ✅ Fall Normal");

  /* --------- FALL DETECTION --------- */
  if (accMag > FALL_ACCEL_THRESHOLD &&
      gyroMag > FALL_GYRO_THRESHOLD &&
      millis() - lastFallTime > FALL_COOLDOWN_MS) {

    Serial.println("  🚨 FALL DETECTED!");
    sendFallToML(accMag, gyroMag);
    lastFallTime = millis();

  } else {
    Serial.println("  ✅ GPS Normal");
  }

  /* --------- GPS (UPDATED) --------- */
/* --------- GPS (CONTINUOUS DISTANCE PRINT) --------- */
if (millis() - lastGPSCheck >= GPS_INTERVAL_MS) {
  lastGPSCheck = millis();

  while (gpsSerial.available()) {
    gps.encode(gpsSerial.read());
  }

  if (gps.location.isValid()) {

    double lat = gps.location.lat();
    double lon = gps.location.lng();

    lastDistance = distanceMeters(
      lat,
      lon,
      HOME_LAT,
      HOME_LON
    );

    /* 🚨 EXIT SAFE ZONE */
    if (lastDistance > SAFE_RADIUS && wasInsideZone) {
      sendGeofenceStatus("OUT", lastDistance, lat, lon);
      wasInsideZone = false;
    }
    /* ✅ BACK INSIDE SAFE ZONE */
    if (lastDistance <= SAFE_RADIUS && !wasInsideZone) {
      sendGeofenceStatus("IN", lastDistance, lat, lon);
      wasInsideZone = true;
    }
  }
}
  delay(200);
}