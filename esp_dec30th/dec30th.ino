#include <WiFi.h>
#include <HTTPClient.h>
#include <Wire.h>
#include <math.h>
#include <TinyGPS++.h>
#include <HardwareSerial.h>

/* ================= MPU6050 ================= */
#define MPU_ADDR 0x68
#define WINDOW_SIZE 5

float mags[WINDOW_SIZE];
int magIndex = 0;

/* ================= GPS ================= */
TinyGPSPlus gps;
HardwareSerial gpsSerial(1);

/* HOME LOCATION */
const double HOME_LAT = 10.077101;
const double HOME_LON = 76.275080;
const double SAFE_RADIUS = 3.0;  // meters

bool wasInsideZone = true;
unsigned long lastGPSCheck = 0;
const unsigned long GPS_INTERVAL_MS = 3000;

/* ================= WIFI ================= */
const char* WIFI_SSID = "Christo's Iphone";
const char* WIFI_PASSWORD = "christopher";
const char* PREDICT_URL  = "http://192.168.248.134:5000/predict";
const char* GEOFENCE_URL = "http://192.168.248.134:5000/geofence";

/* ================= DISTANCE FUNCTION ================= */
double distanceMeters(double lat1, double lon1, double lat2, double lon2) {
  const double R = 6371000;
  double dLat = radians(lat2 - lat1);
  double dLon = radians(lon2 - lon1);

  double a = sin(dLat/2)*sin(dLat/2) +
             cos(radians(lat1))*cos(radians(lat2)) *
             sin(dLon/2)*sin(dLon/2);

  double c = 2 * atan2(sqrt(a), sqrt(1-a));
  return R * c;
}

/* ================= SEND GEOFENCE ================= */
void sendGeofenceStatus(const char* status) {
  if (WiFi.status() != WL_CONNECTED) return;

  HTTPClient http;
  http.begin(GEOFENCE_URL);
  http.addHeader("Content-Type", "application/json");

  String payload = String("{\"status\":\"") + status + "\"}";
  http.POST(payload);
  http.end();

  Serial.println("📍 Geofence sent: " + String(status));
}

/* ================= SETUP ================= */
void setup() {
  Serial.begin(115200);
  delay(2000);

  /* WiFi */
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println("\n✅ WiFi connected");

  /* MPU6050 */
  Wire.begin(21, 22);
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B);
  Wire.write(0x00);
  Wire.endTransmission();

  /* GPS */
  gpsSerial.begin(9600, SERIAL_8N1, 16, 17);

  Serial.println("🚀 ESP32 system started");
}

/* ================= LOOP ================= */
void loop() {

  /* --------- MPU6050 READ --------- */
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x3B);
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_ADDR, 6, true);

  int16_t ax = Wire.read()<<8 | Wire.read();
  int16_t ay = Wire.read()<<8 | Wire.read();
  int16_t az = Wire.read()<<8 | Wire.read();

  // Remove gravity (1g ≈ 16384)
float az_no_g = az - 16384.0;

// Compute motion magnitude only
float mag = sqrt(ax*ax + ay*ay + az_no_g*az_no_g);

// Ignore tiny movements
if (mag < 2000) {
  return;   // nothing meaningful happened
}

  mags[magIndex++] = mag;

  /* --------- WINDOW COMPLETE --------- */
  if (magIndex >= WINDOW_SIZE) {
    magIndex = 0;

    float sum = 0;
    for (int i = 0; i < WINDOW_SIZE; i++) sum += mags[i];
    float mean = sum / WINDOW_SIZE;

    float var = 0;
    for (int i = 0; i < WINDOW_SIZE; i++)
      var += pow(mags[i] - mean, 2);
    float std = sqrt(var / WINDOW_SIZE);

    HTTPClient http;
    http.begin(PREDICT_URL);
    http.addHeader("Content-Type", "application/json");

    String payload =
      "{\"mean\":" + String(mean) +
      ",\"std\":"  + String(std) + "}";

    int code = http.POST(payload);
    if (code > 0) {
      String resp = http.getString();
      Serial.println("🤖 ML Response: " + resp);
    }
    http.end();
  }

  /* --------- GPS CHECK --------- */
  if (millis() - lastGPSCheck >= GPS_INTERVAL_MS) {
    lastGPSCheck = millis();

    for (int i = 0; i < 10 && gpsSerial.available(); i++) {
      gps.encode(gpsSerial.read());
    }

    if (gps.location.isUpdated()) {
      double dist = distanceMeters(
        gps.location.lat(),
        gps.location.lng(),
        HOME_LAT,
        HOME_LON
      );

      if (dist > SAFE_RADIUS && wasInsideZone) {
        Serial.println("🚨 OUT OF SAFE ZONE 🚨");
        sendGeofenceStatus("OUT");
        wasInsideZone = false;
      }

      if (dist <= SAFE_RADIUS && !wasInsideZone) {
        Serial.println("✅ Back inside safe zone");
        sendGeofenceStatus("IN");
        wasInsideZone = true;
      }
    }
  }
}
