#include <ESP8266WiFi.h>
#include <espnow.h>
#include <Wire.h>
#include "SparkFun_BNO08x_Arduino_Library.h"

#define BNO_ADDR      0x4A
#define REPORT_US     10000      // 100 Hz
#define BAUDRATE      115200

const uint8_t TRACKER_ID = 9;    // set unique ID per node
uint8_t peerMAC[] = { 0x34, 0xCD, 0xB0, 0x34, 0x93, 0xE4 };

BNO08x imu;

/* ---- diagnostic packet: the original 42 bytes, then the extra state ----
   The host tells the two apart by length, so trackers on the old firmware keep working. */
typedef struct __attribute__((packed)) {
  uint8_t tracker_id;
  float   ax, ay, az;
  float   roll, pitch, yaw;
  float   qI, qJ, qK, qW;          // rotation vector: accel + gyro + magnetometer (what the pipeline uses today)
  uint8_t battery_percent;
  float   gqI, gqJ, gqK, gqW;      // game rotation vector: accel + gyro only
  float   gx, gy, gz;              // calibrated gyro, rad/s
  float   mx, my, mz;              // calibrated magnetic field, uT
  uint8_t rv_status;               // rotation vector accuracy status, 0 (unreliable) .. 3 (high)
  uint8_t mag_status;              // magnetometer calibration status, 0 .. 3
  float   rv_accuracy_rad;         // the fusion's own estimate of its heading accuracy
  uint8_t reset_count;             // times the BNO08x has reset since power-on
} IMUDataPacket;

IMUDataPacket pkt;

/* ---- battery helpers (factor 5.00) ---- */
const float ADC_TO_VBAT = 5.00f;

float rawToVoltage(uint16_t raw)
{
  return (raw / 1023.0f) * 1.0f * ADC_TO_VBAT;
}
uint8_t vToPercent(float v)
{
  v = constrain(v, 3.3f, 4.2f);
  return (uint8_t)(((v - 3.3f) / 0.9f) * 100.0f);
}

/* Same interval constant as before for every report, so the existing rate is unchanged
   whichever units the installed library version takes. */
void enableReports() {
  imu.enableRotationVector(REPORT_US);
  imu.enableAccelerometer(REPORT_US);
  imu.enableGameRotationVector(REPORT_US);
  imu.enableGyro(REPORT_US);
  imu.enableMagnetometer(REPORT_US);
}

/* ---- latest sensor state, filled by onSensorEvent ---- */
static float ax, ay, az, roll, pitch, yaw, qI, qJ, qK, qW;
static float gqI, gqJ, gqK, gqW = 1.0f, gx, gy, gz, mx, my, mz, rvAcc;
static uint8_t rvStatus = 0, magStatus = 0;
static uint8_t got = 0;

/* Roll, pitch and yaw in degrees, with the same formulas the library's getRoll/getPitch/getYaw use. */
static void quatToEulerDeg(float w, float x, float y, float z, float &r, float &p, float &yw) {
  float norm = sqrt(w * w + x * x + y * y + z * z);
  w /= norm; x /= norm; y /= norm; z /= norm;
  float ysqr = y * y;
  r = atan2(2.0f * (w * x + y * z), 1.0f - 2.0f * (x * x + ysqr)) * 180.0f / PI;
  float t2 = constrain(2.0f * (w * y - z * x), -1.0f, 1.0f);
  p = asin(t2) * 180.0f / PI;
  yw = atan2(2.0f * (w * z + x * y), 1.0f - 2.0f * (ysqr + z * z)) * 180.0f / PI;
}

/* Called once per sensor event. One bus read can carry several events (the two rotation vectors
   arrive together), and imu.getSensorEvent() keeps only the last of them, which dropped every
   rotation vector once the extra reports were enabled. */
static void onSensorEvent(void *cookie, sh2_SensorEvent_t *event) {
  sh2_SensorValue_t v;
  if (sh2_decodeSensorEvent(&v, event) != SH2_OK) return;

  switch (v.sensorId) {
    case SH2_ROTATION_VECTOR:
      qI = v.un.rotationVector.i; qJ = v.un.rotationVector.j;
      qK = v.un.rotationVector.k; qW = v.un.rotationVector.real;
      quatToEulerDeg(qW, qI, qJ, qK, roll, pitch, yaw);
      rvStatus = v.status;
      rvAcc    = v.un.rotationVector.accuracy;
      got |= 0b01; break;

    case SH2_GAME_ROTATION_VECTOR:
      gqI = v.un.gameRotationVector.i; gqJ = v.un.gameRotationVector.j;
      gqK = v.un.gameRotationVector.k; gqW = v.un.gameRotationVector.real;
      break;

    case SH2_GYROSCOPE_CALIBRATED:
      gx = v.un.gyroscope.x; gy = v.un.gyroscope.y; gz = v.un.gyroscope.z;
      break;

    case SH2_MAGNETIC_FIELD_CALIBRATED:
      mx = v.un.magneticField.x; my = v.un.magneticField.y; mz = v.un.magneticField.z;
      magStatus = v.status;
      break;

    case SH2_ACCELEROMETER:
      ax = v.un.accelerometer.x; ay = v.un.accelerometer.y; az = v.un.accelerometer.z;
      got |= 0b10; break;
  }
}

/* ---- setup ---- */
void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin(); Wire.setClock(400000); delay(200);

  /* If the ESP is reset or reflashed while the BNO08x is mid-transfer, the sensor keeps holding SDA low
     and never answers. Wire.status() clocks the line free, so recover the bus and retry before giving up. */
  bool imuOk = false;
  for (uint8_t attempt = 0; attempt < 10 && !imuOk; attempt++) {
    Wire.status();
    imuOk = imu.begin(BNO_ADDR, Wire);
    if (!imuOk) delay(300);
  }
  if (!imuOk) { Serial.println("IMU fail"); delay(1000); ESP.restart(); }

  /* Take the sensor events straight from the sensor hub driver (see onSensorEvent). */
  sh2_setSensorCallback(onSensorEvent, NULL);
  enableReports();

  WiFi.mode(WIFI_STA); WiFi.disconnect();
  if (esp_now_init() != 0) { Serial.println("ESP‑NOW fail"); while (1); }
  esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER);
  esp_now_add_peer(peerMAC, ESP_NOW_ROLE_SLAVE, 1, NULL, 0);

  Serial.println("Tracker: sending full quat + batt%");
}

/* ---- main loop ---- */
void loop() {
  static uint8_t resets = 0;
  if (imu.wasReset()) {            // a reset drops every report: turn them back on and count it
    if (resets < 255) resets++;
    enableReports();
  }
  sh2_service();                   // runs onSensorEvent for each event that has arrived

  if (got == 0b11) {
    got = 0;

    uint16_t raw = analogRead(A0);
    float    v   = rawToVoltage(raw);
    uint8_t  batt= vToPercent(v);

    /* fill packet */
    pkt.tracker_id = TRACKER_ID;
    pkt.ax = ax; pkt.ay = ay; pkt.az = az;
    pkt.roll = roll; pkt.pitch = pitch; pkt.yaw = yaw;
    pkt.qI = qI; pkt.qJ = qJ; pkt.qK = qK; pkt.qW = qW;
    pkt.battery_percent = batt;
    pkt.gqI = gqI; pkt.gqJ = gqJ; pkt.gqK = gqK; pkt.gqW = gqW;
    pkt.gx = gx; pkt.gy = gy; pkt.gz = gz;
    pkt.mx = mx; pkt.my = my; pkt.mz = mz;
    pkt.rv_status = rvStatus; pkt.mag_status = magStatus;
    pkt.rv_accuracy_rad = rvAcc;
    pkt.reset_count = resets;

    esp_now_send(peerMAC, (uint8_t*)&pkt, sizeof(pkt));

    /* local debug (keeps raw for calibration) */
    Serial.printf("ID:%d raw:%u v:%.2f batt:%u%%\n",
                  TRACKER_ID, raw, v, batt);
  }
}


