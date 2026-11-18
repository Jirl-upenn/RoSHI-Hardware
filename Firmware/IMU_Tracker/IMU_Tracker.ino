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

/* ---- setup ---- */
void setup() {
  Serial.begin(BAUDRATE);
  Wire.begin(); Wire.setClock(400000); delay(200);

  if (!imu.begin(BNO_ADDR, Wire)) { Serial.println("IMU fail"); while (1); }
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
  if (!imu.getSensorEvent()) return;

  static float ax, ay, az, roll, pitch, yaw, qI, qJ, qK, qW;
  static float gqI, gqJ, gqK, gqW = 1.0f, gx, gy, gz, mx, my, mz, rvAcc;
  static uint8_t rvStatus = 0, magStatus = 0;
  static uint8_t got = 0;

  switch (imu.getSensorEventID()) {
    case SENSOR_REPORTID_ROTATION_VECTOR:
      qI = imu.getQuatI(); qJ = imu.getQuatJ();
      qK = imu.getQuatK(); qW = imu.getQuatReal();
      roll  = imu.getRoll()  * 180.0f / PI;
      pitch = imu.getPitch() * 180.0f / PI;
      yaw   = imu.getYaw()   * 180.0f / PI;
      rvStatus = imu.getQuatAccuracy();
      rvAcc    = imu.getQuatRadianAccuracy();
      got |= 0b01; break;

    /* The two rotation vectors share one layout in the report, so the same getters read both. */
    case SENSOR_REPORTID_GAME_ROTATION_VECTOR:
      gqI = imu.getQuatI(); gqJ = imu.getQuatJ();
      gqK = imu.getQuatK(); gqW = imu.getQuatReal();
      break;

    case SENSOR_REPORTID_GYROSCOPE_CALIBRATED:
      gx = imu.getGyroX(); gy = imu.getGyroY(); gz = imu.getGyroZ();
      break;

    case SENSOR_REPORTID_MAGNETIC_FIELD:
      mx = imu.getMagX(); my = imu.getMagY(); mz = imu.getMagZ();
      magStatus = imu.getMagAccuracy();
      break;

    case SENSOR_REPORTID_ACCELEROMETER:
      ax = imu.getAccelX(); ay = imu.getAccelY(); az = imu.getAccelZ();
      got |= 0b10; break;
  }

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


