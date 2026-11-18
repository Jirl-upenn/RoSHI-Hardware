#include <WiFi.h>
#include <esp_now.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

/* ===== OLED setup ===== */
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET   -1
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

/* ===== IMU packet: 42 bytes from the original tracker firmware ===== */
typedef struct __attribute__((packed)) {
  uint8_t tracker_id;
  float   ax, ay, az;
  float   roll, pitch, yaw;
  float   qI, qJ, qK, qW;
  uint8_t battery_percent;
} IMUDataPacket;

/* ===== Diagnostic IMU packet: the same 42 bytes, then the extra state (see IMU_Tracker.ino) ===== */
typedef struct __attribute__((packed)) {
  IMUDataPacket base;
  float   gqI, gqJ, gqK, gqW;
  float   gx, gy, gz;
  float   mx, my, mz;
  uint8_t rv_status;
  uint8_t mag_status;
  float   rv_accuracy_rad;
  uint8_t reset_count;
} IMUDiagPacket;

/* ===== Button packet (4 bytes) ===== */
typedef struct __attribute__((packed)) { uint8_t id, batt, b14, b12; } ButtonPacket;

static inline bool looksLikeIMUPacket(int len){ return len == (int)sizeof(IMUDataPacket); }
static inline bool looksLikeDiagPacket(int len){ return len == (int)sizeof(IMUDiagPacket); }
static inline bool looksLikeButtonPacket(int len){ return len == (int)sizeof(ButtonPacket); }

/* ===== timing thresholds ===== */
static const uint32_t IMU_TIMEOUT_MS = 50;   // ~5 frames at 100Hz
static const uint32_t BTN_TIMEOUT_MS = 800;  // UX preference

/* ===== last-seen trackers (for screen) ===== */
static uint8_t lastBatt[10];
static uint32_t lastSeenMs[10];
static bool seen[10];

static uint8_t btn14 = 0, btn12 = 0;   // latest button raw (defaults 0)
static uint32_t lastBTNMs = 0;
static uint8_t lastPrintedBtnState = 255; // force first print

static inline uint8_t clamp2digits(uint8_t v){ return (v > 99) ? 99 : v; }

void drawGrid()
{
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);

  // 5 columns, 2 rows; each cell ~25px wide, rows at y=8 and y=40
  const int startX[5] = { 0, 26, 52, 78, 104 };
  const int rowY[2]   = { 8, 40 };

  display.setTextSize(2);

  for (int idx = 0; idx < 10; ++idx) {
    int row = (idx < 5) ? 0 : 1;
    int col = (idx < 5) ? idx : (idx - 5);
    int x = startX[col];
    int y = rowY[row];

    if (idx == 0) {
      // Button state 0..2; revert to 0 after timeout; both-pressed maps to BTN14 (1)
      uint8_t state = 0;
      uint32_t ageB = millis() - lastBTNMs;
      if (ageB < BTN_TIMEOUT_MS) {
        if (btn14)      state = 1; // BTN14 pressed has priority
        else if (btn12) state = 2; // BTN12 pressed
      }
      display.setTextSize(1); display.setCursor(x, y - 7); display.print("0");
      display.setTextSize(2); display.setCursor(x, y); display.printf("%u", state);
      continue;
    }

    if (seen[idx] && (millis() - lastSeenMs[idx]) < IMU_TIMEOUT_MS) {
      uint8_t val = clamp2digits(lastBatt[idx]);
      display.setTextSize(1); display.setCursor(x, y - 7); display.print(idx);
      display.setTextSize(2); display.setCursor(x, y); display.printf("%u", val);
    }
  }

  display.display();
}

void onReceive(const esp_now_recv_info *, const uint8_t *data, int len)
{
  if (looksLikeIMUPacket(len) || looksLikeDiagPacket(len)) {
    IMUDataPacket p; memcpy(&p, data, sizeof(p));
    if (looksLikeDiagPacket(len)) {
      /* The original line, then the extra fields: readers that match the old prefix still work.
         Quaternions get more digits here so that small heading changes are not lost to rounding. */
      IMUDiagPacket d; memcpy(&d, data, sizeof(d));
      Serial.printf(
        "ID:%d | r:%.1f p:%.1f y:%.1f | "
        "ax:%.2f ay:%.2f az:%.2f | "
        "qI:%.5f qJ:%.5f qK:%.5f qW:%.5f | "
        "batt:%u%% | "
        "gI:%.5f gJ:%.5f gK:%.5f gW:%.5f | "
        "gx:%.4f gy:%.4f gz:%.4f | "
        "mx:%.2f my:%.2f mz:%.2f | "
        "rvs:%u mags:%u rvacc:%.4f rst:%u\n",
        p.tracker_id,
        p.roll, p.pitch, p.yaw,
        p.ax, p.ay, p.az,
        p.qI,  p.qJ,  p.qK,  p.qW,
        p.battery_percent,
        d.gqI, d.gqJ, d.gqK, d.gqW,
        d.gx, d.gy, d.gz,
        d.mx, d.my, d.mz,
        d.rv_status, d.mag_status, d.rv_accuracy_rad, d.reset_count);
    } else
    Serial.printf(
      "ID:%d | r:%.1f p:%.1f y:%.1f | "
      "ax:%.2f ay:%.2f az:%.2f | "
      "qI:%.3f qJ:%.3f qK:%.3f qW:%.3f | "
      "batt:%u%%\n",
      p.tracker_id,
      p.roll, p.pitch, p.yaw,
      p.ax, p.ay, p.az,
      p.qI,  p.qJ,  p.qK,  p.qW,
      p.battery_percent);

    if (p.tracker_id < 10) {
      lastBatt[p.tracker_id] = p.battery_percent;
      lastSeenMs[p.tracker_id] = millis();
      seen[p.tracker_id] = true;
    }
    return;
  }

  if (looksLikeButtonPacket(len)) {
    ButtonPacket b; memcpy(&b, data, sizeof(b));
    // Update cached state/time
    btn14 = b.b14; btn12 = b.b12; lastBTNMs = millis();
    seen[0] = true; lastSeenMs[0] = lastBTNMs;
    // Mapped state (no state 3). Print all non-zero states; print 0 only once on timeout from loop.
    uint8_t state = 0;
    if (btn14)      state = 1;
    else if (btn12) state = 2;
    if (state != 0) {
      Serial.printf("BTN ID:%u | BTN14:%u BTN12:%u\n", b.id, b.b14, b.b12);
      lastPrintedBtnState = state;
    }
    return;
  }
}

void setup() {
  Serial.begin(115200); delay(300);

  memset(seen, 0, sizeof(seen));
  memset(lastBatt, 0, sizeof(lastBatt));
  memset(lastSeenMs, 0, sizeof(lastSeenMs));
  btn14 = 0; btn12 = 0; lastBTNMs = 0; lastPrintedBtnState = 255;

  Wire.begin();
  Wire.setClock(400000); // fast I2C for quicker refresh
  if (display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    display.clearDisplay();
    display.setTextSize(2);
    display.setTextColor(SSD1306_WHITE);
    display.setCursor(0, 0);
    display.println(F("Host"));
    display.display();
  }

  WiFi.mode(WIFI_STA); WiFi.begin();
  if (esp_now_init() != ESP_OK) { while (1); }
  esp_now_register_recv_cb(onReceive);
}

void loop() {
  static uint32_t lastDraw = 0;
  uint32_t now = millis();

  // If button state has timed out to 0 and we haven't printed 0 yet, print it once
  uint32_t ageB = now - lastBTNMs;
  if (ageB >= BTN_TIMEOUT_MS && lastPrintedBtnState != 0 && seen[0]) {
    Serial.printf("BTN ID:%u | BTN14:%u BTN12:%u\n", 0, 0, 0);
    lastPrintedBtnState = 0;
  }

  if (now - lastDraw >= 10) { // refresh ~100Hz
    lastDraw = now;
    drawGrid();
  }
}
