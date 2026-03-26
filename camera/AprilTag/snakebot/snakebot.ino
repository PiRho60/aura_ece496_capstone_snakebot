#include <Arduino.h>
#include <ESP32Servo.h>

#include <Wire.h>
#include <Adafruit_BNO08x.h>
#include <sh2.h>

#include <WiFi.h>
#include <WiFiUdp.h>
#include "esp_wifi.h"
#include "esp_wpa2.h"
#include <string.h>
#include <stdlib.h>

/*********************************************
* WIFI / UDPx
**********************************************/
const char* SSID = "UofT";
const char* EAP_IDENTITY = "quwenhao";
const char* EAP_USERNAME = "quwenhao";
const char* EAP_PASSWORD = "Maplehong0102";

constexpr uint16_t UDP_PORT = 6657;
WiFiUDP Udp;
char udpBuf[256];

/*********************************************
* GLOBALS / MACROS
**********************************************/

static constexpr int PIN_BNO_CS = 27;
static constexpr int PIN_BNO_INT = 26;
static constexpr int PIN_BNO_RST = 25;
const float DECLINATION_DEG = -10.0f; // Toronto ≈ -10
constexpr int NUM_SERVOS = 8;
Adafruit_BNO08x bno(PIN_BNO_RST);
sh2_SensorValue_t imuVal;
int servoPins[NUM_SERVOS] = { 33, 32, 22, 21, 17, 16, 14, 13 };
Servo myServos[NUM_SERVOS];
constexpr float TURNING_DEGREE = 45.0f;
static bool g_offsetInit = false;
static float g_filtOffset = 0.0f;

// Returns true once per `period_ms` for each unique call-site.
#define DO_PERIODIC_MS(period_ms) \
  if ([](uint32_t _p)->bool { \
        static uint32_t _last = 0; \
        const uint32_t _now = millis(); \
        if ((uint32_t)(_now - _last) >= _p) { _last = _now; return true; } \
        return false; \
      }((uint32_t)(period_ms)))
#define PRINT_PERIOD_MS 2000

/*********************************************
* CLASS / STRUCT DEFINITIONS
**********************************************/

enum class RunState : uint8_t {
  Warmup,
  WaitForHeading,
  Idle,
  Running
};

struct HeadingState {
  float trueDeg = 0.0f;
  bool haveHeading = false;
  float filtDeg = 0.0f;
  bool filtInit = false;
};
static HeadingState g_heading;

struct SlitherState {
  float offset;
  int amplitude;
  float speed;
  float wavelengths;
  uint32_t lastUpdateMs;
  float phase;
};
const SlitherState RUN_SLITHER_STATE = { 0.0f, 30, 1.35f, 1.0f, 0, 0.0f };
const SlitherState STOP_SLITHER_STATE = { 0.0f, 0, 0.0f, 1.0f, 0, 0.0f };
SlitherState g_slither = { 0.0f, 30, 1.35f, 1.0f, 0, 0.0f };

/*********************************************
* COMMAND STATE
**********************************************/
static RunState g_runState = RunState::Warmup;
static float g_targetHeading = 0.0f;

/*********************************************
* FUNCTION PROTOTYPES
**********************************************/
static inline float wrap360(float d);
static inline float wrap180(float d);
static float clampf(float x, float lo, float hi);

void setReports();
void initIMU();
void attachAllServos();
void centerAll();
void setSlitherParams(float offset, int Amplitude, float Speed, float Wavelengths);

float get_yaw_game();
void updateHeadingFromIMU();
float getFilteredErr(float targetDeg);
void moveTowardsHeading(float targetDeg);
void slitherStep();

void connectWiFi();
void initUDP();
void processUdpPackets();
bool getParamValue(const char* msg, const char* key, char* out, size_t outSize);
void handleCommand(const char* cmd, float directionDeg);
void stopMotion();

/*********************************************
* FUNCTION DEFINITIONS
**********************************************/

static inline float wrap360(float d) {
  while (d < 0.0f) d += 360.0f;
  while (d >= 360.0f) d -= 360.0f;
  return d;
}

static inline float wrap180(float d) {
  d = wrap360(d);
  if (d >= 180.0f) d -= 360.0f;
  return d;
}

static float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

void setReports() {
  if (!bno.enableReport(SH2_GAME_ROTATION_VECTOR, 20000)) {
    Serial.println("Could not enable rotation vector");
    while (1) delay(10);
  }
}

void initIMU() {
  Serial.println("[DEBUG] initIMU() start");
  Serial.println("Starting BNO08x...");

  if (!bno.begin_SPI(PIN_BNO_CS, PIN_BNO_INT)) {
    Serial.println("Failed to find BNO08x chip");
    while (1) delay(10);
  }

  Serial.println("BNO08x found!");
  setReports();
  delay(100);
}

void attachAllServos() {
  Serial.println("[DEBUG] attachAllServos() start");
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].attach(servoPins[i], 500, 2400);
  }
}

void centerAll() {
  Serial.println("[DEBUG] centerAll() start");
  for (int i = 0; i < NUM_SERVOS; i++) {
    myServos[i].write(90);
    delay(15);
  }
}

void setSlitherParams(float offset, int Amplitude, float Speed, float Wavelengths) {
  g_slither.offset = offset;
  g_slither.amplitude = Amplitude;
  g_slither.speed = Speed;
  g_slither.wavelengths = Wavelengths;
}

float get_yaw_game() {
  float r = imuVal.un.gameRotationVector.real;
  float i = imuVal.un.gameRotationVector.i;
  float j = imuVal.un.gameRotationVector.j;
  float k = imuVal.un.gameRotationVector.k;

  float ys = 2.0f * (r * k + i * j);
  float yc = 1.0f - 2.0f * (j * j + k * k);
  float yaw_game = wrap360(atan2f(ys, yc) * 180.0f / PI);

  return yaw_game;
}

void updateHeadingFromIMU() {
  if (bno.getSensorEvent(&imuVal) &&
      imuVal.sensorId == SH2_GAME_ROTATION_VECTOR) {

    g_heading.trueDeg = get_yaw_game();
    g_heading.haveHeading = true;

    DO_PERIODIC_MS(PRINT_PERIOD_MS) {
      Serial.print("GAME heading (relative): ");
      Serial.println(g_heading.trueDeg);
    }
  }
}

float getFilteredErr(float targetDeg) {
  constexpr float alpha = 0.04f;

  if (!g_heading.filtInit) {
    g_heading.filtDeg = g_heading.trueDeg;
    g_heading.filtInit = true;
  } else {
    const float diff = wrap180(g_heading.trueDeg - g_heading.filtDeg);
    g_heading.filtDeg = wrap360(g_heading.filtDeg + alpha * diff);
  }

  return wrap180(targetDeg - g_heading.filtDeg);
}

void moveTowardsHeading(float targetDeg) {
  DO_PERIODIC_MS(20) {
    if (!g_heading.haveHeading) return;

    float Kp = 0.7f;
    constexpr float maxOffset = 15.0f;
    constexpr float alphaOffset = 0.8f;

    float filtErr = getFilteredErr(targetDeg);
    float offset = clampf(Kp * filtErr, -maxOffset, maxOffset);

    if (!g_offsetInit) {
      g_filtOffset = offset;
      g_offsetInit = true;
    } else {
      g_filtOffset += alphaOffset * (offset - g_filtOffset);
    }

    setSlitherParams(g_filtOffset,
                    RUN_SLITHER_STATE.amplitude,
                    RUN_SLITHER_STATE.speed,
                    RUN_SLITHER_STATE.wavelengths);

    DO_PERIODIC_MS(PRINT_PERIOD_MS) {
      if (imuVal.status < 3) {
        Serial.printf("!!!WARNING: IMU status=%d!!!\n", imuVal.status);
      }
      Serial.print("Filtered error: ");
      Serial.print(filtErr);
      Serial.print(" | Filtered heading: ");
      Serial.print(g_heading.filtDeg);
      Serial.print(" | Target heading: ");
      Serial.print(targetDeg);
      Serial.print(" | Filt Offset: ");
      Serial.println(g_filtOffset);
    }
  }
}

void slitherStep() {
  DO_PERIODIC_MS(20) {
    const float Shift = 2.0f * PI / NUM_SERVOS;
    g_slither.phase += (float)g_slither.speed * 0.12f;

    for (int i = 0; i < NUM_SERVOS; i++) {
      float theta = 90.0f
                  + g_slither.offset
                  + g_slither.amplitude * sinf(g_slither.phase + i * g_slither.wavelengths * Shift);

      myServos[i].write((int)clampf(theta, 0, 180));
    }
  }
}

/*********************************************
* WIFI / UDP HELPERS
**********************************************/
void connectWiFi() {
  WiFi.disconnect(true);
  delay(500);
  WiFi.mode(WIFI_STA);

  WiFi.begin(SSID);

  esp_wifi_sta_wpa2_ent_set_identity((uint8_t*)EAP_IDENTITY, strlen(EAP_IDENTITY));
  esp_wifi_sta_wpa2_ent_set_username((uint8_t*)EAP_USERNAME, strlen(EAP_USERNAME));
  esp_wifi_sta_wpa2_ent_set_password((uint8_t*)EAP_PASSWORD, strlen(EAP_PASSWORD));
  esp_wifi_sta_wpa2_ent_enable();

  Serial.print("Connecting to ");
  Serial.println(SSID);

  unsigned long t0 = millis();
  while (WiFi.status() != WL_CONNECTED && millis() - t0 < 30000) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();

  if (WiFi.status() == WL_CONNECTED) {
    Serial.println("Connected!");
    Serial.print("IP: ");
    Serial.println(WiFi.localIP());
  } else {
    Serial.println("FAILED to connect.");
  }
}

void initUDP() {
  Udp.begin(UDP_PORT);
  Serial.print("UDP listening on port ");
  Serial.println(UDP_PORT);
}

bool getParamValue(const char* msg, const char* key, char* out, size_t outSize) {
  if (!msg || !key || !out || outSize == 0) return false;

  const char* p = strstr(msg, key);
  if (!p) return false;

  p += strlen(key);
  if (*p != '=') return false;
  p++;

  size_t n = 0;
  while (*p && *p != ',' && n + 1 < outSize) {
    out[n++] = *p++;
  }
  out[n] = '\0';
  return n > 0;
}

void stopMotion() {
  setSlitherParams(0.0f, 0, 0.0f, 0.0f);
  centerAll();

  g_offsetInit = false;
  g_filtOffset = 0.0f;

  g_runState = RunState::Idle;
  Serial.println("STOP command executed");
}

void handleCommand(const char* cmd, float directionDeg) {
  if (!cmd) return;

  if (strcmp(cmd, "STOP") == 0) {
    stopMotion();
    return;
  }

  if (strcmp(cmd, "GO") == 0) {
    if (!g_heading.haveHeading) {
      Serial.println("GO received but heading not available yet");
      return;
    }

    g_heading.filtDeg = g_heading.trueDeg;
    g_heading.filtInit = true;

    g_offsetInit = false;
    g_filtOffset = 0.0f;

    g_targetHeading = wrap360(g_heading.trueDeg + directionDeg);
    g_runState = RunState::Running;

    Serial.print("GO received, direction = ");
    Serial.print(directionDeg);
    Serial.print(" deg, current heading = ");
    Serial.print(g_heading.trueDeg);
    Serial.print(" deg, target heading = ");
    Serial.println(g_targetHeading);
    return;
  }

  if (strcmp(cmd, "NORTH") == 0) {
    if (!g_heading.haveHeading) return;
    g_targetHeading = 0.0f;
    g_runState = RunState::Running;
    Serial.println("NORTH received, target heading = 0.0");
    return;
  }

  if (strcmp(cmd, "SOUTH") == 0) {
    if (!g_heading.haveHeading) return;
    g_targetHeading = 180.0f;
    g_runState = RunState::Running;
    Serial.println("SOUTH received, target heading = 180.0");
    return;
  }

  if (strcmp(cmd, "EAST") == 0) {
    if (!g_heading.haveHeading) return;
    g_targetHeading = 270.0f; // wrap360(-90)
    g_runState = RunState::Running;
    Serial.println("EAST received, target heading = 270.0");
    return;
  }

  if (strcmp(cmd, "WEST") == 0) {
    if (!g_heading.haveHeading) return;
    g_targetHeading = 90.0f;
    g_runState = RunState::Running;
    Serial.println("WEST received, target heading = 90.0");
    return;
  }

  Serial.print("Unknown cmd: ");
  Serial.println(cmd);
}

void processUdpPackets() {
  int packetSize = Udp.parsePacket();
  if (packetSize <= 0) return;

  int len = Udp.read(udpBuf, sizeof(udpBuf) - 1);
  if (len <= 0) return;
  udpBuf[len] = '\0';

  Serial.print("UDP RX: ");
  Serial.println(udpBuf);

  char cmdBuf[32] = {0};
  char dirBuf[32] = {0};

  bool haveCmd = getParamValue(udpBuf, "cmd", cmdBuf, sizeof(cmdBuf));
  bool haveDir = getParamValue(udpBuf, "direction", dirBuf, sizeof(dirBuf));

  float directionDeg = 0.0f;
  if (haveDir) {
    directionDeg = atof(dirBuf);
  }

  if (haveCmd) {
    handleCommand(cmdBuf, directionDeg);
  } else {
    Serial.println("Packet missing cmd field");
  }
}

/*********************************************
* SETUP
**********************************************/
void setup() {
  Serial.println("[DEBUG] setup() start");
  Serial.begin(115200);
  delay(1500);

  initIMU();
  attachAllServos();
  centerAll();

  connectWiFi();
  initUDP();

  Serial.println("Setup complete");
}

/*********************************************
* LOOP
**********************************************/
void loop() {
  static const uint32_t bootMs = millis();

  if (bno.wasReset()) {
    Serial.println("!!! WARNING: BNO08x RESET detected !!! -> re-enabling reports");
    setReports();
    g_heading.haveHeading = false;
    g_heading.filtInit = false;
  }

  updateHeadingFromIMU();
  processUdpPackets();

  switch (g_runState) {
    case RunState::Warmup:
      setSlitherParams(0, 0, 0, 0);
      if (millis() - bootMs >= 5000) {
        g_runState = RunState::WaitForHeading;
      }
      break;

    case RunState::WaitForHeading:
      if (!g_heading.haveHeading) return;
      g_runState = RunState::Idle;
      Serial.println("Heading ready, entering IDLE");
      break;

    case RunState::Idle:
      break;

    case RunState::Running:
      moveTowardsHeading(g_targetHeading);
      slitherStep();
      break;
  }
}
