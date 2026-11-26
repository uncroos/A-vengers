/*
 * Finger Stone - 통합 스케치: Flex 제스처(클릭/잠금) + MPU6050 커서 이동, BLE 마우스 하나로 전송
 * 환경: ESP32-DevKitC, arduino-esp32 코어 3.3.x, HijelHID_BLEMouse (+ NimBLE-Arduino)
 * 배선: Flex 36/39/34(엄지/검지/중지)  ※약지 센서 고장으로 제외, MPU6050은 J12 (SDA21, SCL22)
 *
 * 제스처 (아래 GESTURES 표)
 *   주먹(엄지·검지·중지 모두 굽힘) 0.5초 유지     -> 잠금 켜기/끄기
 *   엄지만 굽힘                                      -> 좌클릭
 *   검지만 굽힘                                      -> 우클릭
 *   중지만 굽힘                                      -> 다음 슬라이드 (마우스 휠 아래)
 *   엄지+중지 굽힘                                   -> 이전 슬라이드 (마우스 휠 위)
 *   (슬라이드 쇼 화면에서 휠이 슬라이드를 넘기므로 키보드 없이 마우스만으로 동작)
 *   잠금 중에는 클릭과 커서 이동이 모두 멈춘다 (주먹으로만 해제).
 *   주먹을 쥐면 손이 완전히 펴질 때까지 다른 제스처를 차단 (쥐고 펴는 도중 오발동 방지)
 *
 * 커서 이동 (MPU6050, 손등 부착)
 *   손목 좌우 꺾기(gz) -> 좌우,  손목 위아래 꺾기(gy) -> 상하
 *   클러치: 손가락을 하나라도 굽히고 있는 동안 커서 정지. 손목을 원위치로 돌릴 때는
 *           동작이 배정되지 않은 자세(검지+중지 또는 엄지+검지)를 잡고 있으면 된다.
 *   클릭 자세(엄지/검지 굽힘) 동안에도 커서를 멈춰서 클릭 때 커서가 흔들리지 않게 함 (FREEZE_ON_CLICK_POSE)
 *   방향/감도/가속 설정은 시리얼 키로 바꾸면 자동 저장되어 다음 부팅에도 유지된다.
 *
 * 시리얼 명령 (115200, Newline)
 *   Flex : c=Flex 재캘리브레이션  d=플로터 출력  l=포즈 로그  p=캘리브레이션 값
 *   커서 : e=이동 on/off  x=좌우 반전  y=상하 반전  s=상하 축 gy<->gx  +/-=감도  a/q=가속 강/약
 *           on/off  z=자이로 영점  v=이동 로그  r=커서 설정 초기화  h=도움말
 * BOOT 버튼: 짧게 누름 = 잠금 토글(비상용), 2초 이상 = Flex 재캘리브레이션
 *
 * 처음 쓰는 순서: 이전 flex4 스케치로 Flex 보정을 한 적이 있으면 그 값을 그대로 사용한다(같은 저장소).
 *   없으면 부팅 때 자동으로 캘리브레이션(손 펴기 -> 주먹)이 시작된다.
 *   USE_BLE_HID=0 으로 올리면 PC 연결 없이 시리얼 로그만으로 확인 가능.
 */

#include <Arduino.h>
#include <Wire.h>
#include <Preferences.h>

// ============================ 사용자 설정 ============================

#define USE_BLE_HID 1   // 0: 시리얼 드라이런(액션/이동만 출력)  1: BLE 마우스 전송

#if USE_BLE_HID
  #include <HijelHID_BLEMouse.h>
  // 이름 / 제조사 / 배터리% / 버튼 수(좌,우,가운데) / 가로 스크롤 없음
  HijelBLEMouse mouse("Finger Stone", "A+vengers", 100, 3, false);
#endif

// ---- 핀 ----
const uint8_t NUM_FINGERS = 3;
const uint8_t FLEX_PINS[NUM_FINGERS] = {36, 39, 34};      // 엄지, 검지, 중지 (ADC1)
const char* const FINGER_NAMES[NUM_FINGERS] = {"thumb", "index", "middle"};
const uint8_t CAL_BUTTON_PIN = 0;    // BOOT 버튼
const int8_t  LOCK_LED_PIN   = -1;   // 잠금 표시 LED (-1 = 사용 안 함). 후보: GPIO25, 26
const int PIN_SDA = 21, PIN_SCL = 22;

// ---- Flex 샘플링/필터/캘리브레이션 ----
const uint32_t SAMPLE_INTERVAL_MS = 10;    // 100 Hz
const uint8_t  ADC_OVERSAMPLE     = 8;
const float    EMA_ALPHA          = 0.30f;
const uint32_t CAL_SETTLE_MS  = 2000;
const uint32_t CAL_CAPTURE_MS = 2000;
const uint16_t MIN_RANGE_MV   = 150;
const float BENT_ON[NUM_FINGERS]   = {0.60f, 0.55f, 0.45f};   // 중지는 덜 쥐어도 인식
const float BENT_OFF[NUM_FINGERS]  = {0.40f, 0.35f, 0.30f};
const float OPEN_CLEAR[NUM_FINGERS] = {0.40f, 0.30f, 0.30f};

// ---- 제스처 확정 ----
const uint32_t DEBOUNCE_MS   = 120;
const uint32_t REFRACTORY_MS = 400;
const bool     START_LOCKED  = false;

// ---- 커서 ----
const bool FREEZE_ON_CLICK_POSE = true;    // 엄지/검지가 굽혀진 동안 커서 정지 (클릭 안정화)
const uint32_t I2C_HZ   = 100000;          // 점퍼선 배선이라 100kHz
const uint32_t LOOP_US  = 5000;            // 200 Hz
const uint32_t SEND_MS  = 15;              // BLE 전송 주기
const int   STILL_N     = 60;              // 정지 판정 창 (60샘플 = 300ms)
const float STILL_RANGE = 20.0f;           // 창 내 최대-최소(dps)가 이보다 작으면 '정지'
const float BIAS_TRACK  = 0.02f;           // 정지 중 영점 추종 속도
const float DEAD_DPS    = 6.0f;            // 데드존
const float SMOOTH_A    = 0.45f;           // 각속도 저역필터

// 기본값 (시리얼 키로 바꾸면 저장됨, 'r'로 초기화)
const float DEF_GAIN = 8.0f, DEF_ACCEL_REF = 90.0f;
const int   DEF_SIGN_X = -1, DEF_SIGN_Y = +1;

// ============================ 제스처 정의 ============================

// 포즈 비트: 1 = 굽힘, 0 = 폄
const uint8_t B_THUMB  = 1 << 0;
const uint8_t B_INDEX  = 1 << 1;
const uint8_t B_MIDDLE = 1 << 2;
const uint8_t B_ALL    = 0x07;   // 엄지+검지+중지
const uint8_t B_FIST3  = B_ALL;  // 주먹 판정에 쓰는 손가락

enum Action : uint8_t { ACT_NONE, ACT_LEFT_CLICK, ACT_RIGHT_CLICK, ACT_LOCK_TOGGLE, ACT_NEXT, ACT_PREV };
const char* const ACTION_NAMES[] = {"NONE", "LEFT_CLICK", "RIGHT_CLICK", "LOCK_TOGGLE", "NEXT_SLIDE", "PREV_SLIDE"};

struct Gesture {
  const char* name;
  uint8_t  bent;      // 굽혀져 있어야 하는 손가락
  uint8_t  care;      // 비교할 손가락 (care 중 bent가 아닌 손가락은 "확실히 펴짐"이어야 함)
  Action   action;
  uint16_t holdMs;    // 포즈가 처음 잡힌 시점부터 유지해야 하는 시간
};

// 위에서부터 먼저 일치하는 항목이 선택됨
const Gesture GESTURES[] = {
  // 이름     굽힌 손가락  비교 손가락  액션             hold
  {"FIST",   B_ALL,      B_ALL,      ACT_LOCK_TOGGLE,  500},   // 3손가락 모두 꽉 쥐어야 함 (오발동 방지)
  {"THUMB",  B_THUMB,    B_ALL,      ACT_LEFT_CLICK,   250},
  {"INDEX",  B_INDEX,    B_ALL,      ACT_RIGHT_CLICK,  250},
  {"THUMB_MIDDLE", (uint8_t)(B_THUMB|B_MIDDLE), B_ALL, ACT_PREV, 250},   // 이전 슬라이드
  {"MIDDLE", B_MIDDLE,   B_ALL,      ACT_NEXT,         250},             // 다음 슬라이드
};
const uint8_t NUM_GESTURES = sizeof(GESTURES) / sizeof(GESTURES[0]);

// ============================ 상태 ============================

// 주의: Arduino가 함수 선언을 자동 생성하므로 구조체는 첫 함수보다 앞에 정의한다
struct CalData {
  uint32_t magic;
  uint16_t flat[NUM_FINGERS];   // 손 폈을 때 mV
  uint16_t fist[NUM_FINGERS];   // 주먹 쥐었을 때 mV
};
const uint32_t CAL_MAGIC = 0xF1A60003;   // 3손가락용 (이전 4손가락 보정값은 무시되고 새로 보정함)

struct MouseCfg {
  uint32_t magic;
  float    gain;
  float    accelRef;
  int8_t   signX, signY;
  uint8_t  vertGx;
  uint8_t  reserved;
};
const uint32_t CFG_MAGIC = 0xF1A6C003;

Preferences prefs;    // "fstone": Flex 보정
Preferences mprefs;   // "fsmouse": 커서 설정
CalData  cal;
uint8_t  validMask = B_ALL;
float    mvFilt[NUM_FINGERS];
bool     filtInit = false;
float    bend[NUM_FINGERS];
uint8_t  bentState = 0;
bool     locked = START_LOCKED;
bool     debugPrint = false;
bool     poseLog = true;
uint32_t lastSampleMs = 0, lastDebugMs = 0;

int8_t   candIdx = -1;  uint32_t candSince = 0;
int8_t   curIdx  = -1;  uint32_t curSince  = 0;
bool     fired = false; uint32_t lastFireMs = 0;
bool     everFired = false;
bool     fistLatch = false;

// 커서 설정/상태
float gainDeg   = DEF_GAIN;
float accelRef  = DEF_ACCEL_REF;
int   signX     = DEF_SIGN_X;
int   signY     = DEF_SIGN_Y;
bool  vertFromGx = false;
bool  enabledMove = true;
bool  moveDebug = false;
float accX = 0, accY = 0;     // 소수점 누적(픽셀)

// MPU6050
uint8_t addr = 0x68;
const uint8_t REG_PWR = 0x6B, REG_CFG = 0x1A, REG_GYRO_CFG = 0x1B, REG_GYRO = 0x43;
const float GYR_LSB = 65.5f;          // ±500dps
float bias[3] = {0, 0, 0};
float vel[3]  = {0, 0, 0};            // 필터된 각속도 gx,gy,gz (영점 보정 후)
float win[STILL_N][3];
int   winIdx = 0, winCount = 0;
uint32_t i2cErrors = 0, lastErrPrint = 0;

// ============================ Flex 센서 읽기 ============================

float readFingerMv(uint8_t i) {
  uint32_t sum = 0;
  for (uint8_t k = 0; k < ADC_OVERSAMPLE; k++) sum += analogReadMilliVolts(FLEX_PINS[i]);
  return (float)sum / ADC_OVERSAMPLE;
}

void sampleFingers() {
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    float mv = readFingerMv(i);
    mvFilt[i] = filtInit ? (EMA_ALPHA * mv + (1.0f - EMA_ALPHA) * mvFilt[i]) : mv;
  }
  filtInit = true;
}

void computeBend() {
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    if (!(validMask & (1 << i))) { bend[i] = 0; continue; }
    float range = (float)cal.fist[i] - (float)cal.flat[i];
    float b = (mvFilt[i] - (float)cal.flat[i]) / range;
    bend[i] = constrain(b, 0.0f, 1.0f);
  }
}

void updateBentState() {
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    if (bend[i] > BENT_ON[i])       bentState |=  (1 << i);
    else if (bend[i] < BENT_OFF[i]) bentState &= ~(1 << i);
  }
}

// ============================ Flex 캘리브레이션 ============================

void captureAverage(uint16_t out[], uint32_t durationMs) {
  float sum[NUM_FINGERS] = {0};
  uint32_t n = 0, t0 = millis();
  while (millis() - t0 < durationMs) {
    for (uint8_t i = 0; i < NUM_FINGERS; i++) sum[i] += readFingerMv(i);
    n++;
    delay(SAMPLE_INTERVAL_MS);
  }
  for (uint8_t i = 0; i < NUM_FINGERS; i++) out[i] = n ? (uint16_t)(sum[i] / n) : 0;
}

void countdown(uint32_t ms) {
  for (uint32_t s = ms / 1000; s > 0; s--) { Serial.printf("   %u...\n", (unsigned)s); delay(1000); }
}

void applyCalibration() {
  validMask = 0;
  Serial.println("[CAL] 결과 (mV)   손가락  flat  fist  range");
  for (uint8_t i = 0; i < NUM_FINGERS; i++) {
    int range = (int)cal.fist[i] - (int)cal.flat[i];
    bool ok = abs(range) >= MIN_RANGE_MV;
    if (ok) validMask |= (1 << i);
    Serial.printf("      %-7s %5u %5u %5d  %s\n", FINGER_NAMES[i], cal.flat[i], cal.fist[i], range,
                  ok ? "OK" : "<< 변화 부족: 배선/센서/굽힘 확인 (이 손가락은 무시됨)");
  }
}

void saveCalibration() {
  cal.magic = CAL_MAGIC;
  prefs.putBytes("cal", &cal, sizeof(cal));
}

bool loadCalibration() {
  if (prefs.getBytesLength("cal") != sizeof(CalData)) return false;
  prefs.getBytes("cal", &cal, sizeof(cal));
  return cal.magic == CAL_MAGIC;
}

void runCalibration() {
  Serial.println("[CAL] 1/2 손을 쫙 펴고 유지하세요 (OPEN)");
  countdown(CAL_SETTLE_MS);
  captureAverage(cal.flat, CAL_CAPTURE_MS);
  Serial.println("[CAL] 2/2 주먹을 꽉 쥐고 유지하세요 (FIST)");
  countdown(CAL_SETTLE_MS);
  captureAverage(cal.fist, CAL_CAPTURE_MS);
  applyCalibration();
  saveCalibration();
  filtInit = false;
  bentState = 0;
  fistLatch = false;
  curIdx = candIdx = -1;
  Serial.println("[CAL] 저장 완료");
}

// ============================ 커서 설정 저장 ============================

void saveCfg() {
  MouseCfg c;
  c.magic = CFG_MAGIC;
  c.gain = gainDeg;
  c.accelRef = accelRef;
  c.signX = (int8_t)signX;
  c.signY = (int8_t)signY;
  c.vertGx = vertFromGx ? 1 : 0;
  c.reserved = 0;
  mprefs.putBytes("cfg", &c, sizeof(c));
}

bool loadCfg() {
  if (mprefs.getBytesLength("cfg") != sizeof(MouseCfg)) return false;
  MouseCfg c;
  mprefs.getBytes("cfg", &c, sizeof(c));
  if (c.magic != CFG_MAGIC) return false;
  if (!(c.gain > 0.1f && c.gain < 200.0f) || !(c.accelRef >= 20.0f && c.accelRef <= 1000.0f)) return false;
  gainDeg = c.gain;
  accelRef = c.accelRef;
  signX = c.signX >= 0 ? 1 : -1;
  signY = c.signY >= 0 ? 1 : -1;
  vertFromGx = c.vertGx != 0;
  return true;
}

void printSettings() {
  Serial.printf("[SET] 이동=%s 감도=%.1f 가속기준=%.0f 좌우부호=%+d 상하부호=%+d 상하축=%s\n",
                enabledMove ? "ON" : "OFF", gainDeg, accelRef, signX, signY, vertFromGx ? "gx" : "gy");
}

// ============================ 출력 (BLE / 시리얼) ============================

void setLocked(bool v) {
  locked = v;
  accX = accY = 0;
  if (LOCK_LED_PIN >= 0) digitalWrite(LOCK_LED_PIN, locked ? HIGH : LOW);
  Serial.printf("[LOCK] %s\n", locked ? "잠금 (클릭/커서 정지, 주먹으로 해제)" : "해제");
}

void sendAction(Action a) {
  Serial.printf("[ACTION] %s\n", ACTION_NAMES[a]);
#if USE_BLE_HID
  if (!mouse.isPaired()) { Serial.println("[BLE] 페어링 안 됨"); return; }
  switch (a) {
    case ACT_LEFT_CLICK:  mouse.click(MouseButton::Left);  break;
    case ACT_RIGHT_CLICK: mouse.click(MouseButton::Right); break;
    case ACT_NEXT:        mouse.scroll(-1); break;   // 휠 아래 = 다음 슬라이드 (반대면 +1/-1 맞바꾸기)
    case ACT_PREV:        mouse.scroll(1);  break;   // 휠 위 = 이전 슬라이드
    default: break;
  }
#endif
}

void fire(const Gesture& g) {
  if (g.action == ACT_LOCK_TOGGLE) { setLocked(!locked); return; }
  if (locked) { Serial.printf("[LOCK] 잠금 중이라 무시: %s\n", g.name); return; }
  sendAction(g.action);
}

// ============================ 제스처 엔진 ============================

int8_t matchGesture(uint8_t pose) {
  for (uint8_t g = 0; g < NUM_GESTURES; g++) {
    uint8_t care = GESTURES[g].care & validMask;
    if (care == 0) continue;
    if ((pose & care) != (GESTURES[g].bent & care)) continue;
    bool clearlyOpen = true;
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
      uint8_t bit = 1 << i;
      if ((care & bit) && !(GESTURES[g].bent & bit) && bend[i] > OPEN_CLEAR[i]) clearlyOpen = false;
    }
    if (clearlyOpen) return g;
  }
  return -1;
}

void runGestureEngine(uint32_t now) {
  const uint8_t FIST_MASK = B_FIST3 & validMask;
  if (FIST_MASK) {
    // 래치 해제는 중지/약지만 본다 (검지는 클릭용이라 계속 굽혀 있거나 센서가 고정돼도 래치가 안 풀리는 문제 방지)
    const uint8_t RELEASE_MASK = B_MIDDLE & validMask;
    if ((bentState & FIST_MASK) == FIST_MASK) fistLatch = true;
    else if ((bentState & RELEASE_MASK) == 0) fistLatch = false;
  }
  int8_t m = matchGesture(bentState);
  if (fistLatch && m >= 0 && GESTURES[m].action != ACT_LOCK_TOGGLE) m = -1;

  if (m != candIdx) { candIdx = m; candSince = now; }
  if (now - candSince >= DEBOUNCE_MS && candIdx != curIdx) {
    curIdx = candIdx;
    curSince = candSince;
    fired = false;
  }
  if (curIdx < 0 || fired) return;
  if (candIdx != curIdx) return;

  const Gesture& g = GESTURES[curIdx];
  if (now - curSince < g.holdMs) return;
  if (everFired && now - lastFireMs < REFRACTORY_MS) return;

  fire(g);
  fired = true;
  everFired = true;
  lastFireMs = now;
}

// ============================ MPU6050 ============================

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readGyroRaw(float out[3]) {
  Wire.beginTransmission(addr);
  Wire.write(REG_GYRO);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, 6) != 6) return false;
  uint8_t b[6];
  for (int i = 0; i < 6; i++) b[i] = Wire.read();
  for (int i = 0; i < 3; i++) out[i] = (int16_t)((b[2 * i] << 8) | b[2 * i + 1]) / GYR_LSB;
  return true;
}

bool configImu() {
  bool ok = writeReg(REG_PWR, 0x01);   // sleep 해제, 자이로 PLL 클럭
  delay(20);
  ok &= writeReg(REG_CFG, 0x03);       // 저역필터 ~44Hz
  ok &= writeReg(REG_GYRO_CFG, 0x08);  // ±500dps
  return ok;
}

// SDA가 눌려 있는 상태를 풀기 위해 SCL을 9번 토글한 뒤 버스 재시작
void recoverBus() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT);
  for (int i = 0; i < 9; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(10);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(10);
  }
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(20);
}

void resetWindow() { winIdx = 0; winCount = 0; }

void pushWindow(const float raw[3]) {
  for (int a = 0; a < 3; a++) win[winIdx][a] = raw[a];
  winIdx = (winIdx + 1) % STILL_N;
  if (winCount < STILL_N) winCount++;
}

// 창이 가득 차 있고 모든 축의 흔들림이 작으면 true (mean = 창 평균)
bool isStill(float mean[3]) {
  if (winCount < STILL_N) return false;
  for (int a = 0; a < 3; a++) {
    float mn = 1e9f, mx = -1e9f, s = 0;
    for (int i = 0; i < STILL_N; i++) {
      float x = win[i][a];
      if (x < mn) mn = x;
      if (x > mx) mx = x;
      s += x;
    }
    if (mx - mn > STILL_RANGE) return false;
    mean[a] = s / STILL_N;
  }
  return true;
}

// 손이 가만히 있을 때 영점을 잡는다 (최대 timeoutMs 대기)
bool calibrateGyro(uint32_t timeoutMs) {
  Serial.println("[GYRO] 영점: 손을 가만히 두세요...");
  resetWindow();
  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    float raw[3];
    if (readGyroRaw(raw)) {
      pushWindow(raw);
      float m[3];
      if (isStill(m)) {
        for (int a = 0; a < 3; a++) bias[a] = m[a];
        vel[0] = vel[1] = vel[2] = 0;
        Serial.printf("[GYRO] 완료 gx=%.2f gy=%.2f gz=%.2f dps\n", bias[0], bias[1], bias[2]);
        return true;
      }
    }
    delay(5);
  }
  Serial.println("[GYRO] 시간 초과(계속 움직임). 가만히 있으면 자동 보정됩니다.");
  return false;
}

// 데드존 + 속도 가속 곡선
float shape(float w) {
  float aw = fabsf(w);
  if (aw <= DEAD_DPS) return 0.0f;
  float s = (aw - DEAD_DPS) * (1.0f + aw / accelRef);
  return w < 0 ? -s : s;
}

// 5ms마다 호출: 자이로를 읽고 커서 이동량을 누적/전송
void stepMouse(float dt) {
  static uint32_t lastSend = 0, lastDbg = 0, lastRecover = 0;

  float raw[3];
  if (!readGyroRaw(raw)) {
    i2cErrors++;
    vel[0] = vel[1] = vel[2] = 0;
    accX = accY = 0;
    if (millis() - lastRecover > 200) {
      lastRecover = millis();
      bool print = (millis() - lastErrPrint > 5000);
      if (print) { lastErrPrint = millis(); Serial.printf("[ERR] I2C 읽기 실패(누적 %lu) -> 버스 복구 시도\n", (unsigned long)i2cErrors); }
      recoverBus();
      if (configImu()) { Serial.println("[MPU] 복구 성공 (영점 유지)"); lastErrPrint = 0; }
      resetWindow();
    }
    return;
  }

  // 정지 감지 -> 영점 자동 추종
  pushWindow(raw);
  float m[3];
  if (isStill(m)) {
    for (int a = 0; a < 3; a++) bias[a] += BIAS_TRACK * (m[a] - bias[a]);
  }

  // 각속도 보정 + 필터
  for (int a = 0; a < 3; a++) vel[a] += SMOOTH_A * ((raw[a] - bias[a]) - vel[a]);

  // 클러치 / 클릭 자세 / 잠금 -> 커서 정지
  bool clickPose = FREEZE_ON_CLICK_POSE && ((bentState & (B_THUMB | B_INDEX | B_MIDDLE) & validMask) != 0);
  bool frozen = locked || !enabledMove || clickPose;

  if (frozen) {
    accX = accY = 0;
  } else {
    float vx = shape(vel[2]) * signX;                          // 좌우: gz
    float vy = shape(vertFromGx ? vel[0] : vel[1]) * signY;    // 상하: gy (또는 gx)
    accX += vx * gainDeg * dt;
    accY += vy * gainDeg * dt;
  }

  // 주기적으로 정수 픽셀만 전송
  if (millis() - lastSend >= SEND_MS) {
    lastSend = millis();
    int dx = (int)accX, dy = (int)accY;
    if (dx > 127) dx = 127;
    if (dx < -127) dx = -127;
    if (dy > 127) dy = 127;
    if (dy < -127) dy = -127;
    accX -= dx; accY -= dy;
    if (dx != 0 || dy != 0) {
#if USE_BLE_HID
      if (mouse.isPaired()) mouse.move(dx, dy);
#endif
      if (moveDebug && millis() - lastDbg > 200) {
        lastDbg = millis();
        Serial.printf("[MOVE] gz=%6.1f gy=%6.1f gx=%6.1f -> dx=%4d dy=%4d\n", vel[2], vel[1], vel[0], dx, dy);
      }
    }
  }
}

// ============================ 디버그/입력 ============================

void printDebug(uint32_t now) {
  if (!debugPrint || now - lastDebugMs < 50) return;   // 20 Hz
  lastDebugMs = now;
  for (uint8_t i = 0; i < NUM_FINGERS; i++) Serial.printf("%s:%.2f,", FINGER_NAMES[i], bend[i]);
  Serial.printf("gesture:%d,locked:%d\n", curIdx + 1, locked ? 1 : 0);
}

void logPoseChange() {
  static uint8_t lastPose = 0xFF;
  if (!poseLog || bentState == lastPose) return;
  lastPose = bentState;
  Serial.printf("[POSE] ");
  for (uint8_t i = 0; i < NUM_FINGERS; i++)
    Serial.printf("%s=%d(%.2f)%s ", FINGER_NAMES[i], (bentState >> i) & 1, bend[i], (validMask & (1 << i)) ? "" : "[무시됨]");
  int8_t m = matchGesture(bentState);
  if (fistLatch && (m < 0 || GESTURES[m].action != ACT_LOCK_TOGGLE)) Serial.print("-> 주먹: 손이 펴질 때까지 다른 제스처 차단");
  else Serial.printf("-> %s", m < 0 ? "일치하는 제스처 없음" : GESTURES[m].name);
  Serial.printf("%s\n", locked ? "  [잠금 중]" : "");
}

void printHelp() {
  Serial.println("Flex: c=재캘리브레이션 d=플로터 l=포즈로그 p=보정값 | 커서: e=on/off x=좌우반전 y=상하반전 s=상하축 +/-=감도 a/q=가속  z=영점 v=이동로그 r=초기화 | h=도움말");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'c': runCalibration(); break;
      case 'd': debugPrint = !debugPrint; Serial.printf("[DBG] %s\n", debugPrint ? "ON" : "OFF"); break;
      case 'l': poseLog = !poseLog; Serial.printf("[LOG] 포즈 로그 %s\n", poseLog ? "ON" : "OFF"); break;
      case 'p': applyCalibration(); break;
      case 'e': enabledMove = !enabledMove; printSettings(); break;
      case 'x': signX = -signX; saveCfg(); printSettings(); break;
      case 'y': signY = -signY; saveCfg(); printSettings(); break;
      case 's': vertFromGx = !vertFromGx; saveCfg(); printSettings(); break;
      case '+': gainDeg *= 1.25f; if (gainDeg > 150) gainDeg = 150; saveCfg(); printSettings(); break;
      case '-': gainDeg /= 1.25f; if (gainDeg < 0.5f) gainDeg = 0.5f; saveCfg(); printSettings(); break;
      case 'a': accelRef /= 1.25f; if (accelRef < 20) accelRef = 20; saveCfg(); printSettings(); break;
      case 'q': accelRef *= 1.25f; if (accelRef > 1000) accelRef = 1000; saveCfg(); printSettings(); break;
      case 'z': calibrateGyro(8000); break;
      case 'v': moveDebug = !moveDebug; Serial.printf("[OUT] 이동 로그 %s\n", moveDebug ? "ON" : "OFF"); break;
      case 'r':
        gainDeg = DEF_GAIN; accelRef = DEF_ACCEL_REF; signX = DEF_SIGN_X; signY = DEF_SIGN_Y;
        vertFromGx = false; saveCfg();
        Serial.println("[SET] 커서 설정을 기본값으로 되돌림");
        printSettings();
        break;
      case 'h': printHelp(); break;
      default: break;
    }
  }
}

void handleButton() {
  static uint32_t pressedSince = 0;
  static bool calDone = false;
  if (digitalRead(CAL_BUTTON_PIN) == LOW) {
    if (!pressedSince) { pressedSince = millis(); calDone = false; }
    else if (!calDone && millis() - pressedSince >= 2000) {
      calDone = true;
      runCalibration();
      while (digitalRead(CAL_BUTTON_PIN) == LOW) delay(10);
    }
  } else {
    // 짧게(0.05~1초) 눌렀다 떼면 잠금 토글 (제스처가 안 먹을 때 비상용)
    if (pressedSince && !calDone) {
      uint32_t d = millis() - pressedSince;
      if (d >= 50 && d < 1000) setLocked(!locked);
    }
    pressedSince = 0;
  }
}

// ============================ setup / loop ============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n== Finger Stone 통합 (Flex 제스처 + MPU6050 커서) ==");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);
  if (LOCK_LED_PIN >= 0) { pinMode(LOCK_LED_PIN, OUTPUT); digitalWrite(LOCK_LED_PIN, locked ? HIGH : LOW); }

#if USE_BLE_HID
  mouse.begin();
  Serial.println("[BLE] 마우스 광고 시작: PC에서 'Finger Stone' 페어링 (이전에 키보드로 등록된 것이 있으면 삭제 후 다시)");
#else
  Serial.println("[BLE] 꺼짐 (USE_BLE_HID=0, 시리얼 드라이런)");
#endif

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(20);
  bool imuOk = configImu();
  Serial.println(imuOk ? "[MPU] 초기화 완료" : "[MPU] 초기화 실패: 배선 확인 (SDA21/SCL22). 계속 재시도합니다");

  mprefs.begin("fsmouse", false);
  if (loadCfg()) Serial.println("[SET] 저장된 커서 설정 사용");
  else Serial.println("[SET] 커서 설정: 기본값");
  printSettings();

  prefs.begin("fstone", false);
  if (loadCalibration()) {
    Serial.println("[CAL] 저장된 Flex 캘리브레이션 사용 (재측정: 시리얼 'c' 또는 BOOT 2초)");
    applyCalibration();
  } else {
    Serial.println("[CAL] 저장된 값이 없어 Flex 캘리브레이션을 시작합니다");
    runCalibration();
  }

  if (imuOk) calibrateGyro(8000);    // Flex 보정 이후, 손을 가만히 둔 상태에서
  if (locked) Serial.println("[LOCK] 잠금 상태로 시작");
  printHelp();
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastUs = micros();

  handleSerial();
  handleButton();

  uint32_t nowUs = micros();
  if ((int32_t)(nowUs - nextUs) < 0) return;          // 5ms 주기 대기 (블로킹 없음)
  nextUs += LOOP_US;
  if ((int32_t)(nowUs - nextUs) > 50000) nextUs = nowUs + LOOP_US;   // 오래 밀렸으면 재동기
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = LOOP_US * 1e-6f;

  uint32_t now = millis();
  if (now - lastSampleMs >= SAMPLE_INTERVAL_MS) {     // Flex는 100Hz
    lastSampleMs = now;
    sampleFingers();
    computeBend();
    updateBentState();
    runGestureEngine(now);
    logPoseChange();
    printDebug(now);
  }

  stepMouse(dt);                                       // MPU는 200Hz
}
