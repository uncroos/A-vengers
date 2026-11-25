/*
 * Finger Stone - MPU6050 마우스 커서 이동 (단독 테스트용, Flex 기능 없음)
 *
 * 배선: PCB J12 -> 3V3=VCC, SCL(22), SDA(21), GND
 * 환경: ESP32-DevKitC, arduino-esp32 3.3.x, HijelHID_BLEMouse(+NimBLE-Arduino)
 *
 * 동작
 *  - 손목 좌우 꺾기(gz)  -> 커서 좌우
 *  - 손목 위아래 꺾기(gy) -> 커서 상하   (기본값. 's'로 gx로 바꿀 수 있음)
 *  - 정지 상태를 감지해 자이로 영점을 자동으로 잡고, 정지 중에는 계속 보정(커서 흐름 방지)
 *  - I2C 오류가 나면 버스를 복구하고 영점은 유지
 *
 * 시리얼(115200, Newline) 명령
 *   e  커서 이동 켜기/끄기 (BOOT 버튼도 같은 기능)
 *   x  좌우 방향 반전      y  상하 방향 반전
 *   s  상하 축을 gy <-> gx 로 교체
 *   +  감도 올리기  -  감도 내리기
 *   a  가속 강하게(빠르게 흔들수록 더 멀리)  q  가속 약하게
 *   c  클러치(손가락 굽히면 커서 정지) 켜기/끄기
 *   z  영점 다시 잡기 (손을 가만히)
 *   d  디버그 출력 켜기/끄기     h  도움말
 */
#define USE_BLE_HID 1    // 0: 시리얼 드라이런 (PC에 안 붙이고 dx,dy 값만 확인)

#include <Wire.h>
#include <Preferences.h>
#if USE_BLE_HID
  #include <HijelHID_BLEMouse.h>
  HijelBLEMouse mouse("Finger Stone", "A+vengers", 100, 3, false);
#endif

// ===== 설정 =====
const int   PIN_SDA = 21, PIN_SCL = 22;
const uint32_t I2C_HZ = 100000;      // 점퍼선 배선이라 100kHz로 안정성 우선

const int   BTN_PIN = 0;             // BOOT 버튼: 커서 켜기/끄기

float gainDeg   = 8.0f;              // 감도: 1도 회전당 픽셀 (+/-로 조절)
float accelRef  = 90.0f;             // 가속 기준(dps). 작을수록 빠르게 흔들 때 더 멀리 감 (a/q 키)
float deadDps   = 6.0f;              // 데드존(dps): 이보다 느린 움직임은 무시
float smoothA   = 0.45f;             // 속도 저역필터(0~1, 클수록 반응 빠름)
int   signX     = -1;                // 기본: gz>0(손끝 왼쪽) -> 커서 왼쪽. 반대면 x 키
int   signY     = +1;                // 기본: gy>0(손끝 아래) -> 커서 아래. 반대면 y 키
bool  vertFromGx = false;            // false: 상하=gy, true: 상하=gx
bool  enabledMove = true;

// ===== 클러치: 이 손가락을 굽히고 있는 동안 커서 정지 (손목을 원위치로 돌릴 때 사용) =====
// 0=엄지 1=검지 2=중지 3=약지, -1=사용 안 함. flex4 스케치에서 저장한 보정값(fstone/cal)을 읽어 씀
int8_t CLUTCH_FINGER = 3;
const uint8_t FLEX_PINS[4] = {36, 39, 34, 35};
const float CLUTCH_ON = 0.55f, CLUTCH_OFF = 0.35f;
struct CalData { uint32_t magic; uint16_t flat[4]; uint16_t fist[4]; };
const uint32_t CAL_MAGIC = 0xF1A60004;
CalData cal;
bool calOk = false;
bool clutchOn = false;
float clutchMv = 0;
bool clutchInit = false;
bool  debugOn   = true;

const uint32_t LOOP_US   = 5000;     // 200Hz 읽기
const uint32_t SEND_MS   = 15;       // BLE 전송 주기
const int STILL_N        = 60;       // 정지 판정 창(60샘플=300ms)
const float STILL_RANGE  = 20.0f;    // 창 내 최대-최소가 이보다 작으면 정지 (공중에 든 손의 떨림 고려)
const float BIAS_TRACK   = 0.02f;    // 정지 중 영점 추종 속도

// ===== MPU6050 =====
const uint8_t ADDR_DEFAULT = 0x68;
uint8_t addr = ADDR_DEFAULT;
const uint8_t REG_PWR = 0x6B, REG_CFG = 0x1A, REG_GYRO_CFG = 0x1B, REG_GYRO = 0x43;
const float GYR_LSB = 65.5f;         // ±500dps

float bias[3] = {0, 0, 0};           // gx,gy,gz 영점
float v[3]    = {0, 0, 0};           // 필터된 각속도(영점 보정 후)
float win[STILL_N][3];
int   winIdx = 0, winCount = 0;
uint32_t i2cErrors = 0;
bool imuOk = false;

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

// 정지 창이 가득 찼고 모든 축의 흔들림이 작으면 true. mean에 창 평균 반환
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

void pushWindow(const float raw[3]) {
  for (int a = 0; a < 3; a++) win[winIdx][a] = raw[a];
  winIdx = (winIdx + 1) % STILL_N;
  if (winCount < STILL_N) winCount++;
}

// 손을 가만히 두면 영점을 즉시 잡는다 (최대 timeoutMs 대기)
bool calibrate(uint32_t timeoutMs) {
  Serial.println(F("[CAL] 영점: 손을 가만히 두세요..."));
  resetWindow();
  uint32_t t0 = millis();
  uint32_t nextUs = micros();
  while (millis() - t0 < timeoutMs) {
    float raw[3];
    if (readGyroRaw(raw)) {
      pushWindow(raw);
      float m[3];
      if (isStill(m)) {
        for (int a = 0; a < 3; a++) bias[a] = m[a];
        v[0] = v[1] = v[2] = 0;
        Serial.printf("[CAL] 완료 gx=%.2f gy=%.2f gz=%.2f dps\n", bias[0], bias[1], bias[2]);
        return true;
      }
    }
    nextUs += LOOP_US;
    while ((int32_t)(micros() - nextUs) < 0) { }
  }
  Serial.println(F("[CAL] 시간 초과(계속 움직임). 가만히 있으면 자동 보정됩니다."));
  return false;
}

bool loadFlexCal() {
  Preferences p;
  p.begin("fstone", true);   // 읽기 전용
  bool ok = false;
  if (p.getBytesLength("cal") == sizeof(CalData)) {
    p.getBytes("cal", &cal, sizeof(cal));
    ok = (cal.magic == CAL_MAGIC);
  }
  p.end();
  return ok;
}

// 클러치 손가락의 굽힘(0~1)을 읽어 히스테리시스 적용
void updateClutch() {
  if (CLUTCH_FINGER < 0 || !calOk) { clutchOn = false; return; }
  uint32_t sum = 0;
  for (int k = 0; k < 8; k++) sum += analogReadMilliVolts(FLEX_PINS[CLUTCH_FINGER]);
  float mv = sum / 8.0f;
  clutchMv = clutchInit ? (0.3f * mv + 0.7f * clutchMv) : mv;
  clutchInit = true;
  float range = (float)cal.fist[CLUTCH_FINGER] - (float)cal.flat[CLUTCH_FINGER];
  if (fabsf(range) < 50.0f) { clutchOn = false; return; }   // 보정이 부실하면 사용 안 함
  float b = (clutchMv - (float)cal.flat[CLUTCH_FINGER]) / range;
  bool prev = clutchOn;
  if (b > CLUTCH_ON) clutchOn = true;
  else if (b < CLUTCH_OFF) clutchOn = false;
  if (clutchOn != prev) Serial.println(clutchOn ? F("[CLUTCH] 정지 (손목을 돌려놓으세요)") : F("[CLUTCH] 해제"));
}

void help() {
  Serial.println(F("명령: e 커서on/off | x 좌우반전 | y 상하반전 | s 상하축(gy/gx) | +/- 감도 | a/q 가속 | c 클러치 | z 영점 | d 디버그 | h"));
}

void printSettings() {
  Serial.printf("[SET] 이동=%s 감도=%.1f 가속기준=%.0f 좌우부호=%+d 상하부호=%+d 상하축=%s 클러치=%s\n",
                enabledMove ? "ON" : "OFF", gainDeg, accelRef, signX, signY, vertFromGx ? "gx" : "gy",
                (CLUTCH_FINGER >= 0 && calOk) ? "ON" : "OFF");
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n== Finger Stone MPU6050 마우스 =="));
  pinMode(BTN_PIN, INPUT_PULLUP);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);
  calOk = loadFlexCal();
  if (CLUTCH_FINGER >= 0) {
    if (calOk) Serial.printf("[CLUTCH] 보정값 로드 완료: %s 굽히면 커서 정지\n",
                             CLUTCH_FINGER == 0 ? "엄지" : CLUTCH_FINGER == 1 ? "검지" : CLUTCH_FINGER == 2 ? "중지" : "약지");
    else Serial.println(F("[CLUTCH] Flex 보정값 없음 -> 클러치 꺼짐 (finger_stone_flex4를 한 번 올려 보정하세요)"));
  }

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(20);

  imuOk = configImu();
  Serial.println(imuOk ? F("[MPU] 초기화 완료") : F("[MPU] 초기화 실패: 배선 확인 (SDA21/SCL22)"));
  if (imuOk) calibrate(8000);

#if USE_BLE_HID
  mouse.begin();
  Serial.println(F("[BLE] 마우스 광고 시작: PC/노트북에서 'Finger Stone' 페어링"));
#else
  Serial.println(F("[BLE] 꺼짐 (USE_BLE_HID=0, dx/dy 값만 출력)"));
#endif
  help();
  printSettings();
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'e': enabledMove = !enabledMove; printSettings(); break;
      case 'x': signX = -signX; printSettings(); break;
      case 'y': signY = -signY; printSettings(); break;
      case 's': vertFromGx = !vertFromGx; printSettings(); break;
      case '+': gainDeg *= 1.25f; printSettings(); break;
      case '-': gainDeg /= 1.25f; printSettings(); break;
      case 'a': accelRef /= 1.25f; if (accelRef < 20) accelRef = 20; printSettings(); break;
      case 'q': accelRef *= 1.25f; if (accelRef > 1000) accelRef = 1000; printSettings(); break;
      case 'c': CLUTCH_FINGER = (CLUTCH_FINGER >= 0) ? -1 : 3; clutchOn = false; printSettings(); break;
      case 'z': calibrate(8000); break;
      case 'd': debugOn = !debugOn; Serial.printf("[OUT] 디버그 %s\n", debugOn ? "ON" : "OFF"); break;
      case 'h': help(); break;
      default: break;
    }
  }
}

void loop() {
  static uint32_t nextUs = micros();
  static uint32_t lastUs = micros();
  static uint32_t lastSend = 0, lastDbg = 0, lastBtn = 0, lastRecover = 0;
  static float accX = 0, accY = 0;     // 소수점 누적 (픽셀)
  static uint8_t flexDiv = 0;

  // 고정 주기 대기 (그 사이 시리얼/버튼 처리)
  while ((int32_t)(micros() - nextUs) < 0) { }
  nextUs += LOOP_US;
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.1f) dt = LOOP_US * 1e-6f;

  handleSerial();
  if (digitalRead(BTN_PIN) == LOW && millis() - lastBtn > 400) {
    lastBtn = millis();
    enabledMove = !enabledMove;
    printSettings();
  }

  // ---- 센서 읽기 (+ 오류 복구) ----
  float raw[3];
  if (!readGyroRaw(raw)) {
    i2cErrors++;
    v[0] = v[1] = v[2] = 0;
    accX = accY = 0;
    if (millis() - lastRecover > 200) {
      lastRecover = millis();
      Serial.printf("[ERR] I2C 읽기 실패(누적 %lu) -> 버스 복구 시도\n", (unsigned long)i2cErrors);
      recoverBus();
      if (configImu()) Serial.println(F("[MPU] 복구 성공 (영점 유지)"));
      resetWindow();
    }
    return;
  }

  // ---- 정지 감지 -> 영점 자동 추종 ----
  pushWindow(raw);
  float m[3];
  if (isStill(m)) {
    for (int a = 0; a < 3; a++) bias[a] += BIAS_TRACK * (m[a] - bias[a]);
  }

  // ---- 각속도 보정 + 필터 ----
  for (int a = 0; a < 3; a++) {
    float g = raw[a] - bias[a];
    v[a] += smoothA * (g - v[a]);
  }

  // ---- 데드존 + 가속 곡선 -> 픽셀 ----
  auto shape = [&](float w) {
    float aw = fabsf(w);
    if (aw <= deadDps) return 0.0f;
    float s = (aw - deadDps);
    s *= (1.0f + aw / accelRef);
    return w < 0 ? -s : s;
  };
  float vx = shape(v[2]) * signX;                       // 좌우: gz
  float vy = shape(vertFromGx ? v[0] : v[1]) * signY;   // 상하: gy (또는 gx)

  if ((++flexDiv & 3) == 0) {          // 50Hz로 클러치 손가락 읽기
    bool before = clutchOn;
    updateClutch();
    if (clutchOn && !before) { v[0] = v[1] = v[2] = 0; accX = accY = 0; }
  }

  if (enabledMove && !clutchOn) {
    accX += vx * gainDeg * dt;
    accY += vy * gainDeg * dt;
  } else {
    accX = accY = 0;
  }

  // ---- 주기적으로 정수 픽셀만 전송 ----
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
      if (debugOn && millis() - lastDbg > 200) {
        lastDbg = millis();
        Serial.printf("[MOVE] gz=%6.1f gy=%6.1f gx=%6.1f -> dx=%4d dy=%4d%s\n",
                      v[2], v[1], v[0], dx, dy,
#if USE_BLE_HID
                      mouse.isPaired() ? "" : "  (페어링 안 됨)"
#else
                      ""
#endif
        );
      }
    }
  }
}
