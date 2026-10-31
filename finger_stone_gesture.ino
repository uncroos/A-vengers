/*
 * Finger Stone - Flex 5채널 제스처 컨트롤러 (뼈대 코드)
 * 대상: ESP32-DevKitC (Arduino IDE, arduino-esp32 코어)
 *
 * 흐름:
 *   ADC 읽기(오버샘플) -> EMA 필터 -> 캘리브레이션 기준 0~1 굽힘 정규화
 *   -> 손가락별 히스테리시스(굽힘/폄) -> 5비트 포즈
 *   -> 제스처 테이블 매칭 -> 디바운스/유지시간 -> 액션 -> BLE HID (또는 시리얼 출력)
 *
 * 처음 쓰는 순서:
 *   1) USE_BLE_HID = 0 으로 업로드 (라이브러리 없이 로직만 확인)
 *   2) 시리얼 모니터 115200. 첫 부팅 시 캘리브레이션이 자동 진행됨
 *      (안내에 따라 손 펴기 -> 주먹)
 *   3) 시리얼 플로터에서 손가락을 하나씩 굽혀 "해당 채널만" 변하는지 확인
 *   4) 시리얼 로그의 [ACTION]이 의도대로 나오면 USE_BLE_HID = 1 로 변경
 *
 * 시리얼 명령: c=재캘리브레이션, d=디버그 출력 켜기/끄기, p=캘리브레이션 값 출력, h=도움말
 * 하드웨어 버튼: BOOT 버튼(GPIO0)을 2초 이상 누르면 재캘리브레이션
 *   (GPIO0은 스트랩 핀이므로 부팅이 끝난 뒤 입력으로만 읽음. 외부 회로를 연결하지 말 것)
 */

#include <Arduino.h>
#include <Preferences.h>

// ============================ 사용자 설정 ============================

#define USE_BLE_HID 0   // 0: 시리얼로 액션만 출력(드라이런)  1: BLE HID 전송

#if USE_BLE_HID
  // ESP32 BLE Combo(키보드+마우스) 계열 라이브러리. README의 링크/확인 필요 항목 참고
  #include <BleCombo.h>
#endif

// 핀 (README 핀맵 기준: 엄지, 검지, 중지, 약지, 새끼 / 모두 ADC1)
const uint8_t NUM_FINGERS = 5;
const uint8_t FLEX_PINS[NUM_FINGERS] = {36, 39, 34, 35, 32};
const char* const FINGER_NAMES[NUM_FINGERS] = {"thumb", "index", "middle", "ring", "pinky"};
const uint8_t CAL_BUTTON_PIN = 0;   // 보드의 BOOT 버튼

// 샘플링/필터
const uint32_t SAMPLE_INTERVAL_MS = 10;    // 100 Hz
const uint8_t  ADC_OVERSAMPLE     = 8;     // 한 번 샘플당 ADC 평균 횟수
const float    EMA_ALPHA          = 0.30f; // 클수록 빠르고 노이즈 많음

// 캘리브레이션
const uint32_t CAL_SETTLE_MS = 2000;       // 자세 잡는 시간
const uint32_t CAL_CAPTURE_MS = 2000;      // 자세 유지 측정 시간
const uint16_t MIN_RANGE_MV  = 150;        // 펴/굽 전압차가 이보다 작으면 센서 이상으로 판단

// 손가락별 굽힘 판정 임계값 (히스테리시스: ON보다 커지면 굽힘, OFF보다 작아지면 폄)
// 장갑에서 끝까지 안 굽혀지는 손가락(특히 엄지)은 ON을 낮춰서 조정
const float BENT_ON[NUM_FINGERS]  = {0.60f, 0.55f, 0.55f, 0.55f, 0.55f};
const float BENT_OFF[NUM_FINGERS] = {0.40f, 0.35f, 0.35f, 0.35f, 0.35f};

// 제스처 확정
const uint32_t DEBOUNCE_MS = 120;          // 같은 포즈가 이 시간 이상 유지돼야 인정
const bool     START_LOCKED = false;       // true면 부팅 직후 잠금 상태(주먹 1초로 해제)

// ============================ 제스처 정의 ============================

// 포즈 비트: 1 = 굽힘, 0 = 폄
const uint8_t B_THUMB  = 1 << 0;
const uint8_t B_INDEX  = 1 << 1;
const uint8_t B_MIDDLE = 1 << 2;
const uint8_t B_RING   = 1 << 3;
const uint8_t B_PINKY  = 1 << 4;
const uint8_t B_ALL    = 0x1F;

enum Action : uint8_t {
  ACT_NONE, ACT_NEXT, ACT_PREV, ACT_CLICK, ACT_SCROLL_DOWN, ACT_SCROLL_UP, ACT_LOCK_TOGGLE
};
const char* const ACTION_NAMES[] = {"NONE", "NEXT", "PREV", "CLICK", "SCROLL_DOWN", "SCROLL_UP", "LOCK_TOGGLE"};

struct Gesture {
  const char* name;
  uint8_t  bent;           // 이 포즈에서 굽혀져 있어야 하는 손가락
  uint8_t  care;           // 비교할 손가락 (0인 비트는 무시: 노이즈 큰 엄지 등)
  Action   action;
  uint16_t holdMs;         // 포즈 확정 후 추가로 유지해야 하는 시간
  uint16_t repeatMs;       // 0이면 1회만, 아니면 유지하는 동안 반복
  bool     whenLocked;     // 잠금 상태에서도 동작하는지
};

// 위에서부터 먼저 일치하는 항목이 선택됨. 팀에서 여기만 고치면 제스처가 바뀜.
const Gesture GESTURES[] = {
  // 이름        굽힌 손가락                              비교 손가락                  액션              hold  repeat lock
  {"THUMB_UP",  (uint8_t)(B_INDEX|B_MIDDLE|B_RING|B_PINKY), B_ALL,                      ACT_NEXT,         0,    0,   false},
  {"PINKY_UP",  (uint8_t)(B_INDEX|B_MIDDLE|B_RING),         (uint8_t)(B_ALL & ~B_THUMB), ACT_PREV,         0,    0,   false},
  {"POINT",     (uint8_t)(B_MIDDLE|B_RING|B_PINKY),         (uint8_t)(B_ALL & ~B_THUMB), ACT_CLICK,        150,  0,   false},
  {"PEACE",     (uint8_t)(B_RING|B_PINKY),                  (uint8_t)(B_ALL & ~B_THUMB), ACT_SCROLL_DOWN,  150,  120, false},
  {"THREE",     (uint8_t)(B_PINKY),                         (uint8_t)(B_ALL & ~B_THUMB), ACT_SCROLL_UP,    150,  120, false},
  {"FIST",      B_ALL,                                      B_ALL,                      ACT_LOCK_TOGGLE,  1000, 0,   true },
};
const uint8_t NUM_GESTURES = sizeof(GESTURES) / sizeof(GESTURES[0]);

// ============================ 상태 ============================

struct CalData {
  uint32_t magic;
  uint16_t flat[NUM_FINGERS];   // 손 폈을 때 mV
  uint16_t fist[NUM_FINGERS];   // 주먹 쥐었을 때 mV
};
const uint32_t CAL_MAGIC = 0xF1A60001;

Preferences prefs;
CalData  cal;
uint8_t  validMask = B_ALL;     // 캘리브레이션이 정상인 손가락
float    mvFilt[NUM_FINGERS];
bool     filtInit = false;
float    bend[NUM_FINGERS];     // 0(폄) ~ 1(굽힘)
uint8_t  bentState = 0;         // 히스테리시스를 거친 5비트 포즈
bool     locked = START_LOCKED;
bool     debugPrint = true;
uint32_t lastSampleMs = 0;
uint32_t lastDebugMs = 0;

// 제스처 엔진 상태
int8_t   candIdx = -1;  uint32_t candSince = 0;   // 후보(디바운스 중)
int8_t   curIdx  = -1;  uint32_t curSince  = 0;   // 확정된 제스처
bool     fired = false; uint32_t lastFireMs = 0;

// ============================ 센서 읽기 ============================

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
    // flat -> 0, fist -> 1. 부호는 자동 처리 (분압 방향이 반대여도 동작)
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

// ============================ 캘리브레이션 ============================

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
  curIdx = candIdx = -1;
  Serial.println("[CAL] 저장 완료");
}

// ============================ 출력 (BLE HID / 시리얼) ============================

void sendAction(Action a) {
  Serial.printf("[ACTION] %s\n", ACTION_NAMES[a]);
#if USE_BLE_HID
  if (!Keyboard.isConnected()) { Serial.println("[BLE] 연결 안 됨"); return; }
  switch (a) {
    case ACT_NEXT:        Keyboard.write(KEY_PAGE_DOWN);       break;  // 슬라이드 다음
    case ACT_PREV:        Keyboard.write(KEY_PAGE_UP);         break;  // 슬라이드 이전
    case ACT_CLICK:       Mouse.click(MOUSE_LEFT);             break;
    case ACT_SCROLL_DOWN: Mouse.move(0, 0, -1);                break;  // wheel 부호: 방향 반대면 바꾸기
    case ACT_SCROLL_UP:   Mouse.move(0, 0, 1);                 break;
    default: break;
  }
#endif
}

// ============================ 제스처 엔진 ============================

int8_t matchGesture(uint8_t pose) {
  for (uint8_t g = 0; g < NUM_GESTURES; g++) {
    uint8_t care = GESTURES[g].care & validMask;   // 이상 있는 손가락은 비교에서 제외
    if ((pose & care) == (GESTURES[g].bent & care)) return g;
  }
  return -1;
}

void fire(const Gesture& g) {
  if (g.action == ACT_LOCK_TOGGLE) {
    locked = !locked;
    Serial.printf("[LOCK] %s\n", locked ? "잠금" : "해제");
    return;
  }
  if (locked && !g.whenLocked) return;
  sendAction(g.action);
}

void runGestureEngine(uint32_t now) {
  int8_t m = matchGesture(bentState);

  // 디바운스: 같은 후보가 DEBOUNCE_MS 유지되면 확정
  if (m != candIdx) { candIdx = m; candSince = now; }
  if (now - candSince >= DEBOUNCE_MS && candIdx != curIdx) {
    curIdx = candIdx;
    curSince = candSince;
    fired = false;
  }
  if (curIdx < 0) return;

  const Gesture& g = GESTURES[curIdx];
  if (now - curSince < g.holdMs) return;

  if (!fired) {
    fire(g);
    fired = true;
    lastFireMs = now;
  } else if (g.repeatMs && now - lastFireMs >= g.repeatMs) {
    fire(g);
    lastFireMs = now;
  }
}

// ============================ 디버그/입력 ============================

void printDebug(uint32_t now) {
  if (!debugPrint || now - lastDebugMs < 50) return;   // 20 Hz
  lastDebugMs = now;
  // 시리얼 플로터 형식 (label:value)
  for (uint8_t i = 0; i < NUM_FINGERS; i++) Serial.printf("%s:%.2f,", FINGER_NAMES[i], bend[i]);
  Serial.printf("gesture:%d\n", curIdx + 1);
}

void printHelp() {
  Serial.println("명령: c=캘리브레이션  d=디버그 출력 토글  p=캘리브레이션 값  h=도움말");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') runCalibration();
    else if (c == 'd') { debugPrint = !debugPrint; Serial.printf("[DBG] %s\n", debugPrint ? "ON" : "OFF"); }
    else if (c == 'p') applyCalibration();
    else if (c == 'h') printHelp();
  }
}

void handleButton() {
  static uint32_t pressedSince = 0;
  if (digitalRead(CAL_BUTTON_PIN) == LOW) {
    if (!pressedSince) pressedSince = millis();
    else if (millis() - pressedSince >= 2000) {
      pressedSince = 0;
      runCalibration();
      while (digitalRead(CAL_BUTTON_PIN) == LOW) delay(10);
    }
  } else {
    pressedSince = 0;
  }
}

// ============================ setup / loop ============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n== Finger Stone gesture controller ==");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);          // 0~약 3.1V 구간 (센서는 3V3 구동)
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

#if USE_BLE_HID
  Keyboard.begin();
  Mouse.begin();
#endif

  prefs.begin("fstone", false);
  if (loadCalibration()) {
    Serial.println("[CAL] 저장된 캘리브레이션 사용 (재측정: 시리얼 'c' 또는 BOOT 2초)");
    applyCalibration();
  } else {
    Serial.println("[CAL] 저장된 값이 없어 캘리브레이션을 시작합니다");
    runCalibration();
  }
  printHelp();
}

void loop() {
  handleSerial();
  handleButton();

  uint32_t now = millis();
  if (now - lastSampleMs < SAMPLE_INTERVAL_MS) return;
  lastSampleMs = now;

  sampleFingers();
  computeBend();
  updateBentState();
  runGestureEngine(now);
  printDebug(now);
}
