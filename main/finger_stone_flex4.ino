/*
 * Finger Stone - Flex 4채널(엄지/검지/중지/약지) 제스처 컨트롤러 + BLE 키보드
 * 환경: ESP32-DevKitC, arduino-esp32 코어 3.3.x, HijelHID_BLEKeyboard (+ NimBLE-Arduino)
 *
 * 흐름
 *   ADC 읽기(오버샘플) -> EMA 필터 -> 캘리브레이션(폄=0, 주먹=1)으로 굽힘 정규화
 *   -> 손가락별 히스테리시스(굽힘/폄) -> 4비트 포즈 -> 제스처 테이블 매칭
 *   -> 디바운스/유지시간 -> 액션 1회 전송 (BLE 키보드)
 *
 * 이번 단계의 제스처 (표 하나만 고치면 바뀜. 아래 GESTURES 참고)
 *   검지만 굽힘        -> 다음 슬라이드
 *   중지+약지 굽힘     -> 이전 슬라이드   (검지는 편 채로)
 *   엄지는 두 제스처 모두 무시 (장갑에서 엄지는 끝까지 안 굽혀지는 경우가 많음)
 *   주먹(검지·중지·약지 모두 굽힘)을 쥐면 손이 완전히 펴질 때까지 모든 제스처 차단 (쥐고 펴는 도중 오발동 방지)
 *
 * 처음 쓰는 순서
 *   1) USE_BLE_HID = 0 으로 업로드해서 센서/제스처 로직만 확인 (PC 연결 불필요)
 *   2) 시리얼 모니터 115200. 첫 부팅 때 캘리브레이션이 자동 진행됨 (안내대로 손 펴기 -> 주먹)
 *   3) 시리얼 플로터에서 손가락을 하나씩 굽혀 "해당 채널만" 변하는지 확인
 *   4) [ACTION] 로그가 의도대로 나오면 USE_BLE_HID = 1 로 바꾸고 PC에 페어링
 *
 * 시리얼 명령: c=재캘리브레이션  d=디버그 출력 켜기/끄기  p=캘리브레이션 값 출력  h=도움말
 * 하드웨어: BOOT 버튼(GPIO0)을 2초 이상 누르면 재캘리브레이션
 *   (GPIO0은 스트랩 핀이므로 부팅이 끝난 뒤 입력으로만 읽음. 외부 회로를 연결하지 말 것)
 *
 * 참고: 기본은 포즈가 바뀔 때마다 [POSE] 로그를 출력한다 (어떤 손가락이 굽힘으로 판정됐는지, 굽힘값, 매칭된 제스처).
 *       시리얼 플로터로 보려면 'd'로 플로터 출력을 켜고 'l'로 포즈 로그를 끈다 (텍스트 줄이 섞이면 플로터에 안 그려짐).
 */

#include <Arduino.h>
#include <Preferences.h>

// ============================ 사용자 설정 ============================

#define USE_BLE_HID 1   // 0: 시리얼로 액션만 출력(드라이런)  1: BLE 키보드 전송

#if USE_BLE_HID
  #include <HijelHID_BLEKeyboard.h>
  // PC 블루투스 목록에 보이는 이름 / 제조사 / 배터리%. (이름을 바꾸면 PC에서 기기를 제거하고 다시 페어링)
  HijelHID_BLEKeyboard keyboard("Finger Stone", "A+vengers", 100);

  // 슬라이드 넘김 키 설정.
  // 라이브러리 문서에서 이름이 확인된 상수는 KEY_SPACE, KEY_RETURN, KEY_ESCAPE, KEY_A~KEY_Z 등이고
  // 화살표/PageUp/PageDown 상수 이름은 src/BLEHIDKeys.h 를 봐야 알 수 있다 (아직 미확인).
  //
  // USE_RAW_HID_CODES = 0 : 확인된 상수만 사용 (Space=다음, P=이전). 이전 슬라이드가 안 먹을 수 있음
  // USE_RAW_HID_CODES = 1 : USB HID 키 코드 숫자를 직접 전달 (실험용).
  //     KEY_A 등이 표준 USB HID 코드(KEY_A=0x04, KEY_SPACE=0x2C)와 같은 값일 때만 동작.
  //     컴파일 오류가 나거나 엉뚱한 키가 나가면 0으로 되돌리고 BLEHIDKeys.h 의 ARROW/PAGE 줄을 확인할 것
  #define USE_RAW_HID_CODES 0

  #if USE_RAW_HID_CODES
    // PowerPoint 슬라이드쇼: 아래 화살표 = 다음 슬라이드, 위 화살표 = 이전 슬라이드
    #define SLIDE_NEXT_KEY 0x51   // HID Down Arrow
    #define SLIDE_PREV_KEY 0x52   // HID Up Arrow
  #else
    #define SLIDE_NEXT_KEY KEY_SPACE
    #define SLIDE_PREV_KEY KEY_P
  #endif
#endif

// 핀 (README 핀맵 기준: 엄지, 검지, 중지, 약지 / 모두 ADC1). 새끼 채널(GPIO32)은 센서 고장으로 사용하지 않음
const uint8_t NUM_FINGERS = 4;
const uint8_t FLEX_PINS[NUM_FINGERS] = {36, 39, 34, 35};
const char* const FINGER_NAMES[NUM_FINGERS] = {"thumb", "index", "middle", "ring"};
const uint8_t CAL_BUTTON_PIN = 0;   // 보드의 BOOT 버튼

// 샘플링/필터
const uint32_t SAMPLE_INTERVAL_MS = 10;    // 100 Hz
const uint8_t  ADC_OVERSAMPLE     = 8;     // 한 번 샘플당 ADC 평균 횟수
const float    EMA_ALPHA          = 0.30f; // 클수록 빠르고 노이즈 많음

// 캘리브레이션
const uint32_t CAL_SETTLE_MS  = 2000;      // 자세 잡는 시간
const uint32_t CAL_CAPTURE_MS = 2000;      // 자세 유지 측정 시간
const uint16_t MIN_RANGE_MV   = 150;       // 펴/굽 전압차가 이보다 작으면 센서 이상으로 판단 (그 손가락은 무시)

// 손가락별 굽힘 판정 임계값 (히스테리시스: ON보다 커지면 굽힘, OFF보다 작아지면 폄)
// 장갑에서 끝까지 안 굽혀지는 손가락(특히 엄지)은 ON을 낮춰서 조정
const float BENT_ON[NUM_FINGERS]  = {0.60f, 0.55f, 0.55f, 0.55f};
const float BENT_OFF[NUM_FINGERS] = {0.40f, 0.35f, 0.35f, 0.35f};

// 제스처 확정
const uint32_t DEBOUNCE_MS = 120;          // 같은 포즈가 이 시간 이상 유지돼야 인정
const uint32_t REFRACTORY_MS = 400;        // 제스처 발동 후 이 시간 동안은 다음 제스처 발동 안 함
// 제스처에서 "펴져 있어야 하는" 손가락은 굽힘값이 이 값 이하로 확실히 펴져 있어야 인정 (전환 중 오발동 방지)
const float    OPEN_CLEAR = 0.30f;

// ============================ 제스처 정의 ============================

// 포즈 비트: 1 = 굽힘, 0 = 폄
const uint8_t B_THUMB  = 1 << 0;
const uint8_t B_INDEX  = 1 << 1;
const uint8_t B_MIDDLE = 1 << 2;
const uint8_t B_RING   = 1 << 3;
const uint8_t B_ALL    = 0x0F;

enum Action : uint8_t { ACT_NONE, ACT_NEXT, ACT_PREV };
const char* const ACTION_NAMES[] = {"NONE", "NEXT", "PREV"};

struct Gesture {
  const char* name;
  uint8_t  bent;      // 이 포즈에서 굽혀져 있어야 하는 손가락
  uint8_t  care;      // 비교할 손가락 (0인 비트는 무시)
  Action   action;
  uint16_t holdMs;    // 포즈 확정 후 추가로 유지해야 하는 시간
};

const uint8_t B_NO_THUMB = B_ALL & ~B_THUMB;

// 위에서부터 먼저 일치하는 항목이 선택됨. 팀에서 여기만 고치면 제스처가 바뀜.
const Gesture GESTURES[] = {
  // 이름          굽힌 손가락              비교 손가락    액션       hold
  {"INDEX_BEND", B_INDEX,                B_NO_THUMB,  ACT_NEXT,  250},
  {"RING_MID",   (uint8_t)(B_MIDDLE|B_RING), B_NO_THUMB, ACT_PREV,  250},
};
const uint8_t NUM_GESTURES = sizeof(GESTURES) / sizeof(GESTURES[0]);

// ============================ 상태 ============================

struct CalData {
  uint32_t magic;
  uint16_t flat[NUM_FINGERS];   // 손 폈을 때 mV
  uint16_t fist[NUM_FINGERS];   // 주먹 쥐었을 때 mV
};
const uint32_t CAL_MAGIC = 0xF1A60004;   // 손가락 수가 바뀌어서 이전 5채널 저장값과 구분

Preferences prefs;
CalData  cal;
uint8_t  validMask = B_ALL;     // 캘리브레이션이 정상인 손가락
float    mvFilt[NUM_FINGERS];
bool     filtInit = false;
float    bend[NUM_FINGERS];     // 0(폄) ~ 1(굽힘)
uint8_t  bentState = 0;         // 히스테리시스를 거친 4비트 포즈
bool     debugPrint = false;    // 시리얼 플로터용 출력 (켜려면 'd'). 기본은 꺼 둠: 포즈 로그를 읽기 쉽게
bool     poseLog = true;        // 포즈 변경 로그 (끄려면 'l')
uint32_t lastSampleMs = 0;
uint32_t lastDebugMs = 0;

// 제스처 엔진 상태
int8_t   candIdx = -1;  uint32_t candSince = 0;   // 후보(디바운스 중)
int8_t   curIdx  = -1;  uint32_t curSince  = 0;   // 확정된 제스처
bool     fired = false; uint32_t lastFireMs = 0;
bool     everFired = false;
bool     fistLatch = false;     // 주먹 감지 후 손이 완전히 펴질 때까지 모든 제스처 차단

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

// ============================ 출력 (BLE 키보드 / 시리얼) ============================

void sendAction(Action a) {
  Serial.printf("[ACTION] %s\n", ACTION_NAMES[a]);
#if USE_BLE_HID
  if (!keyboard.isPaired()) { Serial.println("[BLE] 페어링 안 됨"); return; }
  switch (a) {
    case ACT_NEXT: keyboard.tap(SLIDE_NEXT_KEY); break;
    case ACT_PREV: keyboard.tap(SLIDE_PREV_KEY); break;
    default: break;
  }
#endif
}

// ============================ 제스처 엔진 ============================

int8_t matchGesture(uint8_t pose) {
  for (uint8_t g = 0; g < NUM_GESTURES; g++) {
    uint8_t care = GESTURES[g].care & validMask;   // 이상 있는 손가락은 비교에서 제외
    if (care == 0) continue;                       // 비교할 손가락이 하나도 없으면 오발동 방지로 건너뜀
    if ((pose & care) != (GESTURES[g].bent & care)) continue;
    // 펴져 있어야 하는 손가락이 굽히는 중(애매한 구간)이면 인정하지 않음
    bool clearlyOpen = true;
    for (uint8_t i = 0; i < NUM_FINGERS; i++) {
      uint8_t bit = 1 << i;
      if ((care & bit) && !(GESTURES[g].bent & bit) && bend[i] > OPEN_CLEAR) clearlyOpen = false;
    }
    if (clearlyOpen) return g;
  }
  return -1;
}

void runGestureEngine(uint32_t now) {
  // 주먹(검지·중지·약지 모두 굽힘)이 되면 손이 완전히 펴질 때까지 모든 제스처를 차단.
  // 주먹을 쥐거나 펴는 도중에 손가락이 하나씩 굽혀/펴지며 생기는 중간 포즈의 오발동을 막는다.
  const uint8_t FIST_MASK = (B_INDEX | B_MIDDLE | B_RING) & validMask;
  if (FIST_MASK) {
    if ((bentState & FIST_MASK) == FIST_MASK) fistLatch = true;
    else if ((bentState & FIST_MASK) == 0)    fistLatch = false;
  }
  int8_t m = fistLatch ? -1 : matchGesture(bentState);

  // 디바운스: 같은 후보가 DEBOUNCE_MS 유지되면 확정
  if (m != candIdx) { candIdx = m; candSince = now; }
  if (now - candSince >= DEBOUNCE_MS && candIdx != curIdx) {
    curIdx = candIdx;
    curSince = candSince;
    fired = false;
  }
  if (curIdx < 0 || fired) return;
  if (candIdx != curIdx) return;   // 포즈가 이미 바뀌었으면(주먹으로 가는 중 등) 대기 중이던 제스처는 취소

  const Gesture& g = GESTURES[curIdx];
  if (now - curSince < g.holdMs) return;
  if (everFired && now - lastFireMs < REFRACTORY_MS) return;

  sendAction(g.action);
  fired = true;            // 같은 포즈를 유지하는 동안은 1회만. 포즈가 풀려야 다시 발동
  everFired = true;
  lastFireMs = now;
}

// ============================ 디버그/입력 ============================

void printDebug(uint32_t now) {
  if (!debugPrint || now - lastDebugMs < 50) return;   // 20 Hz
  lastDebugMs = now;
  // 시리얼 플로터 형식 (label:value)
  for (uint8_t i = 0; i < NUM_FINGERS; i++) Serial.printf("%s:%.2f,", FINGER_NAMES[i], bend[i]);
  Serial.printf("gesture:%d\n", curIdx + 1);
}

// 포즈가 바뀔 때마다 한 줄 출력: 어떤 손가락이 굽힘으로 판정됐는지, 그때의 굽힘값, 매칭된 제스처
void logPoseChange() {
  static uint8_t lastPose = 0xFF;
  if (!poseLog || bentState == lastPose) return;
  lastPose = bentState;
  Serial.printf("[POSE] ");
  for (uint8_t i = 0; i < NUM_FINGERS; i++)
    Serial.printf("%s=%d(%.2f)%s ", FINGER_NAMES[i], (bentState >> i) & 1, bend[i], (validMask & (1 << i)) ? "" : "[무시됨]");
  int8_t m = matchGesture(bentState);
  if (fistLatch) Serial.println("-> 주먹: 손이 펴질 때까지 모든 제스처 차단");
  else Serial.printf("-> %s\n", m < 0 ? "일치하는 제스처 없음" : GESTURES[m].name);
}

void printHelp() {
  Serial.println("명령: c=캘리브레이션  d=플로터 출력 토글  l=포즈 로그 토글  p=캘리브레이션 값  h=도움말");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'c') runCalibration();
    else if (c == 'd') { debugPrint = !debugPrint; Serial.printf("[DBG] %s\n", debugPrint ? "ON" : "OFF"); }
    else if (c == 'l') { poseLog = !poseLog; Serial.printf("[LOG] 포즈 로그 %s\n", poseLog ? "ON" : "OFF"); }
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
  Serial.println("\n== Finger Stone flex4 gesture controller ==");

  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);          // 0~약 3.1V 구간 (센서는 3V3 구동)
  pinMode(CAL_BUTTON_PIN, INPUT_PULLUP);

#if USE_BLE_HID
  keyboard.begin();
  Serial.println("[BLE] 광고 시작: PC에서 'Finger Stone' 페어링");
#else
  Serial.println("[BLE] 꺼짐 (USE_BLE_HID=0, 시리얼 드라이런)");
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
  logPoseChange();
  printDebug(now);
}
