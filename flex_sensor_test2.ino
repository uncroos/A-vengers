/*
 * Finger Stone - Flex 센서 5채널 하드웨어 테스트 (ESP32-DevKitC, Arduino IDE)
 *
 * 목적: 제스처 코드를 올리기 전에 "각 Flex 센서가 ESP32에서 안정적으로 읽히는지" 확인한다.
 *
 * 사용법
 *   1) 업로드 후 시리얼 모니터 115200 baud, 줄 끝 설정은 "줄 바꿈 없음" 또는 "Newline" 아무거나 가능
 *   2) 명령 한 글자를 입력하고 전송 (아래 메뉴). 테스트 도중 'q'를 보내면 중단
 *   3) 보통은 1 -> 2 -> 4 -> 5 순서 ('a'로 한 번에 실행 가능), 마지막에 'r'로 요약 확인
 *
 * 테스트 항목
 *   [1] 배선 점검   : 편 상태 전압을 읽어 단선(0V 근처) / 분압 저항 누락·3V3 단락(높은 전압) 의심 확인
 *   [2] 정지 안정성 : 편 상태로 가만히 둔 채 10초간 노이즈(표준편차, 최대-최소) 측정
 *   [3] 장시간 드리프트 : 같은 방식으로 60초, 전반/후반 평균 차이로 드리프트 확인
 *   [4] 굽힘 범위   : 폈다 굽혔다 했을 때 전압 변화폭과 방향 확인
 *   [5] 채널 분리   : 손가락을 하나씩 굽혀서 "그 채널만" 변하는지(채널 순서 바뀜, 혼선) 확인
 *   [6] 플로터 스트림 : 5채널 mV를 시리얼 플로터용 형식으로 계속 출력 (중지는 보드 EN 리셋)
 *
 * 합격 기준 수치(아래 상수)는 "처음 측정해 보기 위한 임의 시작값"이다.
 * 실제 측정 결과를 보고 팀에서 조정할 것 (정답이 아니라 출발점).
 */

#include <Arduino.h>
#include <math.h>

// ============================ 설정 ============================

const uint8_t NUM_CH = 5;
// README 핀맵 기준: 엄지, 검지, 중지, 약지, 새끼 (모두 ADC1). PCB와 다르면 여기만 수정
const uint8_t FLEX_PINS[NUM_CH] = {36, 39, 34, 35, 32};
const char* const NAMES[NUM_CH] = {"thumb ", "index ", "middle", "ring  ", "pinky "};        // 표 정렬용(공백 포함)
const char* const PLOT_LABELS[NUM_CH] = {"thumb", "index", "middle", "ring", "pinky"};       // 플로터 범례용

// 저항 추정용 회로 상수 (Flex가 3V3 쪽, 분압 저항이 GND 쪽)
// 실제 3V3 레일을 멀티미터로 재서 VCC_MV를 맞추면 추정이 더 정확해진다
const float VCC_MV    = 3300.0f;
const float R_DIV_OHM = 47000.0f;

// 샘플링 (본 제스처 코드와 같은 조건으로 측정해야 의미가 있음)
const uint8_t  OVERSAMPLE = 8;
const uint32_t SAMPLE_MS  = 10;       // 100 Hz

// 합격 기준 (임의 시작값)
const float OPEN_MV_MAX   = 100.0f;   // 이보다 낮으면 단선/GND 단락 의심
const float RAW_OPEN_MAX  = 30.0f;    // raw 평균이 이 이하여도 0V 근처로 판단 (보정된 mV는 하단에서 0이 아닌 값이 나올 수 있음)
const float HIGH_MV_MIN   = 2800.0f;  // 이보다 높으면 분압 저항 누락/3V3 단락 의심 (ADC 11dB 포화는 약 3100mV 부근)
const float PP_OK_MV      = 40.0f;    // 정지 시 최대-최소: 이하 OK
const float PP_WARN_MV    = 100.0f;   // 이하 WARN, 초과 FAIL
const float DRIFT_OK_MV   = 20.0f;    // 전반/후반 평균 차이
const float RANGE_GOOD_MV = 300.0f;   // 굽힘 변화폭: 이상 OK
const float RANGE_WEAK_MV = 150.0f;   // 이상 WARN, 미만 FAIL
const float ISO_WARN_RATIO = 0.30f;   // 다른 채널 변화폭이 대상의 30% 넘으면 WARN
const float SNR_OK   = 10.0f;         // 굽힘 변화폭 / 정지 노이즈(p-p)
const float SNR_WARN = 5.0f;

const uint32_t STABILITY_MS = 10000;
const uint32_t DRIFT_MS     = 60000;
const uint32_t RANGE_MS     = 6000;
const uint32_t ISO_MS       = 5000;

const bool STREAM_ON_BOOT = false;    // true면 부팅 직후 플로터 스트림 시작

// ============================ 공통 ============================

enum Verdict : int8_t { V_NA = -1, V_OK = 0, V_WARN = 1, V_FAIL = 2 };
const char* vtxt(int8_t v) { return v == V_NA ? "-" : v == V_OK ? "OK" : v == V_WARN ? "WARN" : "FAIL"; }
int8_t worse(int8_t a, int8_t b) { return a > b ? a : b; }

// 결과 저장 (요약 리포트용)
int8_t checkV[NUM_CH]; float checkMv[NUM_CH];
int8_t stabV[NUM_CH];  float stabPP[NUM_CH];
int8_t rangeV[NUM_CH]; float rangeMv[NUM_CH];
int8_t isoV[NUM_CH];

bool g_abort = false;

bool pollAbort() {
  while (Serial.available()) {
    int c = Serial.read();
    if (c == 'q' || c == 'Q') g_abort = true;
  }
  return g_abort;
}

bool waitMs(uint32_t ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < ms) { if (pollAbort()) return false; delay(5); }
  return true;
}

bool countdown(uint8_t s) {
  for (; s > 0; s--) { Serial.printf("   %u...\n", s); if (!waitMs(1000)) return false; }
  return true;
}

void waitUntil(uint32_t t) { while ((int32_t)(millis() - t) < 0) delay(1); }

float readMv(uint8_t i) {
  uint32_t sum = 0;
  for (uint8_t k = 0; k < OVERSAMPLE; k++) sum += analogReadMilliVolts(FLEX_PINS[i]);
  return (float)sum / OVERSAMPLE;
}

float readRaw(uint8_t i) {
  uint32_t sum = 0;
  for (uint8_t k = 0; k < OVERSAMPLE; k++) sum += analogRead(FLEX_PINS[i]);
  return (float)sum / OVERSAMPLE;
}

// 노드 전압 -> Flex 저항 추정 (kΩ). 0V 근처는 -1 (무한대)
float estKohm(float mv) {
  if (mv < 50.0f) return -1.0f;
  float r = R_DIV_OHM * (VCC_MV - mv) / mv;
  return r < 0 ? 0 : r / 1000.0f;
}

void printKohm(float k) {
  if (k < 0) Serial.printf("   inf");
  else Serial.printf("%6.1f", k);
}

void header(const char* title) {
  Serial.println("");
  Serial.println("==================================================");
  Serial.println(title);
  Serial.println("==================================================");
}

void resetAbort() { g_abort = false; }

// ============================ [1] 배선 점검 ============================

void testCheck() {
  header("[1] 배선 점검 (손을 펴고 센서를 평평하게 둔 상태)");
  Serial.println("1.5초간 읽습니다. 센서를 구부리지 마세요.");
  if (!countdown(2)) return;

  float sumMv[NUM_CH] = {0}, sumRaw[NUM_CH] = {0};
  uint32_t n = 0, t0 = millis(), next = t0;
  while (millis() - t0 < 1500) {
    if (pollAbort()) return;
    waitUntil(next); next += SAMPLE_MS;
    for (uint8_t i = 0; i < NUM_CH; i++) { sumMv[i] += readMv(i); sumRaw[i] += readRaw(i); }
    n++;
  }

  Serial.println("채널    GPIO   raw(12bit)    mV    Flex추정(kΩ)  판정");
  for (uint8_t i = 0; i < NUM_CH; i++) {
    float mv = sumMv[i] / n, raw = sumRaw[i] / n, k = estKohm(mv);
    const char* msg = "";
    int8_t v = V_OK;
    if (mv < OPEN_MV_MAX || raw < RAW_OPEN_MAX) { v = V_FAIL; msg = "0V 근처: 센서 단선 또는 GND 단락, 커넥터/납땜 확인"; }
    else if (mv > HIGH_MV_MIN) { v = V_FAIL; msg = "전압 너무 높음: 분압 저항(47k) 누락 또는 3V3 단락 의심"; }
    checkV[i] = v; checkMv[i] = mv;
    Serial.printf("%s  %3u   %8.0f   %6.0f   ", NAMES[i], FLEX_PINS[i], raw, mv);
    printKohm(k);
    Serial.printf("      %s  %s\n", vtxt(v), msg);
  }
  Serial.println("");
  Serial.println("참고: 편 상태 Flex 스펙은 25kΩ, 47k 분압이면 약 2150mV 근처가 예상값입니다.");
  Serial.println("      추정 저항은 VCC_MV/R_DIV_OHM 가정에 의존하므로 대략값으로만 보세요.");
  Serial.println("      채널마다 값이 크게 다르면 센서 개체 편차 또는 분압 저항 값/납땜을 확인하세요.");
}

// ============================ [2][3] 정지 안정성 / 드리프트 ============================

struct Stat {
  double mean, m2; uint32_t n;
  float mn, mx;
  double sumA, sumB; uint32_t nA, nB;
  void reset() { mean = m2 = 0; n = 0; mn = 1e9f; mx = -1e9f; sumA = sumB = 0; nA = nB = 0; }
  void add(float v, bool second) {
    n++; double d = v - mean; mean += d / n; m2 += d * (v - mean);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
    if (second) { sumB += v; nB++; } else { sumA += v; nA++; }
  }
  float stddev() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0; }
  float drift() const { return (nA && nB) ? (float)(sumB / nB - sumA / nA) : 0; }
};

void runStability(uint32_t durMs, const char* title) {
  header(title);
  Serial.println("편 상태로 센서를 가만히 두세요. 손을 책상 위에 올려놓고 움직이지 마세요.");
  Serial.printf("%u초 측정합니다 (중단: q)\n", (unsigned)(durMs / 1000));
  if (!countdown(3)) return;

  Stat st[NUM_CH];
  for (auto& s : st) s.reset();
  uint32_t t0 = millis(), next = t0, lastDot = t0;
  while (millis() - t0 < durMs) {
    if (pollAbort()) { Serial.println("중단됨"); return; }
    waitUntil(next); next += SAMPLE_MS;
    bool second = (millis() - t0) >= durMs / 2;
    for (uint8_t i = 0; i < NUM_CH; i++) st[i].add(readMv(i), second);
    if (millis() - lastDot >= 1000) { lastDot += 1000; Serial.printf("   %u/%u s\n", (unsigned)((millis() - t0) / 1000), (unsigned)(durMs / 1000)); }
  }

  Serial.println("채널      평균mV  표준편차  최소   최대   p-p    드리프트  판정");
  for (uint8_t i = 0; i < NUM_CH; i++) {
    float pp = st[i].mx - st[i].mn, dr = st[i].drift();
    int8_t v = pp <= PP_OK_MV ? V_OK : pp <= PP_WARN_MV ? V_WARN : V_FAIL;
    const char* msg = "";
    if (fabsf(dr) > DRIFT_OK_MV) { v = worse(v, V_WARN); msg = "드리프트 큼(발열/접촉/전원 변동 의심)"; }
    else if (v == V_FAIL)        msg = "노이즈 큼: 100nF 커패시터, 접지, 배선 길이 확인";
    else if (v == V_WARN)        msg = "노이즈 다소 큼: 손이 움직이지 않았는지 확인 후 재측정";
    stabV[i] = v; stabPP[i] = pp;
    Serial.printf("%s  %7.1f  %7.2f  %5.0f  %5.0f  %5.0f  %+7.1f   %s  %s\n",
                  NAMES[i], (float)st[i].mean, st[i].stddev(), st[i].mn, st[i].mx, pp, dr, vtxt(v), msg);
  }
  Serial.println("");
  Serial.printf("기준(임의 시작값): p-p %.0f mV 이하 OK, %.0f mV 이하 WARN / 드리프트 %.0f mV 이내\n", PP_OK_MV, PP_WARN_MV, DRIFT_OK_MV);
  if (durMs < 30000) Serial.println("드리프트는 60초 테스트([3])에서 더 잘 드러납니다.");
}

void testStability() { runStability(STABILITY_MS, "[2] 정지 안정성 (10초)"); }
void testDrift()     { runStability(DRIFT_MS,     "[3] 장시간 드리프트 (60초)"); }

// ============================ [4] 굽힘 범위 ============================

void testRange() {
  header("[4] 굽힘 범위");
  Serial.println("시작할 때 손을 편 채로 1초 유지, 그 뒤 5초 동안 주먹을 끝까지 쥐었다 펴기를 2~3번 천천히 반복하세요.");
  Serial.println("(손가락 전체를 같이 굽혀도 됩니다. 장갑을 낀 상태로 해야 실제 사용 범위가 나옵니다)");
  if (!countdown(3)) return;
  Serial.println("시작! 1초 유지 후 굽혔다 펴기");

  float mn[NUM_CH], mx[NUM_CH], startSum[NUM_CH] = {0};
  for (uint8_t i = 0; i < NUM_CH; i++) { mn[i] = 1e9f; mx[i] = -1e9f; }
  uint32_t startN = 0, t0 = millis(), next = t0;
  while (millis() - t0 < RANGE_MS) {
    if (pollAbort()) { Serial.println("중단됨"); return; }
    waitUntil(next); next += SAMPLE_MS;
    bool inStart = (millis() - t0) < 800;
    for (uint8_t i = 0; i < NUM_CH; i++) {
      float v = readMv(i);
      if (v < mn[i]) mn[i] = v;
      if (v > mx[i]) mx[i] = v;
      if (inStart) startSum[i] += v;
    }
    if (inStart) startN++;
  }
  Serial.println("끝. 결과:");

  Serial.println("채널    시작mV  최소mV  최대mV  변화폭  방향  Flex추정(시작 -> 극값, kΩ)   판정");
  for (uint8_t i = 0; i < NUM_CH; i++) {
    float start = startN ? startSum[i] / startN : 0;
    float range = mx[i] - mn[i];
    bool falling = (start - mn[i]) >= (mx[i] - start);   // 굽힘 시 전압 하강 = 설계와 일치
    float extreme = falling ? mn[i] : mx[i];
    int8_t v = range >= RANGE_GOOD_MV ? V_OK : range >= RANGE_WEAK_MV ? V_WARN : V_FAIL;
    const char* msg = "";
    if (v == V_FAIL)      msg = "변화 거의 없음: 센서가 안 굽혀졌거나 배선/센서 불량";
    else if (v == V_WARN) msg = "변화폭 작음: 더 크게 굽혔는지, 장갑 장착 상태 확인";
    else if (!falling)    { v = V_WARN; msg = "굽힘 시 전압 상승: 분압 방향이 설계(굽힘=전압 하강)와 반대, Flex/저항 위치 확인"; }
    rangeV[i] = v; rangeMv[i] = range;
    Serial.printf("%s  %6.0f  %6.0f  %6.0f  %6.0f  %s  ", NAMES[i], start, mn[i], mx[i], range, falling ? "하강" : "상승");
    printKohm(estKohm(start)); Serial.printf(" -> "); printKohm(estKohm(extreme));
    Serial.printf("          %s  %s\n", vtxt(v), msg);
  }
  Serial.println("");
  Serial.printf("기준(임의 시작값): 변화폭 %.0f mV 이상 OK, %.0f mV 이상 WARN\n", RANGE_GOOD_MV, RANGE_WEAK_MV);
  Serial.println("참고: 47k 분압 기준 이론 변화폭은 약 2150mV(25k) -> 1690mV(45k) ~ 900mV(125k). 장갑에서는 이보다 작을 수 있습니다.");
}

// ============================ [5] 채널 분리 ============================

float isoRange[NUM_CH][NUM_CH];

void testIsolation() {
  header("[5] 채널 분리 (손가락 하나씩)");
  Serial.println("안내하는 손가락 하나만 천천히 끝까지 굽혔다 펴세요. 나머지 손가락은 가만히 두세요.");
  Serial.println("(손가락마다 2초 준비 후 5초 측정, 중단: q)");
  Serial.println("손가락 연동 때문에 판정이 애매하면, 센서를 손에서 떼어 책상 위에서 하나씩 직접 구부려서 해도 됩니다.");
  for (uint8_t i = 0; i < NUM_CH; i++) isoV[i] = V_NA;

  for (uint8_t f = 0; f < NUM_CH; f++) {
    Serial.printf("\n[%u/%u] %s 만 굽혔다 펴기 -- 준비\n", f + 1, NUM_CH, NAMES[f]);
    if (!countdown(2)) return;
    Serial.println("   측정 시작!");

    float mn[NUM_CH], mx[NUM_CH];
    for (uint8_t i = 0; i < NUM_CH; i++) { mn[i] = 1e9f; mx[i] = -1e9f; }
    uint32_t t0 = millis(), next = t0;
    while (millis() - t0 < ISO_MS) {
      if (pollAbort()) { Serial.println("중단됨"); return; }
      waitUntil(next); next += SAMPLE_MS;
      for (uint8_t i = 0; i < NUM_CH; i++) {
        float v = readMv(i);
        if (v < mn[i]) mn[i] = v;
        if (v > mx[i]) mx[i] = v;
      }
    }
    for (uint8_t i = 0; i < NUM_CH; i++) isoRange[f][i] = mx[i] - mn[i];
  }

  Serial.println("\n변화폭 행렬 (mV): 행 = 움직인 손가락, 열 = 읽은 채널");
  Serial.printf("움직인 \\ 채널  ");
  for (uint8_t c = 0; c < NUM_CH; c++) Serial.printf("%s ", NAMES[c]);
  Serial.println("  판정");
  for (uint8_t f = 0; f < NUM_CH; f++) {
    float tr = isoRange[f][f], maxO = 0; uint8_t argO = f;
    for (uint8_t c = 0; c < NUM_CH; c++) if (c != f && isoRange[f][c] > maxO) { maxO = isoRange[f][c]; argO = c; }
    int8_t v = V_OK; const char* msg = "";
    if (maxO >= RANGE_WEAK_MV && maxO > tr) { v = V_FAIL; msg = "대상보다 다른 채널이 더 크게 변함: 채널 순서(핀 연결)가 바뀌었거나 대상 센서 불량"; }
    else if (tr < RANGE_WEAK_MV)     { v = V_FAIL; msg = "대상 채널이 거의 안 변함: 센서/배선 확인"; }
    else if (maxO > ISO_WARN_RATIO * tr) { v = V_WARN; msg = "다른 채널도 같이 변함: 손가락 연동(해부학적) 또는 GND/배선 간섭"; }
    isoV[f] = v;
    Serial.printf("%s     ", NAMES[f]);
    for (uint8_t c = 0; c < NUM_CH; c++) Serial.printf("%s%6.0f%s ", c == f ? "[" : " ", isoRange[f][c], c == f ? "]" : " ");
    Serial.printf("  %s  %s", vtxt(v), msg);
    if (v == V_FAIL && maxO > tr) Serial.printf(" (가장 크게 변한 채널: %s)", NAMES[argO]);
    Serial.println("");
  }
  Serial.println("");
  Serial.printf("기준(임의 시작값): 대상 채널이 가장 크게 변하고, 다른 채널은 대상의 %.0f%% 이하\n", ISO_WARN_RATIO * 100);
  Serial.println("참고: 중지/약지처럼 인접 손가락은 사람 손에서도 함께 움직이므로 WARN은 고장이 아닐 수 있습니다.");
  Serial.println("      FAIL(다른 채널이 더 큼)은 거의 항상 핀/배선 순서 문제입니다.");
}

// ============================ [6] 플로터 스트림 ============================

void runStream() {
  Serial.println("플로터 스트림 시작 (Arduino IDE: 도구 > 시리얼 플로터, 115200). 중지: q 또는 보드 EN 리셋");
  delay(300);
  uint32_t next = millis();
  while (!pollAbort()) {
    waitUntil(next); next += 20;
    for (uint8_t i = 0; i < NUM_CH; i++) Serial.printf("%s:%.0f%s", PLOT_LABELS[i], readMv(i), i < NUM_CH - 1 ? "," : "\n");
  }
  Serial.println("스트림 종료");
}

// ============================ 요약 ============================

void printReport() {
  header("요약 리포트 (실행한 테스트만 반영됨)");
  Serial.println("채널     [1]배선  [2/3]노이즈  [4]범위  SNR(범위/p-p)  [5]분리  종합");
  bool any = false;
  for (uint8_t i = 0; i < NUM_CH; i++) {
    int8_t snrV = V_NA; float snr = 0;
    if (rangeV[i] != V_NA && stabV[i] != V_NA && stabPP[i] > 0) {
      snr = rangeMv[i] / stabPP[i];
      snrV = snr >= SNR_OK ? V_OK : snr >= SNR_WARN ? V_WARN : V_FAIL;
    }
    int8_t all = V_NA;
    int8_t vs[] = {checkV[i], stabV[i], rangeV[i], snrV, isoV[i]};
    for (int8_t v : vs) if (v != V_NA) { any = true; all = (all == V_NA) ? v : worse(all, v); }
    Serial.printf("%s   %-6s   %-8s    %-6s  ", NAMES[i], vtxt(checkV[i]), vtxt(stabV[i]), vtxt(rangeV[i]));
    if (snrV == V_NA) Serial.printf("   -  %-4s     ", "-"); else Serial.printf("%5.1f %-4s     ", snr, vtxt(snrV));
    Serial.printf("%-6s   %s\n", vtxt(isoV[i]), vtxt(all));
  }
  if (!any) Serial.println("아직 실행한 테스트가 없습니다. [1]~[5]를 먼저 실행하세요.");
  Serial.println("");
  Serial.printf("SNR 기준(임의 시작값): %.0f 이상 OK, %.0f 이상 WARN. 제스처 임계값이 안정적으로 동작하려면 SNR이 충분히 커야 합니다.\n", SNR_OK, SNR_WARN);
  Serial.println("'-'는 아직 실행하지 않은 테스트입니다. WARN/FAIL 채널은 해당 테스트의 메시지를 다시 확인하세요.");
}

// ============================ 메뉴 ============================

void printMenu() {
  Serial.println("");
  Serial.println("---------------- Flex 센서 테스트 메뉴 ----------------");
  Serial.println(" 1: 배선 점검 (전압/단선)        2: 정지 안정성 (10초)");
  Serial.println(" 3: 장시간 드리프트 (60초)       4: 굽힘 범위");
  Serial.println(" 5: 채널 분리 (손가락 하나씩)    6: 플로터 스트림");
  Serial.println(" a: 1,2,4,5 순서대로 실행        r: 요약 리포트");
  Serial.println(" q: 진행 중인 테스트 중단        h: 메뉴 다시 보기");
  Serial.println("--------------------------------------------------------");
}

void runAll() {
  testCheck();      if (g_abort) return;
  testStability();  if (g_abort) return;
  testRange();      if (g_abort) return;
  testIsolation();  if (g_abort) return;
  printReport();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);   // 약 0~3.1V 구간

  for (uint8_t i = 0; i < NUM_CH; i++) { checkV[i] = stabV[i] = rangeV[i] = isoV[i] = V_NA; stabPP[i] = rangeMv[i] = checkMv[i] = 0; }

  Serial.println("\n== Finger Stone Flex 센서 테스트 ==");
  Serial.print("핀(GPIO): ");
  for (uint8_t i = 0; i < NUM_CH; i++) Serial.printf("%s=%u  ", NAMES[i], FLEX_PINS[i]);
  Serial.println("");
  Serial.printf("가정: 3V3=%.0f mV, 분압 저항=%.0f ohm, 오버샘플 %u회, %lu ms 주기\n", VCC_MV, R_DIV_OHM, OVERSAMPLE, (unsigned long)SAMPLE_MS);
  printMenu();
  if (STREAM_ON_BOOT) { resetAbort(); runStream(); }
}

void loop() {
  if (!Serial.available()) return;
  int c = Serial.read();
  if (c == '\r' || c == '\n' || c == ' ') return;
  resetAbort();
  switch (c) {
    case '1': testCheck(); break;
    case '2': testStability(); break;
    case '3': testDrift(); break;
    case '4': testRange(); break;
    case '5': testIsolation(); break;
    case '6': runStream(); break;
    case 'a': case 'A': runAll(); break;
    case 'r': case 'R': printReport(); break;
    case 'h': case 'H': printMenu(); break;
    case 'q': case 'Q': break;
    default: Serial.println("알 수 없는 명령 (h: 메뉴)"); break;
  }
  if (g_abort) Serial.println("[중단됨]");
}
