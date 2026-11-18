/*
 * Finger Stone - MyoWare 2.0 근전도(EMG) 센서 하드웨어 테스트 (ESP32-DevKitC, Arduino IDE)
 *
 * 목적: MyoWare 2.0 출력이 ESP32 ADC(GPIO33)에서 제대로 읽히는지,
 *       "근육 이완 vs 수축"을 구분할 수 있는지, 노이즈/포화/임계값 동작이 괜찮은지 확인한다.
 *
 * !!! 안전 (프로젝트 README 8번과 동일) !!!
 *   MyoWare는 절연되어 있지 않다. 전극을 몸에 붙인 상태에서는 PC의 USB에 연결하지 말 것.
 *   -> 업로드/설정은 전극을 뗀 상태에서 USB로 하고,
 *      측정할 때는 USB를 뽑고 배터리로 구동한 채 블루투스(USE_BT_SERIAL 1)로 결과를 받는다.
 *   배터리를 충전(USB-C)하는 동안에도 전극을 붙이지 말 것.
 *
 * 블루투스 시리얼(클래식 SPP) 사용법
 *   - 업로드 후 보드가 "FingerStone-EMG" 이름으로 보임. PC/휴대폰에서 페어링
 *     (Windows: 페어링 후 생기는 "나가는 COM 포트"를 Arduino IDE 시리얼 모니터에서 선택,
 *      Android: Serial Bluetooth Terminal 같은 앱, 115200 설정은 무시됨)
 *   - 스케치 용량이 커서 업로드가 실패하면 보드 설정의 Partition Scheme을 "Huge APP"으로 (확인 필요)
 *   - 블루투스 무선이 ADC 노이즈를 늘릴 수 있으므로 [2] 기준선이 나쁘면 USE_BT_SERIAL 0 + 전극 없이 비교
 *
 * 사용법: 명령 한 글자를 보냄. 테스트 중 'q'를 보내면 중단.
 *   1 전압 점검      : 센서 출력 전압/연결 상태 (1.5초)
 *   2 이완 기준선    : 근육을 완전히 이완한 상태에서 10초간 노이즈 측정
 *   3 수축 반응      : 이완 3초 -> 주먹 세게 4초 -> 이완 3초, 증가량/포화/회복 + 시간대별 타임라인 출력
 *   4 트리거 데모    : [3]에서 자동 계산한 임계값으로 주먹 쥘 때마다 ON/OFF가 깔끔하게 나오는지 확인
 *   5 스트림         : 활동도를 시리얼 플로터용 형식으로 계속 출력 (중지: q)
 *   a 1,2,3,4 순서대로   r 요약 리포트   h 메뉴
 *
 * 판정 기준 수치는 "처음 측정해 보기 위한 임의 시작값"이다. 실측 결과를 보고 조정할 것.
 */

#include <Arduino.h>
#include <math.h>

// ============================ 설정 ============================

#ifndef USE_BT_SERIAL
#define USE_BT_SERIAL 1      // 1: USB + 블루투스 시리얼 둘 다 출력, 0: USB만
#endif
#ifndef EMG_OUTPUT_ENV
#define EMG_OUTPUT_ENV 1     // 1: MyoWare ENV(포락선) 출력을 연결함, 0: RAW 출력을 연결함 (README: 솔더 점퍼 SJ1)
#endif

#if USE_BT_SERIAL
  #include <BluetoothSerial.h>
  BluetoothSerial SerialBT;
#endif

const uint8_t  EMG_PIN = 33;                              // README 핀맵 (ADC1_CH5). PCB와 다르면 수정
const float    VCC_MV  = 3300.0f;

// ENV는 느린 신호라 500 Hz면 충분, RAW는 EMG 대역(약 20~500 Hz) 때문에 1 kHz 이상 필요
const uint32_t SAMPLE_US  = EMG_OUTPUT_ENV ? 2000 : 1000;
const uint8_t  OVERSAMPLE = EMG_OUTPUT_ENV ? 4 : 1;
const uint32_t WINDOW_MS  = 100;                          // 활동도 계산 창 (ENV: 창 평균, RAW: 창 내 AC RMS)

// 판정 기준 (임의 시작값)
const float SAT_MV        = 3000.0f;   // ADC 11dB 포화는 약 3100mV 부근
const float RAW_CENTER_MV = 1650.0f;   // RAW 출력은 VCC/2 중심으로 알고 있음 (데이터시트 확인 필요)
const float BASE_PP_OK    = 60.0f;     // [ENV] 이완 시 최대-최소: 이하 OK
const float BASE_PP_WARN  = 150.0f;    //        이하 WARN
const float BASE_RMS_OK   = 20.0f;     // [RAW] 이완 시 창 RMS 평균: 이하 OK
const float BASE_RMS_WARN = 50.0f;
const float DELTA_OK      = EMG_OUTPUT_ENV ? 300.0f : 60.0f;   // 수축 시 활동도 증가량(mV)
const float DELTA_WEAK    = EMG_OUTPUT_ENV ? 100.0f : 20.0f;
const float NOISE_FLOOR_MIN = EMG_OUTPUT_ENV ? 10.0f : 5.0f;   // SNR 계산 시 분모 하한
const float SNR_OK   = 10.0f;
const float SNR_WARN = 4.0f;
const float SAT_FRAC_WARN = 0.01f;     // 수축 중 샘플의 1% 이상이 포화면 게인 과다

// ============================ 출력 (USB + 블루투스) ============================

class Tee : public Print {
 public:
  size_t write(uint8_t c) override {
    Serial.write(c);
#if USE_BT_SERIAL
    if (SerialBT.hasClient()) SerialBT.write(c);
#endif
    return 1;
  }
  size_t write(const uint8_t* b, size_t n) override {
    Serial.write(b, n);
#if USE_BT_SERIAL
    if (SerialBT.hasClient()) SerialBT.write(b, n);
#endif
    return n;
  }
};
Tee out;

// ============================ 공통 ============================

enum Verdict : int8_t { V_NA = -1, V_OK = 0, V_WARN = 1, V_FAIL = 2 };
const char* vtxt(int8_t v) { return v == V_NA ? "-" : v == V_OK ? "OK" : v == V_WARN ? "WARN" : "FAIL"; }
int8_t worse(int8_t a, int8_t b) { return a > b ? a : b; }

bool g_abort = false;

int readCmdChar() {
  if (Serial.available()) return Serial.read();
#if USE_BT_SERIAL
  if (SerialBT.available()) return SerialBT.read();
#endif
  return -1;
}
bool cmdAvailable() {
#if USE_BT_SERIAL
  if (SerialBT.available()) return true;
#endif
  return Serial.available() > 0;
}

bool pollAbort() {
  while (cmdAvailable()) {
    int c = readCmdChar();
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
  for (; s > 0; s--) { out.printf("   %u...\n", s); if (!waitMs(1000)) return false; }
  return true;
}

void header(const char* title) {
  out.println("");
  out.println("==================================================");
  out.println(title);
  out.println("==================================================");
}

float readMv() {
  uint32_t s = 0;
  for (uint8_t k = 0; k < OVERSAMPLE; k++) s += analogReadMilliVolts(EMG_PIN);
  return (float)s / OVERSAMPLE;
}

float readRawAvg(uint16_t n) {
  uint32_t s = 0;
  for (uint16_t k = 0; k < n; k++) { s += analogRead(EMG_PIN); delayMicroseconds(200); }
  return (float)s / n;
}

// ---- 통계 ----
struct Run {
  double mean, m2; uint32_t n; float mn, mx;
  void reset() { mean = m2 = 0; n = 0; mn = 1e9f; mx = -1e9f; }
  void add(float v) {
    n++; double d = v - mean; mean += d / n; m2 += d * (v - mean);
    if (v < mn) mn = v;
    if (v > mx) mx = v;
  }
  float sd() const { return n > 1 ? sqrt(m2 / (n - 1)) : 0; }
  float pp() const { return n ? mx - mn : 0; }
};

// ---- 활동도: ENV는 창 평균(mV), RAW는 창 내 AC RMS(mV) ----
struct ActMeter {
  double sum, sq; uint32_t n, winUs, winEnd;
  void begin(uint32_t nowUs, uint32_t windowMs) { sum = sq = 0; n = 0; winUs = windowMs * 1000UL; winEnd = nowUs + winUs; }
  bool feed(float v, uint32_t nowUs, float& act) {
    sum += v; sq += (double)v * v; n++;
    if ((int32_t)(nowUs - winEnd) < 0) return false;
    double mean = sum / n;
#if EMG_OUTPUT_ENV
    act = (float)mean;
#else
    double var = sq / n - mean * mean;
    act = var > 0 ? (float)sqrt(var) : 0.0f;
#endif
    sum = sq = 0; n = 0; winEnd += winUs;
    return true;
  }
};

// [3] 수축 테스트용: 창 활동도를 시간순으로 저장 (100ms 창 x 최대 14초)
const int TL_MAX = 140;
float g_tl[TL_MAX]; int g_tlN = 0; bool g_tlOn = false;

struct PhaseOut {
  Run s;               // 샘플(mV)
  Run a;               // 창 활동도
  uint32_t nSat;       // 포화 샘플 수
  float hz;            // 실제 샘플링 속도
  float amp60;         // 60Hz 성분 진폭(mV, 피크). 계산 안 했으면 -1
};

// durMs 동안 SAMPLE_US 간격으로 샘플링. 중단되면 false
bool runPhase(uint32_t durMs, PhaseOut& o, bool wantG60 = false) {
  o.s.reset(); o.a.reset(); o.nSat = 0; o.hz = 0; o.amp60 = -1;
  uint32_t startUs = micros(), next = startUs;
  uint32_t totalUs = durMs * 1000UL;
  ActMeter am; am.begin(startUs, WINDOW_MS);
  const double coef = 2.0 * cos(2.0 * M_PI * 60.0 * SAMPLE_US / 1e6);
  double q1 = 0, q2 = 0;
  while ((uint32_t)(micros() - startUs) < totalUs) {
    while ((int32_t)(micros() - next) < 0) {}
    next += SAMPLE_US;
    float v = readMv();
    o.s.add(v);
    if (v >= SAT_MV) o.nSat++;
    if (wantG60) { double q0 = coef * q1 - q2 + (v - RAW_CENTER_MV); q2 = q1; q1 = q0; }
    float act;
    if (am.feed(v, micros(), act)) {
      o.a.add(act);
      if (g_tlOn && g_tlN < TL_MAX) g_tl[g_tlN++] = act;
      if (pollAbort()) return false;
    }
  }
  float el = (micros() - startUs) / 1e6f;
  o.hz = el > 0 ? o.s.n / el : 0;
  float nominal = 1e6f / SAMPLE_US;
  if (wantG60 && o.s.n > 100 && fabsf(o.hz - nominal) < 0.05f * nominal) {
    double p = q1 * q1 + q2 * q2 - coef * q1 * q2;
    o.amp60 = (float)(2.0 * sqrt(p > 0 ? p : 0) / o.s.n);
  }
  return true;
}

// ---- 결과 저장 ----
int8_t r1 = V_NA, r2 = V_NA, r3 = V_NA;
float  r1Mv = 0, r2Pp = 0, r2Act = 0, r3Delta = 0, r3Snr = 0, r3Sat = 0;
bool   haveThr = false; float thrOn = 0, thrOff = 0;
int    r4Count = -1, r4Chatter = 0;

// ============================ [1] 전압 점검 ============================

void testCheck() {
  header("[1] 전압 점검 (전극 부착, 근육 이완)");
  out.println("팔 힘을 빼고 가만히 계세요. 1.5초 측정합니다.");
  if (!countdown(2)) return;
  PhaseOut P;
  if (!runPhase(1500, P)) return;
  float raw = readRawAvg(200);
  float mv = (float)P.s.mean;

  out.printf("GPIO%u   raw(12bit) 평균: %.0f    mV 평균: %.0f    (최소 %.0f / 최대 %.0f)\n", EMG_PIN, raw, mv, P.s.mn, P.s.mx);
  int8_t v = V_OK; const char* msg = "";
  if (mv > SAT_MV) { v = V_FAIL; msg = "출력이 ADC 상단에 붙음: 센서 전원이 5V로 연결됐거나 게인 과다/단락 의심 (센서는 3V3 구동)"; }
#if EMG_OUTPUT_ENV
  else if (P.s.pp() > 300) { v = V_WARN; msg = "측정 중 신호가 크게 변함(근육이 긴장했거나 전극이 흔들림): 완전히 이완한 뒤 다시 측정하세요"; }
  else if (raw < 30) { v = V_OK; msg = "ENV는 이완 시 0V 근처가 정상일 수 있음(ADC 하단 불감대). 반응은 [3]에서 확인"; }
  else msg = "이완 시 전압 확인됨. 반응은 [3]에서 확인";
#else
  else if (raw < 30) { v = V_FAIL; msg = "RAW 출력이 0V: 센서 전원/출력 연결 확인 (RAW는 VCC/2 근처가 정상으로 알고 있음, 데이터시트 확인 필요)"; }
  else if (fabsf(mv - RAW_CENTER_MV) > 600) { v = V_WARN; msg = "RAW 중심 전압이 VCC/2에서 크게 벗어남 (확인 필요)"; }
#endif
  r1 = v; r1Mv = mv;
  out.printf("판정: %s  %s\n", vtxt(v), msg);
  out.println("참고: 이 테스트만으로는 센서가 살아 있는지 알 수 없습니다. [3]에서 주먹을 쥘 때 값이 올라가야 정상입니다.");
}

// ============================ [2] 이완 기준선 ============================

void testBaseline() {
  header("[2] 이완 기준선 (10초)");
  out.println("전극을 붙인 팔을 책상에 올리고 완전히 힘을 빼세요. 손가락도 움직이지 마세요.");
  if (!countdown(3)) return;
  PhaseOut P;
  if (!runPhase(10000, P, !EMG_OUTPUT_ENV)) { out.println("중단됨"); return; }

  out.printf("실제 샘플링 속도: %.0f Hz (목표 %.0f Hz)%s\n", P.hz, 1e6f / SAMPLE_US,
             P.hz < 0.95f * (1e6f / SAMPLE_US) ? "   << 목표보다 느림: OVERSAMPLE을 줄이세요" : "");
  out.printf("신호 mV: 평균 %.1f  표준편차 %.2f  최소 %.0f  최대 %.0f  p-p %.0f\n", P.s.mean, P.s.sd(), P.s.mn, P.s.mx, P.s.pp());
  out.printf("활동도(%s, %lums 창): 평균 %.1f  최대 %.1f  최소 %.1f\n",
             EMG_OUTPUT_ENV ? "창 평균 mV" : "창 내 RMS mV", (unsigned long)WINDOW_MS, P.a.mean, P.a.mx, P.a.mn);
  if (P.amp60 >= 0) {
    float rms60 = P.amp60 / 1.41421f, total = P.s.sd();
    out.printf("60Hz 성분: 진폭 %.1f mV (전체 RMS 대비 %.0f%%)  -- 전원선 간섭 지표(정보용)\n", P.amp60, total > 0 ? 100.0f * rms60 / total : 0);
  }

  int8_t v; const char* msg = "";
#if EMG_OUTPUT_ENV
  float pp = P.s.pp();
  v = pp <= BASE_PP_OK ? V_OK : pp <= BASE_PP_WARN ? V_WARN : V_FAIL;
  if (v != V_OK) msg = "이완 상태인데 흔들림이 큼: 팔을 완전히 이완했는지, 전극 접촉, 전원/배선 노이즈 확인";
  r2Pp = pp;
#else
  float rms = P.a.mean;
  v = rms <= BASE_RMS_OK ? V_OK : rms <= BASE_RMS_WARN ? V_WARN : V_FAIL;
  if (v != V_OK) msg = "이완 시 AC 성분이 큼: 전극 접촉/피부 상태, 전원선 간섭(60Hz), 케이블 흔들림 확인";
  r2Pp = P.s.pp();
#endif
  r2 = v; r2Act = P.a.mean;
#if EMG_OUTPUT_ENV
  if (readRawAvg(100) < 30)
    out.println("참고: 이완 시 ENV가 ADC 하단(raw 0, 약 142mV 표시)에 붙어 있음. 센서 출력이 거의 0V라는 뜻이라 정상이지만, 아주 약한 수축은 이 아래에서 안 보일 수 있음.");
#endif
  out.printf("판정: %s  %s\n", vtxt(v), msg);
  if (P.s.n > 100 && P.s.sd() == 0)
    out.println("주의: 값이 완전히 고정(표준편차 0). 출력 미연결/전원 없음이거나 ENV가 ADC 하단(0V 근처)에 붙어 있을 수 있음 -> [3]에서 반응 확인");
  out.printf("기준(임의 시작값): %s\n", EMG_OUTPUT_ENV ? "p-p 60mV 이하 OK, 150mV 이하 WARN" : "RMS 20mV 이하 OK, 50mV 이하 WARN");
}

// ============================ [3] 수축 반응 ============================

static void sortAsc(float* a, int n) {
  for (int i = 1; i < n; i++) { float k = a[i]; int j = i - 1; while (j >= 0 && a[j] > k) { a[j + 1] = a[j]; j--; } a[j + 1] = k; }
}

void testContraction() {
  header("[3] 수축 반응 (이완 3초 -> 수축 4초 -> 이완 3초)");
  out.println("전극을 붙인 팔 쪽 손으로 진행합니다.");
  out.println("중요: '>> 지금!' 문구가 화면에 나온 뒤에 주먹을 세게 쥐고, '>> 힘 빼세요'가 나오면 푸세요.");
  out.println("      카운트다운 중이나 '>> 이완' 구간에는 힘을 주지 마세요 (기준선이 오염됩니다).");
  if (!countdown(3)) return;

  g_tlN = 0; g_tlOn = true;
  PhaseOut A, B, C;
  out.println(">> 이완 (3초)");
  if (!runPhase(3000, A)) { g_tlOn = false; out.println("중단됨"); return; }
  int iB = g_tlN;
  out.println(">> 지금! 주먹을 세게 쥐고 유지하세요 (4초)");
  if (!runPhase(4000, B)) { g_tlOn = false; out.println("중단됨"); return; }
  int iC = g_tlN;
  out.println(">> 힘 빼세요 (이완 3초)");
  if (!runPhase(3000, C)) { g_tlOn = false; out.println("중단됨"); return; }
  g_tlOn = false;
  int nA = iB, nB = iC - iB, nC = g_tlN - iC;
  if (nA < 5 || nB < 5 || nC < 5) { out.println("샘플링 창이 부족합니다. 다시 실행하세요."); return; }

  // 타이밍이 조금 어긋나도 되도록 중앙값/상위 평균 같은 강건한 지표를 사용
  float tmp[TL_MAX];
  for (int i = 0; i < nA; i++) tmp[i] = g_tl[i];
  sortAsc(tmp, nA);
  float baseMed = tmp[nA / 2];
  float baseHi  = tmp[(nA * 9) / 10 < nA ? (nA * 9) / 10 : nA - 1];
  float baseMax = tmp[nA - 1];

  for (int i = 0; i < nB; i++) tmp[i] = g_tl[iB + i];
  sortAsc(tmp, nB);
  int topK = nB < 10 ? nB : 10;                        // 가장 높은 1초(10개 창)의 평균
  float clenchTop = 0; for (int i = 0; i < topK; i++) clenchTop += tmp[nB - 1 - i];
  clenchTop /= topK;

  int nTail = nC < 15 ? nC : 15;                       // 이완 후 마지막 1.5초의 중앙값
  for (int i = 0; i < nTail; i++) tmp[i] = g_tl[g_tlN - nTail + i];
  sortAsc(tmp, nTail);
  float recov = tmp[nTail / 2];

  out.printf("\n구간            활동도 대표값(%s)   활동도 최대   신호 mV 최소~최대\n", EMG_OUTPUT_ENV ? "mV" : "RMS mV");
  out.printf("이완(전)  중앙값 %9.1f                  %9.1f     %6.0f ~ %-6.0f\n", baseMed, A.a.mx, A.s.mn, A.s.mx);
  out.printf("수축      최고1초 %8.1f                  %9.1f     %6.0f ~ %-6.0f\n", clenchTop, B.a.mx, B.s.mn, B.s.mx);
  out.printf("이완(후)  중앙값 %9.1f                  %9.1f     %6.0f ~ %-6.0f\n", recov, C.a.mx, C.s.mn, C.s.mx);

  float delta = clenchTop - baseMed;
  float noise = baseHi - baseMed; if (noise < NOISE_FLOOR_MIN) noise = NOISE_FLOOR_MIN;
  float snr = delta / noise;
  uint32_t satN = B.nSat; float satFrac = B.s.n ? (float)satN / B.s.n : 0;
  // 센서가 어느 구간에서든 크게 반응했는가 (기준선이 오염돼도 판단되도록 타임라인 전체의 최대-최소로 확인)
  float tlLo = g_tl[0], tlHi = g_tl[0]; int peakIdx = 0;
  for (int i = 1; i < g_tlN; i++) { if (g_tl[i] < tlLo) tlLo = g_tl[i]; if (g_tl[i] > tlHi) { tlHi = g_tl[i]; peakIdx = i; } }
  bool alive = (tlHi - tlLo) >= DELTA_WEAK;
  bool preCue = (baseMax - baseMed) >= DELTA_WEAK && (baseMax - baseMed) > 0.5f * (delta > DELTA_WEAK ? delta : DELTA_WEAK);

  int8_t v = (delta >= DELTA_OK && snr >= SNR_OK) ? V_OK : (delta >= DELTA_WEAK && snr >= SNR_WARN) ? V_WARN : V_FAIL;
  const char* msg = "";
  if (v == V_FAIL && alive) {
    v = V_WARN;
    if (peakIdx < iB)       msg = "센서는 반응함(하드웨어 OK). 그런데 가장 큰 활동이 '이완' 구간(신호 문구 전)에서 나옴: 문구가 뜨기 전에 미리 쥐었는지 확인하고 재측정";
    else if (peakIdx >= iC) msg = "센서는 반응함(하드웨어 OK). 그런데 가장 큰 활동이 '힘 빼세요' 이후에 나옴: 신호를 늦게 봤거나(블루투스 지연) 늦게 쥠. 타임라인 확인 후 재측정";
    else                    msg = "센서는 반응함. 수축 구간이지만 증가량이 기준보다 작음: 더 세게 쥐고 타임라인 확인";
  }
  else if (v == V_FAIL)     msg = "어느 구간에서도 반응 없음: 전극 부착/위치(팔뚝 근육 중앙), 피부 청결, 전극 케이블, 센서 전원(3V3), 출력 연결(ENV 핀, U2 점퍼) 확인";
  else if (v == V_WARN)     msg = "반응이 약함: 전극 위치 조정 또는 게인 가변저항을 올려 보세요 (게인 조절부는 데이터시트 확인 필요)";
  if (preCue && v != V_FAIL) { v = worse(v, V_WARN); msg = "신호 전(이완 구간)에도 큰 활동이 있었음: '지금!' 문구가 뜬 뒤에 쥐었는지 확인. 기준선이 오염됐을 수 있음"; }
  if (satFrac > SAT_FRAC_WARN) { v = worse(v, V_WARN); msg = "수축 중 포화 구간 있음: 게인을 낮추세요"; }
  bool slow = (recov - baseMed) > 0.2f * (delta > 0 ? delta : 1);
  if (slow && v == V_OK) { v = V_WARN; msg = "수축 후 값이 충분히 내려오지 않음: 이완이 덜 됐거나 ENV 응답이 느림, 전극 접촉 확인"; }

  r3 = v; r3Delta = delta; r3Snr = snr; r3Sat = satFrac;
  out.printf("\n증가량(수축 최고1초 - 이완 중앙값): %.1f mV   대비(SNR=증가량/이완 변동): %.1f   포화 샘플: %.1f%%\n", delta, snr, 100.0f * satFrac);
  out.printf("판정: %s  %s\n", vtxt(v), msg);
  out.printf("기준(임의 시작값): 증가량 %.0f mV 이상 & SNR %.0f 이상 OK, (%.0f mV, SNR %.0f) 이상 WARN\n", DELTA_OK, SNR_OK, DELTA_WEAK, SNR_WARN);

  // 타임라인: 0.5초 단위 최대 활동도 (시간은 100ms 창 개수 기준의 대략값)
  float lo = 1e9f, hi = -1e9f;
  for (int i = 0; i < g_tlN; i++) { if (g_tl[i] < lo) lo = g_tl[i]; if (g_tl[i] > hi) hi = g_tl[i]; }
  out.println("\n타임라인 (0.5초 단위 최대 활동도, 대략적 시간):");
  for (int i = 0; i < g_tlN; i += 5) {
    float m = g_tl[i];
    for (int k = i + 1; k < i + 5 && k < g_tlN; k++) if (g_tl[k] > m) m = g_tl[k];
    int bar = hi > lo ? (int)(30.0f * (m - lo) / (hi - lo)) : 0;
    char b[32]; for (int k = 0; k < 30; k++) b[k] = k < bar ? '#' : ' '; b[30] = 0;
    const char* tag = "";
    if (i == 0) tag = "  <- 이완 시작";
    if (iB >= i && iB < i + 5) tag = "  <- '지금!' 신호";
    if (iC >= i && iC < i + 5) tag = "  <- '힘 빼세요' 신호";
    out.printf("%5.1fs %7.0f |%s|%s\n", i * 0.1f, m, b, tag);
  }
  out.println("-> 막대가 높은 구간이 '지금!'~'힘 빼세요' 사이에 있어야 정상입니다.");

  if (delta > 0 && v != V_FAIL) {
    thrOn  = baseMed + 0.5f * delta;
    thrOff = baseMed + 0.3f * delta;
    haveThr = true;
    out.printf("자동 임계값: ON %.1f / OFF %.1f (이완 중앙값 + 증가량의 50%% / 30%%) -> [4]에서 사용\n", thrOn, thrOff);
  } else {
    haveThr = false;
  }
}

// ============================ [4] 트리거 데모 ============================

void testTrigger() {
  header("[4] 트리거 데모 (10초)");
  if (!haveThr) { out.println("먼저 [3] 수축 반응을 실행해 임계값을 계산하세요 (FAIL이면 센서부터 해결)."); return; }
  out.printf("임계값 ON %.1f / OFF %.1f (히스테리시스)\n", thrOn, thrOff);
  out.println("시작 후 첫 1초는 가만히 있다가, 10초 안에 주먹을 3번 쥐었다 펴세요 (쥐고 1초, 펴고 1초 정도).");
  if (!countdown(3)) return;
  out.println(">> 시작! (10초)");

  const uint32_t WIN_MS = 50;
  uint32_t startUs = micros(), next = startUs, totalUs = 10000000UL;
  ActMeter am; am.begin(startUs, WIN_MS);
  bool on = false; uint32_t onAtMs = 0; int count = 0, chatter = 0; float sumDur = 0;
  while ((uint32_t)(micros() - startUs) < totalUs) {
    while ((int32_t)(micros() - next) < 0) {}
    next += SAMPLE_US;
    float v = readMv(), act;
    if (!am.feed(v, micros(), act)) continue;
    if (pollAbort()) { out.println("중단됨"); return; }
    uint32_t tMs = (micros() - startUs) / 1000UL;
    if (!on && act >= thrOn) { on = true; onAtMs = tMs; count++; out.printf("  ON   t=%5.2fs  활동도 %.0f\n", tMs / 1000.0f, act); }
    else if (on && act <= thrOff) {
      on = false; float dur = (tMs - onAtMs) / 1000.0f; sumDur += dur;
      if (dur < 0.15f) chatter++;
      out.printf("  OFF  t=%5.2fs  (지속 %.2fs)\n", tMs / 1000.0f, dur);
    }
  }
  r4Count = count; r4Chatter = chatter;
  out.printf("\nON 횟수: %d (기대값 3)   0.15초 미만의 짧은 ON: %d회   평균 지속: %.2fs\n", count, chatter, count ? sumDur / count : 0);
  if (count == 3 && chatter == 0) out.println("깔끔하게 3번 감지됨.");
  else if (count > 3 || chatter > 0) out.println("기대보다 많이 감지됨: 임계값을 올리거나(상수 ON/OFF 간격 확대) 이완 기준선 노이즈를 확인하세요.");
  else out.println("기대보다 적게 감지됨: 임계값이 높거나 수축이 약했을 수 있음. [3]을 다시 실행해 보세요.");
}

// ============================ [5] 스트림 ============================

void runStream() {
  out.println("스트림 시작 (시리얼 플로터: act, mv). 중지: q");
  delay(300);
  uint32_t startUs = micros(), next = startUs;
  ActMeter am; am.begin(startUs, 20);
  while (!pollAbort()) {
    while ((int32_t)(micros() - next) < 0) {}
    next += SAMPLE_US;
    float v = readMv(), act;
    if (am.feed(v, micros(), act)) out.printf("act:%.1f,mv:%.0f\n", act, v);
  }
  out.println("스트림 종료");
}

// ============================ 요약/메뉴 ============================

void printReport() {
  header("요약 리포트 (실행한 테스트만 반영됨)");
  out.printf("[1] 전압 점검   : %-4s  (이완 평균 %.0f mV)\n", vtxt(r1), r1Mv);
  out.printf("[2] 이완 기준선 : %-4s  (활동도 평균 %.1f, 신호 p-p %.0f mV)\n", vtxt(r2), r2Act, r2Pp);
  out.printf("[3] 수축 반응   : %-4s  (증가량 %.1f mV, SNR %.1f, 포화 %.1f%%)\n", vtxt(r3), r3Delta, r3Snr, 100.0f * r3Sat);
  if (r4Count < 0) out.println("[4] 트리거 데모 : -");
  else out.printf("[4] 트리거 데모 : ON %d회 (기대 3), 짧은 ON %d회\n", r4Count, r4Chatter);
  out.println("'-'는 아직 실행하지 않은 테스트입니다. 기준 수치는 임의 시작값입니다.");
}

void printMenu() {
  out.println("");
  out.println("---------------- MyoWare 2.0 EMG 테스트 메뉴 ----------------");
  out.printf(" 출력 모드: %s   샘플링 %.0f Hz   핀 GPIO%u\n", EMG_OUTPUT_ENV ? "ENV" : "RAW", 1e6f / SAMPLE_US, EMG_PIN);
  out.println(" 1: 전압 점검        2: 이완 기준선 (10초)");
  out.println(" 3: 수축 반응        4: 트리거 데모");
  out.println(" 5: 스트림           a: 1,2,3,4 순서대로");
  out.println(" r: 요약 리포트      q: 진행 중 중단     h: 메뉴");
  out.println(" 안전: 전극을 붙인 채 USB(PC) 연결 금지. 측정은 배터리+블루투스로.");
  out.println("-------------------------------------------------------------");
}

void runAll() {
  testCheck();        if (g_abort) return;
  testBaseline();     if (g_abort) return;
  testContraction();  if (g_abort) return;
  testTrigger();      if (g_abort) return;
  printReport();
}

void setup() {
  Serial.begin(115200);
  delay(500);
  analogReadResolution(12);
  analogSetPinAttenuation(EMG_PIN, ADC_11db);   // 약 0~3.1V
#if USE_BT_SERIAL
  SerialBT.begin("FingerStone-EMG");
#endif
  out.println("\n== Finger Stone MyoWare 2.0 EMG 테스트 ==");
  out.println("!! 전극을 붙인 상태에서 PC USB 연결 금지 (README 8번). 측정은 배터리 + 블루투스로 !!");
#if USE_BT_SERIAL
  out.println("블루투스 이름: FingerStone-EMG (페어링 후 연결하면 메뉴가 다시 표시됩니다)");
#endif
  printMenu();
}

void loop() {
#if USE_BT_SERIAL
  static bool wasConn = false;
  bool conn = SerialBT.hasClient();
  if (conn && !wasConn) { delay(300); printMenu(); }
  wasConn = conn;
#endif
  if (!cmdAvailable()) return;
  int c = readCmdChar();
  if (c == '\r' || c == '\n' || c == ' ' || c < 0) return;
  g_abort = false;
  switch (c) {
    case '1': testCheck(); break;
    case '2': testBaseline(); break;
    case '3': testContraction(); break;
    case '4': testTrigger(); break;
    case '5': runStream(); break;
    case 'a': case 'A': runAll(); break;
    case 'r': case 'R': printReport(); break;
    case 'h': case 'H': printMenu(); break;
    case 'q': case 'Q': break;
    default: out.println("알 수 없는 명령 (h: 메뉴)"); break;
  }
  if (g_abort) out.println("[중단됨]");
}
