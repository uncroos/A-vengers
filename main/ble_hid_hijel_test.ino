/*
 * Finger Stone - 1단계: BLE 키보드(HID)로 PC 연결 테스트 (최신 코어용)
 * 환경: ESP32-DevKitC, arduino-esp32 코어 3.3.x(정식 최신), HijelHID_BLEKeyboard + NimBLE-Arduino 2.3.8 이상
 *
 * PC에는 아무것도 설치하지 않는다. PC는 이 장치를 일반 블루투스 키보드로 인식한다.
 *
 * 설치 (Arduino IDE)
 *   1) 이전에 zip으로 넣은 T-vK ESP32-BLE-Keyboard 라이브러리는 폴더에서 삭제 (혼동 방지)
 *   2) 라이브러리 매니저에서 "HijelHID_BLEKeyboard" 검색 후 설치 (NimBLE-Arduino 의존성도 같이 설치하는지 확인.
 *      안 깔렸으면 "NimBLE-Arduino"를 직접 설치, 버전 2.3.8 이상)
 *   3) 도구 메뉴에서 "Erase All Flash Before Sketch Upload"를 Enabled로 하고 한 번 업로드
 *      (ESP32에 남아 있을 수 있는 이전 페어링 기록을 지운다. 성공 후에는 Disabled로 되돌려도 됨)
 *   4) PC/노트북 블루투스 목록에 이전에 만든 "Finger Stone" 등 예전 기기가 남아 있으면 '장치 제거'
 *
 * 사용법
 *   1) 업로드 후 시리얼 모니터 115200 baud
 *   2) PC 블루투스 설정 > 기기 추가 > Bluetooth 에서 새로 나타나는 장치를 페어링
 *      (이름은 라이브러리 기본값. 목록에 안 보이면 기존 항목을 제거하고 ESP32 EN 버튼으로 재시작)
 *   3) 시리얼에 [BLE] 페어링됨 이 뜨면 성공
 *   4) 메모장 등 텍스트 창을 클릭해 두고 시리얼로 명령 전송:
 *        t = "ESP32" 입력      n = Space (PowerPoint 슬라이드쇼에서는 다음 슬라이드)
 *        r = Enter             s = 상태 출력
 *      또는 BOOT 버튼을 누르면 Space 전송
 *
 * 이 스케치는 라이브러리 문서에서 확인된 API만 쓴다: begin(), isPaired(), tap(), press(), release(),
 *   KEY_SPACE / KEY_RETURN / KEY_LSHIFT / KEY_E / KEY_S / KEY_P / KEY_3 / KEY_2
 *   Page Down 등 다른 키 상수는 라이브러리 src/BLEHIDKeys.h 에서 이름을 확인한 뒤 추가한다.
 *
 * 주의
 *   - BOOT 버튼(GPIO0)은 스트랩 핀이므로 부팅이 끝난 뒤 입력으로만 읽는다. 외부 회로를 연결하지 말 것.
 *   - 스케치 폴더 이름과 .ino 파일 이름이 같아야 한다.
 */

#include <Arduino.h>
#include <HijelHID_BLEKeyboard.h>

HijelHID_BLEKeyboard keyboard;

const uint8_t  BTN_PIN         = 0;     // 보드의 BOOT 버튼
const uint32_t BTN_DEBOUNCE_MS = 40;
const uint32_t STATUS_EVERY_MS = 5000;  // 미연결 상태 안내 주기

bool     wasPaired = false;
uint32_t lastStatusMs = 0;

bool ready() {
  if (!keyboard.isPaired()) { Serial.println("[KEY] 페어링 안 됨: PC에서 먼저 페어링하세요"); return false; }
  return true;
}

void sendNext() {
  if (!ready()) return;
  keyboard.tap(KEY_SPACE);
  Serial.println("[KEY] Space");
}

void sendEnter() {
  if (!ready()) return;
  keyboard.tap(KEY_RETURN);
  Serial.println("[KEY] Enter");
}

void sendText() {
  if (!ready()) return;
  keyboard.press(KEY_LSHIFT);
  keyboard.tap(KEY_E);
  keyboard.tap(KEY_S);
  keyboard.tap(KEY_P);
  keyboard.release(KEY_LSHIFT);
  keyboard.tap(KEY_3);
  keyboard.tap(KEY_2);
  Serial.println("[KEY] ESP32 입력");
}

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 'n') sendNext();
    else if (c == 'r') sendEnter();
    else if (c == 't') sendText();
    else if (c == 's') Serial.printf("[BLE] %s\n", keyboard.isPaired() ? "페어링됨" : "대기 중");
  }
}

void handleButton() {
  static bool     lastLevel = HIGH;
  static uint32_t changedAt = 0;
  bool level = digitalRead(BTN_PIN);
  if (level != lastLevel && millis() - changedAt > BTN_DEBOUNCE_MS) {
    changedAt = millis();
    lastLevel = level;
    if (level == LOW) sendNext();
  }
}

void handleConnectionChange() {
  bool now = keyboard.isPaired();
  if (now != wasPaired) {
    wasPaired = now;
    Serial.println(now ? "[BLE] 페어링됨" : "[BLE] 연결 끊김");
  }
  if (!now && millis() - lastStatusMs >= STATUS_EVERY_MS) {
    lastStatusMs = millis();
    Serial.println("[BLE] 대기 중: PC 블루투스 설정에서 새 장치를 페어링하세요");
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n== Finger Stone BLE HID 연결 테스트 (Hijel) ==");
  pinMode(BTN_PIN, INPUT_PULLUP);
  keyboard.begin();
  Serial.println("명령: t=ESP32 입력  n=Space  r=Enter  s=상태  / BOOT 버튼=Space");
}

void loop() {
  handleSerial();
  handleButton();
  handleConnectionChange();
  delay(10);
}
