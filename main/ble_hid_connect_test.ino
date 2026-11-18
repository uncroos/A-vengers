/*
 * Finger Stone - 1단계: BLE 키보드(HID)로 PC 연결 테스트 (ESP32-DevKitC, Arduino IDE)
 *
 * 목적: Flex 센서 코드를 붙이기 전에 "ESP32가 블루투스 키보드로 PC에 붙고, 키가 들어가는지"만 확인한다.
 *       이 스케치에는 Flex 센서 코드가 없다. (다음 단계에서 추가)
 *
 * 필요한 라이브러리: BleKeyboard API를 제공하는 라이브러리 (#include <BleKeyboard.h>)
 *   - 이 API(BleKeyboard, isConnected, write, print, KEY_PAGE_DOWN ...)는 T-vK/ESP32-BLE-Keyboard와
 *     그 NimBLE 포크들이 공통으로 쓴다. 어떤 것을 설치할지는 arduino-esp32 코어 버전에 따라 다름
 *     (설치 방법은 채팅 안내 참고). 스케치 본문은 라이브러리가 달라도 그대로 쓴다.
 *
 * 사용법
 *   1) 업로드 후 시리얼 모니터 115200 baud
 *   2) PC의 블루투스 설정에서 "Finger Stone" 을 찾아 페어링 (PC 쪽에서 '기기 추가' -> Bluetooth)
 *   3) 시리얼 모니터에 [BLE] 연결됨 이 뜨면 성공
 *   4) 메모장 등 텍스트 창을 클릭해 두고 시리얼로 명령 전송:
 *        n = Page Down(다음)   p = Page Up(이전)   t = 문자열 "FingerStone OK" 입력
 *        s = 상태 출력         h = 도움말
 *      또는 보드의 BOOT 버튼을 짧게 누르면 Page Down 전송 (시리얼 없이도 테스트 가능)
 *   5) PowerPoint 슬라이드쇼를 켜 두고 n / p 로 슬라이드가 넘어가는지 확인
 *
 * 주의
 *   - BOOT 버튼(GPIO0)은 스트랩 핀이므로 부팅이 끝난 뒤 입력으로만 읽는다. 외부 회로를 연결하지 말 것.
 *   - Arduino IDE는 스케치 폴더 이름과 .ino 파일 이름이 같아야 한다 (ble_hid_connect_test/ble_hid_connect_test.ino).
 */

#include <Arduino.h>
#include <BleKeyboard.h>

// ============================ 설정 ============================

// PC 블루투스 목록에 보이는 이름 (이름, 제조사, 초기 배터리 %)
BleKeyboard bleKeyboard("Finger Stone", "A+vengers", 100);

const uint8_t  BTN_PIN         = 0;     // 보드의 BOOT 버튼
const uint32_t BTN_DEBOUNCE_MS = 40;
const uint32_t STATUS_EVERY_MS = 5000;  // 연결 상태 주기 출력 (0이면 끔)

// ============================ 상태 ============================

bool     wasConnected = false;
uint32_t lastStatusMs = 0;

// ============================ 동작 ============================

void sendNext() {
  if (!bleKeyboard.isConnected()) { Serial.println("[KEY] 연결 안 됨: PC에서 먼저 페어링하세요"); return; }
  bleKeyboard.write(KEY_PAGE_DOWN);
  Serial.println("[KEY] Page Down (다음)");
}

void sendPrev() {
  if (!bleKeyboard.isConnected()) { Serial.println("[KEY] 연결 안 됨: PC에서 먼저 페어링하세요"); return; }
  bleKeyboard.write(KEY_PAGE_UP);
  Serial.println("[KEY] Page Up (이전)");
}

void sendText() {
  if (!bleKeyboard.isConnected()) { Serial.println("[KEY] 연결 안 됨: PC에서 먼저 페어링하세요"); return; }
  bleKeyboard.print("FingerStone OK");
  Serial.println("[KEY] 문자열 전송");
}

void printStatus() {
  Serial.printf("[BLE] %s\n", bleKeyboard.isConnected() ? "연결됨" : "연결 대기 중 (PC 블루투스에서 'Finger Stone' 선택)");
}

void printHelp() {
  Serial.println("명령: n=Page Down  p=Page Up  t=문자열 입력  s=상태  h=도움말  / BOOT 버튼=Page Down");
}

// ============================ 입력 처리 ============================

void handleSerial() {
  while (Serial.available()) {
    char c = Serial.read();
    switch (c) {
      case 'n': sendNext(); break;
      case 'p': sendPrev(); break;
      case 't': sendText(); break;
      case 's': printStatus(); break;
      case 'h': printHelp(); break;
      default: break;   // 줄바꿈 등은 무시
    }
  }
}

void handleButton() {
  static bool     lastLevel = HIGH;
  static uint32_t changedAt = 0;
  bool level = digitalRead(BTN_PIN);
  if (level != lastLevel && millis() - changedAt > BTN_DEBOUNCE_MS) {
    changedAt = millis();
    lastLevel = level;
    if (level == LOW) sendNext();   // 누르는 순간 1회 전송
  }
}

void handleConnectionChange() {
  bool now = bleKeyboard.isConnected();
  if (now != wasConnected) {
    wasConnected = now;
    Serial.println(now ? "[BLE] 연결됨" : "[BLE] 연결 끊김 (자동으로 다시 광고 중)");
  }
  if (STATUS_EVERY_MS && millis() - lastStatusMs >= STATUS_EVERY_MS) {
    lastStatusMs = millis();
    if (!now) printStatus();
  }
}

// ============================ setup / loop ============================

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n== Finger Stone BLE HID 연결 테스트 ==");

  pinMode(BTN_PIN, INPUT_PULLUP);

  bleKeyboard.begin();   // 광고 시작: PC에서 "Finger Stone" 검색 가능
  Serial.println("[BLE] 광고 시작. PC 블루투스 설정에서 'Finger Stone'을 페어링하세요.");
  printHelp();
}

void loop() {
  handleSerial();
  handleButton();
  handleConnectionChange();
  delay(5);
}
