// Finger Stone - MPU6050 연결/값 확인 테스트 (라이브러리 불필요, Wire만 사용)
// 배선: PCB J12 -> 3V3=VCC, SCL(22)=SCL, SDA(21)=SDA, GND=GND (AD0 미연결 -> 주소 0x68)
// 시리얼 모니터 115200, 줄바꿈 "Newline"
//
// 명령: s 스캔 | z 자이로 영점 | p 플로터 출력 토글 | i 재초기화 | h 도움말
#include <Wire.h>

const int PIN_SDA = 21;
const int PIN_SCL = 22;
const uint32_t I2C_HZ = 400000;
const uint8_t REG_WHO = 0x75, REG_PWR = 0x6B, REG_CFG = 0x1A,
              REG_GYRO_CFG = 0x1B, REG_ACC_CFG = 0x1C, REG_DATA = 0x3B;

uint8_t addr = 0x68;
bool ready = false;
bool plotter = false;
uint32_t errCount = 0;

// 감도: 가속도 ±4g -> 8192 LSB/g, 자이로 ±500dps -> 65.5 LSB/dps
const float ACC_LSB = 8192.0f, GYR_LSB = 65.5f;
float gbx = 0, gby = 0, gbz = 0;      // 자이로 오프셋(dps)
float roll = 0, pitch = 0;            // 상보필터 결과(deg)
uint32_t lastUs = 0, lastPrint = 0;
float peakGyro = 0, peakAcc = 0;      // 최근 구간 최대값 (스냅 감지 참고용)

bool writeReg(uint8_t reg, uint8_t val) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  Wire.write(val);
  return Wire.endTransmission() == 0;
}

bool readRegs(uint8_t reg, uint8_t *buf, uint8_t n) {
  Wire.beginTransmission(addr);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return false;
  if (Wire.requestFrom((int)addr, (int)n) != n) return false;
  for (uint8_t i = 0; i < n; i++) buf[i] = Wire.read();
  return true;
}

void scanBus() {
  Serial.println(F("[SCAN] I2C 스캔 중..."));
  int found = 0;
  for (uint8_t a = 1; a < 127; a++) {
    Wire.beginTransmission(a);
    if (Wire.endTransmission() == 0) {
      Serial.printf("[SCAN] 장치 발견: 0x%02X\n", a);
      found++;
    }
  }
  if (!found) {
    Serial.println(F("[SCAN] 장치 없음 -> 배선/도통/전원 확인 (SDA=21, SCL=22, VCC=3V3, GND)"));
  }
}

bool initMpu() {
  ready = false;
  // 0x68 / 0x69 순서로 찾기
  const uint8_t cand[2] = {0x68, 0x69};
  bool got = false;
  for (uint8_t c : cand) {
    addr = c;
    Wire.beginTransmission(addr);
    if (Wire.endTransmission() == 0) { got = true; break; }
  }
  if (!got) {
    Serial.println(F("[MPU] 0x68/0x69 응답 없음. 's'로 스캔해 보세요."));
    return false;
  }
  uint8_t who = 0;
  if (!readRegs(REG_WHO, &who, 1)) {
    Serial.println(F("[MPU] WHO_AM_I 읽기 실패"));
    return false;
  }
  Serial.printf("[MPU] 주소 0x%02X, WHO_AM_I = 0x%02X ", addr, who);
  if (who == 0x68) Serial.println(F("(정상 MPU-6050)"));
  else Serial.println(F("(예상 0x68과 다름: 호환칩/클론일 수 있음, 값이 나오면 계속 사용 가능)"));

  bool ok = writeReg(REG_PWR, 0x01);   // sleep 해제, 클럭 = 자이로 X PLL
  delay(50);
  ok &= writeReg(REG_CFG, 0x03);       // 디지털 저역필터 ~44Hz
  ok &= writeReg(REG_GYRO_CFG, 0x08);  // ±500 dps
  ok &= writeReg(REG_ACC_CFG, 0x08);   // ±4 g
  if (!ok) {
    Serial.println(F("[MPU] 설정 쓰기 실패"));
    return false;
  }
  ready = true;
  Serial.println(F("[MPU] 초기화 완료 (가속도 ±4g, 자이로 ±500dps)"));
  return true;
}

struct Sample { float ax, ay, az, gx, gy, gz, tc; };

bool readSample(Sample &s) {
  uint8_t b[14];
  if (!readRegs(REG_DATA, b, 14)) return false;
  auto v = [&](int i) { return (int16_t)((b[i] << 8) | b[i + 1]); };
  s.ax = v(0) / ACC_LSB;
  s.ay = v(2) / ACC_LSB;
  s.az = v(4) / ACC_LSB;
  s.tc = v(6) / 340.0f + 36.53f;
  s.gx = v(8) / GYR_LSB - gbx;
  s.gy = v(10) / GYR_LSB - gby;
  s.gz = v(12) / GYR_LSB - gbz;
  return true;
}

void calibrateGyro() {
  Serial.println(F("[CAL] 자이로 영점: 약 1초간 손을 가만히 두세요..."));
  float sx = 0, sy = 0, sz = 0;
  gbx = gby = gbz = 0;
  int n = 0;
  uint32_t t0 = millis();
  while (millis() - t0 < 1000) {
    Sample s;
    if (readSample(s)) { sx += s.gx; sy += s.gy; sz += s.gz; n++; }
    delay(5);
  }
  if (n < 20) {
    Serial.println(F("[CAL] 샘플 부족, 실패"));
    return;
  }
  gbx = sx / n; gby = sy / n; gbz = sz / n;
  Serial.printf("[CAL] 오프셋 gx=%.2f gy=%.2f gz=%.2f dps (샘플 %d개)\n", gbx, gby, gbz, n);
}

void help() {
  Serial.println(F("명령: s 스캔 | z 자이로 영점 | p 플로터 출력 | i 재초기화 | h 도움말"));
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println(F("\n=== MPU6050 테스트 ==="));
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(I2C_HZ);
  Wire.setTimeOut(50);
  scanBus();
  if (initMpu()) calibrateGyro();
  help();
  lastUs = micros();
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == 's') scanBus();
    else if (c == 'z') { if (ready) calibrateGyro(); else Serial.println(F("[MPU] 먼저 초기화 필요")); }
    else if (c == 'p') { plotter = !plotter; Serial.printf("[OUT] 플로터 %s\n", plotter ? "ON" : "OFF"); }
    else if (c == 'i') { if (initMpu()) calibrateGyro(); }
    else if (c == 'h') help();
  }

  if (!ready) {
    // 연결 대기: 2초마다 재시도
    static uint32_t lastTry = 0;
    if (millis() - lastTry > 2000) {
      lastTry = millis();
      if (initMpu()) calibrateGyro();
    }
    return;
  }

  Sample s;
  if (!readSample(s)) {
    errCount++;
    Serial.printf("[ERR] 읽기 실패 (누적 %lu회) -> 재초기화 시도\n", (unsigned long)errCount);
    ready = false;
    return;
  }

  // 상보필터로 기울기(roll/pitch) 추정
  uint32_t nowUs = micros();
  float dt = (nowUs - lastUs) * 1e-6f;
  lastUs = nowUs;
  if (dt <= 0 || dt > 0.5f) dt = 0.01f;
  float accRoll = atan2f(s.ay, s.az) * 57.29578f;
  float accPitch = atan2f(-s.ax, sqrtf(s.ay * s.ay + s.az * s.az)) * 57.29578f;
  roll = 0.98f * (roll + s.gx * dt) + 0.02f * accRoll;
  pitch = 0.98f * (pitch + s.gy * dt) + 0.02f * accPitch;

  float gmag = sqrtf(s.gx * s.gx + s.gy * s.gy + s.gz * s.gz);
  float amag = sqrtf(s.ax * s.ax + s.ay * s.ay + s.az * s.az);
  if (gmag > peakGyro) peakGyro = gmag;
  float dAcc = fabsf(amag - 1.0f);
  if (dAcc > peakAcc) peakAcc = dAcc;

  if (millis() - lastPrint >= 50) {   // 20Hz 출력
    lastPrint = millis();
    if (plotter) {
      Serial.printf("gx:%.1f,gy:%.1f,gz:%.1f,ax:%.2f,ay:%.2f,az:%.2f\n",
                    s.gx, s.gy, s.gz, s.ax, s.ay, s.az);
    } else {
      Serial.printf("acc[g] %6.2f %6.2f %6.2f |A|=%.2f | gyro[dps] %7.1f %7.1f %7.1f |G|=%6.1f | roll %6.1f pitch %6.1f | %.1fC\n",
                    s.ax, s.ay, s.az, amag, s.gx, s.gy, s.gz, gmag, roll, pitch, s.tc);
    }
  }
  // 1초마다 구간 최대값 출력 (스냅 임계값 정할 때 참고)
  static uint32_t lastPeak = 0;
  if (!plotter && millis() - lastPeak >= 1000) {
    lastPeak = millis();
    Serial.printf("[PEAK] 최근 1초 최대 |G|=%.0f dps, |가속도-1g|=%.2f g\n", peakGyro, peakAcc);
    peakGyro = 0; peakAcc = 0;
  }
  delay(2);
}
