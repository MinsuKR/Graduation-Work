#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// === 핀 정의 ===
#define HX711_DT   PIN_PD3   // DOUT
#define HX711_SCK  PIN_PD2   // SCK
#define TARE_BTN   PIN_PE4   // 영점 버튼, GND에 연결

// === LCD (주소 확인 필요: 보통 0x27 또는 0x3F) ===
LiquidCrystal_I2C lcd(0x27, 16, 2);

// === HX711 객체 ===
HX711 scale;
long tare_offset = 0;  // 영점 오프셋

// === 영점 조정 함수 ===
void doTare(uint8_t avg = 15) {
  if (!scale.is_ready()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("TARE SKIPPED");
    lcd.setCursor(0, 1); lcd.print("HX711 NOT READY");
    delay(800);
    return;
  }
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("TARE...");
  scale.tare(avg);
  tare_offset = scale.get_offset();
  lcd.setCursor(0, 1);
  lcd.print("OK off="); lcd.print(tare_offset);
  delay(700);
}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms = 100) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scale.is_ready()) return true;
  }
  return false;
}

void setup() {
  pinMode(TARE_BTN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  scale.begin(HX711_DT, HX711_SCK);
  scale.set_gain(128);  // 기본 A채널 게인 명시

  // HX711 준비 대기 (3초)
  uint32_t t0 = millis();
  while (!scale.is_ready() && (millis() - t0 < 3000)) {}
  if (!scale.is_ready()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    delay(1500);
  }

  // 최초 영점 설정
  doTare();
}

void loop() {
  static uint32_t last = 0;

  // 200ms마다 데이터 갱신
  if (millis() - last >= 200) {
    last = millis();

    if (waitReady(80)) {  // 80ms 이내에 준비되면
      long raw = scale.read_average(5);  // 5회 평균
      long net = raw - tare_offset;

      char line1[17], line2[17];
      snprintf(line1, sizeof(line1), "NET:%-10ld", net);
      snprintf(line2, sizeof(line2), "RAW:%-10ld", raw);
      lcd.setCursor(0, 0); lcd.print(line1);
      lcd.setCursor(0, 1); lcd.print(line2);
    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

  // 버튼 눌렀을 때 TARE 실행
  if (digitalRead(TARE_BTN) == LOW) {
    delay(20);
    if (digitalRead(TARE_BTN) == LOW) {
      doTare();
      while (digitalRead(TARE_BTN) == LOW) {} // 뗄 때까지 대기
    }
  }
}
