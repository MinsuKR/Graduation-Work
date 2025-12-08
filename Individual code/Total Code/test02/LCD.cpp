/* Test complete */
// SDA = PD1 , SCL = PD0
#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ===== HX711 핀 (ATmega128 / MegaCore) =====
#define HX711_DT   PIN_PD3   // HX711 DOUT -> PD3
#define HX711_SCK  PIN_PD2   // HX711 SCK  -> PD2

// ===== TARE 버튼 =====
#define TARE_BTN   PIN_PE4   // 버튼 -> PE4, 다른 쪽은 GND

// ===== LCD (0x27가 흔함. 다르면 0x3F 등으로 변경) =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

HX711 scale;
long tare_offset = 0;

// 간단 영점 함수
void doTare(uint8_t avg = 15) {
  if (!scale.is_ready()) {
    // LCD에 안내
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
  lcd.setCursor(0, 1); lcd.print("OK off=");
  lcd.print(tare_offset);
  delay(700);
}

void setup() {
  // 버튼 내부 풀업
  pinMode(TARE_BTN, INPUT_PULLUP);

  // LCD 초기화
  lcd.init();        // 또는 lcd.begin();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 시작
  scale.begin(HX711_DT, HX711_SCK);

  // 준비 대기 (최대 3초)
  uint32_t t0 = millis();
  while (!scale.is_ready() && (millis() - t0 < 3000)) {}

  if (!scale.is_ready()) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check wiring");
    // 계속 진행은 하되, 읽기 때마다 not ready 표시됨
    delay(1200);
  }

  // 최초 영점
  doTare();
}

void loop() {
  static uint32_t last = 0;
  static uint32_t btnLastMs = 0;
  static bool btnPrev = HIGH;

  // 200ms마다 측정/표시
  if (millis() - last >= 200) {
    last = millis();

    if (scale.is_ready()) {
      long raw = scale.read_average(5); // 원시값
      long net = raw - tare_offset;     // 영점 보정

      // LCD에 두 줄 표시
      // 1행: NET(순카운트)   2행: RAW(원시)
      char line1[17];
      char line2[17];
      snprintf(line1, sizeof(line1), "NET:%-10ld", net);
      snprintf(line2, sizeof(line2), "RAW:%-10ld", raw);

      lcd.setCursor(0, 0); lcd.print(line1);
      lcd.setCursor(0, 1); lcd.print(line2);
    } else {
      lcd.setCursor(0, 0); lcd.print("HX711 NOT READY ");
      lcd.setCursor(0, 1); lcd.print("Check VCC/DT/SCK");
    }
  }

  // 버튼(PE4) 눌림 감지 → 길게/짧게 구분 없이 영점
  bool btnNow = digitalRead(TARE_BTN); // 풀업: 눌리면 LOW
  if (btnPrev == HIGH && btnNow == LOW) {
    btnLastMs = millis(); // 눌린 시점
  }
  if (btnPrev == LOW && btnNow == HIGH) {
    // 뗀 시점, 디바운스(누른 시간 30ms 이상이면 유효)
    if (millis() - btnLastMs > 30) {
      doTare();
    }
  }
  btnPrev = btnNow;
}
