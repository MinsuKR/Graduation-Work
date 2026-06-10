#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== LCD =====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ===== 버튼 핀 =====
#define BTN_TARE_PIN PIN_PE4
#define BTN_CAL_PIN  PIN_PE5
#define BTN_SEND_PIN PIN_PE6

// ===== 사용 방식 선택 =====
// 외부 풀업 저항 사용 중이면 1
// 내부 풀업만 사용할 거면 0
#define USE_EXTERNAL_PULLUP 1

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();
  lcd.clear();

#if USE_EXTERNAL_PULLUP
  pinMode(BTN_TARE_PIN, INPUT);
  pinMode(BTN_CAL_PIN, INPUT);
  pinMode(BTN_SEND_PIN, INPUT);
#else
  pinMode(BTN_TARE_PIN, INPUT_PULLUP);
  pinMode(BTN_CAL_PIN, INPUT_PULLUP);
  pinMode(BTN_SEND_PIN, INPUT_PULLUP);
#endif

  lcd.setCursor(0, 0);
  lcd.print("Button Test");
  delay(1000);
  lcd.clear();
}

void loop() {
  int t = digitalRead(BTN_TARE_PIN);
  int c = digitalRead(BTN_CAL_PIN);
  int s = digitalRead(BTN_SEND_PIN);

  lcd.setCursor(0, 0);
  lcd.print("T:");
  lcd.print(t ? "H" : "L");
  lcd.print(" C:");
  lcd.print(c ? "H" : "L");
  lcd.print(" S:");
  lcd.print(s ? "H" : "L");
  lcd.print(" ");

  lcd.setCursor(0, 1);

#if USE_EXTERNAL_PULLUP
  lcd.print("EXT_PULLUP MODE ");
#else
  lcd.print("INT_PULLUP MODE ");
#endif

  delay(100);
}