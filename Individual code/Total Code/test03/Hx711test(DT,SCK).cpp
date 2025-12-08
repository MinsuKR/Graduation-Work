#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define HX711_DT   PIN_PD3   // DOUT
#define HX711_SCK  PIN_PD2   // SCK

LiquidCrystal_I2C lcd(0x27, 16, 2); // I2C LCD 주소가 다르면 0x3F 등으로 변경

void setup() {
  lcd.init();
  lcd.backlight();

  pinMode(HX711_DT, INPUT);
  pinMode(HX711_SCK, OUTPUT);
  digitalWrite(HX711_SCK, LOW);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HX711 Test Start");

  delay(1000); // 전원 안정화 대기
}

void loop() {
  // 1. 초기 상태 읽기
  bool dt_before = digitalRead(HX711_DT);
  bool sck_before = digitalRead(HX711_SCK);

  // LCD 표시
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("DT:");
  lcd.print(dt_before ? "H " : "L ");
  lcd.print("SCK:");
  lcd.print(sck_before ? "H" : "L");
  lcd.setCursor(0, 1);
  lcd.print("Toggling SCK...");

  delay(1000);

  // 2. SCK를 27번 토글
  for (int i = 0; i < 27; i++) {
    digitalWrite(HX711_SCK, HIGH);
    delayMicroseconds(5);
    digitalWrite(HX711_SCK, LOW);
    delayMicroseconds(5);
  }

  // 3. 토글 후 상태 읽기
  bool dt_after = digitalRead(HX711_DT);
  bool sck_after = digitalRead(HX711_SCK);

  // LCD에 결과 표시
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Bef: DT=");
  lcd.print(dt_before ? "H " : "L ");
  lcd.print("SCK=");
  lcd.print(sck_before ? "H" : "L");

  lcd.setCursor(0, 1);
  lcd.print("Af : DT=");
  lcd.print(dt_after ? "H " : "L ");
  lcd.print("SCK=");
  lcd.print(sck_after ? "H" : "L");

  delay(2000); // 2초 후 다시 루프
}
