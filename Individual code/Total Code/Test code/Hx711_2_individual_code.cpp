#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ==== LCD ====
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ==== HX711 인스턴스 ====
HX711 scaleL;   // 왼쪽 로드셀
HX711 scaleR;   // 오른쪽 로드셀

// ==== 핀 설정 ====
// 왼쪽 HX711
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2

// 오른쪽 HX711
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

// ==== 영점 오프셋 ====
long offsetL = 0;
long offsetR = 0;

// ==== 보정 계수 (실제 CAL) ====
float factorL = -0.005398f;
float factorR = -0.004933f;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("HX711 L/R Test");

  delay(600);

  // HX711 시작
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  delay(300);

  // ==== TARE (초기 영점) ====
  lcd.clear();
  lcd.setCursor(0,0); lcd.print("Tare...");
  offsetL = scaleL.read_average(20);
  offsetR = scaleR.read_average(20);
  lcd.setCursor(0,1); lcd.print("Done");
  delay(500);

  lcd.clear();
}

void loop() {
  // 각 로드셀의 RAW 평균값 10번 측정
  long rawL = scaleL.read_average(10);
  long rawR = scaleR.read_average(10);

  // net = raw - offset
  long netL = rawL - offsetL;
  long netR = rawR - offsetR;

  // 보정 후 무게
  float wL = netL * factorL;
  float wR = netR * factorR;

  // ==== LCD 출력 ====
  lcd.setCursor(0,0);
  lcd.print("L:");
  lcd.print(wL, 1);
  lcd.print("g      ");

  lcd.setCursor(0,1);
  lcd.print("R:");
  lcd.print(wR, 1);
  lcd.print("g      ");

  delay(150);
}
