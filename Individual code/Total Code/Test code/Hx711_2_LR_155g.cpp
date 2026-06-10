#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ==== LCD ====
LiquidCrystal_I2C lcd(0x27, 16, 2); // 주소 다르면 0x3F 등으로 변경

// ==== HX711 인스턴스 2개 ====
HX711 scaleL;   // 왼쪽 로드셀
HX711 scaleR;   // 오른쪽 로드셀

// ==== 핀 설정 ====
// 왼쪽 HX711
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2

// 오른쪽 HX711
#define DT_R   PIN_PD5   // DOUT
#define SCK_R  PIN_PD4   // SCK

// ==== 보정 계수 (초기값은 대충, 부팅 시 155g로 다시 계산) ====
float factorL = -0.005398f;
float factorR = -0.004933f;

// ==== 영점 오프셋 ====
long offsetL = 0;
long offsetR = 0;

// ==== 캘리브용 기준 무게 ====
const float CAL_WEIGHT = 155.0f;

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("HX711 Dual Test");
  lcd.setCursor(0, 1);
  lcd.print("Init...");

  // HX711 시작
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  delay(500);

  // --- 1) 양쪽 로드셀 TARE (빈 상태에서) ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Tare L/R...");

  offsetL = scaleL.read_average(20);  // 왼쪽 평균
  offsetR = scaleR.read_average(20);  // 오른쪽 평균

  lcd.setCursor(0, 1);
  lcd.print("Done");
  delay(800);

  // --- 2) 왼쪽 155g 캘리브 ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Put 155g on L");
  lcd.setCursor(0, 1);
  lcd.print("Wait...");

  delay(3000); // 올릴 시간

  {
    long sum = 0;
    const int N = 30;
    for (int i = 0; i < N; i++) {
      long raw = scaleL.read();
      sum += raw;
      delay(30);
    }
    long avgRaw = sum / N;
    long net = avgRaw - offsetL;

    if (net != 0) {
      factorL = CAL_WEIGHT / (float)net;  // 부호 포함해서 자동 계산
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("L Cal Done");
    lcd.setCursor(0, 1);
    lcd.print("fL=");
    lcd.print(factorL, 6);
    delay(1500);
  }

  // --- 3) 오른쪽 155g 캘리브 ---
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Put 155g on R");
  lcd.setCursor(0, 1);
  lcd.print("Wait...");

  delay(3000); // 옮길 시간

  {
    long sum = 0;
    const int N = 30;
    for (int i = 0; i < N; i++) {
      long raw = scaleR.read();
      sum += raw;
      delay(30);
    }
    long avgRaw = sum / N;
    long net = avgRaw - offsetR;

    if (net != 0) {
      factorR = CAL_WEIGHT / (float)net;
    }

    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("R Cal Done");
    lcd.setCursor(0, 1);
    lcd.print("fR=");
    lcd.print(factorR, 6);
    delay(1500);
  }

  // --- 측정 모드 진입 ---
  lcd.clear();
}

void loop() {
  // 각각 여러 번 평균 내기 (노이즈 줄이기)
  long rawL = scaleL.read_average(10);
  long rawR = scaleR.read_average(10);

  long netL = rawL - offsetL;
  long netR = rawR - offsetR;

  float wL = (float)netL * factorL;
  float wR = (float)netR * factorR;

  float Wtotal = wL + wR;  // ★ 전체 무게

  // ===== LCD 출력 =====
  // 1줄: 왼쪽 / 오른쪽
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(wL, 1);
  lcd.print("g  ");

  lcd.print("R:");
  lcd.print(wR, 1);
  lcd.print("g ");

  // 2줄: 합계
  lcd.setCursor(0, 1);
  lcd.print("TOT:");
  lcd.print(Wtotal, 1);
  lcd.print("g    ");   // 남은 자리 지우기용 공백

  delay(150); // 150ms마다 갱신
}
