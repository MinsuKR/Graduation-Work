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

// ==== 로드셀 기본 보정 계수 (이미 맞춰둔 값 사용) ====
float factorL = -0.005398f;
float factorR = -0.004933f;

// ==== 영점 오프셋 ====
long offsetL = 0;
long offsetR = 0;

// ==== 총무게 보정용 계수 ====
float a_coef = 1.0f;   // Wtotal = a*L + b*R
float b_coef = 1.0f;

// ==== 캘리브용 기준 무게 ====
const float CAL_WEIGHT = 622.0f;

// ==== 평균 측정 함수 ====
void measureLR(int samples, float &outL, float &outR) {
  long sumL = 0;
  long sumR = 0;

  for (int i = 0; i < samples; i++) {
    long rawL = scaleL.read();
    long rawR = scaleR.read();
    sumL += rawL;
    sumR += rawR;
    delay(30);
  }

  long avgRawL = sumL / samples;
  long avgRawR = sumR / samples;

  long netL = avgRawL - offsetL;
  long netR = avgRawR - offsetR;

  outL = (float)netL * factorL;
  outR = (float)netR * factorR;
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 Dual a,b");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 시작
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);
  delay(500);

  // --- 1) TARE (빈 상태에서) ---
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Tare ....");
  offsetL = scaleL.read_average(20);
  offsetR = scaleR.read_average(20);
  lcd.setCursor(0, 1); lcd.print("Done");
  delay(800);

  // --- 2) 왼쪽 끝에 155g 올려서 측정 ---
  float L_left = 0.0f, R_left = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g LEFT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 올릴 시간

  measureLR(30, L_left, R_left);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Lpos L=");
  lcd.print(L_left, 1);
  lcd.setCursor(0, 1); lcd.print("Lpos R=");
  lcd.print(R_left, 1);
  delay(1500);

  // --- 3) 오른쪽 끝에 155g 올려서 측정 ---
  float L_right = 0.0f, R_right = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g RIGHT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 옮길 시간

  measureLR(30, L_right, R_right);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Rpos L=");
  lcd.print(L_right, 1);
  lcd.setCursor(0, 1); lcd.print("Rpos R=");
  lcd.print(R_right, 1);
  delay(1500);

  // --- 4) a,b 계산 ---
  //  CAL_WEIGHT = a*L_left  + b*R_left
  //  CAL_WEIGHT = a*L_right + b*R_right
  float det = (L_left * R_right) - (L_right * R_left);

  if (fabs(det) < 1e-6f) {
    // 특이 케이스: 거의 대칭이라면 일단 a=b=1로 사용
    a_coef = 1.0f;
    b_coef = 1.0f;
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("det~0, use 1,1");
  } else {
    a_coef = CAL_WEIGHT * (R_right - R_left) / det;
    b_coef = CAL_WEIGHT * (L_left  - L_right) / det;

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("a=");
    lcd.print(a_coef, 4);
    lcd.setCursor(0,1); lcd.print("b=");
    lcd.print(b_coef, 4);
  }

  delay(2000);
  lcd.clear();
}

void loop() {
  // 한 번 읽어서 무게 계산
  long rawL = scaleL.read_average(20);
  long rawR = scaleR.read_average(20);

  long netL = rawL - offsetL;
  long netR = rawR - offsetR;

  float wL = (float)netL * factorL;
  float wR = (float)netR * factorR;

  // 위치 보정 포함한 총무게
  float Wtotal = a_coef * wL + b_coef * wR;

  // ===== LCD 출력 =====
  lcd.setCursor(0, 0);
  lcd.print("L:");
  lcd.print(wL, 1);
  lcd.print("g ");
  lcd.print("R:");
  lcd.print(wR, 1);
  lcd.print("g");

  lcd.setCursor(0, 1);
  lcd.print("TOT:");
  lcd.print(Wtotal, 1);
  lcd.print("g    ");

  delay(150);
}
