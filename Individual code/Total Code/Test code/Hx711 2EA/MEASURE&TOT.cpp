// 첫번째 코드
// 두 개의 로드셀을 각각의 독립된 저울로 보고 보정하는 방식
// 왼쪽 오른쪽 센서 각각 Factor구하는 두 로드셀이 물리적으로 완전히 분리되어 있거나, 상호 간섭이 적을 때 사용하는 표준적인 보정 방식

// 두번째 코드
// 하나의 판을 두 센서가 받치는 구조'에서 발생하는 간섭(Cross-talk) 현상을 해결 방식
// 왼쪽에 물체를 놓아도 오른쪽 센서에 무게가 전달되는 현상을 수학적으로
// 두 번의 측정 데이터($nL\_left, nR\_left$ 및 $nL\_right, nR\_right$)를 수집하여 연립 방정식을 풀어 두 계수를 동시에 산출
// 하나의 상판을 공유하는 듀얼 로드셀 시스템에서는 하중의 분산으로 인해 센서 간 상호 간섭이 발생
// 이를 해결하기 위해 선형 연립 방정식 모델을 도입하였고, 행렬 연산을 통해 물체의 위치에 관계없이 일관된 전체 무게(TOTAL)를 산출하도록 구현

// <각 왼쪽 오른쪽 155g 나옴>
// #include <Arduino.h>
// #include <Wire.h>
// #include <HX711.h>
// #include <LiquidCrystal_I2C.h>

// // ATmega128 전용 핀 정의
// #define DT_L   PIN_PD3
// #define SCK_L  PIN_PD2
// #define DT_R   PIN_PD5
// #define SCK_R  PIN_PD4

// HX711 scaleL, scaleR;
// LiquidCrystal_I2C lcd(0x27, 16, 2);

// // 전역 변수로 관리 (EEPROM 없이 실시간으로 계산하여 전달)
// long offsetL = 0, offsetR = 0;
// float factorL = 1.0f, factorR = 1.0f;
// const float CAL_WEIGHT = 155.0f; // 기준 무게 155g

// void lcd2(const char* a, const char* b) {
//   lcd.clear();
//   lcd.setCursor(0, 0); lcd.print(a);
//   lcd.setCursor(0, 1); lcd.print(b);
// }

// void setup() {
//   lcd.init();
//   lcd.backlight();
  
//   scaleL.begin(DT_L, SCK_L);
//   scaleR.begin(DT_R, SCK_R);

//   // --- [1단계: TARE (영점 조절)] ---
//   lcd2("STEP 1: TARE", "Keep Empty!!");
//   delay(3000); // 카메라 포커스 맞출 시간
  
//   if (scaleL.wait_ready_timeout(1000) && scaleR.wait_ready_timeout(1000)) {
//     offsetL = scaleL.read_average(30);
//     offsetR = scaleR.read_average(30);
//     lcd2("TARE Done!", "Next: CAL");
//     delay(2000);
//   }

//   // --- [2단계: LEFT Calibration (왼쪽 보정)] ---
//   lcd2("STEP 2: LEFT", "Put 155g on L");
//   delay(8000); // 155g 올리는 시간
  
//   lcd2("L: Calibrating", "Hold still...");
//   long netL = scaleL.read_average(30) - offsetL;
//   if (netL != 0) {
//     factorL = CAL_WEIGHT / (float)netL;
//   }
//   lcd2("LEFT CAL OK!", "Next: RIGHT");
//   delay(2000);

//   // --- [3단계: RIGHT Calibration (오른쪽 보정)] ---
//   lcd2("STEP 3: RIGHT", "Put 155g on R");
//   delay(8000); // 155g 옮기는 시간
  
//   lcd2("R: Calibrating", "Hold still...");
//   long netR = scaleR.read_average(30) - offsetR;
//   if (netR != 0) {
//     factorR = CAL_WEIGHT / (float)netR;
//   }
//   lcd2("RIGHT CAL OK!", "Starting Measure");
//   delay(2000);
  
//   lcd.clear();
// }

// void loop() {
//   // --- [4단계: 실시간 MEASURE (무게 측정)] ---
//   if (scaleL.is_ready() && scaleR.is_ready()) {
//     long rawL = scaleL.read_average(5);
//     long rawR = scaleR.read_average(5);

//     // 보정된 그램(g) 계산
//     float wL = (float)(rawL - offsetL) * factorL;
//     float wR = (float)(rawR - offsetR) * factorR;
//     float total = wL + wR;

//     // 0점 근처 미세 떨림 방지
//     if (total < 0.2 && total > -0.5) total = 0.0;

//     // LCD 출력
//     char line1[17], line2[17];
//     char sL[7], sR[7], sT[8];
    
//     dtostrf(wL, 5, 1, sL);
//     dtostrf(wR, 5, 1, sR);
//     dtostrf(total, 6, 1, sT);

//     sprintf(line1, "L:%s R:%s", sL, sR);
//     sprintf(line2, "TOTAL: %s g", sT);

//     lcd.setCursor(0, 0); lcd.print(line1);
//     lcd.setCursor(0, 1); lcd.print(line2);
//   }
//   delay(100);
// }

// <왼쪽 오른쪽 합쳐서 155g 나옴>
#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ATmega128 핀 정의
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

HX711 scaleL, scaleR;
LiquidCrystal_I2C lcd(0x27, 16, 2);

long offsetL = 0, offsetR = 0;
float factorL = 0.0f, factorR = 0.0f;
const float CAL_WEIGHT = 155.0f;

void lcd2(const char* a, const char* b) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print(b);
}

void setup() {
  lcd.init();
  lcd.backlight();
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  // --- [1단계: TARE] ---
  lcd2("STEP 1: TARE", "Keep Empty!!");
  delay(3000);
  if (scaleL.wait_ready_timeout(1000) && scaleR.wait_ready_timeout(1000)) {
    offsetL = scaleL.read_average(20);
    offsetR = scaleR.read_average(20);
    lcd2("TARE Done!", "Next: CAL Left");
    delay(2000);
  }

  // --- [2단계: LEFT Calibration 데이터 수집] ---
  lcd2("Put 155g on LEFT", "Wait 5s...");
  delay(5000);
  long nL_left = scaleL.read_average(30) - offsetL;
  long nR_left = scaleR.read_average(30) - offsetR;
  lcd2("L-Data OK", "Next: CAL Right");
  delay(2000);

  // --- [3단계: RIGHT Calibration 데이터 수집] ---
  lcd2("Put 155g on RIGHT", "Wait 5s...");
  delay(5000);
  long nL_right = scaleL.read_average(30) - offsetL;
  long nR_right = scaleR.read_average(30) - offsetR;

  // --- [4단계: 연립 방정식 산출] ---
  // Cramer's Rule 적용
  float det = (float)nL_left * nR_right - (float)nR_left * nL_right;

  if (abs(det) > 0.0001) { // 0 나누기 방지
    factorL = (CAL_WEIGHT * (float)nR_right - CAL_WEIGHT * (float)nR_left) / det;
    factorR = ((float)nL_left * CAL_WEIGHT - (float)nL_right * CAL_WEIGHT) / det;
    
    lcd.clear();
    lcd.setCursor(0,0); lcd.print("fL:"); lcd.print(factorL, 6);
    lcd.setCursor(0,1); lcd.print("fR:"); lcd.print(factorR, 6);
    delay(4000);
    lcd.clear();
  } else {
    // 행렬식이 0이면 보정 실패 (보통 센서 한쪽이 반응 없거나 물체 위치가 너무 중앙일 때)
    lcd2("CAL ERROR", "Det is 0");
    while(1);
  }
}

void loop() {
  if (scaleL.is_ready() && scaleR.is_ready()) {
    long rawL = scaleL.read_average(5);
    long rawR = scaleR.read_average(5);

    // 계산된 계수가 0이 아닐 때만 무게 계산 진행
    float wL = (float)(rawL - offsetL) * factorL;
    float wR = (float)(rawR - offsetR) * factorR;
    float total = wL + wR;

    // 미세 오차 0점 처리
    if (total < 0.2 && total > -0.5) {
        wL = 0.0; wR = 0.0; total = 0.0;
    }

    // LCD 출력
    char sL[7], sR[7], sT[8];
    dtostrf(wL, 5, 1, sL);
    dtostrf(wR, 5, 1, sR);
    dtostrf(total, 6, 1, sT);

    lcd.setCursor(0, 0);
    lcd.print("L:"); lcd.print(sL);
    lcd.print(" R:"); lcd.print(sR);

    lcd.setCursor(0, 1);
    lcd.print("TOTAL: "); lcd.print(sT); lcd.print(" g   ");
  }
  delay(150);
}