#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

/* =========================
   실험 모드 선택
   ========================= */
#define EXP_POS_ERROR   1 // 위치 편차
#define EXP_REPEAT      2 // 반복 측정
#define EXP_CAL_STAB    3 // 캘 안정성
#define EXP_DRIFT       4 // 드리프트 측정

// 실행하고자 하는 모드를 여기서 선택하세요
#define EXPERIMENT_MODE EXP_DRIFT

/* =========================
   하드웨어 및 보정 설정
   ========================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);
HX711 scaleL, scaleR;

#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

float factorL = -0.005398f;
float factorR = -0.004933f;
long offsetL = 0, offsetR = 0;
float a_coef = 1.0f, b_coef = 1.0f;
const float CAL_WEIGHT = 155.0f;
#define AVG_CAL_SAMPLES 30
#define FACTOR_ABS_MIN  0.0001f

/* ==============================================
   정밀 측정 및 보정 함수들
   ============================================== */

long readFilteredRaw(HX711 &scale, int samples) {
  long sum = 0;
  long minVal = 2147483647;
  long maxVal = -2147483648;
  for (int i = 0; i < samples; i++) {
    while(!scale.is_ready());
    long raw = scale.read();
    if (raw < minVal) minVal = raw;
    if (raw > maxVal) maxVal = raw;
    sum += raw;
    delay(5);
  }
  return (sum - minVal - maxVal) / (samples - 2);
}

float getSingleWeight(HX711 &scale, long offset, float factor, int samples) {
  return (float)(readFilteredRaw(scale, samples) - offset) * factor;
}

float readTotalWeight(uint8_t samples) {
  float wL = getSingleWeight(scaleL, offsetL, factorL, samples);
  float wR = getSingleWeight(scaleR, offsetR, factorR, samples);
  return (a_coef * wL) + (b_coef * wR);
}

void doTare(uint8_t avg) {
  lcd.clear(); lcd.print("Auto TARE...");
  long sumL = 0, sumR = 0;
  for(int i=0; i<5; i++) {
    sumL += readFilteredRaw(scaleL, avg);
    sumR += readFilteredRaw(scaleR, avg);
    lcd.setCursor(i, 1); lcd.print(".");
  }
  offsetL = sumL / 5;
  offsetR = sumR / 5;
  lcd.setCursor(0, 1); lcd.print("Done           ");
  delay(1000);
}

void doCalibration() {
  float L_left, R_left, L_right, R_right, L_center, R_center;
  
  // 1. 왼쪽 측정
  lcd.clear(); lcd.print("1. Put 155g LEFT");
  delay(4000);
  L_left = getSingleWeight(scaleL, offsetL, factorL, AVG_CAL_SAMPLES);
  R_left = getSingleWeight(scaleR, offsetR, factorR, AVG_CAL_SAMPLES);

  // 2. 가운데 측정 (추가된 부분)
  lcd.clear(); lcd.print("2. Put 155g CTR");
  delay(4000);
  L_center = getSingleWeight(scaleL, offsetL, factorL, AVG_CAL_SAMPLES);
  R_center = getSingleWeight(scaleR, offsetR, factorR, AVG_CAL_SAMPLES);
  
  // 가운데 측정값 시리얼 출력 (디버깅용)
  Serial.print("Center Raw Check - L: "); Serial.print(L_center);
  Serial.print(" R: "); Serial.println(R_center);

  // 3. 오른쪽 측정
  lcd.clear(); lcd.print("3. Put 155g RIGHT");
  delay(4000);
  L_right = getSingleWeight(scaleL, offsetL, factorL, AVG_CAL_SAMPLES);
  R_right = getSingleWeight(scaleR, offsetR, factorR, AVG_CAL_SAMPLES);

  // 4. 연립 방정식 계산 (왼쪽과 오른쪽 데이터를 기준으로 계수 산출)
  float det = (L_left * R_right) - (L_right * R_left);
  if (abs(det) < FACTOR_ABS_MIN) {
    a_coef = 1.0f; b_coef = 1.0f;
    lcd.clear(); lcd.print("DET ERROR");
  } else {
    a_coef = CAL_WEIGHT * (R_right - R_left) / det;
    b_coef = CAL_WEIGHT * (L_left - L_right) / det;
    
    // 결과 출력
    lcd.clear();
    lcd.print("a:"); lcd.print(a_coef, 4);
    lcd.setCursor(0, 1);
    lcd.print("b:"); lcd.print(b_coef, 4);
    
    // 가운데 지점에서의 최종 보정 무게 확인 (시리얼)
    float calibratedCenter = (a_coef * L_center) + (b_coef * R_center);
    Serial.print("Calibrated Center Weight: "); Serial.println(calibratedCenter);
  }
  delay(3000);
}

/* =========================
   실험 모드 함수들
   ========================= */

void experimentPositionError() {
  const char* posName[7] = {"LU", "L", "LD", "C", "RU", "R", "RD"};
  Serial.println("Position,Weight(g)");
  for (int i = 0; i < 7; i++) {
    lcd.clear(); lcd.print("Put 155g "); lcd.print(posName[i]);
    delay(4000); 
    float w = readTotalWeight(40);
    Serial.print(posName[i]); Serial.print(","); Serial.println(w, 2);
    lcd.setCursor(0, 1); lcd.print("W: "); lcd.print(w, 2); lcd.print("g");
    delay(2000);
  }
  lcd.clear(); lcd.print("POS TEST DONE");
}

void experimentRepeatability() {
  Serial.println("Trial,Weight(g)");
  for (int i = 1; i <= 10; i++) {
    lcd.clear(); lcd.print("Trial "); lcd.print(i);
    lcd.setCursor(0, 1); lcd.print("Put 155g");
    delay(4000);
    float w = readTotalWeight(40);
    Serial.print(i); Serial.print(","); Serial.println(w, 2);
    lcd.clear(); lcd.print("W="); lcd.print(w, 2);
    delay(2000);
    lcd.clear(); lcd.print("Remove Weight");
    delay(3000);
  }
  lcd.clear(); lcd.print("REPEAT DONE");
}

void experimentCalStability() {
  Serial.println("Cycle,Weight(g)");
  for (int c = 1; c <= 5; c++) {
    lcd.clear(); lcd.print("Cycle "); lcd.print(c);
    delay(3000);
    float w = readTotalWeight(40);
    Serial.print(c); Serial.print(","); Serial.println(w, 2);
    lcd.setCursor(0, 1); lcd.print("W="); lcd.print(w, 2);
    delay(2000);
    lcd.clear(); lcd.print("Manual Check Done");
  }
}

void experimentDrift() {
  Serial.println("Time(min),Weight(g)");
  unsigned long start = millis();
  unsigned long duration = 600000UL; // 10분 (600,000ms)

  // 10분 정각 데이터까지 포함하기 위해 조건을 살짝 늘리거나 
  // 루프 내부에서 시간을 체크하는 방식을 사용합니다.
  while (true) { 
    unsigned long elapsed = millis() - start;
    unsigned long t = elapsed / 60000UL; // 현재 분(min)
    
    float w = readTotalWeight(30);
    
    // 시리얼 출력
    Serial.print(t); Serial.print(","); Serial.println(w, 3);
    
    // LCD 출력
    lcd.setCursor(0, 0);
    lcd.print("Drift: "); lcd.print(6); lcd.print("min     "); // 뒤에 빈칸을 넣어 잔상 제거
    
    lcd.setCursor(0, 1);
    lcd.print("W: "); lcd.print(w, 2); lcd.print("g      "); // g 단위 추가 및 잔상 제거

    // 10분에 도달하면 측정 후 종료
    if (elapsed >= duration) break;

    delay(30000); // 30초 대기
  }
  
  lcd.clear();
  lcd.print("Drift Test Done");
  Serial.println("Drift Test Done");
}

/* =========================
   Main Setup & Loop
   ========================= */

void setup() {
  Serial.begin(9600);
  lcd.init();
  lcd.backlight();

  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  // 1. 초기 정밀 Tare
  doTare(20);

  // 2. 초기 정밀 보정 (실험 전 보정 수행)
  doCalibration();

  lcd.clear();
  lcd.print("Start Experiment");
  delay(1500);

  // 3. 선택된 실험 모드 실행
#if EXPERIMENT_MODE == EXP_POS_ERROR
  experimentPositionError();
#elif EXPERIMENT_MODE == EXP_REPEAT
  experimentRepeatability();
#elif EXPERIMENT_MODE == EXP_CAL_STAB
  experimentCalStability();
#elif EXPERIMENT_MODE == EXP_DRIFT
  experimentDrift();
#endif

  lcd.clear();
  lcd.print("ALL TEST DONE");
}

void loop() {
  // 실험 종료 후 대기
}