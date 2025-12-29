#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

#include "LED.h"
#include "Buzzer.h"
#include "Button.h"

// === LCD 정의 ===
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// ==== LCD & HX711 인스턴스 2개 ====
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
HX711 scaleL;   // 왼쪽 로드셀
HX711 scaleR;   // 오른쪽 로드셀

// ==== 핀 설정 ====
// 왼쪽 HX711
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2

// 오른쪽 HX711
#define DT_R   PIN_PD5   // DOUT
#define SCK_R  PIN_PD4   // SCK

// EEPROM 주소 매핑 (연속 저장)
#define EE_FACTOR_L   0   
#define EE_FACTOR_R   4
#define EE_A_COEF     8
#define EE_B_COEF     12
#define EE_TARE_L     16
#define EE_TARE_R     20

// ==== 로드셀 기본 보정 계수 (이미 맞춰둔 값 사용) ====
float factorL = -0.005398f;
float factorR = -0.004933f;

// ==== 영점 오프셋 ====
long offsetL = 0;
long offsetR = 0;

// ==== 총무게 보정용 계수 ====
float a_coef = 1.0f;   // Wtotal = a*L + b*R
float b_coef = 1.0f;

// 샘플링/안정도/스냅 파라미터
const uint8_t STABLE_SAMPLING   = 10;
const uint8_t AVG_TARE_SAMPLES  = 20;
const uint8_t AVG_CAL_SAMPLES   = 35;
const uint8_t AVG_MEAS_SAMPLES  = 10;

const long  STABLE_DIFF_MAX = 500;
const float ZERO_SNAP_G     = 2.0f;

// ==== 캘리브용 파라미터 ====
const float CAL_WEIGHT = 155.0f;
const float FACTOR_ABS_MIN = 1e-5f;
const float FACTOR_ABS_MAX = 1.0f;

// 표시용 필터 상태값
float DisplayW = 0.0f;
float PrevW    = 0.0f;

// 현재 장치의 보정값들을 EEPROM(영구메모리)에 저장하는 함수
void saveAllToEEPROM() {
  EEPROM.put(EE_FACTOR_L, factorL);
  EEPROM.put(EE_FACTOR_R, factorR);
  EEPROM.put(EE_A_COEF,  a_coef);
  EEPROM.put(EE_B_COEF,  b_coef);
  EEPROM.put(EE_TARE_L,  offsetL);
  EEPROM.put(EE_TARE_R,  offsetR);
}

// EEPROM에서 저장해둔 보정값을 읽어오는 함수
void loadAllFromEEPROM() {
  EEPROM.get(EE_FACTOR_L, factorL);
  EEPROM.get(EE_FACTOR_R, factorR);
  EEPROM.get(EE_A_COEF,  a_coef);
  EEPROM.get(EE_B_COEF,  b_coef);
  EEPROM.get(EE_TARE_L,  offsetL);
  EEPROM.get(EE_TARE_R,  offsetR);

  // 유효범위 체크(값이 정상인지)
  if (!isfinite(factorL) || fabs(factorL) < 1e-6) factorL = -0.005398f;
  if (!isfinite(factorR) || fabs(factorR) < 1e-6) factorR = -0.004933f;
  if (!isfinite(a_coef)) a_coef = 1.0f;
  if (!isfinite(b_coef)) b_coef = 1.0f;
  if (!isfinite(offsetL)) offsetL = 0;
  if (!isfinite(offsetR)) offsetR = 0;
}

// === 함수 선언 ===
void saveCalibration(float value);
float loadCalibration();
bool waitReady(uint16_t timeout_ms = 100);
void doTare(uint8_t avg = AVG_TARE_SAMPLES);
void doCalibration();
bool isStable(long* avg_out = nullptr);
void measureLR(int samples, float &outL, float &outR);

void setup() 
{
    
  // LED / Buzzer / Button 모듈 초기화
  LED_Init();
  Buzzer_Init();
  Button_Init();

  // 부팅 알림
  LED_Set(LEDSTATE_IDLE);
  Buzzer_Click();

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 Dual a,b");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 시작
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);
  scaleL.set_gain(128);
  scaleR.set_gain(128);

  loadAllFromEEPROM();
  delay(500);

   // HX711 준비 대기
  if (!waitReady(3000)) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  // 자동 안정도 확인
  long avgRaw = 0;
  uint32_t startTime = millis();
  while (!isStable(&avgRaw)) {
    lcd.setCursor(0, 0); lcd.print("RAW="); lcd.print(avgRaw);
    lcd.print("    ");
    lcd.setCursor(0, 1); lcd.print("Stabilizing...");
    delay(500);
    if (millis() - startTime > 10000) break;
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("RAW Stable OK!");
  delay(800);

  // --- 1) TARE (빈 상태에서) ---
  doTare();

  // 정상 대기 상태 LED
  LED_Set(LEDSTATE_IDLE);

}

void loop() 
{
  static uint32_t last = 0;

  if (millis() - last >= 50) // 300 -> 50 바꿈 
  {
    last = millis();

    if (waitReady(100)) {
        // 한 번 읽어서 무게 계산
        long rawL = scaleL.read_average(5);
        long rawR = scaleR.read_average(5);

        long netL = rawL - offsetL;
        long netR = rawR - offsetR;

        float wL = (float)netL * factorL;
        float wR = (float)netR * factorR;

        // 위치 보정 포함한 총무게
        float Wtotal = a_coef * wL + b_coef * wR;

        if (isnan(Wtotal) || isinf(Wtotal)) Wtotal = 0.0f;

        float absW = fabs(Wtotal);

        // 1) 0 근처는 스냅(바로 0으로)
      if (absW < ZERO_SNAP_G) {   // ZERO_SNAP_G = 2.0f 유지
        DisplayW = 0.0f;
      }
      // 2) 나머지는 필터 없이 그대로 사용 (즉각 반응)
      else {
        DisplayW = Wtotal;
      }

      // x.x g 단위로 표현(원래대로)
      float shown = roundf(DisplayW * 10.0f) / 10.0f;

      // ===== LCD 출력 =====
      lcd.setCursor(0, 0);
      lcd.print("L:");
      lcd.print(wL, 1);
      lcd.print("g ");
      lcd.print("R:");
      lcd.print(wR, 1);
      lcd.print("g ");

      lcd.setCursor(0, 1);
      lcd.print("TOT:");
      lcd.print(shown, 1);
      lcd.print("g    ");

      // ===== ESP32로 무게 전송 =====
      Serial.print("W=");
      Serial.println(shown, 1);

    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

    // ===== 버튼 처리 =====

  // TARE 버튼
  ButtonEvent eTare = Button_ReadTare();
  if (eTare == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    doTare();
    Buzzer_Click();
    LED_Set(LEDSTATE_IDLE);
  } else if (eTare == BTN_LONG) {
    // 필요하면: 장기 TARE 기능 넣기
    LED_Set(LEDSTATE_PROCESSING);
    doTare();
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  }

  // CAL 버튼
  ButtonEvent eCal = Button_ReadCal();
  if (eCal == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    doCalibration();
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  } else if (eCal == BTN_LONG) {
    // 장기 CAL 기능 필요하면 나중에 추가
  }
  
}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scaleL.is_ready() && scaleR.is_ready()) return true;
  }
  return false;
}

// === 영점 조정 ===
void doTare(uint8_t avg) {
  if (!waitReady(800)) return;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  offsetL = scaleL.read_average(avg);
  offsetR = scaleR.read_average(avg);

  saveAllToEEPROM();

  lcd.setCursor(0, 1); lcd.print("Done");
  delay(700);
  lcd.clear();
}

// === 캘리브레이션 ===
void doCalibration() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put weight (155g)");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);

  if (!waitReady(1000)) return;

  // --- 2) 왼쪽 끝에 155g 올려서 측정 ---
  float L_left = 0.0f, R_left = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g LEFT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 올릴 시간

  measureLR(30, L_left, R_left);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L_left=");
  lcd.print(L_left, 1);
  lcd.setCursor(0, 1); lcd.print("R_left=");
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
  lcd.setCursor(0, 0); lcd.print("L_right=");
  lcd.print(L_right, 1);
  lcd.setCursor(0, 1); lcd.print("R_right=");
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

    saveAllToEEPROM();

    lcd.clear();
    lcd.setCursor(0,0); lcd.print("a=");
    lcd.print(a_coef, 4);
    lcd.setCursor(0,1); lcd.print("b=");
    lcd.print(b_coef, 4);
  }

}

// === 안정도 측정 함수 ===
bool isStable(long* avg_out) {
  long samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false;
    long rawL = scaleL.read();
    long rawR = scaleR.read();
    samples[i] = rawL + rawR;
  }

  long minV = samples[0], maxV = samples[0];
  long sum = 0;
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  if (avg_out) *avg_out = sum / STABLE_SAMPLING;
  long diff = maxV - minV;
  return (diff < STABLE_DIFF_MAX);
}

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