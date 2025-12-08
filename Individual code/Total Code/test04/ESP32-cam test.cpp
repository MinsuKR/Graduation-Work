// USART PE0, PE1 
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

// EEPROM 주소 (지금은 사용 안 함: 필요하면 a,b 저장용으로 확장 가능)
#define EEPROM_ADDR 0

// 샘플링/안정도/스냅 파라미터
const uint8_t STABLE_SAMPLING   = 10;
const uint8_t AVG_TARE_SAMPLES  = 20;
const uint8_t AVG_CAL_SAMPLES   = 35;
const uint8_t AVG_MEAS_SAMPLES  = 8;

const long  STABLE_DIFF_MAX = 500;
const float ZERO_SNAP_G     = 2.0f;

// === 듀얼 HX711 설정 ===

// 왼쪽 HX711
#define HX711_DT_L   PIN_PD3
#define HX711_SCK_L  PIN_PD2

// 오른쪽 HX711
#define HX711_DT_R   PIN_PD5
#define HX711_SCK_R  PIN_PD4

// 로드셀 기본 보정 계수 (이미 L/R 각각 맞춰둔 값 사용)
const float FACTOR_L_DEFAULT = -0.005398f;
const float FACTOR_R_DEFAULT = -0.004933f;

const float FACTOR_ABS_MIN   = 1e-5f;
const float FACTOR_ABS_MAX   = 1.0f;

// 위치 보정용 기준 무게 (원하면 155.0f 등으로 변경)
const float CAL_WEIGHT = 622.0f;

// 표시용 필터 상태값 (총무게용)
float DisplayW = 0.0f;
float PrevW    = 0.0f;

// === LCD & HX711 ===
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// 듀얼 HX711
HX711 scaleL;   // 왼쪽 로드셀
HX711 scaleR;   // 오른쪽 로드셀

// 영점 오프셋
long tare_offset_L = 0;
long tare_offset_R = 0;

// 로드셀 개별 계수 (기본값에서 시작)
float factorL = FACTOR_L_DEFAULT;
float factorR = FACTOR_R_DEFAULT;

// 위치 보정 계수 (Wtotal = a*L + b*R)
float a_coef = 1.0f;
float b_coef = 1.0f;

// === 함수 선언 ===
bool waitReadyDual(uint16_t timeout_ms = 100);
bool isStableDual(long* avg_out = nullptr);

void doTare(uint8_t avg = AVG_TARE_SAMPLES);
void doCalibration();   // 위치보정(a,b)용 CAL

// (이전 호환용, 지금은 실질적 동작 X)
void saveCalibration(float value) {
  (void)value; // 사용 안 함. 나중에 EEPROM 구조 확장 가능.
}
float loadCalibration() {
  // 듀얼 구조에서는 factorL/factorR/a/b를 별도로 관리할 수 있음.
  // 여기서는 일단 FACTOR_DEFAULT만 리턴 (사용하지 않지만 호환성 유지).
  return FACTOR_L_DEFAULT;
}

// === 평균 측정 함수 (L,R 각각 g단위로 반환) ===
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

  long netL = avgRawL - tare_offset_L;
  long netR = avgRawR - tare_offset_R;

  outL = (float)netL * factorL;
  outR = (float)netR * factorR;
}

// === HX711 준비 대기 (양쪽 모두 ready) ===
bool waitReadyDual(uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    bool rL = scaleL.is_ready();
    bool rR = scaleR.is_ready();
    if (rL && rR) return true;
  }
  return false;
}

// === 안정도 측정 (L+R 합 기준으로) ===
bool isStableDual(long* avg_out) {
  long samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReadyDual(300)) return false;
    long rawL = scaleL.read();
    long rawR = scaleR.read();
    samples[i] = rawL + rawR;  // 합으로 안정도 판단
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

// === 영점 조정 (양쪽 TARE) ===
void doTare(uint8_t avg) {
  if (!waitReadyDual(800)) return;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  long rawL = scaleL.read_average(avg);
  long rawR = scaleR.read_average(avg);

  tare_offset_L = rawL;
  tare_offset_R = rawR;

  DisplayW = 0.0f;
  PrevW    = 0.0f;

  lcd.setCursor(0, 1);
  lcd.print("OffL:");
  lcd.print(tare_offset_L);
  lcd.print(" ");

  delay(700);
  lcd.clear();
}

// === 위치 보정 캘리브레이션 (a,b 계산) ===
void doCalibration() {
  // 1) 왼쪽 끝에 CAL_WEIGHT 올리기
  float L_left = 0.0f, R_left = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put ");
  lcd.print(CAL_WEIGHT, 0);
  lcd.print("g LEFT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 올릴 시간

  measureLR(AVG_CAL_SAMPLES, L_left, R_left);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Lpos L=");
  lcd.print(L_left, 1);
  lcd.setCursor(0, 1); lcd.print("Lpos R=");
  lcd.print(R_left, 1);
  delay(1500);

  // 2) 오른쪽 끝에 CAL_WEIGHT 옮기기
  float L_right = 0.0f, R_right = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put ");
  lcd.print(CAL_WEIGHT, 0);
  lcd.print("g RIGHT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 옮길 시간

  measureLR(AVG_CAL_SAMPLES, L_right, R_right);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Rpos L=");
  lcd.print(L_right, 1);
  lcd.setCursor(0, 1); lcd.print("Rpos R=");
  lcd.print(R_right, 1);
  delay(1500);

  // 3) a,b 계산
  //  CAL_WEIGHT = a*L_left  + b*R_left
  //  CAL_WEIGHT = a*L_right + b*R_right
  float det = (L_left * R_right) - (L_right * R_left);

  if (fabs(det) < 1e-6f) {
    // 거의 대칭일 때: 그냥 합사용
    a_coef = 1.0f;
    b_coef = 1.0f;
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("det~0 -> a=b=1");
  } else {
    a_coef = CAL_WEIGHT * (R_right - R_left) / det;
    b_coef = CAL_WEIGHT * (L_left  - L_right) / det;

    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("a=");
    lcd.print(a_coef, 4);
    lcd.setCursor(0, 1); lcd.print("b=");
    lcd.print(b_coef, 4);
  }

  delay(2000);
  lcd.clear();
}

// === SETUP ===
void setup() {
  // ★ USART0 초기화
  Serial.begin(9600);
  delay(100);

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
  lcd.setCursor(0, 0); lcd.print("HX711 Dual");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 초기화 (듀얼)
  scaleL.begin(HX711_DT_L, HX711_SCK_L);
  scaleR.begin(HX711_DT_R, HX711_SCK_R);
  scaleL.set_gain(128);
  scaleR.set_gain(128);

  // (필요시: EEPROM에서 factorL/factorR/a/b 로딩 가능)
  // 여기서는 factorL/factorR은 상수, a/b는 기본 1.0

  // HX711 준비 대기
  if (!waitReadyDual(3000)) {
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
  while (!isStableDual(&avgRaw)) {
    lcd.setCursor(0, 0); lcd.print("RAW=");
    lcd.print(avgRaw);
    lcd.print("    ");
    lcd.setCursor(0, 1); lcd.print("Stabilizing...");
    delay(500);
    if (millis() - startTime > 10000) break;
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("RAW Stable OK!");
  delay(800);

  // 자동 영점 (양쪽)
  doTare();

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L,R Ready");
  delay(1000);

  // 정상 대기 상태 LED
  LED_Set(LEDSTATE_IDLE);

  Serial.println("=== USART L/R/TOTAL Output Start ===");
}

// === LOOP ===
void loop() {
  static uint32_t last = 0;

  // USART 출력용
  static float lastWL = 0.0f;
  static float lastWR = 0.0f;
  static float lastWtot = 0.0f;

  // 주기 측정/표시 (원래 300ms -> 지금 50ms)
  if (millis() - last >= 50) {
    last = millis();

    if (waitReadyDual(100)) {
      long rawL = scaleL.read_average(AVG_MEAS_SAMPLES);
      long rawR = scaleR.read_average(AVG_MEAS_SAMPLES);

      long netL = rawL - tare_offset_L;
      long netR = rawR - tare_offset_R;

      float wL = (float)netL * factorL;
      float wR = (float)netR * factorR;

      // 위치 보정 포함한 총무게
      float weight = a_coef * wL + b_coef * wR;

      if (isnan(weight) || isinf(weight)) weight = 0.0f;

      float absW = fabs(weight);

      // 0 근처는 스냅(바로 0으로)
      if (absW < ZERO_SNAP_G) {
        DisplayW = 0.0f;
      } else {
        // 바로 반응 (필터 없이)
        DisplayW = weight;
      }
      PrevW = weight;

      float shown = roundf(DisplayW * 1.0f) / 1.0f;

      // LCD 출력
      lcd.setCursor(0, 0);
      lcd.print("W:");
      lcd.print(shown, 1);
      lcd.print("g   ");

      lcd.setCursor(0, 1);
      lcd.print("L:");
      lcd.print(wL, 1);
      lcd.print(" R:");
      lcd.print(wR, 1);
      lcd.print("   ");

      // SERIAL 출력용 저장
      lastWL   = wL;
      lastWR   = wR;
      lastWtot = shown;

    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

  // ====== SERIAL DEBUG OUTPUT (1초마다) ======
  static uint32_t lastTx = 0;
  if (millis() - lastTx >= 1000) {
    lastTx = millis();

    Serial.print("L=");
    Serial.print(lastWL, 2);
    Serial.print("g  R=");
    Serial.print(lastWR, 2);
    Serial.print("g  Wtot=");
    Serial.print(lastWtot, 2);
    Serial.println(" g");
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
    LED_Set(LEDSTATE_PROCESSING);
    doTare();
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  }

  // CAL 버튼 (위치 보정 a,b 재계산)
  ButtonEvent eCal = Button_ReadCal();
  if (eCal == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    doCalibration();
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  } else if (eCal == BTN_LONG) {
    // 필요 시: 다른 캘리브 모드 추가 가능
  }
}
