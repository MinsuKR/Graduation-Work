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

// EEPROM 주소
#define EEPROM_ADDR 0

// 샘플링/안정도/스냅 파라미터
const uint8_t STABLE_SAMPLING   = 10;
const uint8_t AVG_TARE_SAMPLES  = 20;
const uint8_t AVG_CAL_SAMPLES   = 35;
const uint8_t AVG_MEAS_SAMPLES  = 8;

const long  STABLE_DIFF_MAX = 500;
const float ZERO_SNAP_G     = 2.0f;

// 캘리브레이션 파라미터
const float FACTOR_DEFAULT = -0.015856f; // 로드셀 1개 - -0.095f / 로드셀 2개 - 
const float FACTOR_ABS_MIN = 1e-5f;
const float FACTOR_ABS_MAX = 1.0f;

// 표시용 필터 상태값
float DisplayW = 0.0f;
float PrevW    = 0.0f;

// === LCD & HX711 ===
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
HX711 scale;

long  tare_offset        = 0;
float calibration_factor = FACTOR_DEFAULT;

// === 함수 선언 ===
void saveCalibration(float value);
float loadCalibration();
bool waitReady(uint16_t timeout_ms = 100);
void doTare(uint8_t avg = AVG_TARE_SAMPLES);
void doCalibration();
bool isStable(long* avg_out = nullptr);

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
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 초기화
  scale.begin(PIN_PD3, PIN_PD2); // HX711_DT, HX711_SCK
  scale.set_gain(128);

  // 캘리브레이션 계수 로드
  calibration_factor = loadCalibration();

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

  // 자동 영점
  doTare();

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Factor:");
  lcd.setCursor(0, 1); lcd.print(calibration_factor, 6);
  delay(1000);

  // 정상 대기 상태 LED
  LED_Set(LEDSTATE_IDLE);

  Serial.println("=== USART RAW/NET/WEIGHT Output Start ===");
}

// === LOOP ===
void loop() {
  static uint32_t last = 0;

  //USART에 띄울 값
  static long lastRaw = 0;
  static long lastNet = 0;
  static float lastW  = 0;

  // 300ms마다 무게 측정/표시
  if (millis() - last >= 50) // 300 -> 50 바꿈 
  {
    last = millis();

    if (waitReady(100)) {
      long raw = scale.read_average(AVG_MEAS_SAMPLES); // 35 -> 4 -> 8바꿈
      long net = raw - tare_offset;
      float weight = (float)net * calibration_factor;

      if (isnan(weight) || isinf(weight)) weight = 0.0f;

      // ===== 표시용 필터 ===== // ALPHA(천천히 변함)부드럽게  
      // float absW = fabs(weight);
      // const float ALPHA = 0.3f;

      // if (absW < ZERO_SNAP_G) {
      //   DisplayW = 0.0f;
      // } else if (fabs(PrevW) > 20.0f && absW < 10.0f) {
      //   DisplayW = 0.0f;
      // } else {
      //   DisplayW = ALPHA * weight + (1.0f - ALPHA) * DisplayW;
      // }
      // PrevW = weight;

      float absW = fabs(weight);

      // 1) 0 근처는 스냅(바로 0으로)
      if (absW < ZERO_SNAP_G) {   // ZERO_SNAP_G = 2.0f 유지
        DisplayW = 0.0f;
      }
      // 2) 나머지는 필터 없이 그대로 사용 (즉각 반응)
      else {
        DisplayW = weight;
      }

      PrevW = weight;  // 지금은 크게 안 쓰이지만 일단 유지

      // 0.1g 단위로 반올림(원래대로)
      float shown = roundf(DisplayW * 1.0f) / 1.0f;

      // LCD 출력
      lcd.setCursor(0, 0);
      lcd.print("W:");
      lcd.print(shown, 1);
      lcd.print(" g      ");

      lcd.setCursor(0, 1);
      lcd.print("R:");
      lcd.print(raw);
      lcd.print(" N:");
      lcd.print(net);
      lcd.print("  ");

      // SERIAL 출력용 저장
      lastRaw = raw;
      lastNet = net;
      lastW   = shown;

    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

  // ====== SERIAL DEBUG OUTPUT (1초마다) ======
  static uint32_t lastTx = 0;
  if (millis() - lastTx >= 1000) {
    lastTx = millis();

    Serial.print("RAW=");
    Serial.print(lastRaw);
    Serial.print("  NET=");
    Serial.print(lastNet);
    Serial.print("  W=");
    Serial.print(lastW, 2);
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

// === EEPROM 저장 ===
void saveCalibration(float value) {
  EEPROM.put(EEPROM_ADDR, value);
}

// === EEPROM 불러오기 (비정상값 자동복구 포함) ===
float loadCalibration() {
  float val;
  EEPROM.get(EEPROM_ADDR, val);

  if (!isfinite(val) ||
      fabs(val) < FACTOR_ABS_MIN ||
      fabs(val) > FACTOR_ABS_MAX) {
    return FACTOR_DEFAULT;
  }
  return val;
}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scale.is_ready()) return true;
  }
  return false;
}

// === 영점 조정 ===
void doTare(uint8_t avg) {
  if (!waitReady(800)) return;

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  long raw = scale.read_average(avg);

  tare_offset = raw;
  scale.set_offset(tare_offset);

  DisplayW = 0.0f;
  PrevW    = 0.0f;

  lcd.setCursor(0, 1); lcd.print("Offset:");
  lcd.print(tare_offset);
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

  long raw = scale.read_average(AVG_CAL_SAMPLES);
  long net = raw - tare_offset;
  float known_weight = 155.0f;

  calibration_factor = known_weight / (float)net;
  saveCalibration(calibration_factor);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Cal Done!");
  lcd.setCursor(0, 1); lcd.print("Factor=");
  lcd.print(calibration_factor, 6);
  delay(2000);

  Serial.print("[CAL] factor=");
  Serial.println(calibration_factor, 6);
}

// === 안정도 측정 함수 ===
bool isStable(long* avg_out) {
  long samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false;
    samples[i] = scale.read();
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

