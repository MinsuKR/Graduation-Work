#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

// === 핀 정의 ===
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define HX711_DT   PIN_PD3   // DOUT
#define HX711_SCK  PIN_PD2   // SCK
#define TARE_BTN   PIN_PE4   // 영점 버튼 (GND와 연결)
#define CAL_BTN    PIN_PE5   // 캘리브레이션 버튼 (GND와 연결)

// EEPROM 주소
#define EEPROM_ADDR 0 // EEPROM에 보관할 위치(주소 0)

const uint8_t STABLE_SAMPLING = 10;
const uint8_t AVG_TARE_SAMPLES = 15; 
const uint8_t AVG_CAL_SAMPLES = 25; 

const long STABLE_DIFF_MAX = 200; 
const float ZERO_SNAP_G = 2.0f;


const float FACTOR_DEFAULT = -0.067f;
const float FACTOR_ABS_MIN = 1e-5f;
const float FACTOR_ABS_MAX = 1.0f; 

float DisplayW = 0.0f; // 화면에 보여줄 필터링된 무게
float PrevW =0.0f; // 직전 측정 원본 무게

// === LCD ===
LiquidCrystal_I2C lcd(0x27, 16, 2); // LCD 주소 0x27 16*2

// === HX711 객체 ===
HX711 scale;  
long tare_offset = 0; 
float calibration_factor = FACTOR_DEFAULT ; 

void saveCalibration(float value);
float loadCalibration();
bool waitReady(uint16_t timeout_ms = 100);
void doTare(uint8_t avg = AVG_TARE_SAMPLES);
void doCalibration();
bool isStable(long* avg_out = nullptr);

// === SETUP ===
void setup() {
  pinMode(TARE_BTN, INPUT_PULLUP); 
  pinMode(CAL_BTN, INPUT_PULLUP); 

  // === LCD 초기화/백라이트 ON/부팅 메시지 ===
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  scale.begin(HX711_DT, HX711_SCK); 
  scale.set_gain(128); 
 
  calibration_factor = loadCalibration(); 

  // 리팩터 포인트: 부팅 준비 대기 → waitReady(3000) 한 줄로 통일
  if (!waitReady(3000)) {   
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    delay(1500);
  }

  // === 자동 안정도 확인 ===
  long avgRaw = 0;
  uint32_t startTime = millis();
  while (!isStable(&avgRaw)) 
  {
    lcd.setCursor(0, 0); lcd.print("RAW="); lcd.print(avgRaw); 
    lcd.print("    "); 
    lcd.setCursor(0, 1); lcd.print("Stabilizing...");
    delay(500);
    if (millis() - startTime > 10000) break; 
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("RAW Stable OK!");
  delay(800);

  // === 자동 영점 ===
  doTare();

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Factor:");
  lcd.setCursor(0, 1); lcd.print(calibration_factor, 6); 
  delay(1000);
}

// === LOOP ===
void loop() {
  static uint32_t last = 0; 

  if (millis() - last >= 300) {
    last = millis(); 

    if (waitReady(100)) {  
      long raw = scale.read_average(AVG_CAL_SAMPLES); 
      long net = raw - tare_offset;  
      float weight = (float)net * calibration_factor; 

      // === 노이즈 억제 ===
      if (isnan(weight) || isinf(weight)) weight = 0.0;
      //if (fabs(weight) < ZERO_SNAP_G) weight = 0.0; 

       // === 저울처럼 "딱딱" 보이게 하는 표시용 필터 ===
      float absW = fabs(weight);
      const float ALPHA = 0.3f;  // 0~1, 클수록 빠르고, 작을수록 부드러움

      if (absW < ZERO_SNAP_G) {
        // ① 2g 이하는 그냥 0으로 스냅 → 빈 저울에서 출렁임 제거
        DisplayW = 0.0f;
      } else if (fabs(PrevW) > 20.0f && absW < 10.0f) {
        // ② 방금 전엔 20g 이상이었는데, 지금은 10g 미만 → 무게 뺀 상황으로 보고 바로 0
        DisplayW = 0.0f;
      } else {
        // ③ 그 외에는 지수 이동 평균으로 부드럽게 필터링
        DisplayW = ALPHA * weight + (1.0f - ALPHA) * DisplayW;
      }
      PrevW = weight;

      // 0.5g 단위로 반올림 (필요하면 1g 단위로 변경 가능)
      float shown = roundf(DisplayW * 2.0f) / 2.0f;

      lcd.setCursor(0, 0);
      lcd.print("W:");
      lcd.print(weight, 1);
      lcd.print(" g      "); 

      lcd.setCursor(0, 1);
      lcd.print("R:");
      lcd.print(raw);
      lcd.print(" N:");
      lcd.print(net);
      lcd.print("  ");

    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

  // === TARE 버튼 === > TARE 버튼(눌리면 LOW) 디바운스 → 영점 실행 → 버튼에서 손 뗄 때까지 대기
  if (digitalRead(TARE_BTN) == LOW) { //
    delay(20);
    if (digitalRead(TARE_BTN) == LOW) {
      doTare();
      while (digitalRead(TARE_BTN) == LOW) {}
    }
  }

  // === CAL 버튼 === > CAL 버튼 동작(동일 패턴) → 캘리브레이션 실행.
  if (digitalRead(CAL_BTN) == LOW) {
    delay(20);
    if (digitalRead(CAL_BTN) == LOW) {
      doCalibration();
      while (digitalRead(CAL_BTN) == LOW) {}
    }
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

  // 비정상 값 필터링 
  if (!isfinite(val) || fabs(val) < FACTOR_ABS_MIN || fabs(val) > FACTOR_ABS_MAX) // 무한대/NaN || 0에 너무 가까우면 복구 || 너무 큰 값(이상치) 복구
  { 
    return FACTOR_DEFAULT;  
  }
  return val; 
}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms = 100) 
{ 
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scale.is_ready()) return true; 
  }
  return false; 
}

// === 영점 조정 ===
void doTare(uint8_t avg = AVG_TARE_SAMPLES) 
{
  if (!waitReady(800)) return; 

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  long raw = scale.read_average(avg); 
  tare_offset = raw; // ★ 중요: 실제 영점 오프셋 저장
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
}

 // === 안정도 측정 함수 ===
bool isStable(long* avg_out = nullptr) 
{
  long samples[STABLE_SAMPLING];
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false; 
    samples[i] = scale.read(); 
  }

  // === 최소/최대/합 계산 ===
  long minV = samples[0], maxV = samples[0];
  long sum = 0;
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  if (avg_out) *avg_out = sum / 10; 
  long diff = maxV - minV; 
  return (diff < STABLE_DIFF_MAX);
} 
