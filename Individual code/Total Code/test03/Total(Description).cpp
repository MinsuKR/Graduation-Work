// bool : 논리형(True / False)값
// millis() : 부팅 후 지난 시간을 ms 단위로 반환 / 내부적으로 타이머0 인터럽트로 증가시키는 전역 카운터를 읽음
// fabs : 절대값
// isnan : 무한(NaN)이면 1 , 아니면 0
// isinf : +∞면 1, −∞면 −1, finite면 0
// isfinite : 유한(NaN X)이면 True <-> !isfinite 무한이면 True
// lcd.print("  "); : 여유 공백을 출력해서 이전 문자(잔상)를 지워줌
// nullptr : C++에서 어떤 객체도 가리키지 않는 포인터  + 비어있음을 의미
// <파일명.h> = 표준 라이브러리 /  "파일명.h" = 내가 만든 헤더 파일 
// #ifndef, #define, #endif = 헤더 중복 포함 방지 즉, 같은 헤더가 여러 번 포함(Include)되더라도 단 한 번만 처리되게 한다.
// USART PE0, PE1 

#include <Arduino.h> //Arduino 기본 함수들(pinMode, digitalWrite, millis 등)
#include <Wire.h> // I2C 통신 라이브러리
#include <HX711.h> // HX711(로드셀 ADC읽기) 제어 라이브러리
#include <LiquidCrystal_I2C.h> // I2C 타입 16×2 LCD 제어 라이브러리
#include <EEPROM.h> // 보드 내부(factor) EEPROM에 값 저장/불러오기용
#include <math.h> // fabs, isnan, isinf, roundf 같은 수학 함수 사용하기 위한 라이브러리(AVR에선 이미 포함되어 있는 경우가 많지만 안전하게 사용하기 위해)

#include "LED.h" // 사용자 LED 헤더 함수
#include "Buzzer.h" // 사용자 Buzzer 헤더 함수
#include "Button.h" // // 사용자 Button(tare,cal) 헤더 함수

// === LCD 정의 ===
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// EEPROM 주소
#define EEPROM_ADDR 0 // EEPROM에 calibration factor를 저장할 시작 주소(0번)

// 샘플링/안정도/스냅 파라미터
const uint8_t STABLE_SAMPLING   = 10; // 안정도 검사할 때 몇 샘플을 모을지
const uint8_t AVG_TARE_SAMPLES  = 20; // Tare 할 때 평균 내는 샘플 수
const uint8_t AVG_CAL_SAMPLES   = 35; // 캘리브레이션 때 평균 내는 샘플 수
const uint8_t AVG_MEAS_SAMPLES  = 4; // 평소 측정에서 평균낼 샘플 수

const long  STABLE_DIFF_MAX = 200; // 안정도 검사에서 최대–최소 차이가 200 이하이면 “안정”으로 간주
const float ZERO_SNAP_G     = 2.0f; // ±2g 이내는 0g으로 스냅시키기 위한 기준값

// 캘리브레이션 파라미터
const float FACTOR_DEFAULT = -0.095f; // EEPROM에 유효한 값이 없을 때 사용할 기본 환산계수(로드셀 병렬시 여기만 바뀜)
const float FACTOR_ABS_MIN = 1e-5f; // 불러온 factor가 0에 너무 가까운 값(거의 0)을 보고 기본값으로 복구
const float FACTOR_ABS_MAX = 1.0f; // 불러온 factor가 비정상적으로 큰 값을 보고 기본값으로 복구

// 표시용 필터 상태값
float DisplayW = 0.0f; // 저울 값이 LCD표시에 부드럽게 하기 위한 필터링된 무게 값
float PrevW    = 0.0f; // 저울 값이 LCD표시에 부드럽게 하기 위한 직전 측정 원본 무게(필터 판단 조건에 사용)

// === LCD & HX711 === 
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS); //객체 생성
HX711 scale; //객체 생성

long  tare_offset = 0; // RAW에서 빼줄 영점 값(Offset)
float calibration_factor = FACTOR_DEFAULT; // NET → g로 바꾸는 factor. 초기엔 기본값

// === 함수 선언 ===
void saveCalibration(float value); // EEPROM 저장
float loadCalibration(); // EEPROM 로드(+이상치 복구)
bool waitReady(uint16_t timeout_ms = 100); // HX711 준비 대기
void doTare(uint8_t avg = AVG_TARE_SAMPLES); // 자동 영점
void doCalibration(); // 캘리브레이션 루틴
bool isStable(long* avg_out = nullptr); // 안정도 측정

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
  // LED를 IDLE 상태로 켜고, 부저를 한 번 클릭해서 “부팅됨” 피드백
  LED_Set(LEDSTATE_IDLE);
  Buzzer_Click();

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 초기화
  scale.begin(PIN_PD3, PIN_PD2); // HX711 데이터를 읽기 위해 DOUT(=DT)을 PD3, SCK를 PD2에 연결
  scale.set_gain(128); // A채널 게인 128로 설정

  // 캘리브레이션 계수 로드
  calibration_factor = loadCalibration(); // EEPROM에서 저장된 factor 읽어오기. 비정상이면 FACTOR_DEFAULT로 대체

  // HX711 준비 대기
  // 3초 동안 HX711이 ready 되는지 확인 후 준비 안 되면 LCD에 에러 메시지, LED ERROR 상태, 에러 부저
  if (!waitReady(3000)) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  // 자동 안정도 확인
  // isStable()로 일정 시간(최대 10초) 동안 RAW 값이 안정되길 기다리고 안정될 때까지 현재 RAW 평균값을 LCD에 출력
  long avgRaw = 0;
  uint32_t startTime = millis();
  while (!isStable(&avgRaw)) {
    lcd.setCursor(0, 0); lcd.print("RAW="); lcd.print(avgRaw);
    lcd.print("    ");
    lcd.setCursor(0, 1); lcd.print("Stabilizing...");
    delay(500);
    if (millis() - startTime > 10000) break;
  }
  // 완료 메세지
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("RAW Stable OK!");
  delay(800);

  // 자동 영점
  doTare(); // 현재 상태를 기준으로 자동 TARE 수행(Offset 세팅)

  // 현재 calibration factor를 LCD에 한 번 표시
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

    if (waitReady(100)) { // Hx711가 준비될 때까지 100ms 준비
      long raw = scale.read_average(AVG_MEAS_SAMPLES); // 35 -> 4바꿈 , 준비되면 샘플 평균으로 RAW값을 얻음
      long net = raw - tare_offset; // Tare 적용.
      float weight = (float)net * calibration_factor; // g 단위로 환산

      if (isnan(weight) || isinf(weight)) weight = 0.0f; // 계산 중 NaN이나 무한대가 나오면 0으로 보호

      // ===== 표시용 필터 ===== 부드럽게
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
      
      // 필터 없이 그냥 weight를 그대로 사용
      DisplayW = weight;
      PrevW    = weight;

      // 1g 단위로 반올림
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
  ButtonEvent eTare = Button_ReadTare(); // Button_ReadTare()로 TARE 버튼 상태 읽기
  
  if (eTare == BTN_SHORT) { // 짧게 누르면 dotare()실행, 부저 click, LED blue들어왔다 -> 다시 green
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
  ButtonEvent eCal = Button_ReadCal(); // doCalibration()으로  Cal 버튼 상태 읽기

  if (eCal == BTN_SHORT) { // CAL 버튼 짧은 클릭 → doCalibration() 실행, 완료 후 성공음, LED 상태 복구
    LED_Set(LEDSTATE_PROCESSING);
    doCalibration();
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  } else if (eCal == BTN_LONG) {
    // 장기 CAL 기능 필요하면 나중에 추가
  }
}

// === EEPROM 저장 ===
// value를 EEPROM의 주소 0부터 저장
void saveCalibration(float value) { 
  EEPROM.put(EEPROM_ADDR, value);
}

// === EEPROM 불러오기 (비정상값 자동복구 포함) ===
// EEPROM에서 float 하나 읽어옴, NaN/무한대이거나, 너무 작거나 너무 크면 비정상 데이터로 보고 FACTOR_DEFAULT 반환, 정상 범위이면 그대로 사용
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
// 주어진 시간(ms) 안에 scale.is_ready() 가 true 되면 true 리턴, 타임아웃 넘기면 false 리턴
bool waitReady(uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scale.is_ready()) return true;
  }
  return false;
}

// === 영점 조정 ===
// 
void doTare(uint8_t avg) {
  if (!waitReady(800)) return; // 최대 800ms 동안 HX711 준비 기다림

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  long raw = scale.read_average(avg); // avg개(기본 20개) 평균 읽어서 raw 얻음

  tare_offset = raw;
  scale.set_offset(tare_offset); // 그 값을 tare_offset으로 저장, HX711 라이브러리 내부 offset에도 설정

  // 표시용 무게 변수 0으로 초기화
  DisplayW = 0.0f;
  PrevW    = 0.0f;

  lcd.setCursor(0, 1); lcd.print("Offset:");
  lcd.print(tare_offset);
  delay(700);
  lcd.clear();
}

// === 캘리브레이션 ===
void doCalibration() { // 사용자에게 “155g 올려두고 기다리라”는 안내, 3초 기다림
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put weight (155g)");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);

  if (!waitReady(1000)) return;

  long raw = scale.read_average(AVG_CAL_SAMPLES); // HX711 준비 확인 후, 35샘플 평균으로 RAW 측정
  long net = raw - tare_offset; // 현재 offset을 빼서 NET 계산
  float known_weight = 155.0f; // 기준 무게를 155g로 설정

  calibration_factor = known_weight / (float)net; // factor = 155 / NET → g/카운트 단위
  saveCalibration(calibration_factor); // EEPROM에 저장(전원 꺼져도 유지)

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Cal Done!");
  lcd.setCursor(0, 1); lcd.print("Factor=");
  lcd.print(calibration_factor, 6);
  delay(2000);

  Serial.print("[CAL] factor=");
  Serial.println(calibration_factor, 6);
}

// === 안정도 측정 함수 ===
// 10번(STABLE_SAMPLING) 반복하면서 HX711 준비를 최대 300ms 기다린 후 read()로 샘플을 연속으로 읽어 배열에 저장
bool isStable(long* avg_out) {
  long samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false;
    samples[i] = scale.read();
  }

  // 10개 샘플 중 최소값/최대값/합계를 계산
  long minV = samples[0], maxV = samples[0];
  long sum = 0;
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  if (avg_out) *avg_out = sum / STABLE_SAMPLING; // 평균값이 필요하면 *avg_out에 넣어줌
  long diff = maxV - minV; // 최대–최소 차이가 STABLE_DIFF_MAX(200)보다 작으면 true(안정), 아니면 false
  return (diff < STABLE_DIFF_MAX);
}

