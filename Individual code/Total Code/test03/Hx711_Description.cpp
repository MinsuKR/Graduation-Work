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

#include <Arduino.h> // Arduino 코어 함수
#include <Wire.h> // I2C 통신 라이브러리
#include <HX711.h> // HX711 전용 라이브러리(로드셀 ADC 읽기)
#include <LiquidCrystal_I2C.h> // I2C 타입 16×2 LCD 제어 라이브러리
#include <EEPROM.h> // 보드 내부 EEPROM에 값 저장/불러오기용
#include <math.h> // AVR에선 이미 포함되어 있는 경우가 많다 , 그래도 안전하게 쓰려면 사용, fabs, isnan, isinf 등 수학 함수

// === LCD/핀 정의 ===
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

#define HX711_DT   PIN_PD3    // HX711 DOUT
#define HX711_SCK  PIN_PD2    // HX711 SCK
#define TARE_BTN   PIN_PE4    // 영점 버튼 (GND와 연결, 눌리면 LOW)
#define CAL_BTN    PIN_PE5    // 캘리브레이션 버튼 (GND와 연결, 눌리면 LOW)

// EEPROM 주소
#define EEPROM_ADDR 0 // EEPROM에 보관할 위치(주소 0)

// === 튜닝용 상수들 ===
const uint8_t STABLE_SAMPLING   = 10;    // 안정도 판단용 샘플 개수
const uint8_t AVG_TARE_SAMPLES  = 15;    // TARE 시 평균 샘플 개수
const uint8_t AVG_CAL_SAMPLES   = 25;    // 캘리브레이션 시 평균 샘플 개수

const long  STABLE_DIFF_MAX = 200;       // 최근 샘플의 max-min 허용 범위(카운트)
const float ZERO_SNAP_G     = 2.0f;      // ±2g 이내는 0g로 스냅

// 캘리브레이션 팩터에 대한 안전 범위
const float FACTOR_DEFAULT  = -0.067f;   // 기본 환산 계수(카운트 → g)
const float FACTOR_ABS_MIN  = 1e-5f;     // 0에 너무 가까운 값(거의 0) 보호
const float FACTOR_ABS_MAX  = 1.0f;      // 비정상적으로 큰 값 보호

// === 표시용 필터 변수 ===
// DisplayW : LCD에 보여줄 필터링된 무게 값
// PrevW    : 직전 측정 원본 무게(필터 판단 조건에 사용)
float DisplayW = 0.0f;
float PrevW    = 0.0f;

// === LCD 객체 ===
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// === HX711 객체 ===
HX711 scale;
long  tare_offset = 0; // 영점 오프셋(ADC 카운트)
float calibration_factor = FACTOR_DEFAULT; // 카운트 → g 환산 계수

// === 함수 프로토타입 ===
void  saveCalibration(float value);              // EEPROM 저장
float loadCalibration();                         // EEPROM 로드(+이상치 복구)
bool  waitReady(uint16_t timeout_ms = 100);      // HX711 준비 대기
void  doTare(uint8_t avg = AVG_TARE_SAMPLES);    // 자동 영점
void  doCalibration();                           // 캘리브레이션 루틴
bool  isStable(long* avg_out = nullptr);         // 안정도 측정

// === SETUP ===
void setup() {
  // 버튼은 풀업으로 사용 (기본 HIGH, 눌리면 GND로 연결되어 LOW)
  pinMode(TARE_BTN, INPUT_PULLUP);
  pinMode(CAL_BTN, INPUT_PULLUP);

  // === LCD 초기화/백라이트 ON/부팅 메시지 ===
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 + LCD");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // === HX711 초기화 ===
  scale.begin(HX711_DT, HX711_SCK); // HX711을 DT/SCK 핀으로 시작
  scale.set_gain(128);              // A채널 게인 128로 설정

  // === 캘리브레이션 팩터 EEPROM에서 불러오기(이상치면 기본값으로 복구) ===
  calibration_factor = loadCalibration();

  // // HX711 준비 대기 > 배선/전원이 이상할 때 초기에 알림을 줌
  // uint32_t t0 = millis(); // 지금 시각(ms) 저장, 기준점
  // while (!scale.is_ready() && (millis() - t0 < 3000)) {} // false(= 준비 안 됨, HX711의 DOUT이 HIGH)인 동안, 그리고 경과 시간이 3초 미만인 동안 계속 대기
  // if (!scale.is_ready()) // 루프를 빠져나왔는데도 아직 준비가 안 됐다면(= 배선문제, RATE 설정 문제, 전원문제)
  // {
  //   lcd.clear();
  //   lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
  //   lcd.setCursor(0, 1); lcd.print("Check Wiring");
  //   delay(1500);
  // }
  
  // 리팩터 포인트: 부팅 준비 대기 → waitReady(3000) 한 줄로 통일
  if (!waitReady(3000)) {   // 3초 안에 준비 못 하면 경고
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    delay(1500);
  }

   // === 자동 안정도 확인 ===
  // 부팅 직후 로드셀 출력이 출렁일 수 있으므로, 일정 수준 이하로 들어올 때까지 대기
  long     avgRaw     = 0;
  uint32_t startTime  = millis();

  while (!isStable(&avgRaw)) { // 최근 RAW가 덜 흔들릴 때까지 반복
    lcd.setCursor(0, 0);
    lcd.print("RAW=");
    lcd.print(avgRaw);
    lcd.print("    "); // 잔상 지우기

    lcd.setCursor(0, 1);
    lcd.print("Stabilizing...");
    lcd.print("   ");

    delay(500);
    if (millis() - startTime > 10000) break; // 10초 넘으면 포기하고 다음 단계
  }

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("RAW Stable OK!");
  delay(800);

   // === 자동 영점(TARE) ===
  // 빈 저울 상태에서 평균 RAW를 읽어 tare_offset으로 저장
  doTare();

   // 현재 사용 중인 calibration_factor를 한 번 보여줌
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Factor:");
  lcd.setCursor(0, 1); lcd.print(calibration_factor, 6); //소수점 이하 자리수 지정(6자리 0.000000)
  delay(1000);
}

// === LOOP ===
void loop() {
  // 주기 제어용 타이머 (마지막 갱신 시각 저장)
  static uint32_t last = 0;

  if (millis() - last >= 300) {
    last = millis(); // 300ms마다 표시 갱신

    if (waitReady(100)) {  // 100ms안에 준비가 되면
      long raw = scale.read_average(25); // 평균 Raw 25회 읽기
      long net = raw - tare_offset; // net 계산 
      
      // === 카운트를 g 단위로 변환 ===
      float weight = (float)net * calibration_factor; // g단위로 변환, weight = net * factor


      // === 노이즈 억제 ===
      if (isnan(weight) || isinf(weight)) weight = 0.0f;
      //if (fabs(weight) < 2.0) weight = 0.0; // 수학적 예외 처리 + ±2 g 이내는 0으로 스냅

      // === 저울처럼 "딱딱" 보이게 하는 표시용 필터 ===
      //  - DisplayW : 화면에 실제로 보여줄 필터링된 값
      //  - PrevW    : 직전 생(raw→g)의 값
      float absW       = fabs(weight);
      const float ALPHA = 0.3f;  // 0~1, 클수록 반응 빠름 / 작을수록 부드러움

      if (absW < ZERO_SNAP_G) {
        // ① 2g 이하는 그냥 0으로 스냅 → 빈 저울에서 ±몇 g 출렁임 제거
        DisplayW = 0.0f;
      } else if (fabs(PrevW) > 20.0f && absW < 10.0f) {
        // ② 방금 전에는 20g 이상이었는데, 지금은 10g 미만이면,
        //    "물건을 막 치운 상황"으로 판단하고 바로 0으로 떨어뜨림
        DisplayW = 0.0f;
      } else {
        // ③ 그 외에는 지수 이동 평균(EWMA)으로 부드럽게 필터링
        //    DisplayW(n) = ALPHA * 현재값 + (1-ALPHA) * 이전 DisplayW
        DisplayW = ALPHA * weight + (1.0f - ALPHA) * DisplayW;
      }
      PrevW = weight;  // 다음 판정을 위해 현재 원본 weight 저장

      // === 0.5g 단위로 반올림 ===
      //   * 필요하면 1g 단위 등으로 쉽게 변경 가능
      float shown = roundf(DisplayW * 2.0f) / 2.0f;
      
      lcd.setCursor(0, 0);
      lcd.print("W:");
      lcd.print(weight, 1);
      //lcd.print(shown, 1);       // 필터링 + 0.5g 반올림된 값을 표시
      lcd.print(" g      "); // 잔상 지우기

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
  EEPROM.put(EEPROM_ADDR, value); // 바이트 단위로 EEPROM에 씀
}

// === EEPROM 불러오기 (비정상값 자동복구 포함) ===
float loadCalibration() {
  float val;
  EEPROM.get(EEPROM_ADDR, val); // EEPROM에서 바이트 단위로 읽어 변수 채움

  // 비정상 값 필터링 
  if (!isfinite(val) || fabs(val) < 1e-5f || fabs(val) > 1.0f) // 무한대/NaN || 0에 너무 가까우면 복구 || 너무 큰 값(이상치) 복구
  { 
    return FACTOR_DEFAULT;  // 안전 기본값 복구 (무한/NaN/0/너무 큰 값)
  }
  return val; // 아니면 저장값 반환
}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms = 100) // 100은 디폴트 값 따라서 waitReady(1000);는 1000으로 덮어씀
{ 
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scale.is_ready()) return true; // Hx711가 현재 시간에서 0.1초안에 준비되면 True
  }
  return false; // Hx711 준비되면 True
}

// === 영점 조정 ===
void doTare(uint8_t avg) // waitReady와 비슷한 디폴트 값 + 노이즈가 심하면 바꿔도 됨
{
  if (!waitReady(800)) return; // 최대 800ms 대기해 준비 안 되면 영점 스킵

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  // avg번 읽어 평균한 RAW(HX711의 24bit ADC 카운트)를 획득
  // → 이 값을 영점 기준 offset으로 사용
  long raw = scale.read_average(avg); // avg번 읽어 평균한 RAW(Hx711의 24bit ADC 카운트)를 획득 / 로드셀의 **미세한 차동전압(ΣΔ 변조)**을 디지털 카운트로 변환한 값(아날로그 값은 아님)
  
  // 실제 영점 오프셋 저장
  tare_offset = raw; // 얻은 raw값을 영점 tare_offset에 저장
  scale.set_offset(tare_offset);  // 라이브러리 내부 offset도 동기화
  
  // 표시용 필터/이전 값도 초기화 (영점 후 바로 0g에서 시작하도록)
  DisplayW = 0.0f;
  PrevW    = 0.0f;

  lcd.setCursor(0, 1);
  lcd.print("Offset:");
  lcd.print(tare_offset);
  lcd.print("   ");
  delay(700);
  lcd.clear();
}

// === 캘리브레이션 ===
void doCalibration() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put weight (155g)");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 155 g 기준무게 3초 대기

  if (!waitReady(1000)) return; // 1초 안에 준비 안 되면 캘리브레이션 중단

  long raw = scale.read_average(AVG_CAL_SAMPLES); // 25번 평균 Raw값 읽고 + 노이즈 감소
  long net = raw - tare_offset; // 영점 보정(순카운트 net) 계산 + 직전에 TARE에서 tare_offset = raw;로 저장했으므로, 그 즉시엔 net=0 이후 시간이 지나 무게가 변하면 raw가 변화고 net이 바뀜
  
  float known_weight = 155.0;  // 기준 무게(g)

  // 표준식: weight = net * factor 가 되도록
  calibration_factor = known_weight / (float)net; 

  // 새 팩터를 EEPROM에 저장
  saveCalibration(calibration_factor); 

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Cal Done!");
  lcd.setCursor(0, 1); lcd.print("Factor=");
  lcd.print(calibration_factor, 6);
  delay(2000);
}

 // === 안정도 측정 함수 ===
bool isStable(long* avg_out) // 포인터가 nullptr가 아니면 *avg_out = 평균 저장, nullptr면 평균을 안 돌려줌
{
  long samples[STABLE_SAMPLING];

  // STABLE_SAMPLING 회 연속 샘플 수집
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false; // 각 샘플마다 최대 300ms 대기
    samples[i] = scale.read();         // 단일 RAW 읽기
  }

  // === 최소/최대/합 계산 ===
  long minV = samples[0];
  long maxV = samples[0];
  long sum = 0;

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  // if (avg_out) = avg_out != nullptr 즉, 포인터가 전달됐을 때만 평균을 써줌
  if (avg_out) {
    *avg_out = sum / STABLE_SAMPLING;
  }  

  long diff = maxV - minV; // diff = maxV - minV 최근 10샘플의 최대-최소 = 흔들림 크기
  
  return (diff < 200); // 200 counts 이내면 안정으로 간주 + 임계값은 환경에 맞게 조정 가능
} 
