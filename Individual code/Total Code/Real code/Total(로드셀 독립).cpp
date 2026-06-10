// bool : 논리형(True / False)값
// millis() : 부팅 후 지난 시간을 ms 단위로 반환 / 내부적으로 타이머0 인터럽트로 증가시키는 전역 카운터를 읽음 / loop상관없이 계속 증가
// fabs() : 실수(double,float) 절대값 & abs() : 정수(int) 절대값 
// isnan(Not a Number) : 무한(NaN)이면 1 , 아니면 0 & "숫자가 아님" ex) 0/0
// isinf(Infinity) : +∞면 1, −∞면 −1, finite면 0 & "무한대" ex) 1/0
// isfinite : 유한(NaN X)이면 True <-> !isfinite 무한이면 True
// -> 실수형(float/double)에만 의미가 있으며 정수형은 항상 참이 되므로 효과x
// lcd.print("  "); : 여유 공백을 출력해서 이전 문자(잔상)를 지워줌
// nullptr : C++에서 어떤 객체도 가리키지 않는 포인터  + 비어있음을 의미
// <파일명.h> = 표준 라이브러리 /  "파일명.h" = 내가 만든 헤더 파일 
// #ifndef, #define, #endif = 헤더 중복 포함 방지 즉, 같은 헤더가 여러 번 포함(Include)되더라도 단 한 번만 처리되게 한다.
// diff : 부팅 직후/손으로 건드릴 때/진동 있을 때 raw가 요동치면 그 상태로 tare/cal 하면 오프셋이 틀어짐. 그래서 “지금 센서가 흔들리지 않는다”는 기준이 필요
// ZERO_SNAP_G : 너무 크면(5g) = 작은 무게를 못 읽음 / 너무 작으면(0.5g) = 0근처에 흔들림이 보임
// votile : ISR(인터럽트)에서 이 변수들을 바꾸는 코드일 때 사용 / 다른 경우 사용 x
// delay(1) : While문 안에서 쓰는 이유는 CPU 점유를 줄이고, 다른 작업(시리얼 버퍼 처리 등)도 조금 더 여유가 생김
// 원래 While은 준비될 때까지 CPU를 100%로 계속 돌리는 바쁜 루프(busy-wait)다.
// static : 함수가 끝나도 값 유지
// strncpy : 길이 제한 복사(오버플로 방지) + strcpy는 buf이 20 넘으면 메모리가 깨짐
// const : 값이 바뀌지 않게 해줌
// unsigned : 부호없는 (+)만있는 32비트
// ! : 부정 (ex) true -> false)
// 캘리브레이션 : 측정 기기나 시스템이 나타내는 값이 **표준값(참값)**과 일치하도록 조정하거나, 그 오차를 확인하는 과정
// HX711 → Wtotal(연산) -> DisplayW (표시 안정화) -> g_lastShown (사용자 확정 값) -> SEND → ESP32 -> ACK 수신 -> 화면 잠금 + 피드백
// '\0' - C문자열은 "널문자"로 끝나야함 & 문자열 끝 표시 & 없으면 계속 읽음 


// 기본 표준 함수
#include <Arduino.h> //Arduino 기본 함수들(pinMode, digitalWrite, millis 등)
#include <Wire.h> // I2C 통신 라이브러리 + I2C 장치(LCD)랑 통신하려고 필요
#include <HX711.h> // HX711(로드셀 ADC읽기) 제어 라이브러리 + 로드셀 값(24bit RAW)을 읽어오기 위한 핵심
#include <LiquidCrystal_I2C.h> // I2C 타입 16×2 LCD 제어 라이브러리 + LCD에 표시하는 모든 동작
#include <EEPROM.h> // 보드 내부(factor) EEPROM에 값 저장/불러오기용
#include <math.h> // 무게 계산 안정화(이상값 방지 + 반올림) & fabs, isnan, isinf, roundf 같은 수학 함수 사용하기 위한 라이브러리(AVR에선 이미 포함되어 있는 경우가 많지만 안전하게 사용하기 위해)
#include <stdint.h> // int32_t 등등 함수 사용하기 위한 라이브러리 + 정확한 비트폭을 가진 정수 타입
#include <string.h> // strncpy 함수 사용하기 위한 라이브러리 + C 문자열 처리 함수(Serial로 받은 텍스트(ACK)를 문자열로 검사/복사)


//사용자 헤더 함수
#include "LED.h"  // LED 헤더 
#include "Buzzer.h" // Buzzer 헤더
#include "Button.h" // Button(tare,cal) 헤더

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
#define DT_L   PIN_PD3 // DOUT
#define SCK_L  PIN_PD2 // SCK

// 오른쪽 HX711
#define DT_R   PIN_PD5 // DOUT
#define SCK_R  PIN_PD4 // SCK

// EEPROM 바이트 단위 저장 공간 (연속 주소 저장) 
// float(4바이트), long(4바이트)라서 4바이트 간격으로 배치
#define EE_FACTOR_L   0    
#define EE_FACTOR_R   4
#define EE_A_COEF     8
#define EE_B_COEF     12
#define EE_TARE_L     16
#define EE_TARE_R     20

// ==== 로드셀 기본 보정 계수 ====
// HX711 raw 값 → g 단위로 바꾸는 스케일링 계수
// Cal = Known_Weight(CAL_WEIGHT) / Net(Raw - Offset) 
float factorL = -0.005398f;
float factorR = -0.004933f;

// ==== 영점 오프셋 ====
// long -> int32_t
int32_t offsetL = 0;
int32_t offsetR = 0;

// ==== 총무게 보정용 계수(가중치 보정) ====
// Wtotal = a*L + b*R
// 특이 케이스: 거의 대칭이라면 일단 a=b=1로 사용
float a_coef = 1.0f;  
float b_coef = 1.0f;

// 샘플링/안정도/스냅 파라미터
const uint8_t STABLE_SAMPLING   = 10; // 안정도 검사할 때 몇 샘플을 모을지
// 즉, HX711 값을 10번 연속으로 읽고, 10개 값 중 최대값 − 최소값 < STABLE_DIFF_MAX 이면 안정이라 판단
// 너무 적으면 → 우연한 노이즈에 속음 & 너무 많으면 → 부팅·대기 시간이 길어짐

const uint8_t AVG_TARE_SAMPLES  = 20; // Tare 할 때 평균 내는 샘플 수
// 즉, 20번 평균 → 노이즈 제거 + 안정적인 영점
// 정확도가 중요하므로 AVG_MEAS_SAMPLES  = 5; 보다 많아야함

const uint8_t AVG_CAL_SAMPLES   = 30; // 캘리브레이션 때 평균 내는 샘플 수
// 즉, 저울 평생 정확도를 좌우 + 한 번 잘못 잡히면 계속 오차 누적 + 시간이 길어야 정확함

const uint8_t AVG_MEAS_SAMPLES  = 5; // 평소 측정에서 평균낼 샘플 수
// 즉, 너무 적으면 → 숫자가 덜덜 떨림 & 너무 많으면 → 반응이 느려짐

const int32_t  STABLE_DIFF_MAX = 500; // 안정하다고 인정할 최대 흔들림 폭(RAW 기준)
// 즉, isStable()에서 10번 샘플을 모아 (max - min) 값이 500보다 작으면 “안정” 

const float ZERO_SNAP_G     = 2.0f; // 0 근처 튐(±0.x g)을 0으로 붙이는 스냅
// 즉, 빈 저울에서도 0.3g, -0.7g, 1.1g 같은 값이 튐 따라서 0으로 고정시킴

const int32_t OFFSET_LIMIT = 10000000; // ±10M (HX711 24bit(±8,388,608 근처) 기준)
// 즉, EEPROM에서 불러온 영점(offset)이 정상 범위인지 검사하는 한계값(EEPROM 깨짐 방어 장치)

// ==== 캘리브용 파라미터 ====
const float CAL_WEIGHT = 155.0f; // 캘리브레이션 하기 위한 기준 무게 값(변경가능)
const float FACTOR_ABS = 1e-6f; //보정 계수(factor)의 최소 절댓값 기준
// 즉, a,b의 factor가 0에 가깝거나 깨진 값(NaN/Inf)이면 계산이 폭주함 따라서 1로 맞춤

// 표시용 필터 상태값
float DisplayW = 0.0f; // 저울 값이 LCD표시에 부드럽게 하기 위한 필터링된 무게 값
// 실제 계산값(Wtotal)은 잡읍 + 순간 변화가 큼 따라서, DisplayW 이용하여 ZERO_SNAP 적용

// 무게저장 전역변수 + 실제 측정 무게를 보증해주는 플래그 역할 true시 버튼 허용
float g_lastShown = 0.0f;     // 가장 최근에 계산된 표시 무게 저장
// 즉, 버튼 누르는 순간 계산 중이면 이상한 값 전송 따라서 표시된 값을 보냄
bool  g_weightValid = false;  // 측정값 유효 여부(초기 이상한 값 방지)
// 즉, 아직 false로 정상x 현재 무게 값이 “정상적으로 계산된 상태인지” 확인 후 true가 되면 정상

// ACK 보이는 전역변수
bool g_waitingAck = false;   // SEND 후 ACK 대기 까지 화면 잠금(측정화면이 덮어쓰지 못하게)
// 아직 false로 정상x 현재 ESP32의 ACK 응답을 기다리는 상태 확인 후 true가 되면 측정 화면이 SEND/ACK 화면을 덮어쓰지 못함
// 즉, 평상시에는 새로운 무게 값 계산 LCD표시 , true시 측정값을 계속 갱신하는 코드가 LCD에 글자를 다시 쓰지 못하게 막는다는 뜻(SEND화면을 씀)

bool g_ackReceived = false;  // ACK 수신 여부
// 즉, ACK 수신 받으면 true

uint32_t g_ackHoldUntil = 0;          // ACK 받은 뒤 몇 ms 동안 ACK 화면 유지할지
// delay사용x , millis기반

char g_lastAck[20] = {0};             // LCD에 표시할 ACK 문자열 저장
// 즉, ESP32가 보낸 ACK 문자열 저장 버퍼 + AVR환경 char[] + strncpy()가 안정적

// === 함수 선언 ===
bool waitReady(uint16_t timeout_ms = 100); 
// HX711 두 개가 모두 ready 상태가 될 때까지 기다리는 함수 / timeout_ms = 최대 몇까지 기다릴지?
// true -> 모두 준비 완료 & false -> 시간 초과


// doTare & doTareAndSave 나눈 이유?
// EEPROM은 “쓰기 횟수 제한”(ATmega128 = 100,000)로 사용 빈도를 아껴야함
// 매번 저장하면 환경의 영향으로 다음 부팅에 이상한 영점이 잡힐 수 있음 + 안정적이다고 생각 시 저장 가능
void doTare(uint8_t avg = AVG_TARE_SAMPLES); 
// 영점(Tare)잡는 함수 & avg수 만큼 평균을 잡음
// EEPROM 저장 x

void doTareAndSave(uint8_t avg = AVG_TARE_SAMPLES); 
// 영점(Tare)잡는 함수 & avg수 만큼 평균을 잡음
// EEPROM 저장 o

void doCalibration();
// 기준 무게로 보정값 계산 후 저울 표준 값과 일치하도록 만듬
// EEPROM 저장 o

bool isStable(int32_t* avg_out = nullptr);
// 현재 상태가 안정적인지 판단 & 부팅직후 흔들림 제거를 위함
// true -> 안정 & false -> 불안정

void measureLR(uint8_t samples, float &outL, float &outR);
// 좌/우 로드셀을 동시에 여러 번 측정해서 평균 g값을 반환
// 함수 밖 변수에 직접 결과를 써주기 위해(return하나로 L/R값을 둘다 못내보냄)

void handleEspAckOnLcd();
// ESP32에서 오는 ACK 문자열 처리 + LCD 반영
// Seial 수신 + ACK 검사 + 통신 UI처리

void saveAllToEEPROM(); 
// 현재 장치의 보정값들을 EEPROM(영구메모리)에 저장하는 함수

void loadAllFromEEPROM(); 
// EEPROM에서 저장해둔 보정값을 읽어오는 함수

void setup() 
{
  // ESP32 UART0 통신 시작
  // 이후 Serial.print(), Serial.available(), Serial.read() 가능하게 함
  Serial.begin(9600);  

  // LED / Buzzer / Button 모듈 초기화
  LED_Init();
  Buzzer_Init();
  Button_Init();

  // 부팅 알림
  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();

  // LCD 초기화
  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 Dual a,b");
  lcd.setCursor(0, 1); lcd.print("Init...");

  // HX711 핀 지정 후 시작 + gain 채널 A 128배 증폭(표준)
  // gain = 128 이유?
  // Gain 값에 따라 ADC가 받아들일 수 있는 입력 전압 범위(FSR)가 달라짐 = "허용 입력 범위"(증폭 비율x)
  // ex) 128(20mV) => 24bit로 표현 => 비트가 가장 작고 해상도 최고 
  // 로드셀 정격(1mV -> 수백 uV ~ 수mV) = 128이 포화가 안되고, 분해능이 최고 (미세변화를 확인해야함)
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);
  scaleL.set_gain(128);
  scaleR.set_gain(128);

  // 이전 보정값 복구
  loadAllFromEEPROM();
  // 안정화 시간
  delay(500);

   // HX711 준비 대기
   // 3초 동안 HX711이 ready 되는지 확인 후 준비 안 되면 LCD에 에러 메시지, LED ERROR 상태, 에러 부저
   // waitReady(3000) - 3초동안 준비가 됐는지 확인 후 true반환 !붙으면 false 반환 
   if (!waitReady(3000)) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  // 자동 안정도 확인
  // 초기 흔들림이 안정될 때까지 RAW 안정화 대기 후 대기 시작 시간 저장
  int32_t avgRaw = 0;
  uint32_t startTime = millis();

  // isStable()가 true가 될 때까지 반복하면서 “안정화 대기”
  // "avgRaw 변수의 주소”를 넘겨서, 함수가 평균 raw 값을 밖으로 돌려주게 함
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

  // === 영점잡기 (빈 상태에서) ===
  doTare();

  // 정상 대기 상태 LED
  LED_Set(LEDSTATE_IDLE);

}

void loop() 
{
  // stactic으로 last값 유지
  static uint32_t last = 0;

  // 50ms 주기로 무게 갱신
  if (millis() - last >= 50) // 300 -> 50 바꿈 
  {
    // 루프가 1번 끝나면 이 값을 유지 
    last = millis();

    // HX711 준비되면 읽기
    if (waitReady(100)) {
        
      // Weight(g) = Net(RAW - Offset) x factor

      // raw : HX711 ADC 값
        int32_t rawL = (int32_t)scaleL.read_average(AVG_MEAS_SAMPLES);
        int32_t rawR = (int32_t)scaleR.read_average(AVG_MEAS_SAMPLES);

      // net : offset(영점) 빼기
        int32_t netL = rawL - offsetL;
        int32_t netR = rawR - offsetR;

      // wL/wR : 계수 곱해서 g로 변환
        float wL = (float)netL * factorL;
        float wR = (float)netR * factorR;

      // 위치 보정 포함한 총무게
        float Wtotal = a_coef * wL + b_coef * wR;

      // 연산이 망가지거나 이상할 경우 방지
      // 숫자가 아니거나 무한대면 0으로 만듬
        if (isnan(Wtotal) || isinf(Wtotal)) Wtotal = 0.0f;

        // 절댓 값을 구함
        float absW = fabs(Wtotal);

      // 0 근처는 스냅(바로 0으로)
      // 빈 저울에서 0.3g, -0.7g 같은 잡노이즈가 LCD에 계속 뜨는 걸 막기 위한 처리(2   )
      if (absW < ZERO_SNAP_G) {   // ZERO_SNAP_G = 2.0f 유지
        DisplayW = 0.0f;
      }
      // 2) 나머지는 필터 없이 그대로 사용 (즉각 반응)
      else {
        DisplayW = Wtotal;
      }

      // x.x g 단위로 표현(소수 1자리 반올림)
      // ex) 12.34 -> 123.4 -> roundf() -> 123 -> 12.3
      float shown = roundf(DisplayW * 10.0f) / 10.0f;

      // 지금 계산한 값을 “SEND 버튼에서 쓸 수 있게” 전역으로 저장
      g_lastShown = shown;
      g_weightValid = true;

      // SEND 버튼 누른 뒤 / ACK 유지 시간에는 LCD를 덮어쓰지 않음 -> SEND화면 유지
      // SEND 후 ACK 기다릴 때(g_waitingAck=true) 또는 ACK 받고 1.2초 유지 시간(g_ackHoldUntil) 동안 측정 화면이 SEND/ACK 화면을 덮어쓰지 못하게 막음
      bool lockScreen = g_waitingAck || (millis() < g_ackHoldUntil);

      // ===== LCD 출력 =====
      // 평상시 g_waitingAck = false -> send 버튼 누르면 g_waitingAck = true -> SEND LCD 화면 사용
      if (!lockScreen)  // 차단스위치 역할
      {
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
      }

    } else {
      lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
      lcd.setCursor(0, 1); lcd.print("...            ");
    }
  }

  // ESP32 응답(ACK) 수신 처리
  // eSend 버튼 안에말고 loop안에 쓰는 이유?
  // 즉, ESP32의 네트워크/Wi-Fi 환경의 응답지연으로 인해 버튼 누를시(eSend)에 넣으면 ACK수신이 1번만 실행되어 
  // 수신을 놓칠 가능성이 커서 항상(loop) 해주지않으면 ACK가 도착했을 때는 eSend를 벗어난다.
  // ACK는 버튼 이벤트 내부에서 발생하는 동기 이벤트가 아니라 통신 지연이 포함된 "비동기 이벤트"여서 지속적으로 Serial감시 필요
  handleEspAckOnLcd();

  // ===== 버튼 처리 =====
  // loop 안에 있는 이유?
  // 지금 버튼 상태이고 Button_Read는 주기적으로 호출되어야함 & c++ 전역에서 if (eSend==...) 같은 “실행문”을 둘 수 없음
  // 폴링(polling 방식)

  // TARE 버튼
  ButtonEvent eTare = Button_ReadTare(); // 지금 버튼 상태 읽음
  if (eTare == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    doTare(); // ★ 저장 X
    Buzzer_Click();
    LED_Set(LEDSTATE_IDLE);
  } else if (eTare == BTN_LONG) {
    // 필요하면: 장기 TARE 기능 넣기
    LED_Set(LEDSTATE_PROCESSING);
    doTareAndSave();  // ★ 저장 O
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

  // ===== SEND 버튼 (무게 전송) =====
  ButtonEvent eSend = Button_ReadSend();
  if (eSend == BTN_SHORT) {   
   // 화면 잠금 시작
  g_waitingAck = true;
  g_ackReceived = false;

  // 1행 SEND 표시
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SEND ");
  lcd.print(g_lastShown, 1);
  lcd.print("g   ");  // 뒤 지우기용

  // 2행: RX 대기 표시
  lcd.setCursor(0, 1);
  lcd.print("RX: waiting... ");

  // 전송
  Serial.print("W=");
  Serial.println(g_lastShown, 1);

  // LED/Buzzer 피드백
  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();
  }

}

// === HX711 준비 대기 ===
// t0에 시작 시간 저장 후 (현재 - 시작) 반복 
bool waitReady(uint16_t timeout_ms) {
  uint32_t t0 = millis(); // t0 = millis()로 시작 시각 저장
  while (millis() - t0 < timeout_ms) // timeout_ms 동안 반복
  {
    if (scaleL.is_ready() && scaleR.is_ready()) return true;
    delay(1); // busy-wait(바쁜 대기) 완화(안정성) & CPU를 1ms 쉬게 함
  }
  return false;
}

// === 영점 조정 ===
void doTare(uint8_t avg) {
  if (!waitReady(800)) return; // 800ms 안에 HX711 준비가 안 되면 그냥 종료(안정성)

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Auto TARE...");

  
  // 평균으로 tare하면 노이즈 감소
  // offset이 이후에 raw - offset으로 영점 제거에 사용
  offsetL = scaleL.read_average(avg);
  offsetR = scaleR.read_average(avg);

  // EEPROM은 “쓰기 횟수 제한"이 있어 자주쓰면 수명이 줄어듬
  // Tare은 사용자가 자주 사용하므로 임시/영구를 나눌 가치가 큼
  //saveAllToEEPROM(); 

  lcd.setCursor(0, 1); lcd.print("Done");
  delay(700);
  lcd.clear();
}

// === 영점 조정 + EEPROM 저장 (저장 O) ===
void doTareAndSave(uint8_t avg) {
  doTare(avg);          // offsetL/R 갱신
  saveAllToEEPROM();    // ★ 롱프레스일 때만 저장
  lcd.setCursor(0, 0); lcd.print("Auto TARE Val");
  lcd.setCursor(0, 1); lcd.print("Saved");
  delay(500);
  lcd.clear();
}

// === 캘리브레이션 ===
void doCalibration() {

  // Cal_F = Known_Weight(CAL_WEIGHT) / Net(Raw - Offset) 

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put weight (155g)");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);

  if (!waitReady(1000)) return; // 1000ms 안에 HX711 준비가 안 되면 그냥 종료(안정성)

  // === 왼쪽 끝에 155g 올려서 측정 ===
  float L_left = 0.0f, R_left = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g LEFT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 올릴 시간

  measureLR(AVG_CAL_SAMPLES, L_left, R_left);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L_left=");
  lcd.print(L_left, 1);
  lcd.setCursor(0, 1); lcd.print("R_left=");
  lcd.print(R_left, 1);
  delay(1500);

  // === 오른쪽 끝에 155g 올려서 측정 ===
  float L_right = 0.0f, R_right = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g RIGHT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);  // 옮길 시간

  measureLR(AVG_CAL_SAMPLES, L_right, R_right);

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

  if (fabs(det) < FACTOR_ABS) {
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
// 샘플 수에 따라 raw(좌+우)가 “최대-최소”가 500 미만이면 안정이라고 판단
bool isStable(int32_t* avg_out) // 안정한지 검사해서 true/false 돌려주고, avg_out가 nullptr이 아니면 평균 값을 줌
{
  int32_t samples[STABLE_SAMPLING]; // 배열을 만들고, 안정도 판단을 위한 raw 샘플들을 저장할 공간을 확보

  // 샘플을 채움
  for (int i = 0; i < STABLE_SAMPLING; i++) 
  {
    if (!waitReady(300)) return false; // HX711이 준비 안 되면 안정도 판단을 못 하니까 바로 false 반환
    int32_t rawL = (int32_t)scaleL.read();
    int32_t rawR = (int32_t)scaleR.read();
    samples[i] = rawL + rawR; // 전체 하중이 안정적인지 확인하기 위해
  }

  // 샘플의 최소/최대/평균을 계산
  int32_t minV = samples[0], maxV = samples[0];
  int32_t sum = 0;
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) 
    minV = samples[i];
    if (samples[i] > maxV)
    maxV = samples[i];

    sum += samples[i];
  }

  if (avg_out) 
  *avg_out = sum / STABLE_SAMPLING; // avg_out이 Null이 아니면 평균 값 *avg_out써줌 & 평균 raw를 얻음
  int32_t diff = maxV - minV; // 샘플의 “흔들림 폭” 계산

  return (diff < STABLE_DIFF_MAX); // 흔들림이 기준보다 작으면 안정(true), 크면 불안정(false)
}

// ==== 평균 측정 함수 ====
// (가능한 만큼) 여러 번 읽어 평균 → 영점 제거 → g로 변환 → outL/outR로 반환
void measureLR(uint8_t samples, float &outL, float &outR) 
{
  // outL = 하나의 로드셀의 왼쪽에 가해진 값
  int32_t sumL = 0; // 왼쪽 raw 합
  int32_t sumR = 0; // 오른쪽 raw 합
  uint8_t got = 0; // 몇 개를 성공적으로 모았는지 
  uint32_t t0 = millis();

  while (got < samples) // samples개 모을 때까지 반복
  {
    if ((uint32_t)(millis() - t0) > 3000UL) break; // 3초 넘으면 break → 무한 대기 방지

    if (!waitReady(200)) continue; // HX711 ready가 아니면 skip하고 다시 시도(continue)

    // ready일 때만 읽기
    int32_t rawL = (int32_t)scaleL.read();
    int32_t rawR = (int32_t)scaleR.read();
    sumL += rawL;
    sumR += rawR;
    got++;
    delay(5);
  }

  if (got == 0) { outL = 0; outR = 0; return;} // ready가 계속 안 떠서 샘플을 하나도 못 모았으면 out을 0으로 두고 종료

  // sample -> got 변경
  int32_t avgRawL = sumL / got;
  int32_t avgRawR = sumR / got; 

  // offset 빼서 net 만듬
  int32_t netL = avgRawL - offsetL;
  int32_t netR = avgRawR - offsetR;

  // factor 곱해서 g로 변환해서 밖으로 전달
  outL = (float)netL * factorL;
  outR = (float)netR * factorR;
}

// ===== ESP32로부터 ACK 수신해서 LCD에 표시 =====
// Serial로 들어오는 문장을 한 줄 단위로 모아서 처리
void handleEspAckOnLcd() 
{
  static char buf[48]; // 수신 문자열을 모아두는 버퍼
  static uint8_t idx = 0; // buf에 몇 글자까지 쌓였는지 위치

  while (Serial.available()) // 시리얼 버퍼에 읽을 데이터가 남아있는 동안 계속 처리(Serial.available()가 0이 될 때까지 반복)
  { 
    char c = (char)Serial.read(); // 시리얼에서 문자 1개 읽기(한 글자 씩 읽음)
    if (c == '\r') continue; // \r(캐리지리턴)은 무시하고 \n만 줄끝으로 사용

    if (c == '\n') // 줄바꿈 \n을 만나면 “한 줄 수신 완료”로 판단
     {
      buf[idx] = '\0'; // C 문자열의 끝 표시(널 종료) / buf를 문자열로 처리하려면 반드시 필요
      idx = 0; // 다음 줄을 받기 위해 인덱스를 0으로 초기화

      if (buf[0] == '\0') continue; //빈 줄이면 무시

      // ✅ "ACK"로 시작하는 응답만 인정 (잡데이터 방지)
      if (strncmp(buf, "ACK", 3) == 0)  //받은 문자열이 "ACK"로 시작하는지 검사
      {
        g_ackReceived = true; // ACK를 받았다는 상태 플래그 세팅
        g_waitingAck = false; // SEND 후 ACK 기다리던 상태 종료 → 화면 잠금 해제
        g_ackHoldUntil = millis() + 1200;   // ACK 화면 1.2초 유지 / millis() < g_ackHoldUntil이면 평상시 화면 출력 막음

        LED_Set(LEDSTATE_IDLE);

        // LCD 표시용으로 ACK 문자열(buf에 들어온 문자열)을 g_lastAck에 복사
        // "최대 sizeof(g_lastAck) - 1 글자 복사 후 잘려도 마지막에는 무조건 문자열 끝을 보장"
        strncpy(g_lastAck, buf, sizeof(g_lastAck) - 1);
        g_lastAck[sizeof(g_lastAck) - 1] = '\0'; // 버퍼 오버런과 문자열 깨짐을 방지

        // 2행에 ACK 표시
        lcd.setCursor(0, 1);
        lcd.print("RX:");
        for (int i = 3; i < 16; i++) lcd.print(" "); // 2행의 3열~15열까지 공백을 찍어서 “기존 글자 지우기” + RX: 뒤에 남아있던 글자를 깨끗하게 비움
        lcd.setCursor(3, 1);
        for (int i = 0; i < 13 && g_lastAck[i] != '\0'; i++) lcd.print(g_lastAck[i]); // ACK 문자열을 최대 13글자까지만 출력(16칸 LCD에서 RX: 3칸 뺀 나머지) + 문자열 끝(\0)이면 중단
      }

    } else {
      // 비정상이라 판단 후 idx를 0으로 리셋 & 지금까지 모은 buf 내용은 사실상 무효 처리
      if (idx < sizeof(buf) - 1) 
      buf[idx++] = c; // 버퍼가 남아있으면 현재 문자를 저장하고 idx 증가 / -1은 마지막 널 종료 공간을 남겨두기 위해서
      else idx = 0; // 버퍼가 꽉 찼으면 그냥 인덱스를 0으로 리셋(오버플로 방지)
    }
  }
}

// 현재 장치의 보정값들을 EEPROM(영구메모리)에 저장하는 함수
// EEPROM.put(주소, 값) → saveAllToEEPROM()
void saveAllToEEPROM() {
  EEPROM.put(EE_FACTOR_L, factorL);
  EEPROM.put(EE_FACTOR_R, factorR);
  EEPROM.put(EE_A_COEF,  a_coef);
  EEPROM.put(EE_B_COEF,  b_coef);
  EEPROM.put(EE_TARE_L,  offsetL);
  EEPROM.put(EE_TARE_R,  offsetR);
}

// EEPROM에서 저장해둔 보정값을 읽어오는 함수
// EEPROM.get(주소, 값) → loadAllFromEEPROM()
void loadAllFromEEPROM() {
  EEPROM.get(EE_FACTOR_L, factorL);
  EEPROM.get(EE_FACTOR_R, factorR);
  EEPROM.get(EE_A_COEF,  a_coef);
  EEPROM.get(EE_B_COEF,  b_coef);
  EEPROM.get(EE_TARE_L,  offsetL);
  EEPROM.get(EE_TARE_R,  offsetR);

  // 유효범위 체크(값이 정상인지) -> 이상 시 기본 값 복구
  if (!isfinite(factorL) || fabs(factorL) < 1e-6) factorL = -0.005398f;
  if (!isfinite(factorR) || fabs(factorR) < 1e-6) factorR = -0.004933f;
  if (!isfinite(a_coef)) a_coef = 1.0f;
  if (!isfinite(b_coef)) b_coef = 1.0f;
  if (offsetL < -OFFSET_LIMIT || offsetL > OFFSET_LIMIT) offsetL = 0;
  if (offsetR < -OFFSET_LIMIT || offsetR > OFFSET_LIMIT) offsetR = 0;
}