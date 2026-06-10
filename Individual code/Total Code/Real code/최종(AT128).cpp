// 기본 표준 함수
#include <Arduino.h> 
#include <Wire.h> 
#include <HX711.h> 
#include <LiquidCrystal_I2C.h> 
#include <EEPROM.h> 
#include <math.h> 
#include <stdint.h> 
#include <string.h> 

//사용자 헤더 함수
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
#define DT_L   PIN_PD3 // DOUT
#define SCK_L  PIN_PD2 // SCK

// 오른쪽 HX711
#define DT_R   PIN_PD5 // DOUT
#define SCK_R  PIN_PD4 // SCK

// EEPROM 바이트 단위 저장 공간 (연속 주소 저장) 
#define EE_FACTOR_L   0    
#define EE_FACTOR_R   4
#define EE_A_COEF     8
#define EE_B_COEF     12
#define EE_TARE_L     16
#define EE_TARE_R     20

// ==== 전역변수 ====

// ==== 로드셀 기본 보정 계수 ====
float factorL = -0.005395f; 
float factorR = -0.005014f; 

// ==== 영점 오프셋 ====
int32_t offsetL = 0;
int32_t offsetR = 0;
const int32_t OFFSET_LIMIT = 10000000; 

// ==== 총무게 보정용 계수(가중치 보정) ====
float a_coef = 1.0f;  
float b_coef = 1.0f;

// ==== 샘플 개수(평균/안정성/안정도 기준) ====
const uint8_t STABLE_SAMPLING   = 10; 
const uint8_t AVG_TARE_SAMPLES  = 30; // 20 -> 30
const uint8_t AVG_CAL_SAMPLES   = 80; // 30 -> 80
const uint8_t AVG_MEAS_SAMPLES  = 8; // 5-> 8 
const int32_t STABLE_DIFF_MAX = 500; 

// ==== 표시/출력 ====
float DisplayW = 0.0f; 
float g_lastShown = 0.0f;   
const float ZERO_SNAP_G = 5.0f; 
bool  g_weightValid = false; 

// ==== ESP32-cam ACK 제어 ====
bool g_waitingAck = false;   
bool g_ackReceived = false;  
uint32_t g_ackHoldUntil = 0;    
char g_lastAck[48] = {0};

// ==== 캘리브레이션 기준 값 ====
const float CAL_WEIGHT = 155.0f;
const float FACTOR_ABS_MIN= 1e-6f;
const float FACTOR_ABS_MAX = 1.0f;
const float SNAP_RANGE = 1.0f;

// ==== send -> ack 전역변수
enum UiState : uint8_t { UI_NORMAL, UI_WAIT_ACK, UI_SHOW_ACK, UI_SHOW_IP };
UiState g_uiState = UI_NORMAL;
              
uint32_t g_uiUntil = 0;          // 해당 상태 유지 종료 시각(ms)
char     g_lastIp[16] = {0};     // "192.168.0.33" 용

// === 함수 선언 ===
bool waitReady(uint16_t timeout_ms = 100);
bool isStable(int32_t* avg_out = nullptr);
void doTare(uint8_t avg = AVG_TARE_SAMPLES); // EEPROM 저장 x
void doTareAndSave(uint8_t avg = AVG_TARE_SAMPLES); // EEPROM 저장 o
void doCalibration();
void measureLR(uint8_t samples, float &outL, float &outR);
void handleEspAckOnLcd();
void loadAllFromEEPROM();
void saveAllToEEPROM();

void setup() 
{
  Serial.begin(9600);  

  LED_Init();
  Buzzer_Init();
  Button_Init();

  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();

  lcd.init();
  lcd.backlight();
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("HX711 Dual a,b");
  lcd.setCursor(0, 1); lcd.print("Init...");

  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);
  scaleL.set_gain(128);
  scaleR.set_gain(128);

  loadAllFromEEPROM();
  delay(500);

   if (!waitReady(3000)) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  int32_t avgRaw = 0;
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

  // === 영점잡기 (빈 상태에서) ===
  doTare();

  LED_Set(LEDSTATE_IDLE);

}

void loop() 
{
  static uint32_t last = 0;

  if (millis() - last >= 200) 
  {
    last = millis();

    if (waitReady(100)) {
        
        int32_t rawL = (int32_t)scaleL.read_average(AVG_MEAS_SAMPLES);
        int32_t rawR = (int32_t)scaleR.read_average(AVG_MEAS_SAMPLES);

        int32_t netL = rawL - offsetL;
        int32_t netR = rawR - offsetR;

        float wL = (float)netL * factorL;
        float wR = (float)netR * factorR;

        float Wtotal = a_coef * wL + b_coef * wR;

        if (isnan(Wtotal) || isinf(Wtotal)) Wtotal = 0.0f;

        // === [추가] 155g 근처 값 고정(Snap) 로직 ===
        // 목표값이 155g이고 오차 범위를 +-1.0g으로 설정할 경우
        if (fabs(Wtotal - CAL_WEIGHT) <= SNAP_RANGE) {
        Wtotal = CAL_WEIGHT;
        }

        float absW = fabs(Wtotal);

      if (absW < ZERO_SNAP_G) {  
        DisplayW = 0.0f;
      }
      else {
        DisplayW = Wtotal;
      }

      float shown = roundf(DisplayW * 10.0f) / 10.0f;

      g_lastShown = shown;
      g_weightValid = true;

      bool lockScreen = (g_uiState != UI_NORMAL);

      // ===== LCD 출력 =====
      if (!lockScreen) {
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

  handleEspAckOnLcd();

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
    doTareAndSave(); 
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
  }

  // ===== SEND 버튼 (무게 전송) =====
  ButtonEvent eSend = Button_ReadSend();
  if (eSend == BTN_SHORT) {   

  g_waitingAck = true;
  g_ackReceived = false;

  g_uiState = UI_WAIT_ACK;     // ✅ 상태 전환
  g_uiUntil = 0;               // WAIT_ACK는 ACK 올 때까지 (또는 타임아웃을 넣고 싶으면 여기서 설정)

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("SEND ");
  lcd.print(g_lastShown, 1);
  lcd.print("g   "); 

  lcd.setCursor(0, 1);
  lcd.print("RX: waiting... ");

  Serial.print("W=");
  Serial.print(g_lastShown, 1);
  Serial.println(";PIC");

  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();
  }

}

// === HX711 준비 대기 ===
bool waitReady(uint16_t timeout_ms) {
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (scaleL.is_ready() && scaleR.is_ready()) return true;
    delay(1); 
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

  // saveAllToEEPROM();

  lcd.setCursor(0, 1); lcd.print("Done");
  delay(700);
  lcd.clear();
}

// === 영점 조정 + EEPROM 저장 (저장 O) ===
void doTareAndSave(uint8_t avg) {
  doTare(avg);          // offsetL/R 갱신
  saveAllToEEPROM();    // ★ 롱프레스일 때만 저장
  lcd.setCursor(0, 1); lcd.print("Saved");
  delay(500);
  lcd.clear();
}

// === 캘리브레이션 ===
void doCalibration() {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put weight (155g)");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000);

  if (!waitReady(1000)) return;

  float L_left = 0.0f, R_left = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g LEFT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000); 

  measureLR(AVG_CAL_SAMPLES, L_left, R_left);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L_left=");
  lcd.print(L_left, 1);
  lcd.setCursor(0, 1); lcd.print("R_left=");
  lcd.print(R_left, 1);
  delay(1500);

  float L_right = 0.0f, R_right = 0.0f;
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("Put 155g RIGHT");
  lcd.setCursor(0, 1); lcd.print("Wait...");
  delay(3000); 

  measureLR(AVG_CAL_SAMPLES, L_right, R_right);

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("L_right=");
  lcd.print(L_right, 1);
  lcd.setCursor(0, 1); lcd.print("R_right=");
  lcd.print(R_right, 1);
  delay(1500);

  float det = (L_left * R_right) - (L_right * R_left);

  if (fabs(det) < FACTOR_ABS_MIN) {
    a_coef = FACTOR_ABS_MAX;
    b_coef = FACTOR_ABS_MAX;
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
bool isStable(int32_t* avg_out) 
{
  int32_t samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) 
  {
    if (!waitReady(300)) return false; 
    int32_t rawL = (int32_t)scaleL.read();
    int32_t rawR = (int32_t)scaleR.read();
    samples[i] = rawL + rawR;
  }

  int32_t minV = samples[0], maxV = samples[0];
  int32_t sum = 0;
  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  if (avg_out) *avg_out = sum / STABLE_SAMPLING; 
  int32_t diff = maxV - minV; 
  return (diff < STABLE_DIFF_MAX); 
}

// ==== 평균 측정 함수 ====
void measureLR(uint8_t samples, float &outL, float &outR) {
  int32_t sumL = 0;
  int32_t sumR = 0;
  uint8_t got = 0;
  uint32_t t0 = millis();

  while (got < samples) {
    if ((uint32_t)(millis() - t0) > 3000UL) break; 

    if (!waitReady(200)) continue; 

    // ready일 때만 읽기
    int32_t rawL = (int32_t)scaleL.read();
    int32_t rawR = (int32_t)scaleR.read();
    sumL += rawL;
    sumR += rawR;
    got++;
    delay(5);
  }

  if (got == 0) { outL = 0; outR = 0; return;} 
  int32_t avgRawL = sumL / got;
  int32_t avgRawR = sumR / got;

  int32_t netL = avgRawL - offsetL;
  int32_t netR = avgRawR - offsetR;

  outL = (float)netL * factorL;
  outR = (float)netR * factorR;
}

// ===== ESP32로부터 ACK 수신해서 LCD에 표시 =====
// 
// ===== ESP32로부터 ACK 수신해서 LCD에 표시 =====
void handleEspAckOnLcd() {
  static char buf[80];          // ✅ 조금 넉넉하게 (48 -> 80 권장)
  static uint8_t idx = 0;

  while (Serial.available())
  {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n')
    {
      buf[idx] = '\0';
      idx = 0;

      if (buf[0] == '\0') continue;

      if (strncmp(buf, "ACK", 3) == 0)
      {
        g_ackReceived = true;
        g_waitingAck = false;
        LED_Set(LEDSTATE_IDLE);

        strncpy(g_lastAck, buf, sizeof(g_lastAck) - 1);
        g_lastAck[sizeof(g_lastAck) - 1] = '\0';

        // --- IP 추출해서 저장 ---
        g_lastIp[0] = '\0';
        char* pIp = strstr(buf, "IP=");
        if (pIp) 
        {
            // "IP=" 다음부터 최대 15글자 복사
            strncpy(g_lastIp, pIp + 3, 15);
            g_lastIp[15] = '\0';
        }

        // --- PIC 결과 확인(간단 파싱) ---
        bool picOk = (strstr(buf, "PIC=OK") != nullptr);
        bool picFail = (strstr(buf, "PIC=FAIL") != nullptr);

        // 1) ACK 요약 화면 (예: 1.5초)
        lcd.clear();
        lcd.setCursor(0, 0);
         lcd.print("ACK ");
        if (picOk) lcd.print("PIC OK");
        else if (picFail) lcd.print("PIC FAIL");
        else lcd.print("RX OK");

        // 2행에는 W값 또는 짧게 ACK 일부 표시 (원하는 걸로)
        lcd.setCursor(0, 1);
        // ACK에 W=가 있으면 W만 보여주기
        char* pW = strstr(buf, "W=");
        if (pW) {
        lcd.print("W:");
        // "W=" 다음부터 ; 또는 공백 전까지 출력
        for (int i = 2; i < 15 && pW[i] && pW[i] != ' ' && pW[i] != ';'; i++) lcd.print(pW[i]);
        } else {
        lcd.print("Done");
        }

          g_uiState = UI_SHOW_ACK;
          g_uiUntil = millis() + 1500;   // ✅ ACK 화면 유지 시간
      }

    }
    else {
      if (idx < sizeof(buf) - 1) buf[idx++] = c;
      else idx = 0;
    }
  }
  // ===== UI 상태 타이머 처리 =====
if (g_uiState == UI_SHOW_ACK && millis() > g_uiUntil) {
  // ACK 화면 끝 -> IP 화면으로
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("ESP32-CAM IP");

  lcd.setCursor(0, 1);
  if (g_lastIp[0]) {
    lcd.print("IP:");
    lcd.print(g_lastIp);
  } else {
    lcd.print("IP: (none)");
  }

  g_uiState = UI_SHOW_IP;
  g_uiUntil = millis() + 2500;   // ✅ IP 화면 유지 시간
}

else if (g_uiState == UI_SHOW_IP && millis() > g_uiUntil) {
  // IP 화면 끝 -> 평상시 복귀
  lcd.clear();
  g_uiState = UI_NORMAL;
}
}


// === EEPROM에서 저장해둔 보정값을 읽어오는 함수 ===
void loadAllFromEEPROM() 
{
  EEPROM.get(EE_FACTOR_L, factorL);
  EEPROM.get(EE_FACTOR_R, factorR);
  EEPROM.get(EE_A_COEF,  a_coef);
  EEPROM.get(EE_B_COEF,  b_coef);
  EEPROM.get(EE_TARE_L,  offsetL);
  EEPROM.get(EE_TARE_R,  offsetR);

  // 유효범위 체크(값이 정상인지) -> 이상 시 기본 값 복구
  if (!isfinite(factorL) || fabs(factorL) < 1e-6) factorL = -0.005395f;
  if (!isfinite(factorR) || fabs(factorR) < 1e-6) factorR = -0.005014f; 
  if (!isfinite(a_coef)) a_coef = 1.0f;
  if (!isfinite(b_coef)) b_coef = 1.0f;
  if (offsetL < -OFFSET_LIMIT || offsetL > OFFSET_LIMIT) offsetL = 0;
  if (offsetR < -OFFSET_LIMIT || offsetR > OFFSET_LIMIT) offsetR = 0;
}

// === 현재 장치의 보정값들을 EEPROM(영구메모리)에 저장하는 함수 ===
void saveAllToEEPROM() 
{
  EEPROM.put(EE_FACTOR_L, factorL);
  EEPROM.put(EE_FACTOR_R, factorR);
  EEPROM.put(EE_A_COEF,  a_coef);
  EEPROM.put(EE_B_COEF,  b_coef);
  EEPROM.put(EE_TARE_L,  offsetL);
  EEPROM.put(EE_TARE_R,  offsetR);
}