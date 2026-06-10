// 기본 표준 함수
#include <Arduino.h> 
#include <Wire.h> 
#include <LiquidCrystal_I2C.h> 
#include <math.h> 

//사용자 헤더 함수
#include "LED.h"  
#include "Buzzer.h" 
#include "Button.h" 
#include "Config.h"
#include "Dual_Scale.h"
#include "Eeprom_S.h"
#include "Tare.h"
#include "Calib.h"
#include "Uart_Ack.h"
#include "LCD.h"
#include "AutoZero.h"  

// === 객체 생성 ===
LiquidCrystal_I2C lcdWeight(LCD_WEIGHT_ADDR, LCD_COLS, LCD_ROWS); // 무게 상시 표시용
LiquidCrystal_I2C lcdStatus(LCD_STATUS_ADDR, LCD_COLS, LCD_ROWS); // ESP32/상태 표시용
DualScale scale; // 저울 로직/상태
AckState ack; // UART ACK 상태

// === UI 상태 === 
// 프로그램 전체에서 마지막으로 LCD에 보여준 무게를 기억하기 위해 필요
static float ack_lastShown = 0.0f;
static void LcdBack(LiquidCrystal_I2C& lcd);
// === 버튼 이벤트 ===
static void HandleButtons();
// === AutoZero 상태 ===
static AutoZeroState g_az;
// === 측정 주기 ===
static uint32_t last = 0;

void setup() 
{
  Serial.begin(9600);  

  LED_Init();
  Buzzer_Init();
  Button_Init();

  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();

  LCD_Init(lcdWeight,lcdStatus);
  LCD_Msg(lcdWeight,"HX711 Dual a,b","Init...");
  LCD_Msg(lcdStatus,"ESP32/NET", "Init...");

  scale.begin(DT_L, SCK_L, DT_R, SCK_R);

  EepromStore::LoadAll(scale);
  UartAck_Begin(ack);

  delay(500);

   if (!scale.waitReady(3000)) {
    LCD_Msg(lcdStatus,"HX711 NOT READY","Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  // ===== 부팅 안정화 =====
  int32_t avgRaw = 0;
  uint32_t startTime = millis();

  while (!scale.isStable(&avgRaw))  
  {
    lcdStatus.setCursor(0, 0); lcdStatus.print("RAW="); lcdStatus.print(avgRaw); lcdStatus.print("    ");
    lcdStatus.setCursor(0, 1); lcdStatus.print("Stabilizing...");
    delay(500);
    if (millis() - startTime > 10000) break;
  }

  LCD_Msg(lcdStatus,"RAW Stable OK!","Auto Tare....");
  delay(800);

  // === 영점잡기 (빈 상태에서) ===
  DoTare(scale, lcdStatus, AVG_TARE_SAMPLES);

  // AutoZero 초기화 (초기 표시값 0 기준)
  AutoZero_Init(g_az, 0);

  LED_Set(LEDSTATE_IDLE);
  LcdBack(lcdStatus);

  // lcdWeight 초기 표시(선택)
  lcdWeight.setCursor(0, 0);
  lcdWeight.print("L:0g R:0g      ");
  lcdWeight.setCursor(0, 1);
  lcdWeight.print("TOT:0g         ");
}

void loop() 
{
  // ===== 1) 주기적 측정 및 표시 =====
  if (millis() - last >= 200) {
    last = millis();

    if (!scale.waitReady(100)) {
      // lcdWeight는 덮어쓰지 않고 유지
      return;
    }

    ScaleReading value = scale.readOnce(AVG_MEAS_SAMPLES);
  if (!value.valid) return;

  // --- 1) AutoZero 판단값(중요! ZERO_SNAP 적용 전) ---
  int az_g = (int)lroundf(value.total);   // ★ 작은 물건(3g 등) 올렸을 때 auto tare 방지 핵심

  // --- 2) 표시값(기존처럼 ZERO_SNAP 적용) ---
  float totalSnap = (fabs(value.total) < ZERO_SNAP_G) ? 0.0f : value.total;
  int shown_g = (int)lroundf(totalSnap);

  // UI/통신용 값 갱신
  ack_lastShown = (float)shown_g;
  ack.lastShown = (float)shown_g;

  // LCD 출력(정수 g)
  int wL_g = (int)lroundf(value.wL);
  int wR_g = (int)lroundf(value.wR);

  lcdWeight.setCursor(0, 0);
  lcdWeight.print("L:"); lcdWeight.print(wL_g);
  lcdWeight.print("g R:"); lcdWeight.print(wR_g);
  lcdWeight.print("g   ");

  lcdWeight.setCursor(0, 1);
  lcdWeight.print("TOT:");
  lcdWeight.print(shown_g);
  lcdWeight.print("g      ");

  // AutoZero 업데이트는 az_g로!
  bool allowUiMsg = (ack.UI_State == UI_NORMAL);
  AutoZero_Update(g_az, scale, az_g, &lcdStatus, allowUiMsg, AVG_TARE_SAMPLES);

  }

  // ===== 3) UART ACK 처리 =====
  UartAck_Handle(ack,lcdStatus);

  static UiState UI = UI_NORMAL;
  if(UI != ack.UI_State)
  {
    if(ack.UI_State == UI_NORMAL)
    {
      LcdBack(lcdStatus);
    }
    UI = ack.UI_State;
  }

  // AutoZero가 UI 복귀 예약을 걸어놨고, 복귀 시간이 지났다면 기본 화면으로 복귀
  if (AutoZero_UiReturnDue(g_az)) {
  // ACK 화면 등 UI가 다른 상태면 덮어쓰지 않음 → UI_NORMAL일 때만 복귀
  if (ack.UI_State == UI_NORMAL) {
    LcdBack(lcdStatus);            // "Button Waiting" 화면으로 복귀
    AutoZero_ClearUiReturn(g_az);  // 예약 해제
  }
  // UI_NORMAL이 아니면 예약을 유지해서, 나중에 UI_NORMAL이 되면 복귀하도록 둠
  }

  // ===== 버튼 처리 =====
  HandleButtons();
}
   

static void LcdBack(LiquidCrystal_I2C& lcd)
{
  LCD_Msg(lcd,"Button Waiting","1.T 2.C 3.S");
}

static void HandleButtons()
{
  // TARE 버튼
  ButtonEvent eTare = Button_ReadTare();
  if (eTare == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    DoTare(scale,lcdStatus,AVG_TARE_SAMPLES);
    Buzzer_Click();
    LED_Set(LEDSTATE_IDLE);

    // 수동 tare 후 AutoZero 타이머 리셋
    AutoZero_Init(g_az, 0);

    if(ack.UI_State == UI_NORMAL) LcdBack(lcdStatus);

  } else if (eTare == BTN_LONG) {
    LED_Set(LEDSTATE_PROCESSING);
    DoTare(scale, lcdStatus, AVG_TARE_SAMPLES);
    EepromStore::SaveAll(scale); 
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);

    // 수동 tare 후 AutoZero 타이머 리셋
    AutoZero_Init(g_az, 0);

    if(ack.UI_State == UI_NORMAL) LcdBack(lcdStatus);
  }

  // CAL 버튼
  ButtonEvent eCal = Button_ReadCal();
  if (eCal == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    bool ok = DoCalibration(scale,lcdStatus);
    if(ok)
    {
      EepromStore::SaveAll(scale);
      Buzzer_Success();
    }
    else
    {
      Buzzer_Error();
    }
    LED_Set(LEDSTATE_IDLE);

    // 수동 tare 후 AutoZero 타이머 리셋
    AutoZero_Init(g_az, 0);

    if(ack.UI_State == UI_NORMAL) LcdBack(lcdStatus);
  }

  // ===== SEND 버튼 (무게 전송) =====
  ButtonEvent eSend = Button_ReadSend();
  if (eSend == BTN_SHORT) {   
  UartAck_SendWeight(ack,lcdStatus,ack_lastShown);
  }
}


// ✅ 부팅 시 자동 안정화 → 자동 TARE
// ✅ 무게는 항상 lcdWeight에 표시(절대 다른 화면으로 덮어쓰기 X)
// ✅ 표시는 소수점 없이 정수(g)로 반올림
// ✅ ZERO SNAP(±5g) 유지
// ✅ Auto Zero Tracking(조건부 자동 영점 재보정) 추가
// “비어있고(0 근처), 안정적이면” 일정 시간 후 자동으로 다시 0 맞춤