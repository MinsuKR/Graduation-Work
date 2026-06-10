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
#include "Config.h"
#include "Dual_Scale.h"
#include "Eeprom_S.h"
#include "Tare.h"
#include "Calib.h"
#include "Uart_Ack.h"

// === 객체 생성 ===
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS); // UI 하드웨어
DualScale scale; // 저울 로직/상태
AckState ack; // UART ACK 상태

// === UI 상태 === 
// 프로그램 전체에서 마지막으로 LCD에 보여준 무게를 기억하기 위해 필요
static float ack_lastShown = 0.0f;

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

  scale.begin(DT_L, SCK_L, DT_R, SCK_R);

  EepromStore::LoadAll(scale);
  UartAck_Begin(ack);

  delay(500);

   if (!scale.waitReady(3000)) {
    lcd.clear();
    lcd.setCursor(0, 0); lcd.print("HX711 NOT READY");
    lcd.setCursor(0, 1); lcd.print("Check Wiring");
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  int32_t avgRaw = 0;
  uint32_t startTime = millis();

  while (!scale.isStable(&avgRaw))  
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
  DoTare(scale, lcd, AVG_TARE_SAMPLES);

  LED_Set(LEDSTATE_IDLE);

}

void loop() 
{
  static uint32_t last = 0;

  if (millis() - last >= 200) 
  {
    last = millis();

    bool lockScreen = (ack.UI_State != UI_NORMAL);

    if (scale.waitReady(100)) {
        
        ScaleReading value = scale.readOnce(AVG_MEAS_SAMPLES);

        // 조건 ? 참일때값 : 거짓일때값
        // float displayW = (fabs(value.total) < ZERO_SNAP_G) ? 0.0f : value.total; 사용가능
        float DisplayW = 0.0f;
        float absW = fabs(value.total);

        if(absW < ZERO_SNAP_G)
        {
          DisplayW = 0.0f;
        }
        else
        {
          DisplayW = value.total;
        }
        
        float shown = roundf(DisplayW * 10.0f) / 10.0f;

        // UI(화면)
        ack_lastShown = shown;
        // ack상태(통신)
        ack.lastShown = shown;

      // ===== LCD 출력 =====
      if (!lockScreen) {
      lcd.setCursor(0, 0);
      lcd.print("L:");
      lcd.print(value.wL, 1);
      lcd.print("g ");
      lcd.print("R:");
      lcd.print(value.wR, 1);
      lcd.print("g ");

      lcd.setCursor(0, 1);
      lcd.print("TOT:");
      lcd.print(shown, 1);
      lcd.print("g    ");
      }

    } else {
      // UI 보호 때문에 사용
      if (!lockScreen) {
        lcd.setCursor(0, 0); lcd.print("Waiting HX711  ");
        lcd.setCursor(0, 1); lcd.print("...            ");
      }
    }
  }

  UartAck_Handle(ack,lcd);

  // ===== 버튼 처리 =====

  // TARE 버튼
  ButtonEvent eTare = Button_ReadTare();
  if (eTare == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    DoTare(scale,lcd,AVG_TARE_SAMPLES);
    Buzzer_Click();
    LED_Set(LEDSTATE_IDLE);
  } else if (eTare == BTN_LONG) {
    LED_Set(LEDSTATE_PROCESSING);
    DoTare(scale, lcd, AVG_TARE_SAMPLES);
    EepromStore::SaveAll(scale); 
    Buzzer_Success();
    LED_Set(LEDSTATE_IDLE);
  }

  // CAL 버튼
  ButtonEvent eCal = Button_ReadCal();
  if (eCal == BTN_SHORT) {
    LED_Set(LEDSTATE_PROCESSING);
    bool ok = DoCalibration(scale,lcd);
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
  }

  // ===== SEND 버튼 (무게 전송) =====
  ButtonEvent eSend = Button_ReadSend();
  if (eSend == BTN_SHORT) {   
  UartAck_SendWeight(ack,lcd,ack_lastShown);
  }
}
   