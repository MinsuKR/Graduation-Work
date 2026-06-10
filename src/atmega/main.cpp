#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <math.h>

#include "LED.h"
#include "Buzzer.h"
#include "Button.h"
#include "Config.h"
#include "Dual_Scale.h"
#include "Eeprom_S.h"
#include "Tare.h"
#include "Calib.h"
#include "LCD.h"
#include "AutoZero.h"
#include "MeatAnalysis.h"
#include "Wiz550.h"
#include "Wizfi360.h"

static const uint32_t NET_UI_SHOW_MS = 8000;
static const uint32_t AI_REPLY_TIMEOUT_MS = 60000;

LiquidCrystal_I2C lcdWeight(LCD_WEIGHT_ADDR, LCD_COLS, LCD_ROWS);
LiquidCrystal_I2C lcdStatus(LCD_STATUS_ADDR, LCD_COLS, LCD_ROWS);
DualScale scale;

static AutoZeroState g_az;
static uint32_t lastMeasureMs = 0;
static bool wifiReady = false;
static bool netUiOverlay = false;
static uint32_t netUiUntil = 0;
static uint32_t weightIgnoreUntil = 0;
static bool needWeightRecoveryAverage = false;
static int displayTotalInt = 0;
static bool displayTotalInitialized = false;

static void LcdBack(LiquidCrystal_I2C& lcd);
static void HandleButtons();
static bool ReadCurrentWeight(float& outWeight, uint8_t samples);
static void DisplayWeight(const ScaleReading& value);
static int GetStableDisplayTotal(float totalWeight);
static void ResetStableDisplayTotal();
static void StartButtonWeightIgnore();
static bool IsWeightIgnoreActive();
static void Net_StartOverlay(uint32_t ms = NET_UI_SHOW_MS);
static bool StatusUiNormal();
static void StatusUiUpdate();

void setup()
{
  LED_Init();
  Buzzer_Init();
  Button_Init();

  LED_Set(LEDSTATE_PROCESSING);
  Buzzer_Click();

  LCD_Init(lcdWeight, lcdStatus);
  LCD_Msg(lcdWeight, F("HX711 Dual a,b"), F("Init..."));
  LCD_Msg(lcdStatus, F("WIZ550io NET"), F("Init..."));

  scale.begin(DT_L, SCK_L, DT_R, SCK_R);
  EepromStore::LoadAll(scale);

  bool netReady = Wiz550_Begin();
  if (netReady) {
    LED_Set(LEDSTATE_PROCESSING);
    Wiz550_ShowIp(lcdStatus);
  } else {
    LCD_Msg(lcdStatus, F("NET INIT FAIL"), F("Check W5500"));
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
  }
  delay(1200);

  LED_Set(LEDSTATE_PROCESSING);
  LCD_Msg(lcdStatus, F("WizFi360 NET"), F("Init aux..."));
  
  wifiReady = Wizfi360_Begin(lcdStatus);
  if (wifiReady) {
    LED_Set(LEDSTATE_PROCESSING);
    Wizfi360_ShowIp(lcdStatus);
  } else {
    LCD_Msg(lcdStatus, F("Wifi FAIL"), F("Check Wifi"));
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
  }
  delay(1200);

  if (!scale.waitReady(3000)) {
    LCD_Msg(lcdStatus, F("HX711 NOT READY"), F("Check Wiring"));
    LED_Set(LEDSTATE_ERROR);
    Buzzer_Error();
    delay(1500);
  }

  int32_t avgRaw = 0;
  uint32_t startTime = millis();

  while (!scale.isStable(&avgRaw)) {
    lcdStatus.setCursor(0, 0);
    lcdStatus.print(F("RAW="));
    lcdStatus.print(avgRaw);
    lcdStatus.print(F("    "));
    lcdStatus.setCursor(0, 1);
    lcdStatus.print(F("Stabilizing..."));
    delay(500);
    if (millis() - startTime > 10000) break;
  }

  LCD_Msg(lcdStatus, F("RAW Stable OK!"), F("Auto Tare...."));
  LED_Set(LEDSTATE_PROCESSING);
  delay(800);

  DoTare(scale, lcdStatus, AVG_TARE_SAMPLES);
  AutoZero_Init(g_az, 0);

  LED_Set(LEDSTATE_NORMAL);
  LcdBack(lcdStatus);

  lcdWeight.setCursor(0, 0);
  lcdWeight.print(F("L:0g R:0g      "));
  lcdWeight.setCursor(0, 1);
  lcdWeight.print(F("TOT:0g         "));
}

void loop()
{
  HandleButtons();

  if (millis() - lastMeasureMs >= 30) {
    lastMeasureMs = millis();

    if (IsWeightIgnoreActive()) {
      return;
    }

    if (scale.waitReady(5)) {
      uint8_t samples = needWeightRecoveryAverage ? BUTTON_RECOVERY_SAMPLES : AVG_DISPLAY_SAMPLES;
      ScaleReading value = scale.readOnce(samples);

      if (value.valid) {
        int az_g = (int)lroundf(value.total);

        DisplayWeight(value);
        needWeightRecoveryAverage = false;
        AutoZero_Update(g_az, scale, az_g, &lcdStatus, StatusUiNormal(), AVG_TARE_SAMPLES);
      }
    }
  }

  StatusUiUpdate();

  if (AutoZero_UiReturnDue(g_az) && StatusUiNormal()) {
    LcdBack(lcdStatus);
    AutoZero_ClearUiReturn(g_az);
  }
}

static void LcdBack(LiquidCrystal_I2C& lcd)
{
  LCD_Msg(lcd, F("T:Tare(L:Save)"), F("C:Cal S:Send"));
}

static bool StatusUiNormal()
{
  return !netUiOverlay;
}

static bool ReadCurrentWeight(float& outWeight, uint8_t samples)
{
  if (!scale.waitReady(500)) {
    return false;
  }

  ScaleReading value = scale.readOnce(samples);
  if (!value.valid) {
    return false;
  }

  // float totalSnap = (fabs(value.total) < ZERO_SNAP_G) ? 0.0f : value.total;
  float totalSnap;
  if (fabs(value.total) < ZERO_SNAP_G) {
    totalSnap = 0.0f;
  } else {
    totalSnap = value.total;
  }
  outWeight = roundf(totalSnap * 10.0f) / 10.0f;
  return true;
}

static void DisplayWeight(const ScaleReading& value)
{
  float wL_disp;
  if (fabs(value.wL) < ZERO_SNAP_G)
  {wL_disp = 0.0f;}
  else {wL_disp = value.wL;}

  float wR_disp;
  if (fabs(value.wR) < ZERO_SNAP_G)
  {wR_disp = 0.0f;}
  else {wR_disp = value.wR;}

  float totalSnap;
  if (fabs(value.total) < ZERO_SNAP_G)
  {totalSnap = 0.0f;}
  else {totalSnap = value.total;}

  float wL_g = roundf(wL_disp * 10.0f) / 10.0f;
  float wR_g = roundf(wR_disp * 10.0f) / 10.0f;
  int shown_g = GetStableDisplayTotal(totalSnap);

  lcdWeight.setCursor(0, 0);
  lcdWeight.print(F("L:"));
  lcdWeight.print(wL_g, 1);
  lcdWeight.print(F("g R:"));
  lcdWeight.print(wR_g, 1);
  lcdWeight.print(F("g   "));

  lcdWeight.setCursor(0, 1);
  lcdWeight.print(F("TOT:"));
  lcdWeight.print(shown_g);
  lcdWeight.print(F("g           "));
}

static int GetStableDisplayTotal(float totalWeight)
{
  if (!displayTotalInitialized) {
    displayTotalInt = (int)lroundf(totalWeight);
    displayTotalInitialized = true;
    return displayTotalInt;
  }

  if (fabs(totalWeight - (float)displayTotalInt) > DISPLAY_HOLD_BAND_G) {
    displayTotalInt = (int)lroundf(totalWeight);
  }

  return displayTotalInt;
}

static void ResetStableDisplayTotal()
{
  displayTotalInitialized = false;
}

static void StartButtonWeightIgnore()
{
  weightIgnoreUntil = millis() + BUTTON_WEIGHT_IGNORE_MS;
  needWeightRecoveryAverage = true;
}

static bool IsWeightIgnoreActive()
{
  return needWeightRecoveryAverage && ((int32_t)(millis() - weightIgnoreUntil) < 0);
}

static void Net_StartOverlay(uint32_t ms)
{
  netUiOverlay = true;
  netUiUntil = millis() + ms;
}

static void StatusUiUpdate()
{
  if (netUiOverlay && millis() > netUiUntil) {
    netUiOverlay = false;
    LED_Set(LEDSTATE_NORMAL);
    LcdBack(lcdStatus);
  }
}

static void HandleButtons()
{
  ButtonEvent eTare = Button_ReadTare();
  if (eTare == BTN_SHORT) {
    StartButtonWeightIgnore();
    LED_Set(LEDSTATE_PROCESSING);
    bool ok = DoTare(scale, lcdStatus, AVG_TARE_SAMPLES,
                     "Manual TARE...",
                     "TARE Done",
                     "TARE Fail");
    if (ok) Buzzer_Click();
    else Buzzer_Error();

    LED_Set(LEDSTATE_NORMAL);
    ResetStableDisplayTotal();
    AutoZero_Init(g_az, 0);
    LcdBack(lcdStatus);
  } else if (eTare == BTN_LONG) {
    StartButtonWeightIgnore();
    LED_Set(LEDSTATE_PROCESSING);
    bool ok = DoTare(scale, lcdStatus, AVG_TARE_SAMPLES,
                     "Save TARE...",
                     "Saved OK",
                     "Save Fail");
    if (ok) {
      EepromStore::SaveAll(scale);
      Buzzer_Success();
    } else {
      Buzzer_Error();
    }
    LED_Set(LEDSTATE_NORMAL);
    ResetStableDisplayTotal();
    AutoZero_Init(g_az, 0);
    LcdBack(lcdStatus);
  }

  ButtonEvent eCal = Button_ReadCal();
  if (eCal == BTN_SHORT) {
    StartButtonWeightIgnore();
    LED_Set(LEDSTATE_PROCESSING);
    bool ok = DoCalibration(scale, lcdStatus);
    if (ok) {
      EepromStore::SaveAll(scale);
      Buzzer_Success();
    } else {
      Buzzer_Error();
      LED_Set(LEDSTATE_ERROR);
    }
    LED_Set(LEDSTATE_NORMAL);
    ResetStableDisplayTotal();
    AutoZero_Init(g_az, 0);
    LcdBack(lcdStatus);
  }

  ButtonEvent eSend = Button_ReadSend();
  if (eSend == BTN_SHORT) {
    StartButtonWeightIgnore();
    LED_Set(LEDSTATE_PROCESSING);
    LCD_Msg(lcdStatus, F("SEND PRESSED"), F("Stabilizing..."));

    while (IsWeightIgnoreActive()) {
      delay(5);
    }

    float sendWeight = 0.0f;
    bool weightOk = ReadCurrentWeight(sendWeight, BUTTON_RECOVERY_SAMPLES);
    if (!weightOk) {
      Buzzer_Error();
      LED_Set(LEDSTATE_ERROR);
      LCD_Msg(lcdStatus, F("WEIGHT READ FAIL"), F("Try again"));
      Net_StartOverlay(2500);
      return;
    }

    char request[40];
    MeatAnalysis_BuildCaptureRequest(request, sizeof(request), sendWeight);
    char reply[96];
    bool ok = Wiz550_SendRequest(request, lcdStatus, "CAPTURE W5500", reply, sizeof(reply), AI_REPLY_TIMEOUT_MS);
    bool usedWifi = false;

    if (!ok) {
      delay(1500);
      LCD_Msg(lcdStatus, F("W5500 FAIL"), F("Try WiFi..."));
      if (!wifiReady) {
        wifiReady = Wizfi360_Begin(lcdStatus);
      }
      if (wifiReady) {
        ok = Wizfi360_SendRequest(request, lcdStatus, "CAPTURE WIFI", reply, sizeof(reply), AI_REPLY_TIMEOUT_MS);
        usedWifi = ok;
      }
    }

    if (ok) {
      MeatAnalysisResult result;
      if (MeatAnalysis_ParseResponse(reply, result)) {
        MeatAnalysis_ShowResult(lcdStatus, result, sendWeight);
      } else if (usedWifi) {
        LCD_Msg(lcdStatus, F("WIFI TX/RX OK"), F("RX DATA"));
      } else {
        LCD_Msg(lcdStatus, F("W5500 TX/RX OK"), F("RX DATA"));
      }
      Buzzer_Success();
    } else {
      Buzzer_Error();
      LED_Set(LEDSTATE_ERROR);
      delay(1500);
      LCD_Msg(lcdStatus, F("SEND/RX FAIL"), F("Check NET/RPI"));
    }

    // Net_StartOverlay(ok ? NET_UI_SHOW_MS : 2500);
    if (ok) {
      Net_StartOverlay(NET_UI_SHOW_MS);
    } else {
      Net_StartOverlay(2500);
    }
  }
}
