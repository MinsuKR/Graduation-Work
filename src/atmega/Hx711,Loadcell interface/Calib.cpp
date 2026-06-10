#include "Calib.h"
#include "Config.h"
#include "LCD.h"
#include <math.h>

static bool factorInRange(float f) {
  float af = fabsf(f);
  return (af >= FACTOR_ABS_MIN) && (af <= FACTOR_ABS_MAX);
}

static bool detTooSmall(float det, float ref) {
  if (ref <= 1.0f) return true;
  return (fabsf(det) < (CAL_DET_REL_EPS * ref));
}

static void showLRg(LiquidCrystal_I2C& lcd, const char* title, float Lg, float Rg) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print("L:");
  lcd.print(Lg, 1);
  lcd.print(" R:");
  lcd.print(Rg, 1);
  delay(1800);
}

static void showSum(LiquidCrystal_I2C& lcd, const char* title, float sum) {
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(title);
  lcd.setCursor(0, 1);
  lcd.print("SUM=");
  lcd.print(sum, 1);
  lcd.print("g");
  delay(1400);
}

bool DoCalibration(DualScale& scale, LiquidCrystal_I2C& lcdStatus)
{
  // 0) 시작 안내: 빈 상태
  LCD_Msg(lcdStatus, F("Remove all load"), F("Stabilize..."));
  delay(1800);

  // 0-1) 안정화 체크(선택)
  int32_t avgRaw = 0;
  uint32_t t0 = millis();
  while (!scale.isStable(&avgRaw)) {
    lcdStatus.clear();
    lcdStatus.setCursor(0, 0);
    lcdStatus.print("RAW=");
    lcdStatus.print(avgRaw);
    lcdStatus.setCursor(0, 1);
    lcdStatus.print("Wait stable...");
    delay(400);
    if (millis() - t0 > 8000UL) break;
  }

  // 0-2) TARE 강제
  LCD_Msg(lcdStatus, F("Auto Tare"), F("Wait..."));
  delay(500);
  if (!scale.tare(AVG_TARE_SAMPLES)) {
    LCD_Msg(lcdStatus, F("Tare FAIL"), F("Check HX711"));
    delay(1400);
    return false;
  }

  // 1) LEFT 155g
  LCD_Msg(lcdStatus, F("Put 155g LEFT"), F("Wait..."));
  delay(5000);

  int32_t nL_left = 0, nR_left = 0;
  if (!scale.measureNetLR(AVG_CAL_SAMPLES, nL_left, nR_left)) {
    LCD_Msg(lcdStatus, F("Read FAIL"), F("LEFT"));
    delay(1400);
    return false;
  }

  LCD_Msg(lcdStatus, F("Remove weight"), F("Wait..."));
  delay(1000);
  uint32_t t = millis();
  while (millis() - t < 6000UL) {
  int32_t avg=0;
  if (scale.isStable(&avg)) break;
  delay(200);
}
  LCD_Msg(lcdStatus, F("OK"), F("Next..."));
  delay(1000);
  // 2) RIGHT 155g
  LCD_Msg(lcdStatus, F("Put 155g RIGHT"), F("Wait..."));
  delay(5000);

  int32_t nL_right = 0, nR_right = 0;
  if (!scale.measureNetLR(AVG_CAL_SAMPLES, nL_right, nR_right)) {
    LCD_Msg(lcdStatus, F("Read FAIL"), F("RIGHT"));
    delay(1400);
    return false;
  }

  // 3) factorL, factorR 풀기 (raw(net) 기반)
  // factorL*nL_left  + factorR*nR_left  = CAL_WEIGHT
  // factorL*nL_right + factorR*nR_right = CAL_WEIGHT
  float det = (float)nL_left * (float)nR_right - (float)nL_right * (float)nR_left;
  float ref = fabsf((float)nL_left * (float)nR_right) + fabsf((float)nL_right * (float)nR_left);


  float fL = 0.0f, fR = 0.0f;
  bool useFallback = detTooSmall(det, ref);

  if (!useFallback) {
    fL = CAL_WEIGHT * ((float)nR_right - (float)nR_left) / det;
    fR = CAL_WEIGHT * ((float)nL_left  - (float)nL_right) / det;

    if (!isfinite(fL) || !isfinite(fR) || !factorInRange(fL) || !factorInRange(fR)) {
      useFallback = true;
    }
  }

  // 3-1) det가 작거나 값이 튀면 fallback
  if (useFallback) {
    // LEFT는 L 기여가 크고, RIGHT는 R 기여가 크다는 가정
    if (abs(nL_left) < 10000 || abs(nR_right) < 10000) {
    LCD_Msg(lcdStatus, F("Fallback FAIL"), F("net too small"));
    delay(1500);
    return false;
}

    fL = CAL_WEIGHT / (float)nL_left;
    fR = CAL_WEIGHT / (float)nR_right;

    if (!isfinite(fL) || !isfinite(fR) || !factorInRange(fL) || !factorInRange(fR)) {
      lcdStatus.clear();
      lcdStatus.setCursor(0, 0); lcdStatus.print("fL="); lcdStatus.print(fL, 6);
      lcdStatus.setCursor(0, 1); lcdStatus.print("fR="); lcdStatus.print(fR, 6);
      delay(2000);

      LCD_Msg(lcdStatus, F("FACTOR RANGE"), F("Retry"));
      delay(1400);
      return false;
    }

    LCD_Msg(lcdStatus, F("det small"), F("fallback used"));
    delay(1000);
  }

  // 5) 디버그: 계산된 factor로 LEFT/RIGHT에서 L/R g값 및 합계 표시
  float L_left_g  = (float)nL_left  * fL;
  float R_left_g  = (float)nR_left  * fR;
  float L_right_g = (float)nL_right * fL;
  float R_right_g = (float)nR_right * fR;

  float sum_left  = L_left_g + R_left_g;
  float sum_right = L_right_g + R_right_g;

  showLRg(lcdStatus, "LEFT 155g ->",  L_left_g,  R_left_g);
  showSum(lcdStatus, "LEFT CHECK", sum_left);

  showLRg(lcdStatus, "RIGHT 155g ->", L_right_g, R_right_g);
  showSum(lcdStatus, "RIGHT CHECK", sum_right);

  // 6) 최종 적용
  scale.factorL = fL;
  scale.factorR = fR;

  // total은 wL+wR로 쓰기 위해 a=b=1
  scale.a_coef = 1.0f;
  scale.b_coef = 1.0f;

  // 결과 표시
  lcdStatus.clear();
  lcdStatus.setCursor(0, 0);
  lcdStatus.print("fL=");
  lcdStatus.print(scale.factorL, 6);
  lcdStatus.setCursor(0, 1);
  lcdStatus.print("fR=");
  lcdStatus.print(scale.factorR, 6);
  delay(2000);

  LCD_Msg(lcdStatus, F("CAL OK"), F("Saved by main"));
  delay(800);

  return true;
}
