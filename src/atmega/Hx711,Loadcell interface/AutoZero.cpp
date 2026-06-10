#include "AutoZero.h"
#include "Config.h"
#include "LCD.h"

static bool nearZero(int g) {
  return (abs(g) <= AUTO_ZERO_BAND_G);
}

void AutoZero_Init(AutoZeroState& st, int initialAzG)
{
  st.lastAutoZeroMs  = 0;
  st.lastStableChkMs = 0;
  st.stableCached    = false;

  st.lastAzG   = initialAzG;
  st.lastMoveMs = millis();

  st.uiReturnMs = 0;
}

bool AutoZero_UiReturnDue(const AutoZeroState& st)
{
  if (st.uiReturnMs == 0) return false;
  return ((int32_t)(millis() - st.uiReturnMs) >= 0);
}

void AutoZero_ClearUiReturn(AutoZeroState& st)
{
  st.uiReturnMs = 0;
}

void AutoZero_Update(AutoZeroState& st,
                     DualScale& scale,
                     int az_g,
                     LiquidCrystal_I2C* lcdStatus,
                     bool allowUiMsg,
                     uint8_t tareAvgSamples)
{
  const uint32_t now = millis();

  // stable 캐시 갱신(주기적으로만)
  if (now - st.lastStableChkMs >= AUTO_ZERO_STABLE_CHECK_MS) {
    st.lastStableChkMs = now;
    int32_t rawAvgTmp = 0;
    st.stableCached = scale.isStable(&rawAvgTmp);
  }

  // 빈 상태가 아니면(= 물건 올라간 상태 가능) 타이머 리셋 후 종료
  if (!nearZero(az_g)) {
    st.lastAzG   = az_g;
    st.lastMoveMs = now;
    return;
  }

  // nearZero일 때만 "움직임" 판단 (잡음은 threshold로 무시)
  if (abs(az_g - st.lastAzG) >= AUTO_ZERO_MOVE_THRESHOLD_G) {
    st.lastAzG   = az_g;
    st.lastMoveMs = now;
  }

  const bool noMoveEnough = (now - st.lastMoveMs) >= AUTO_ZERO_NO_MOVE_MS;
  const bool coolDone     = (now - st.lastAutoZeroMs) >= AUTO_ZERO_MIN_INTERVAL_MS;

  if (noMoveEnough && st.stableCached && coolDone) {

    // AUTO ZERO 메시지 표시 + 복귀 예약
    if (lcdStatus && allowUiMsg) {
      LCD_Msg(*lcdStatus, F("AUTO ZERO"), F("Tare..."));
      st.uiReturnMs = now + AUTO_ZERO_UI_RETURN_MS;
    }

    // tare 수행
    scale.tare(tareAvgSamples);

    // 상태 리셋
    st.lastAutoZeroMs = now;
    st.lastAzG        = 0;
    st.lastMoveMs     = now;
  }
}
