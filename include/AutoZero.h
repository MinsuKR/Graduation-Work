#ifndef AUTO_ZERO_H
#define AUTO_ZERO_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Dual_Scale.h"

// AutoZero 상태(실행 중 변하는 값)
struct AutoZeroState {
  uint32_t lastAutoZeroMs;   // 마지막 auto tare 시각
  uint32_t lastStableChkMs;  // stable 체크 마지막 시각
  bool     stableCached;     // stable 캐시

  // "무게가 안 움직인 시간" 측정용
  int      lastAzG;       // 직전 기준 표시값(g)   
  uint32_t lastMoveMs;       // 마지막으로 "움직임"이 감지된 시각
  uint32_t uiReturnMs;       // UI 복귀 예약 시간
};

void AutoZero_Init(AutoZeroState& st, int initialShownG = 0);

// shown_g: 표시 정수 g (ZERO SNAP/반올림 적용 후 값을 넣는 것을 권장)
// lcdStatus == nullptr이면 LCD 메시지 없이 동작
// allowUiMsg: UI 충돌 방지(예: ACK 화면일 때 false)
// tareAvgSamples: tare 시 평균 샘플 수(AVG_TARE_SAMPLES)
void AutoZero_Update(AutoZeroState& st,
                     DualScale& scale,
                     int az_g,
                     LiquidCrystal_I2C* lcdStatus,
                     bool allowUiMsg,
                     uint8_t tareAvgSamples);

// ★ 추가: UI 복귀 예약이 “지금 복귀할 시간인지” 확인
bool AutoZero_UiReturnDue(const AutoZeroState& st);

// ★ 추가: UI 복귀 예약 해제
void AutoZero_ClearUiReturn(AutoZeroState& st);


#endif
