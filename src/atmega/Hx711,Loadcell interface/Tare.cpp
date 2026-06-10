#include "Tare.h"

// DualScale 객체를 복사하지 않고, 원본 객체를 그대로 함수 안에서 사용하고 수정하기 위해 참조로 전달한 것
// "&" 참조 
bool DoTare(DualScale& scale, LiquidCrystal_I2C& lcdStatus, uint8_t avg,
            const char* line1,
            const char* doneMsg,
            const char* failMsg) {
  lcdStatus.clear();
  lcdStatus.setCursor(0, 0); 
  lcdStatus.print(line1);

  bool ok = scale.tare(avg);

  lcdStatus.setCursor(0, 1);
  if (ok) lcdStatus.print(doneMsg);
  else    lcdStatus.print(failMsg);

  delay(500);
  lcdStatus.clear();
  return ok;
}