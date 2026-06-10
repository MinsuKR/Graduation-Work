#ifndef TARE_H
#define TARE_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Dual_Scale.h"

// “DualScale 객체 하나를 빌려와서 scale이라는 이름으로 쓰겠다”
bool DoTare(DualScale& scale, LiquidCrystal_I2C& lcd, uint8_t avg,
            const char* line1 = "Auto TARE...",
            const char* doneMsg = "Done",
            const char* failMsg = "Fail");

#endif