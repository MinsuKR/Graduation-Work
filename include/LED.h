#ifndef LED_H
#define LED_H

#include <Arduino.h>
#include "PinMap.h"

// LED 핀 (필요에 맞게 변경 가능)
#define LED_RED   PIN_LED_R
#define LED_GREEN PIN_LED_G
#define LED_BLUE  PIN_LED_B

// LED 상태
enum LedState {
    LEDSTATE_OFF,
    LEDSTATE_NORMAL,     // Green: normal/ready
    LEDSTATE_PROCESSING, // 파랑: 분석/통신 중
    LEDSTATE_ERROR       // 빨강: 오류
};

void LED_Init();
void LED_AllOff();
void LED_Set(LedState state);

#endif
