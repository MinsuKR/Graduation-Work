#ifndef BUTTON_H
#define BUTTON_H

#include <Arduino.h>

// ATmega128 핀 (지금 네 코드와 맞춤)
#define BTN_TARE_PIN PIN_PE4
#define BTN_CAL_PIN  PIN_PE5
#define BTN_SEND_PIN PIN_PE6

enum ButtonEvent {
    BTN_NONE=0,
    BTN_SHORT,
    BTN_LONG
};

void Button_Init();

// 각각의 버튼에 대한 이벤트 읽기
ButtonEvent Button_ReadTare();
ButtonEvent Button_ReadCal();
ButtonEvent Button_ReadSend();

#endif
