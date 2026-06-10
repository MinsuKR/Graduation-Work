#include "Button.h"
#include <Arduino.h>

static const uint32_t DEBOUNCE_MS  = 5;
static const uint32_t LONGPRESS_MS = 2000;

enum {
    BTN_IDX_TARE = 0,
    BTN_IDX_CAL,
    BTN_IDX_SEND,
    BTN_COUNT
};

struct ButtonState {
    uint8_t pin;
    bool lastReading;          // 직전 raw 입력값
    bool stableState;          // 디바운스 후 확정 상태
    uint32_t lastDebounceTime; // 마지막 변화 시각
    uint32_t pressStartTime;   // 눌리기 시작한 시각
    bool longFired;            // LONG 발생 여부
    bool shortFired;           // press 즉시 SHORT 1회 발생 여부
};

static ButtonState buttons[BTN_COUNT] = {
    { BTN_TARE_PIN, HIGH, HIGH, 0, 0, false, false },
    { BTN_CAL_PIN,  HIGH, HIGH, 0, 0, false, false },
    { BTN_SEND_PIN, HIGH, HIGH, 0, 0, false, false }
};

// TARE 전용: 짧게(뗄 때), 길게(2초)
static ButtonEvent readButtonTare(ButtonState &btn)
{
    bool reading = digitalRead(btn.pin);

    if (reading != btn.lastReading) {
        btn.lastDebounceTime = millis();
    }

    btn.lastReading = reading;

    if (millis() - btn.lastDebounceTime >= DEBOUNCE_MS) {
        if (reading != btn.stableState) {
            btn.stableState = reading;

            // 눌림
            if (btn.stableState == LOW) {
                btn.pressStartTime = millis();
                btn.longFired = false;
            }
            // 뗌
            else {
                if (!btn.longFired) {
                    return BTN_SHORT;
                }
            }
        }

        // 길게 누름
        if (btn.stableState == LOW && !btn.longFired) {
            if (millis() - btn.pressStartTime >= LONGPRESS_MS) {
                btn.longFired = true;
                return BTN_LONG;
            }
        }
    }

    return BTN_NONE;
}

// CAL/SEND 전용: 누르는 순간 바로 SHORT
static ButtonEvent readButtonImmediate(ButtonState &btn)
{
    bool reading = digitalRead(btn.pin);

    if (reading != btn.lastReading) {
        btn.lastDebounceTime = millis();
    }

    btn.lastReading = reading;

    if (millis() - btn.lastDebounceTime >= DEBOUNCE_MS) {
        if (reading != btn.stableState) {
            btn.stableState = reading;

            // 눌리는 순간 1회 SHORT 발생
            if (btn.stableState == LOW) {
                if (!btn.shortFired) {
                    btn.shortFired = true;
                    return BTN_SHORT;
                }
            }
            // 떼면 다시 다음 입력 가능하도록 초기화
            else {
                btn.shortFired = false;
            }
        }
    }

    return BTN_NONE;
}

void Button_Init()
{
    for (int i = 0; i < BTN_COUNT; i++) {
        // 외부 풀업 사용 경우 INPUT
        // 내부 풀업 사용 경우 INPUT_PULLUP
        pinMode(buttons[i].pin, INPUT);   

        bool now = digitalRead(buttons[i].pin);
        buttons[i].lastReading = now;
        buttons[i].stableState = now;
        buttons[i].lastDebounceTime = millis();
        buttons[i].pressStartTime = 0;
        buttons[i].longFired = false;
        buttons[i].shortFired = false;
    }
}

ButtonEvent Button_ReadTare()
{
    return readButtonTare(buttons[BTN_IDX_TARE]);
}

ButtonEvent Button_ReadCal()
{
    return readButtonImmediate(buttons[BTN_IDX_CAL]);
}

ButtonEvent Button_ReadSend()
{
    return readButtonImmediate(buttons[BTN_IDX_SEND]);
}
