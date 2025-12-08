#include "Button.h"

static ButtonEvent readButton(uint8_t pin) {
    // 눌리는 순간(LOW edge) 바로 반응
    if (digitalRead(pin) == LOW) {
        delay(20); // 디바운스 최소
        if (digitalRead(pin) == LOW) {
            return BTN_SHORT;  
        }
    }

    return BTN_NONE;
}

void Button_Init() {
    pinMode(BTN_TARE_PIN, INPUT_PULLUP);
    pinMode(BTN_CAL_PIN, INPUT_PULLUP);
}

ButtonEvent Button_ReadTare() {
    return readButton(BTN_TARE_PIN);
}

ButtonEvent Button_ReadCal() {
    return readButton(BTN_CAL_PIN);
}
