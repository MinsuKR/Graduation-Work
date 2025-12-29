#include "LED.h"

void LED_Init() {
    pinMode(LED_GREEN, OUTPUT);
    pinMode(LED_BLUE, OUTPUT);
    pinMode(LED_RED, OUTPUT);

    LED_AllOff();
}

void LED_AllOff() {
    digitalWrite(LED_GREEN, LOW);
    digitalWrite(LED_BLUE, LOW);
    digitalWrite(LED_RED, LOW);
}

void LED_Set(LedState state) {
    LED_AllOff();

    switch (state) {
        case LEDSTATE_IDLE:
            digitalWrite(LED_GREEN, HIGH);
            break;
        case LEDSTATE_PROCESSING:
            digitalWrite(LED_BLUE, HIGH);
            break;
        case LEDSTATE_ERROR:
            digitalWrite(LED_RED, HIGH);
            break;
        case LEDSTATE_OFF:
        default:
            // 이미 전부 OFF
            break;
    }
}
