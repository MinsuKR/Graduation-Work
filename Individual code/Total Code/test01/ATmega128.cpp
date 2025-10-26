#include <Arduino.h>

#define LED_PIN  PA0    // ATmega128의 PORTA.0 핀에 LED 연결
#define BUZZER_PIN PA1  // ATmega128의 PORTA.1 핀에 부저 연결


void setup() {
    DDRA |= (1 << PA0) | (1 << PA1);
}

void loop() {
    PORTA |= (1 << PA0);
    PORTA |= (1 << PA1);
    delay(500);

    PORTA &= ~(1 << PA0);
    PORTA &= ~(1 << PA1);
    delay(500);
}