/* Test complete */
// PA0 - 저항(330) - LED - GND , PA0 - 저항 - BUZZER - GND
#include <Arduino.h>

#define INNER_LED_PIN PG0 // ATmega128의 PORTG.0 핀에 내부 LED
#define LED_PIN PA0 // ATmega128의 PORTA.0 핀에 LED 연결
#define BUZZER_PIN PA1 // ATmega128의 PORTA.1 핀에 BUZZER 연결

void setup() {
    DDRG |= (1 << PG0);
    DDRA |= (1 << PA0);
    DDRA |= (1 << PA1);
}

void loop() {
    PORTG |= (1 << PG0);
    PORTA |= (1 << PA0); // PB0 High
    PORTA |= (1 << PA1);
    delay(500);

    PORTG &= ~(1 << PG0);
    PORTA &= ~(1 << PA0); // PB0 Low
    PORTA &= ~(1 << PA1);
    delay(500);
}