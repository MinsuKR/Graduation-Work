// 수동부저
#include "Buzzer.h"

void Buzzer_Init() {
    DDRB |= _BV(PB0);
    PORTB &= ~_BV(PB0);
}

void Buzzer_Beep(uint16_t duration) {
    uint16_t cycles = duration << 1;
    while (cycles--) {
        PORTB |= _BV(PB0);
        delayMicroseconds(244);
        PORTB &= ~_BV(PB0);
        delayMicroseconds(244);
    }
    delay(20);
}

void Buzzer_Click() {
    Buzzer_Beep(40);
}

void Buzzer_Success() {
    Buzzer_Beep(120);
    delay(70);
    Buzzer_Beep(180);
}

void Buzzer_Error() {
    for (int i = 0; i < 3; i++) {
        Buzzer_Beep(80);
        delay(70);
    }
}

// #include "Buzzer.h"

// void Buzzer_Init() {
//     pinMode(BUZZER_PIN, OUTPUT);
//     digitalWrite(BUZZER_PIN, LOW); // 평소에는 꺼둠 (noTone 대신 LOW)
// }

// void Buzzer_Beep(uint16_t duration) {
//     digitalWrite(BUZZER_PIN, HIGH); // 부저 켜기 (tone 대신 HIGH)
//     delay(duration);
//     digitalWrite(BUZZER_PIN, LOW);  // 부저 끄기 (noTone 대신 LOW)
//     delay(20);                      // 음과 음 사이의 최소 여백
// }

// void Buzzer_Click() {
//     Buzzer_Beep(40);
// }

// void Buzzer_Success() {
//     Buzzer_Beep(120);
//     delay(70);
//     Buzzer_Beep(180);
// }

// void Buzzer_Error() {
//     for (int i = 0; i < 3; i++) {
//         Buzzer_Beep(80);
//         delay(70);
//     }
// }
