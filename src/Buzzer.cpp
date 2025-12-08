#include "Buzzer.h"

void Buzzer_Init() {
    pinMode(BUZZER_PIN, OUTPUT);
    digitalWrite(BUZZER_PIN, LOW);
}

void Buzzer_Beep(uint16_t duration) {
    digitalWrite(BUZZER_PIN, HIGH);
    delay(duration);
    digitalWrite(BUZZER_PIN, LOW);
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
