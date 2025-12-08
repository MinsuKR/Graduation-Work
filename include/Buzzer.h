#ifndef BUZZER_H
#define BUZZER_H

#include <Arduino.h>

#define BUZZER_PIN PIN_PA7

void Buzzer_Init();

// 짧은 삑 (기본 알림)
void Buzzer_Beep(uint16_t duration = 70);

// 버튼 클릭 느낌
void Buzzer_Click();

// 성공/완료음
void Buzzer_Success();

// 에러음
void Buzzer_Error();

#endif
