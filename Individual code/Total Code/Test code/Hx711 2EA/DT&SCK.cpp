// H (Busy) : 지금 준비 대기중 상태
// L (Ready) : 계산 끝! 24비트 데이터 준비됐다고 알려줌
// SCK(펄스) : 데이터 전송 SCK를 24번 거쳐서
// 그후 H (Busy) : 데이터 줬을 때 

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

LiquidCrystal_I2C lcd(0x27, 16, 2);

void setup() {
  lcd.init();
  lcd.backlight();
  pinMode(DT_L, INPUT);
  pinMode(SCK_L, OUTPUT);
  pinMode(DT_R, INPUT);
  pinMode(SCK_R, OUTPUT);
  digitalWrite(SCK_L, LOW);
  digitalWrite(SCK_R, LOW);
  lcd.clear();
}

void loop() {
  // --- [왼쪽 센서 시연] ---
  lcd.setCursor(0, 0);
  lcd.print("L: BUSY (H)...  "); // 먼저 Busy를 출력해서 보여줌
  delay(1000);                   // 카메라에 Busy가 담길 시간을 줌

  // DT가 Low가 될 때까지 기다림 (실제 준비 완료 순간 포착)
  while(digitalRead(DT_L) == HIGH); 

  lcd.setCursor(0, 0);
  lcd.print("L: READY (L)!!  "); // 준비 완료 표시
  
  // 데이터 읽기 완료 처리 (27번 토글)
  for(int i = 0; i < 27; i++) {
    digitalWrite(SCK_L, HIGH); delayMicroseconds(2);
    digitalWrite(SCK_L, LOW);  delayMicroseconds(2);
  }
  delay(2000);                  // READY 상태를 길게 보여줌

  // --- [오른쪽 센서 시연] ---
  lcd.setCursor(0, 1);
  lcd.print("R: BUSY (H)...  ");
  delay(1000);

  while(digitalRead(DT_R) == HIGH);

  lcd.setCursor(0, 1);
  lcd.print("R: READY (L)!!  ");
  
  for(int i = 0; i < 27; i++) {
    digitalWrite(SCK_R, HIGH); delayMicroseconds(2);
    digitalWrite(SCK_R, LOW);  delayMicroseconds(2);
  }
  delay(2000);
}