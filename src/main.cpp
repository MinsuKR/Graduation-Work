#include <Arduino.h>
#include <Wire.h> // I²C 통신
#include <LiquidCrystal_I2C.h>
#include <HX711.h>
#include <EEPROM.h> // 전원이 꺼져도 데이터가 유지되는 작은 메모리 공간

#define LED_PIN     PA0 // LED
#define BUZZER_PIN  PA1 // Buzzer

// constexpr uint8_t => C++ 컴파일 시점 상수]

// LiquidCrystal_I2C lcd(0x27, 16, 2)랑 같은말(제어)
constexpr uint8_t LCD_ADDR   = 0x27;  // 보통 0x27 또는 0x3F
constexpr uint8_t LCD_COLS   = 16;
constexpr uint8_t LCD_ROWS   = 2;

// HX711 핀 (임의의 디지털 핀 2개)
constexpr uint8_t HX_DT_PIN  = 6;   // HX711 DT
constexpr uint8_t HX_SCK_PIN = 7;   // HX711 SCK

// 측정/표시 파라미터
constexpr uint16_t SAMPLE_AVG   = 5;   // 평균 샘플 수
constexpr uint16_t UPDATE_MS    = 200; // LCD/로그 업데이트 주기(ms)
constexpr uint16_t BEEP_MS      = 120; // 경고음 길이(ms)

LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);
HX711 scale;

void beep(uint16_t ms = BEEP_MS) {
  PORTA |= (1 << BUZZER_PIN);
  delay(ms);
  PORTA &= ~(1 << BUZZER_PIN);
}

void ledOn()  { PORTA |=  (1 << LED_PIN); }
void ledOff() { PORTA &= ~(1 << LED_PIN); }

void setup() {

}

void loop() {

}