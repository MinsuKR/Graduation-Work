#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

unsigned long lastSend = 0;
int counter = 0;

char rxBuf[40];
uint8_t rxIdx = 0;

void showLCD(const char* l1, const char* l2) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(l1);
  lcd.setCursor(0, 1); lcd.print(l2);
}

void setup() {
  Serial.begin(9600);

  Wire.begin();
  lcd.init();
  lcd.backlight();

  showLCD("ATmega UART0", "Boot...");
  delay(1000);

  // ESP32 부팅 안정화 시간
  showLCD("Wait ESP32", "3 sec");
  delay(3000);

  showLCD("Ready", "Send PING");
}

void loop() {
  // 1초마다 PING 보내기
  if (millis() - lastSend >= 1000) {
    lastSend = millis();
    counter++;

    Serial.print("PING ");
    Serial.println(counter);

    char l1[17], l2[17];
    snprintf(l1, sizeof(l1), "TX PING %d", counter);
    snprintf(l2, sizeof(l2), "wait ACK");
    showLCD(l1, l2);
  }

  // ACK 수신 → LCD 표시
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      rxBuf[rxIdx] = '\0';
      rxIdx = 0;

      // LCD 2줄에 표시 (길면 잘림)
      showLCD("RX:", rxBuf);

    } else {
      if (rxIdx < sizeof(rxBuf) - 1) rxBuf[rxIdx++] = c;
      else rxIdx = 0;
    }
  }
}
