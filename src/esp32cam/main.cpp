#include <Arduino.h>

#define FLASH_LED 4

String line;

void blinkOnce() {
  digitalWrite(FLASH_LED, HIGH);
  delay(200);
  digitalWrite(FLASH_LED, LOW);
}

void setup() {
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  // ATmega와 UART0로 통신 (U0RXD=GPIO3, U0TXD=GPIO1)
  Serial.begin(9600);

  delay(500);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        // 예: "W=123.4" 수신
        blinkOnce();

        // ACK 회신
        Serial.print("ACK ");
        Serial.println(line);
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}
