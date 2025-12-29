// #include <Arduino.h>

// static const int ESP_RX = 15; // 네가 쓰는 RX
// static const int ESP_TX = 14; // 네가 쓰는 TX
// static const int FLASH_LED = 4;

// HardwareSerial Uart1(1);
// String line;

// void blinkOnce() {
//   digitalWrite(FLASH_LED, HIGH);
//   delay(80);
//   digitalWrite(FLASH_LED, LOW);
// }

// void setup() {
//   pinMode(FLASH_LED, OUTPUT);
//   digitalWrite(FLASH_LED, LOW);

//   Uart1.begin(9600, SERIAL_8N1, ESP_RX, ESP_TX);
//   delay(2000); // 부팅 안정
// }

// void loop() {
//   while (Uart1.available()) {
//     char c = (char)Uart1.read();
//     if (c == '\n') {
//       line.trim();
//       if (line.length()) {
//         blinkOnce();                 // ✅ 수신 확인
//         Uart1.print("ACK ");
//         Uart1.println(line);         // ✅ 응답
//       }
//       line = "";
//     } else if (c != '\r') {
//       line += c;
//     }
//   }
// }
#include <Arduino.h>

#define FLASH_LED 4

String line;

void blinkOnce() {
  digitalWrite(FLASH_LED, HIGH);
  delay(80);
  digitalWrite(FLASH_LED, LOW);
}

void setup() {
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  // UART0를 ATmega와 통신에 사용
  Serial.begin(9600);
  delay(2000);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length()) {
        blinkOnce();              // ✅ PING 수신 눈으로 확인
        Serial.print("ACK ");
        Serial.println(line);     // ✅ ACK 전송
      }
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}
