#include <Arduino.h>

#define FLASH_LED 4 // 플래시 LED 핀 번호를 4번(GPIO4)로 정의

String line; // 시리얼로 받은 한 줄(문장)을 누적할 문자열 버퍼

void blinkOnce() {
  digitalWrite(FLASH_LED, HIGH);
  delay(200);
  digitalWrite(FLASH_LED, LOW);
}

void setup() {
  // 플래시 LED 핀을 출력으로 설정하고 처음에는 꺼둠
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  // ATmega와 UART0로 통신 (U0RXD=GPIO3, U0TXD=GPIO1)
  Serial.begin(9600);

  delay(500);
}

void loop() {
  while (Serial.available()) // 시리얼 버퍼에 읽을 데이터가 있는 동안 계속 처리
  {
    char c = (char)Serial.read(); // 문자 1개를 읽음
    if (c == '\n') // 줄바꿈을 만나면 “한 줄 수신 완료”로 판단
    {
      line.trim(); // 문자열 앞뒤 공백/개행 같은 걸 정리

      if (line.length()) // 빈 줄이면 무시, 내용이 있으면 처리
      {
        // 예: "W=123.4" 수신

        blinkOnce(); // “뭔가 받았다”는 표시로 LED 한번 점등

        // ACK 회신
        Serial.print("ACK ");
        Serial.println(line);
      }
      line = ""; // 다음 줄을 받기 위해 버퍼 비움
    } 
    else if (c != '\r') // \r(캐리지리턴)은 무시하고, 나머지 문자는 line에 계속 누적
    {
      line += c;
    }
  }
}
