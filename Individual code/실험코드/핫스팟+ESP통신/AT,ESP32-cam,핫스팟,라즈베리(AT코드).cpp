/*
 * ATmega128 양방향 통신 테스트
 * ESP32를 통해 라즈베리파이와 통신
 *
 * 구조:
 *   ATmega128 --UART(Serial)--> ESP32 --WiFi--> 라즈베리파이
 *
 * 배선:
 *   PE1 (TX0) → ESP32 GPIO16 (RX2)
 *   PE0 (RX0) ← ESP32 GPIO17 (TX2)
 *   GND       ↔ ESP32 GND
 *
 * 주의: Serial을 ESP32 통신에 사용하므로 PC 시리얼 모니터 사용 불가
 *       통신 확인은 라즈베리파이 터미널에서 확인
 */

#include <Arduino.h>

static String espBuf;

void handleResponse(const String& line);

void setup() {
    Serial.begin(115200);  // ESP32 UART 통신 (PE0=RX0, PE1=TX0)
    delay(10000);           // ESP32 부팅 + WiFi 접속 대기
}

void loop() {
    // ===== ESP32로부터 수신 =====
    while (Serial.available() > 0) {
        char c = (char)Serial.read();
        if (c == '\r') continue;

        if (c == '\n') {
            if (espBuf.length() > 0) {
                handleResponse(espBuf);
                espBuf = "";
            }
        } else {
            if (espBuf.length() < 255) espBuf += c;
        }
    }

    // ===== 3초마다 PING 자동 전송 =====
    static unsigned long lastSend = 0;
    static int count = 0;
    if (millis() - lastSend > 3000) {
        count++;
        String msg = "PING " + String(count);
        Serial.println(msg);
        lastSend = millis();
    }
}

void handleResponse(const String& line) {
    // 라즈베리파이에서 PONG이 돌아오면 양방향 통신 성공
    // (시리얼 모니터 없으므로 여기서는 확인 불가, 라즈베리파이에서 확인)
}