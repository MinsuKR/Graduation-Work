// esp-cam main
#include <Arduino.h>

static String g_line = "";
static const char* TEST_IP = "192.168.0.100";

void setup() {
    Serial.begin(115200);
    Serial.print("ACK BOOT OK IP=");
    Serial.println(TEST_IP);
}

void handleLine(String s) {
    if (s.indexOf("PIC") < 0) return;

    // [1단계] LCD에 "ST:CONNECT" 표시 (1.5초)
    Serial.print("ACK B ST=CONNECT IP=");
    Serial.println(TEST_IP);
    delay(1500); 

    // [2단계] LCD에 "ST:SYNC" 표시 (1.5초)
    Serial.print("ACK B ST=SYNC IP=");
    Serial.println(TEST_IP);
    delay(1500);

    // [3단계] ATmega에게 결과 화면 전환 신호 보냄
    // 이제 ATmega 내부에서 문구를 직접 출력하므로 데이터 내용은 중요하지 않음
    Serial.println("ACK OK G=1 T=1 C=1"); 
    delay(200); 

    // [4단계] 무게 값 전달 및 최종 화면 표시
    int wPos = s.indexOf("W=");
    String wVal = "0.0";
    if (wPos >= 0) {
        int endPos = s.indexOf(';', wPos);
        if (endPos == -1) endPos = s.length();
        wVal = s.substring(wPos + 2, endPos);
        wVal.trim();
    }
    
    Serial.print("ACK W=");
    Serial.print(wVal);
    Serial.print(" IP=");
    Serial.println(TEST_IP);
}

void loop() {
    while (Serial.available()) {
        char c = (char)Serial.read();
        if (c == '\n') {
            if (g_line.length() > 0) handleLine(g_line);
            g_line = "";
        } else if (c != '\r') {
            g_line += c;
        }
    }
}

// ATmega uart.ack

// Uart_Ack.cpp 파일의 showResult 함수를 수정합니다.
static void showResult(LiquidCrystal_I2C& lcd, const char* g, const char* t, const char* c, const char* w) {
    lcd.clear();
    
    // 첫 번째 줄: 통신 상태 제목
    lcd.setCursor(0, 0);
    lcd.print("UART COMM TEST"); 

    // 두 번째 줄: 결과와 무게 표시
    lcd.setCursor(0, 1);
    lcd.print("SUCCESS! W=");
    lcd.print((w && w[0]) ? w : "0.0");
    lcd.print("g");
}