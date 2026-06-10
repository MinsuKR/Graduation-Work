#include <Arduino.h>

#include "Wifi_Unit.h"

// ===== 공유기 정보 =====
static const char* WIFI_SSID = "ldh";
static const char* WIFI_PASS = "dlehdgns2001";

// ===== 라즈베리파이 TCP 서버 정보 =====
static const char* PI_SERVER_IP  = "10.221.11.227";   // ★ 라즈베리파이 IP (확인 후 수정)
static const uint16_t PI_SERVER_PORT = 5000;

static String uartBuf;
// 이전 연결 상태 기억 + 딱 한 번 로그 찍음
static bool lastWifiOk = false;
static bool lastServerOk = false;

// 연결된 기기 상태 메세지 (모든 응답)
static void sendAck(const String& s) {
  Serial.print(s);
  Serial.print("\n");
}


// 데이터가 들어오면 실행되는 로직 핵심
static void handleAtLine(const String& line) {
  // 예: W=123.4;PIC W시작 아니면 무시
  if (!line.startsWith("W=")) return;

  // WIFI 끊겼을 때
  if (WiFi.status() != WL_CONNECTED) {
    sendAck("ACK B ST=WIFI");
    return;
  }

  // 서버 끊겼을 때
  if (!WifiUnit_ServerConnected()) {
    sendAck("ACK B ST=SERVER");
    return;
  }

  // 모든 상태 ok
  sendAck("ACK B ST=UPLOAD IP=" + WifiUnit_IpStr());

  // 에러 시
  if (!WifiUnit_SendLine(line)) {
    sendAck("ACK F R=NO_SERVER IP=" + WifiUnit_IpStr());
  }
}

void setup() {
  Serial.begin(115200);
  delay(300);

  sendAck("ACK B ST=BOOT");

  bool ok = WifiUnit_Ensure(WIFI_SSID, WIFI_PASS, 10000);
  if (ok) {  
    sendAck("ACK B ST=WIFI_OK IP=" + WifiUnit_IpStr());
  } else {
    sendAck("ACK B ST=WIFI_FAIL");
  }

  WifiUnit_BeginBridge(WIFI_SSID, WIFI_PASS, PI_SERVER_IP, PI_SERVER_PORT);
}

void loop() {
  // 자동 복구
  WifiUnit_LoopBridge();

  // 와이파이 신호 잡고, 서버까지 연결되었다는 뜻
  bool wifiOk = (WiFi.status() == WL_CONNECTED);
  bool serverOk = WifiUnit_ServerConnected();

  // 방금 막 연결되었을 때 상태 알려줌
  if (wifiOk && !lastWifiOk) {
    sendAck("ACK B ST=WIFI_OK IP=" + WifiUnit_IpStr());
  }
  lastWifiOk = wifiOk;

  if (serverOk && !lastServerOk) {
    sendAck("ACK B ST=SERVER_OK IP=" + WifiUnit_IpStr());
  }
  lastServerOk = serverOk;

  // ===== UART 수신 =====
  while (Serial.available() > 0) {
    char c = (char)Serial.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (uartBuf.length() > 0) {
        handleAtLine(uartBuf);
        uartBuf = "";
      }
    } else {
      if (uartBuf.length() < 255) uartBuf += c;
    }
  }

  // ===== 서버 수신 =====
  String netLine;
  if (WifiUnit_ReadLine(netLine)) {
    // 라즈베리파이에서 아래처럼 보낸다고 가정
    // ACK OK G=1+ T=pork C=91
    // ACK W=123.4 IP=192.168.0.55
    // ACK F R=MODEL
    sendAck(netLine);
  }
}
