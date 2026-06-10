// // namespace는 c++사용하는 = static랑 같음

// #include "WiFi_Unit.h"

// namespace {
//   const char* g_ssid = nullptr;
//   const char* g_pass = nullptr;
//   const char* g_serverIp = nullptr;
//   uint16_t g_serverPort = 0;

//   WiFiClient g_client;

//   uint32_t g_lastWifiRetryMs = 0;
//   uint32_t g_lastServerRetryMs = 0;

//   const uint32_t WIFI_RETRY_MS   = 5000;
//   const uint32_t SERVER_RETRY_MS = 3000;

//   // 서버에서 온 데이터를 한 글자씩 담아두는 저장소
//   String g_rxBuf;

//   // Wi-Fi가 끊기면 5초마다 재연결
//   void ensureWifiInternal() {
//     if (!g_ssid || !g_pass) return;
//     if (WiFi.status() == WL_CONNECTED) return;

//     uint32_t now = millis();
//     if (now - g_lastWifiRetryMs < WIFI_RETRY_MS) return;
//     g_lastWifiRetryMs = now;

//     WiFi.mode(WIFI_STA);
//     WiFi.begin(g_ssid, g_pass);
//   }

//   // Wi-Fi가 연결된 상태에서 서버와 연결이 끊기면 3초마다 재접속
//   void ensureServerInternal() {
//     if (WiFi.status() != WL_CONNECTED) return;
//     if (!g_serverIp || g_serverPort == 0) return;
//     if (g_client.connected()) return;

//     uint32_t now = millis();
//     if (now - g_lastServerRetryMs < SERVER_RETRY_MS) return;
//     g_lastServerRetryMs = now;

//     g_client.stop();
//     g_client.connect(g_serverIp, g_serverPort);
//   }
// }

// // 처음 시작 시 호출하면 그 자리에서 기다림 = 강제 대기(setup에서 사용) 
// bool WifiUnit_Ensure(const char* ssid, const char* pass, uint32_t timeoutMs) {
//   if (WiFi.status() == WL_CONNECTED) return true;

//   WiFi.mode(WIFI_STA);
//   WiFi.begin(ssid, pass);

//   uint32_t t0 = millis();
//   while (millis() - t0 < timeoutMs) {
//     if (WiFi.status() == WL_CONNECTED) return true;
//     delay(200);
//   }
//   return false;
// }

// // 기기가 할당 받은 IP주소 문자열로 반환
// String WifiUnit_IpStr() {
//   if (WiFi.status() != WL_CONNECTED) return "0.0.0.0";
//   return WiFi.localIP().toString();
// }

// // 통신에 필요한 모든 정보를 관리 해줌 (Wi-Fi 처음 연결)
// // 자동 복구 하려고 시도
// void WifiUnit_BeginBridge(const char* ssid, const char* pass, const char* serverIp, uint16_t serverPort) {
//   g_ssid = ssid;
//   g_pass = pass;
//   g_serverIp = serverIp;
//   g_serverPort = serverPort;

//   WiFi.mode(WIFI_STA);
//   WiFi.begin(g_ssid, g_pass);
// }

// // 무한 반복으로 연결 시도해 자동 복구해줌
// void WifiUnit_LoopBridge() {
//   ensureWifiInternal();
//   ensureServerInternal();
// }

// // 서버가 연결되어 있을때, 데이터 보냄 ('\n'로 한 줄 인식)
// bool WifiUnit_SendLine(const String& line) {
//   if (!g_client.connected()) return false;
//   g_client.print(line);
//   g_client.print("\n");
//   return true;
// }

// // 서버로 온 데이터를 읽음 (outline에 모은 문자열을 저장 그리고 "" 비움)
// bool WifiUnit_ReadLine(String& outLine) {
//   outLine = "";

//   if (!g_client.connected()) return false;

//   while (g_client.available() > 0) {
//     char c = (char)g_client.read();
//     if (c == '\r') continue;

//     if (c == '\n') {
//       if (g_rxBuf.length() == 0) continue;
//       outLine = g_rxBuf;
//       g_rxBuf = "";
//       return true;
//     } else {
//       if (g_rxBuf.length() < 255) {
//         g_rxBuf += c;
//       } else {
//         g_rxBuf = "";
//       }
//     }
//   }
//   return false;
// }

// // 서버 상태 확인
// bool WifiUnit_ServerConnected() {
//   return g_client.connected();
// }

// namespace는 c++사용하는 = static랑 같음

#include "WiFi_Unit.h"

namespace {
  const char* g_ssid = nullptr;
  const char* g_pass = nullptr;
  const char* g_serverIp = nullptr;
  uint16_t g_serverPort = 0;

  WiFiClient g_client;

  uint32_t g_lastWifiRetryMs = 0;
  uint32_t g_lastServerRetryMs = 0;

  const uint32_t WIFI_RETRY_MS   = 5000;
  const uint32_t SERVER_RETRY_MS = 3000;

  // 서버에서 온 데이터를 한 글자씩 담아두는 저장소
  String g_rxBuf;

  // Wi-Fi가 끊기면 5초마다 재연결
  void ensureWifiInternal() {
    if (!g_ssid || !g_pass) return;
    if (WiFi.status() == WL_CONNECTED) return;

    uint32_t now = millis();
    if (now - g_lastWifiRetryMs < WIFI_RETRY_MS) return;
    g_lastWifiRetryMs = now;

    WiFi.mode(WIFI_STA);
    WiFi.begin(g_ssid, g_pass);
  }

  // Wi-Fi가 연결된 상태에서 서버와 연결이 끊기면 3초마다 재접속
  void ensureServerInternal() {
    if (WiFi.status() != WL_CONNECTED) return;
    if (!g_serverIp || g_serverPort == 0) return;
    if (g_client.connected()) return;

    uint32_t now = millis();
    if (now - g_lastServerRetryMs < SERVER_RETRY_MS) return;
    g_lastServerRetryMs = now;

    g_client.stop();
    g_client.connect(g_serverIp, g_serverPort);
  }
}

// 처음 시작 시 호출하면 그 자리에서 기다림 = 강제 대기(setup에서 사용) 
bool WifiUnit_Ensure(const char* ssid, const char* pass, uint32_t timeoutMs) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, pass);

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return false;
}

// 기기가 할당 받은 IP주소 문자열로 반환
String WifiUnit_IpStr() {
  if (WiFi.status() != WL_CONNECTED) return "0.0.0.0";
  return WiFi.localIP().toString();
}

// 통신에 필요한 모든 정보를 관리 해줌 (Wi-Fi 처음 연결)
// 자동 복구 하려고 시도
// ★ 수정: 이미 WiFi 연결되어 있으면 WiFi.begin() 재호출하지 않음
void WifiUnit_BeginBridge(const char* ssid, const char* pass, const char* serverIp, uint16_t serverPort) {
  g_ssid = ssid;
  g_pass = pass;
  g_serverIp = serverIp;
  g_serverPort = serverPort;

  // // 이미 WiFi 연결되어 있으면 재시작하지 않음 (연결 끊김 방지)
  // if (WiFi.status() != WL_CONNECTED) {
  //   WiFi.mode(WIFI_STA);
  //   WiFi.begin(g_ssid, g_pass);
  // }

  // // ★ 추가: WiFi 연결 상태면 즉시 서버 연결 시도
  // if (WiFi.status() == WL_CONNECTED && g_serverIp && g_serverPort > 0) {
  //   g_client.connect(g_serverIp, g_serverPort);
  // }
}

// 무한 반복으로 연결 시도해 자동 복구해줌
void WifiUnit_LoopBridge() {
  ensureWifiInternal();
  ensureServerInternal();
}

// 서버가 연결되어 있을때, 데이터 보냄 ('\n'로 한 줄 인식)
bool WifiUnit_SendLine(const String& line) {
  if (!g_client.connected()) return false;
  g_client.print(line);
  g_client.print("\n");
  return true;
}

// 서버로 온 데이터를 읽음 (outline에 모은 문자열을 저장 그리고 "" 비움)
bool WifiUnit_ReadLine(String& outLine) {
  outLine = "";

  if (!g_client.connected()) return false;

  while (g_client.available() > 0) {
    char c = (char)g_client.read();
    if (c == '\r') continue;

    if (c == '\n') {
      if (g_rxBuf.length() == 0) continue;
      outLine = g_rxBuf;
      g_rxBuf = "";
      return true;
    } else {
      if (g_rxBuf.length() < 255) {
        g_rxBuf += c;
      } else {
        g_rxBuf = "";
      }
    }
  }
  return false;
}

// 서버 상태 확인
bool WifiUnit_ServerConnected() {
  return g_client.connected();
}