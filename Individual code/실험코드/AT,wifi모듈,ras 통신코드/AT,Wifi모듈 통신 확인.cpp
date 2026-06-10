
// #include <Arduino.h>
// #include <Wire.h>
// #include <LiquidCrystal_I2C.h>

// // ===== 사용자 설정 =====
// static const char* WIFI_SSID = "AIscale";
// static const char* WIFI_PASS = "shinhan1234";

// // LCD 주소: 보통 0x27 또는 0x3F
// LiquidCrystal_I2C lcd(0x3F, 16, 2);

// // WizFi360 UART
// #define WIFI_SERIAL Serial

// String rxBuf;
// String ipAddr = "";

// // =============================
// // LCD 함수
// // =============================
// void lcdShow(const char* line1, const char* line2 = "")
// {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print(line1);
//   lcd.setCursor(0, 1);
//   lcd.print(line2);
// }

// void lcdShowStr(String line1, String line2 = "")
// {
//   lcd.clear();
//   lcd.setCursor(0, 0);
//   lcd.print(line1.substring(0, 16));
//   lcd.setCursor(0, 1);
//   lcd.print(line2.substring(0, 16));
// }

// // =============================
// // UART / AT 함수
// // =============================
// void flushWifiSerial()
// {
//   while (WIFI_SERIAL.available()) {
//     WIFI_SERIAL.read();
//   }
// }

// void sendCmd(const String& cmd)
// {
//   WIFI_SERIAL.print(cmd);
//   WIFI_SERIAL.print("\r\n");
// }

// bool waitForKeyword(const char* keyword, unsigned long timeoutMs)
// {
//   rxBuf = "";
//   unsigned long start = millis();

//   while (millis() - start < timeoutMs) {
//     while (WIFI_SERIAL.available()) {
//       char c = (char)WIFI_SERIAL.read();
//       rxBuf += c;

//       if (rxBuf.indexOf(keyword) >= 0) {
//         return true;
//       }
//     }
//   }
//   return false;
// }

// bool waitForAnyKeyword(const char* key1, const char* key2, unsigned long timeoutMs)
// {
//   rxBuf = "";
//   unsigned long start = millis();

//   while (millis() - start < timeoutMs) {
//     while (WIFI_SERIAL.available()) {
//       char c = (char)WIFI_SERIAL.read();
//       rxBuf += c;

//       if (rxBuf.indexOf(key1) >= 0) return true;
//       if (rxBuf.indexOf(key2) >= 0) return true;
//     }
//   }
//   return false;
// }

// bool sendCmdWait(const String& cmd, const char* keyword, unsigned long timeoutMs)
// {
//   flushWifiSerial();
//   sendCmd(cmd);
//   return waitForKeyword(keyword, timeoutMs);
// }

// // =============================
// // IP 파싱 함수
// // =============================

// // "123.45.67.89" 형태의 IP를 rxBuf 안에서 찾아서 반환
// String extractIPAddress(const String& s)
// {
//   for (int i = 0; i < s.length(); i++) {
//     if (!isDigit((unsigned char)s[i])) continue;

//     int start = i;
//     int dotCount = 0;
//     bool valid = true;

//     while (i < s.length()) {
//       char c = s[i];
//       if (isDigit((unsigned char)c)) {
//         i++;
//       }
//       else if (c == '.') {
//         dotCount++;
//         i++;
//       }
//       else {
//         break;
//       }
//     }

//     String token = s.substring(start, i);

//     // 점이 3개여야 IPv4 가능성 있음
//     if (dotCount != 3) {
//       continue;
//     }

//     // 토큰 검증
//     int partStart = 0;
//     int parts = 0;

//     for (int j = 0; j <= token.length(); j++) {
//       if (j == token.length() || token[j] == '.') {
//         String part = token.substring(partStart, j);

//         if (part.length() == 0 || part.length() > 3) {
//           valid = false;
//           break;
//         }

//         int value = part.toInt();
//         if (value < 0 || value > 255) {
//           valid = false;
//           break;
//         }

//         parts++;
//         partStart = j + 1;
//       }
//     }

//     if (valid && parts == 4) {
//       return token;
//     }
//   }

//   return "";
// }

// // =============================
// // WiFi 단계 함수
// // =============================
// bool wifiATCheck()
// {
//   lcdShow("AT check...", "");
//   return sendCmdWait("AT", "OK", 2000);
// }

// bool wifiDisableEcho()
// {
//   lcdShow("ATE0...", "");
//   return sendCmdWait("ATE0", "OK", 2000);
// }

// bool wifiSetMode()
// {
//   lcdShow("Set STA mode", "");
//   return sendCmdWait("AT+CWMODE=1", "OK", 3000);
// }

// bool wifiJoinAP()
// {
//   lcdShow("WiFi Join...", "");
//   String cmd = String("AT+CWJAP=\"") + WIFI_SSID + "\",\"" + WIFI_PASS + "\"";

//   flushWifiSerial();
//   sendCmd(cmd);

//   unsigned long start = millis();
//   rxBuf = "";

//   while (millis() - start < 20000) {
//     while (WIFI_SERIAL.available()) {
//       char c = (char)WIFI_SERIAL.read();
//       rxBuf += c;

//       if (rxBuf.indexOf("OK") >= 0) return true;
//       if (rxBuf.indexOf("FAIL") >= 0) return false;
//       if (rxBuf.indexOf("ERROR") >= 0) return false;
//     }
//   }

//   if (rxBuf.indexOf("GOT IP") >= 0) return true;
//   if (rxBuf.indexOf("WIFI CONNECTED") >= 0) return true;

//   return false;
// }

// bool wifiGetIP()
// {
//   lcdShow("Checking IP...", "Wait DHCP...");

//   flushWifiSerial();
//   sendCmd("AT+CIFSR");

//   unsigned long start = millis();
//   rxBuf = "";

//   while (millis() - start < 5000) {
//     while (WIFI_SERIAL.available()) {
//       char c = (char)WIFI_SERIAL.read();
//       rxBuf += c;
//     }
//   }

//   ipAddr = extractIPAddress(rxBuf);

//   if (ipAddr.length() > 0) {
//     return true;
//   }

//   return false;
// }

// void stopHere(const char* line1, const char* line2 = "")
// {
//   lcdShow(line1, line2);
//   while (1) {
//     delay(100);
//   }
// }

// // =============================
// // setup / loop
// // =============================
// void setup()
// {
//   lcd.init();
//   lcd.backlight();
//   lcdShow("Booting...", "");
//   delay(1000);

//   WIFI_SERIAL.begin(115200);
//   delay(3000);

//   if (!wifiATCheck()) {
//     stopHere("AT FAIL", "Check TX/RX");
//   }
//   lcdShow("AT OK", "");
//   delay(1000);

//   if (!wifiDisableEcho()) {
//     stopHere("ATE0 FAIL", "");
//   }
//   lcdShow("ECHO OFF", "");
//   delay(800);

//   if (!wifiSetMode()) {
//     stopHere("MODE FAIL", "");
//   }
//   lcdShow("STA MODE OK", "");
//   delay(1000);

//   if (!wifiJoinAP()) {
//     stopHere("WIFI FAIL", "SSID/PW ERR");
//   }
//   lcdShow("WIFI CONNECTED", "");
//   delay(1000);

//   if (!wifiGetIP()) {
//     stopHere("IP FAIL", "No DHCP");
//   }

//   lcdShowStr("SSID:" + String(WIFI_SSID), "IP:" + ipAddr);
// }

// void loop()
// {
//   // 필요하면 주기적으로 IP 재확인 가능
// }

#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// ===== 사용자 설정 =====
static const char* WIFI_SSID = "AIscale";
static const char* WIFI_PASS = "shinhan1234";

// ===== 라즈베리파이 TCP 서버 정보 =====
static const char* RPI_IP   = "10.147.83.227";  // ★ 라즈베리파이 IP (확인 후 수정)
static const uint16_t RPI_PORT = 5000;

// LCD 주소: 보통 0x27 또는 0x3F
LiquidCrystal_I2C lcd(0x3F, 16, 2);

// WizFi360 UART
#define WIFI_SERIAL Serial

String rxBuf;
String ipAddr = "";
bool serverConnected = false;
int pingCount = 0;

// =============================
// LCD 함수
// =============================
void lcdShow(const char* line1, const char* line2 = "")
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

void lcdShowStr(String line1, String line2 = "")
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

// =============================
// UART / AT 함수
// =============================
void flushWifiSerial()
{
  while (WIFI_SERIAL.available()) {
    WIFI_SERIAL.read();
  }
}

void sendCmd(const String& cmd)
{
  WIFI_SERIAL.print(cmd);
  WIFI_SERIAL.print("\r\n");
}

bool waitForKeyword(const char* keyword, unsigned long timeoutMs)
{
  rxBuf = "";
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      rxBuf += c;

      if (rxBuf.indexOf(keyword) >= 0) {
        return true;
      }
    }
  }
  return false;
}

bool sendCmdWait(const String& cmd, const char* keyword, unsigned long timeoutMs)
{
  flushWifiSerial();
  sendCmd(cmd);
  return waitForKeyword(keyword, timeoutMs);
}

// =============================
// IP 파싱 함수
// =============================
String extractIPAddress(const String& s)
{
  for (int i = 0; i < (int)s.length(); i++) {
    if (!isDigit((unsigned char)s[i])) continue;

    int start = i;
    int dotCount = 0;
    bool valid = true;

    while (i < (int)s.length()) {
      char c = s[i];
      if (isDigit((unsigned char)c)) { i++; }
      else if (c == '.') { dotCount++; i++; }
      else { break; }
    }

    String token = s.substring(start, i);
    if (dotCount != 3) continue;

    int partStart = 0;
    int parts = 0;

    for (int j = 0; j <= (int)token.length(); j++) {
      if (j == (int)token.length() || token[j] == '.') {
        String part = token.substring(partStart, j);
        if (part.length() == 0 || part.length() > 3) { valid = false; break; }
        int value = part.toInt();
        if (value < 0 || value > 255) { valid = false; break; }
        parts++;
        partStart = j + 1;
      }
    }

    if (valid && parts == 4) return token;
  }
  return "";
}

// =============================
// WiFi 단계 함수
// =============================
bool wifiATCheck()
{
  lcdShow("AT check...", "");
  return sendCmdWait("AT", "OK", 2000);
}

bool wifiDisableEcho()
{
  lcdShow("ATE0...", "");
  return sendCmdWait("ATE0", "OK", 2000);
}

bool wifiSetMode()
{
  lcdShow("Set STA mode", "");
  return sendCmdWait("AT+CWMODE=1", "OK", 3000);
}

bool wifiJoinAP()
{
  lcdShow("WiFi Join...", "");
  String cmd = String("AT+CWJAP=\"") + WIFI_SSID + "\",\"" + WIFI_PASS + "\"";

  flushWifiSerial();
  sendCmd(cmd);

  unsigned long start = millis();
  rxBuf = "";

  while (millis() - start < 20000) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      rxBuf += c;

      if (rxBuf.indexOf("OK") >= 0) return true;
      if (rxBuf.indexOf("FAIL") >= 0) return false;
      if (rxBuf.indexOf("ERROR") >= 0) return false;
    }
  }

  if (rxBuf.indexOf("GOT IP") >= 0) return true;
  if (rxBuf.indexOf("WIFI CONNECTED") >= 0) return true;

  return false;
}

bool wifiGetIP()
{
  lcdShow("Checking IP...", "Wait DHCP...");

  flushWifiSerial();
  sendCmd("AT+CIFSR");

  unsigned long start = millis();
  rxBuf = "";

  while (millis() - start < 5000) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      rxBuf += c;
    }
  }

  ipAddr = extractIPAddress(rxBuf);
  return (ipAddr.length() > 0);
}

// =============================
// TCP 연결 + 데이터 송수신
// =============================
bool tcpConnect()
{
  lcdShow("TCP Connect...", RPI_IP);
  String cmd = String("AT+CIPSTART=\"TCP\",\"") + RPI_IP + "\"," + String(RPI_PORT);
  return sendCmdWait(cmd, "OK", 10000);
}

bool tcpSend(const String& data)
{
  // 전송할 데이터 길이 (줄바꿈 포함)
  int len = data.length() + 1;  // +1 for \n
  String cmd = "AT+CIPSEND=" + String(len);

  flushWifiSerial();
  sendCmd(cmd);

  // '>' 프롬프트 대기
  if (!waitForKeyword(">", 3000)) return false;

  // 데이터 전송
  WIFI_SERIAL.print(data);
  WIFI_SERIAL.print("\n");

  // 전송 완료 확인
  return waitForKeyword("SEND OK", 5000);
}

String tcpReceive(unsigned long timeoutMs)
{
  // +IPD,길이:데이터 형식에서 데이터 추출
  rxBuf = "";
  unsigned long start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      rxBuf += c;

      if (rxBuf.indexOf("\n") >= 0) {
        // +IPD,길이:데이터 파싱
        int colonIdx = rxBuf.indexOf(':');
        if (colonIdx >= 0) {
          String data = rxBuf.substring(colonIdx + 1);
          data.trim();
          return data;
        }
        return rxBuf;
      }
    }
  }
  return "";
}

void stopHere(const char* line1, const char* line2 = "")
{
  lcdShow(line1, line2);
  while (1) { delay(100); }
}

// =============================
// setup
// =============================
void setup()
{
  lcd.init();
  lcd.backlight();
  lcdShow("Booting...", "");
  delay(1000);

  WIFI_SERIAL.begin(115200);
  delay(3000);

  // AT 확인
  if (!wifiATCheck()) stopHere("AT FAIL", "Check TX/RX");
  lcdShow("AT OK", "");
  delay(500);

  // 에코 끄기
  if (!wifiDisableEcho()) stopHere("ATE0 FAIL", "");
  delay(500);

  // STA 모드
  if (!wifiSetMode()) stopHere("MODE FAIL", "");
  delay(500);

  // WiFi 접속
  if (!wifiJoinAP()) stopHere("WIFI FAIL", "SSID/PW ERR");
  lcdShow("WIFI OK", "");
  delay(500);

  // IP 확인
  if (!wifiGetIP()) stopHere("IP FAIL", "No DHCP");
  lcdShowStr("IP:" + ipAddr, "Connecting...");
  delay(500);

  // TCP 서버 연결
  if (!tcpConnect()) stopHere("TCP FAIL", "Check RPI");
  serverConnected = true;
  lcdShowStr("Connected!", "RPI:" + String(RPI_IP));
  delay(1000);
}

// =============================
// loop
// =============================
void loop()
{
  // 연결 끊김 감지 시 재연결
  if (!serverConnected) {
    lcdShow("Reconnecting...", "");
    if (tcpConnect()) {
      serverConnected = true;
      lcdShow("Reconnected!", "");
    } else {
      lcdShow("TCP FAIL", "Retry 5s...");
      delay(5000);
      return;
    }
  }

  // 3초마다 PING 전송
  static unsigned long lastSend = 0;
  if (millis() - lastSend > 3000) {
    pingCount++;
    String msg = "PING " + String(pingCount);

    if (tcpSend(msg)) {
      lcdShowStr("TX: " + msg, "RPI:" + String(RPI_IP));

      // 응답 대기 (2초)
      String reply = tcpReceive(2000);
      if (reply.length() > 0) {
        lcdShowStr("TX: " + msg, "RX: " + reply);
      }
    } else {
      serverConnected = false;
      lcdShow("SEND FAIL", "Reconnecting...");
    }

    lastSend = millis();
  }

  // 서버에서 데이터가 먼저 온 경우 처리
  if (WIFI_SERIAL.available()) {
    String data = tcpReceive(1000);
    if (data.length() > 0) {
      lcdShowStr("RX:", data);
    }
  }
}