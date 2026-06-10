#include <Arduino.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

// Wi-Fi AP setting
static const char* WIFI_SSID = "AIscale";
static const char* WIFI_PASS = "shinhan1234";

// Raspberry Pi TCP server setting
static const char* RPI_IP = "192.168.0.100";
static const uint16_t RPI_PORT = 5000;

// WizFi360-C UART. Use Serial1 if the module is wired to UART1.
#define WIFI_SERIAL Serial
static const uint32_t WIFI_BAUD = 115200;

LiquidCrystal_I2C lcd(0x3F, 16, 2);

String rxBuf;
String localIp = "";
uint16_t pingCount = 0;
bool tcpReady = false;
bool replyReceived = false;

void lcdShow(const char* line1, const char* line2 = "")
{
  char buf1[17];
  char buf2[17];
  snprintf(buf1, sizeof(buf1), "%-16.16s", line1);
  snprintf(buf2, sizeof(buf2), "%-16.16s", line2);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(buf1);
  lcd.setCursor(0, 1);
  lcd.print(buf2);
}

void lcdShowStr(const String& line1, const String& line2 = "")
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1.substring(0, 16));
  lcd.setCursor(0, 1);
  lcd.print(line2.substring(0, 16));
}

void flushWifiSerial()
{
  while (WIFI_SERIAL.available()) {
    WIFI_SERIAL.read();
  }
}

void sendAt(const String& cmd)
{
  WIFI_SERIAL.print(cmd);
  WIFI_SERIAL.print("\r\n");
}

bool waitForAny(const char* key1, const char* key2, const char* key3, uint32_t timeoutMs)
{
  rxBuf = "";
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      rxBuf += c;

      if (key1 && rxBuf.indexOf(key1) >= 0) return true;
      if (key2 && rxBuf.indexOf(key2) >= 0) return true;
      if (key3 && rxBuf.indexOf(key3) >= 0) return true;
      if (rxBuf.indexOf("ERROR") >= 0) return false;
      if (rxBuf.indexOf("FAIL") >= 0) return false;
    }
  }

  return false;
}

bool sendAtWait(const String& cmd, const char* okKeyword, uint32_t timeoutMs)
{
  flushWifiSerial();
  sendAt(cmd);
  return waitForAny(okKeyword, NULL, NULL, timeoutMs);
}

String extractIPv4(const String& text)
{
  for (int i = 0; i < (int)text.length(); i++) {
    if (!isDigit((unsigned char)text[i])) continue;

    int start = i;
    int dotCount = 0;

    while (i < (int)text.length()) {
      char c = text[i];
      if (isDigit((unsigned char)c)) {
        i++;
      } else if (c == '.') {
        dotCount++;
        i++;
      } else {
        break;
      }
    }

    String token = text.substring(start, i);
    if (dotCount != 3) continue;

    bool valid = true;
    int parts = 0;
    int partStart = 0;

    for (int j = 0; j <= (int)token.length(); j++) {
      if (j == (int)token.length() || token[j] == '.') {
        String part = token.substring(partStart, j);
        if (part.length() == 0 || part.length() > 3) {
          valid = false;
          break;
        }

        int value = part.toInt();
        if (value < 0 || value > 255) {
          valid = false;
          break;
        }

        parts++;
        partStart = j + 1;
      }
    }

    if (valid && parts == 4) return token;
  }

  return "";
}

String extractIpdPayload()
{
  int ipd = rxBuf.indexOf("+IPD");
  if (ipd < 0) return "";

  int comma = rxBuf.indexOf(',', ipd);
  int colon = rxBuf.indexOf(':', ipd);
  if (comma < 0 || colon < 0 || colon <= comma) return "";

  int len = rxBuf.substring(comma + 1, colon).toInt();
  int dataStart = colon + 1;
  int availableLen = rxBuf.length() - dataStart;

  if (len > 0 && availableLen >= len) {
    String data = rxBuf.substring(dataStart, dataStart + len);
    data.trim();
    rxBuf = "";
    return data;
  }

  int newline = rxBuf.indexOf('\n', dataStart);
  if (newline >= 0) {
    String data = rxBuf.substring(dataStart, newline);
    data.trim();
    rxBuf = "";
    return data;
  }

  return "";
}

void stopHere(const char* line1, const char* line2 = "")
{
  lcdShow(line1, line2);
  while (true) {
    delay(100);
  }
}

bool wifiAtCheck()
{
  lcdShow("WizFi AT...", "");
  return sendAtWait("AT", "OK", 2000);
}

bool wifiPrepare()
{
  lcdShow("WizFi setup...", "Echo off");
  if (!sendAtWait("ATE0", "OK", 2000)) return false;

  lcdShow("WizFi setup...", "STA mode");
  if (!sendAtWait("AT+CWMODE=1", "OK", 3000)) return false;

  lcdShow("WizFi setup...", "Single conn");
  if (!sendAtWait("AT+CIPMUX=0", "OK", 3000)) return false;

  return true;
}

bool wifiJoinAp()
{
  lcdShow("WiFi join...", WIFI_SSID);

  String cmd = String("AT+CWJAP=\"") + WIFI_SSID + "\",\"" + WIFI_PASS + "\"";
  flushWifiSerial();
  sendAt(cmd);

  return waitForAny("WIFI GOT IP", "GOT IP", "OK", 25000);
}

bool wifiGetIp()
{
  lcdShow("DHCP check...", "AT+CIFSR");

  flushWifiSerial();
  sendAt("AT+CIFSR");

  rxBuf = "";
  uint32_t start = millis();
  while (millis() - start < 5000) {
    while (WIFI_SERIAL.available()) {
      rxBuf += (char)WIFI_SERIAL.read();
    }
  }

  localIp = extractIPv4(rxBuf);
  return localIp.length() > 0;
}

bool tcpConnect()
{
  lcdShow("TCP connect...", RPI_IP);

  sendAtWait("AT+CIPCLOSE", "OK", 1500);

  String cmd = String("AT+CIPSTART=\"TCP\",\"") + RPI_IP + "\"," + String(RPI_PORT);
  flushWifiSerial();
  sendAt(cmd);

  return waitForAny("CONNECT", "ALREADY", "OK", 12000);
}

String tcpRead(uint32_t timeoutMs)
{
  String cached = extractIpdPayload();
  if (cached.length() > 0) return cached;

  rxBuf = "";
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      rxBuf += (char)WIFI_SERIAL.read();

      String data = extractIpdPayload();
      if (data.length() > 0) return data;
    }
  }

  return "";
}

bool tcpSendLine(const String& data)
{
  String payload = data + "\n";
  String cmd = "AT+CIPSEND=" + String(payload.length());

  flushWifiSerial();
  sendAt(cmd);
  if (!waitForAny(">", NULL, NULL, 3000)) return false;

  WIFI_SERIAL.print(payload);
  return waitForAny("SEND OK", "OK", NULL, 5000);
}

void showSuccess()
{
  replyReceived = true;
  lcdShow("Raspberry hello", "Success");
}

bool checkReply(uint32_t timeoutMs)
{
  String reply = tcpRead(timeoutMs);
  if (reply.length() == 0) return false;

  showSuccess();
  return true;
}

void setup()
{
  lcd.init();
  lcd.backlight();
  lcdShow("Booting...", "WizFi360-C");
  delay(1000);

  WIFI_SERIAL.begin(WIFI_BAUD);
  delay(3000);

  if (!wifiAtCheck()) stopHere("AT FAIL", "Check TX/RX");
  lcdShow("AT OK", "");
  delay(700);

  if (!wifiPrepare()) stopHere("SETUP FAIL", "Check module");
  lcdShow("SETUP OK", "");
  delay(700);

  if (!wifiJoinAp()) stopHere("WIFI FAIL", "SSID/PASS");
  lcdShow("WIFI OK", "AIscale");
  delay(700);

  if (!wifiGetIp()) stopHere("IP FAIL", "No DHCP");
  lcdShowStr("IP:" + localIp, "TCP next...");
  delay(1200);

  if (!tcpConnect()) stopHere("TCP FAIL", "Check RPI");
  tcpReady = true;
  lcdShow("TCP OK", "Send HELLO");
  delay(700);

  if (tcpSendLine("HELLO FROM ATMEGA128A")) {
    if (!checkReply(8000)) {
      lcdShow("HELLO SENT", "No reply");
    }
  } else {
    lcdShow("HELLO FAIL", "Send error");
  }

  delay(1500);
}

void loop()
{
  static uint32_t lastSendMs = 0;

  if (replyReceived) {
    return;
  }

  // Catch a delayed Raspberry Pi reply before the next send clears UART input.
  if (checkReply(100)) {
    return;
  }

  if (millis() - lastSendMs < 3000) {
    return;
  }
  lastSendMs = millis();

  if (!tcpReady) {
    lcdShow("TCP retry...", RPI_IP);
    tcpReady = tcpConnect();
    if (!tcpReady) {
      lcdShow("TCP FAIL", "Retry 3s");
      return;
    }
  }

  pingCount++;
  String msg = "PING " + String(pingCount) + " FROM ATMEGA";

  if (!tcpSendLine(msg)) {
    tcpReady = false;
    lcdShow("SEND FAIL", "Reconnect");
    return;
  }

  lcdShowStr("TX:" + String(pingCount), "Wait reply...");

  if (!checkReply(10000)) {
    lcdShowStr("TX:" + String(pingCount), "No reply");
  }
}
