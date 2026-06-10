#include "Wizfi360.h"

#include <string.h>

// D0 <- WizFi360-C TX, D1 -> WizFi360-C RX on ATmega328P UART0.
#define WIFI_SERIAL Serial

static const uint32_t WIFI_BAUD = 115200;
static const char* WIFI_SSID = "AIscale";
static const char* WIFI_PASS = "shinhan1234";
static const char* SERVER_IP = "192.168.0.6";
static const uint16_t SERVER_PORT = 5000;

static char g_rx[192];
static char g_pendingReply[64];
static char g_localIp[16] = "0.0.0.0";
static bool g_wifiReady = false;
static bool g_tcpReady = false;

static void lcdShowSafe(LiquidCrystal_I2C& lcd,
                        const __FlashStringHelper* line1,
                        const __FlashStringHelper* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

static void lcdShowSafe(LiquidCrystal_I2C& lcd,
                        const __FlashStringHelper* line1,
                        const char* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

static void lcdShowSafe(LiquidCrystal_I2C& lcd,
                        const char* line1,
                        const __FlashStringHelper* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

static void clearRx()
{
  memset(g_rx, 0, sizeof(g_rx));
}

static void flushSerial()
{
  while (WIFI_SERIAL.available()) {
    WIFI_SERIAL.read();
  }
}

static void sendCmd(const char* cmd)
{
  WIFI_SERIAL.print(cmd);
  WIFI_SERIAL.print("\r\n");
}

static bool extractIpdPayload()
{
  char* ipd = strstr(g_rx, "+IPD");
  if (!ipd) return false;

  char* comma = strchr(ipd, ',');
  char* colon = strchr(ipd, ':');
  if (!comma || !colon || colon <= comma) return false;

  int len = atoi(comma + 1);
  char* data = colon + 1;
  int availableLen = strlen(data);

  if (len > 0 && availableLen >= len) {
    // size_t copyLen = (len < (int)sizeof(g_pendingReply) - 1) ? len : sizeof(g_pendingReply) - 1;
    size_t copyLen;
    if (len < (int)sizeof(g_pendingReply) - 1) {
      copyLen = len;
    } else {
      copyLen = sizeof(g_pendingReply) - 1;
    }
    memcpy(g_pendingReply, data, copyLen);
    g_pendingReply[copyLen] = '\0';
    return true;
  }

  char* newline = strchr(data, '\n');
  if (newline) {
    size_t copyLen = newline - data;
    if (copyLen >= sizeof(g_pendingReply)) copyLen = sizeof(g_pendingReply) - 1;
    memcpy(g_pendingReply, data, copyLen);
    g_pendingReply[copyLen] = '\0';
    return true;
  }

  return false;
}

static bool waitForAny(const char* key1, const char* key2, const char* key3, uint32_t timeoutMs)
{
  clearRx();
  uint16_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();

      if (idx < sizeof(g_rx) - 1) {
        g_rx[idx++] = c;
        g_rx[idx] = '\0';
      }

      if (extractIpdPayload()) return true;
      if (key1 && strstr(g_rx, key1)) return true;
      if (key2 && strstr(g_rx, key2)) return true;
      if (key3 && strstr(g_rx, key3)) return true;
      if (strstr(g_rx, "ERROR")) return false;
      if (strstr(g_rx, "FAIL")) return false;
    }
  }

  return false;
}

static bool waitForIpdPayload(uint32_t timeoutMs)
{
  clearRx();
  uint16_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();

      if (idx < sizeof(g_rx) - 1) {
        g_rx[idx++] = c;
        g_rx[idx] = '\0';
      }

      if (extractIpdPayload()) {
        return true;
      }

      if (strstr(g_rx, "CLOSED")) {
        g_tcpReady = false;
      }
    }
  }

  return false;
}

static bool sendCmdWait(const char* cmd, const char* keyword, uint32_t timeoutMs)
{
  flushSerial();
  sendCmd(cmd);
  return waitForAny(keyword, nullptr, nullptr, timeoutMs);
}

static bool isIpChar(char c)
{
  return (c >= '0' && c <= '9') || c == '.';
}

static bool extractIPv4(const char* text, char* out, size_t outSize)
{
  if (!text || !out || outSize == 0) return false;

  for (size_t i = 0; text[i]; i++) {
    if (text[i] < '0' || text[i] > '9') continue;

    char token[16];
    size_t j = 0;
    size_t k = i;
    int dots = 0;

    while (text[k] && isIpChar(text[k]) && j < sizeof(token) - 1) {
      if (text[k] == '.') dots++;
      token[j++] = text[k++];
    }
    token[j] = '\0';

    if (dots == 3) {
      strncpy(out, token, outSize - 1);
      out[outSize - 1] = '\0';
      return true;
    }
  }

  return false;
}

static bool updateLocalIp()
{
  flushSerial();
  sendCmd("AT+CIFSR");

  clearRx();
  uint16_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < 5000) {
    while (WIFI_SERIAL.available()) {
      char c = (char)WIFI_SERIAL.read();
      if (idx < sizeof(g_rx) - 1) {
        g_rx[idx++] = c;
        g_rx[idx] = '\0';
      }
    }
  }

  return extractIPv4(g_rx, g_localIp, sizeof(g_localIp));
}

static bool connectTcp()
{
  if (!g_wifiReady) return false;

  g_tcpReady = false;
  sendCmdWait("AT+CIPCLOSE", "OK", 1200);
  delay(200);
  flushSerial();

  char cmd[64];
  snprintf(cmd, sizeof(cmd), "AT+CIPSTART=\"TCP\",\"%s\",%u", SERVER_IP, SERVER_PORT);
  sendCmd(cmd);

  g_tcpReady = waitForAny("CONNECT", "ALREADY", nullptr, 10000);
  return g_tcpReady;
}

static void closeTcp()
{
  if (g_tcpReady) {
    sendCmdWait("AT+CIPCLOSE", "OK", 1200);
  }
  g_tcpReady = false;
  flushSerial();
}

static bool readReply(char* reply, size_t replySize, uint32_t timeoutMs)
{
  if (!reply || replySize == 0) return false;
  memset(reply, 0, replySize);

  if (g_pendingReply[0]) {
    strncpy(reply, g_pendingReply, replySize - 1);
    g_pendingReply[0] = '\0';
    return true;
  }

  bool got = waitForIpdPayload(timeoutMs);
  if (!got || !g_pendingReply[0]) return false;

  strncpy(reply, g_pendingReply, replySize - 1);
  g_pendingReply[0] = '\0';
  return true;
}

bool Wizfi360_Begin(LiquidCrystal_I2C& lcd)
{
  g_wifiReady = false;
  g_tcpReady = false;
  g_pendingReply[0] = '\0';

  WIFI_SERIAL.begin(WIFI_BAUD);
  delay(2500);

  lcdShowSafe(lcd, F("WizFi360 AT"), F("Checking..."));
  if (!sendCmdWait("AT", "OK", 2000)) return false;

  lcdShowSafe(lcd, F("WizFi setup"), F("Echo off"));
  if (!sendCmdWait("ATE0", "OK", 2000)) return false;

  lcdShowSafe(lcd, F("WizFi setup"), F("STA mode"));
  if (!sendCmdWait("AT+CWMODE=1", "OK", 3000)) return false;

  lcdShowSafe(lcd, F("WizFi setup"), F("Single conn"));
  if (!sendCmdWait("AT+CIPMUX=0", "OK", 3000)) return false;

  char joinCmd[80];
  snprintf(joinCmd, sizeof(joinCmd), "AT+CWJAP=\"%s\",\"%s\"", WIFI_SSID, WIFI_PASS);

  lcdShowSafe(lcd, F("WiFi join..."), WIFI_SSID);
  flushSerial();
  sendCmd(joinCmd);

  g_wifiReady = waitForAny("WIFI GOT IP", "GOT IP", "OK", 25000);
  if (!g_wifiReady) return false;

  if (g_wifiReady) {
    lcdShowSafe(lcd, F("WiFi IP check"), F("AT+CIFSR"));
    if (!updateLocalIp()) {
      strncpy(g_localIp, "0.0.0.0", sizeof(g_localIp) - 1);
      g_localIp[sizeof(g_localIp) - 1] = '\0';
    }
  }
  return g_wifiReady;
}

bool Wizfi360_IsReady()
{
  return g_wifiReady;
}

void Wizfi360_ShowIp(LiquidCrystal_I2C& lcd)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  // lcd.print(g_wifiReady ? "WIFI AUX OK" : "WIFI AUX FAIL");
  if (g_wifiReady) {
    lcd.print(F("WIFI AUX OK"));
  } else {
    lcd.print(F("WIFI AUX FAIL"));
  }
  lcd.setCursor(0, 1);
  lcd.print(F("IP:"));
  lcd.print(g_localIp);
}

bool Wizfi360_SendRequest(const char* request,
                          LiquidCrystal_I2C& lcd,
                          const char* title,
                          char* reply,
                          size_t replySize,
                          uint32_t timeoutMs)
{
  if (reply && replySize > 0) {
    memset(reply, 0, replySize);
  }
  g_pendingReply[0] = '\0';

  char line[17];
  // snprintf(line, sizeof(line), "%-16.16s", title ? title : "WIFI SEND REQ");
  if (title) {
    snprintf(line, sizeof(line), "%-16.16s", title);
  } else {
    snprintf(line, sizeof(line), "%-16.16s", "WIFI SEND REQ");
  }

  lcdShowSafe(lcd, line, F("Connecting"));
  if (!connectTcp()) {
    lcdShowSafe(lcd, F("WIFI TCP FAIL"), F("IP/port"));
    g_tcpReady = false;
    return false;
  }

  // size_t payloadLen = request ? strlen(request) : 0;
  size_t payloadLen;
  if (request) {
    payloadLen = strlen(request);
  } else {
    payloadLen = 0;
  }
  if (payloadLen == 0) {
    closeTcp();
    return false;
  }

  char sendCmdBuf[24];
  snprintf(sendCmdBuf, sizeof(sendCmdBuf), "AT+CIPSEND=%u", (unsigned)payloadLen);

  flushSerial();
  sendCmd(sendCmdBuf);
  if (!waitForAny(">", nullptr, nullptr, 3000)) {
    lcdShowSafe(lcd, F("WIFI SENDFAIL"), F("No prompt"));
    closeTcp();
    return false;
  }

  lcdShowSafe(lcd, line, F("Waiting RX"));
  WIFI_SERIAL.print(request);

  bool sent = waitForAny("SEND OK", nullptr, nullptr, 5000);
  if (!sent) {
    lcdShowSafe(lcd, F("WIFI SENDFAIL"), F("No SENDOK"));
    closeTcp();
    return false;
  }

  bool gotReply = readReply(reply, replySize, timeoutMs);
  if (!gotReply) {
    lcdShowSafe(lcd, F("WIFI NO REPLY"), F("No ACK"));
    closeTcp();
    return false;
  }

  closeTcp();
  return true;
}
