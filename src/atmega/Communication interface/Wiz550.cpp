#include "Wiz550.h"

#include <SPI.h>
#include <Ethernet3.h>
#include <string.h>

#include "PinMap.h"

static const uint8_t ETH_CS_PIN = PIN_WIZ_CS;
static const uint8_t ETH_RST_PIN = PIN_WIZ_RST;

static byte g_mac[] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x58 };

static IPAddress g_localIp(192, 168, 0, 120);
static IPAddress g_subnet(255, 255, 255, 0);
static IPAddress g_gateway(192, 168, 0, 1);
static IPAddress g_dns(8, 8, 8, 8);

static IPAddress g_serverIp(192, 168, 0, 6);
static const uint16_t g_serverPort = 5000;

static EthernetClient g_client;
static bool g_ready = false;

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
                        const char* line1,
                        const __FlashStringHelper* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}

static bool readReply(char* out, size_t outSize, uint32_t timeoutMs)
{
  if (!out || outSize == 0) return false;

  memset(out, 0, outSize);
  size_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (g_client.available()) {
      char c = (char)g_client.read();

      if (idx < outSize - 1) {
        out[idx++] = c;
      }

      if (c == '\n') {
        out[idx] = '\0';
        return true;
      }
    }

    if (!g_client.connected() && !g_client.available()) {
      break;
    }

    delay(10);
  }

  out[idx] = '\0';
  return idx > 0;
}

static bool connectServer()
{
  if (Ethernet.link() == 0) {
    g_client.stop();
    g_ready = false;
    return false;
  }

  if (!g_ready) {
    g_ready = Wiz550_Begin();
    if (!g_ready) return false;
  }

  if (g_client.connected()) {
    return true;
  }

  g_client.stop();
  delay(500);

  for (uint8_t attempt = 0; attempt < 2; attempt++) {
    if (Ethernet.link() == 0) {
      g_client.stop();
      g_ready = false;
      return false;
    }

    if (g_client.connect(g_serverIp, g_serverPort) > 0) {
      delay(300);
      return true;
    }

    g_client.stop();
    delay(1000);
  }

  return false;
}

bool Wiz550_Begin()
{
  pinMode(ETH_RST_PIN, OUTPUT);
  digitalWrite(ETH_RST_PIN, LOW);
  delay(20);
  digitalWrite(ETH_RST_PIN, HIGH);
  delay(150);

  pinMode(ETH_CS_PIN, OUTPUT);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.begin();

  Ethernet.setCsPin(ETH_CS_PIN);
  Ethernet.begin(g_mac, g_localIp, g_subnet, g_gateway, g_dns);
  delay(1500);

  Ethernet.setRtTimeOut(3000);
  Ethernet.setRtCount(4);

  g_client.stop();
  uint32_t start = millis();
  while (Ethernet.link() == 0 && millis() - start < 3000) {
    delay(100);
  }
  g_ready = (Ethernet.link() != 0);
  return g_ready;
}

bool Wiz550_IsReady()
{
  return g_ready && (Ethernet.link() != 0);
}

void Wiz550_ShowIp(LiquidCrystal_I2C& lcd)
{
  lcd.clear();
  lcd.setCursor(0, 0);
  if (Wiz550_IsReady()) {
    lcd.print(F("W5500 LINK OK"));
  } else {
    lcd.print(F("W5500 INIT FAIL"));
  }
  lcd.setCursor(0, 1);
  if (Ethernet.link() == 0) {
    lcd.print(F("NO LINK/CABLE"));
  } else {
    lcd.print(F("IP:"));
    lcd.print(Ethernet.localIP());
  }
}

bool Wiz550_SendRequest(const char* request,
                        LiquidCrystal_I2C& lcd,
                        const char* title,
                        char* reply,
                        size_t replySize,
                        uint32_t timeoutMs)
{
  if (reply && replySize > 0) {
    memset(reply, 0, replySize);
  }

  // lcdShowSafe(lcd, title ? title : "SEND REQ", "Connecting...");
  if (title) {
    lcdShowSafe(lcd, title, F("Connecting"));
  } else {
    lcdShowSafe(lcd, F("SEND REQ"), F("Connecting"));
  }

  if (!connectServer()) {
    if (Ethernet.link() == 0) {
      lcdShowSafe(lcd, F("W5500 NO LINK"), F("Cable"));
    } else {
      lcdShowSafe(lcd, F("W5500 TCP FAIL"), F("IP/port"));
    }
    g_client.stop();
    return false;
  }

  // lcdShowSafe(lcd, title ? title : "SEND REQ", "Waiting RX...");
  if (title) {
    lcdShowSafe(lcd, title, F("Waiting RX"));
  } else {
    lcdShowSafe(lcd, F("SEND REQ"), F("Waiting RX"));
  }

  if (request && request[0]) {
    size_t sent = g_client.print(request);
    if (sent == 0 && !g_client.connected()) {
      lcdShowSafe(lcd, F("W5500 SENDFAIL"), F("TCP closed"));
      g_client.stop();
      return false;
    }
  }

  bool gotReply = readReply(reply, replySize, timeoutMs);
  if (!gotReply) {
    lcdShowSafe(lcd, F("W5500 NO REPLY"), F("No ACK"));
    g_client.stop();
    return false;
  }

  return (reply && reply[0] != '\0');
}
