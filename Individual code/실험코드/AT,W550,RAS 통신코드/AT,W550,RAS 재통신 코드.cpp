#include <Arduino.h>
#include <SPI.h>
#include <Ethernet3.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
#include <string.h>

#ifndef PIN_PB0
#define PIN_PB0 8
#endif

LiquidCrystal_I2C lcd(0x3F, 16, 2);

// WIZ550io wiring on ATmega128A:
// MOSI -> PB2, MISO -> PB3, SCLK -> PB1, SCSn -> PB0.
// SPI.begin() uses PB1/PB2/PB3 automatically; only CS is controlled here.
static const uint8_t ETH_CS_PIN = PIN_PB0;

// Network setting
byte mac[] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x58 };

IPAddress localIp(192, 168, 0, 120);
IPAddress subnet(255, 255, 255, 0);
IPAddress gateway(192, 168, 0, 1);
IPAddress myDns(8, 8, 8, 8);

IPAddress serverIp(192, 168, 0, 100);
const uint16_t serverPort = 5000;

const uint32_t SEND_INTERVAL_MS = 3000;
const uint32_t REPLY_TIMEOUT_MS = 5000;

EthernetClient client;
uint16_t txCount = 0;
uint32_t lastSendMs = 0;

void lcdShowSafe(const char* line1, const char* line2 = "")
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

bool ensureTcpConnected()
{
  if (client.connected()) {
    return true;
  }

  client.stop();

  if (Ethernet.link() == 0) {
    lcdShowSafe("LINK FAIL", "Check cable");
    delay(1000);
    return false;
  }

  lcdShowSafe("TCP Connect...", "Raspberry Pi");
  int rc = client.connect(serverIp, serverPort);

  if (rc <= 0) {
    lcdShowSafe("TCP FAIL", "Retry soon");
    client.stop();
    delay(1000);
    return false;
  }

  lcdShowSafe("TCP Connected", "Ready");
  delay(500);
  return true;
}

bool readReply(char* out, size_t outSize, uint32_t timeoutMs)
{
  if (outSize == 0) {
    return false;
  }

  size_t idx = 0;
  uint32_t start = millis();

  while (millis() - start < timeoutMs) {
    while (client.available()) {
      char c = (char)client.read();

      if (idx < outSize - 1) {
        out[idx++] = c;
      }

      if (c == '\n') {
        out[idx] = '\0';
        return true;
      }
    }

    if (!client.connected() && !client.available()) {
      break;
    }

    delay(10);
  }

  out[idx] = '\0';
  return idx > 0;
}

bool sendAndReceive()
{
  txCount++;
  String msg = "PING " + String(txCount) + " FROM ATMEGA";

  client.println(msg);
  lcdShowStr("TX:" + String(txCount), "Wait reply...");

  char rxBuf[64];
  bool gotReply = readReply(rxBuf, sizeof(rxBuf), REPLY_TIMEOUT_MS);

  if (!gotReply) {
    lcdShowStr("TX:" + String(txCount), "No reply");
    return false;
  }

  if (strstr(rxBuf, "ACK") != NULL || strstr(rxBuf, "PING") != NULL || strlen(rxBuf) > 0) {
    lcdShowSafe("Raspberry hello", "Success");
  } else {
    lcdShowSafe("RX DATA", rxBuf);
  }

  return true;
}

void setup()
{
  lcd.init();
  lcd.backlight();

  Serial.begin(115200);
  delay(1000);

  lcdShowSafe("Booting...", "WIZ550io");
  delay(1000);

  pinMode(ETH_CS_PIN, OUTPUT);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.begin();

  Ethernet.setCsPin(ETH_CS_PIN);
  Ethernet.begin(mac, localIp, subnet, gateway, myDns);
  delay(1500);

  lcdShowStr("IP:192.168.0.120", "Start TCP...");
  delay(1000);
}

void loop()
{
  if (millis() - lastSendMs < SEND_INTERVAL_MS) {
    return;
  }
  lastSendMs = millis();

  if (!ensureTcpConnected()) {
    return;
  }

  if (!sendAndReceive()) {
    client.stop();
  }
}
