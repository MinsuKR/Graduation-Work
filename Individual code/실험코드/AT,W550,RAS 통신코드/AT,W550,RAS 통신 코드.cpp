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

EthernetClient client;

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

void stopHere(const char* line1, const char* line2 = "")
{
  lcdShowSafe(line1, line2);
  while (1) {
    delay(100);
  }
}

void setup()
{
  lcd.init();
  lcd.backlight();

  Serial.begin(115200);
  delay(1000);

  lcdShowSafe("Booting...", "");
  delay(1000);

  pinMode(ETH_CS_PIN, OUTPUT);
  digitalWrite(ETH_CS_PIN, HIGH);
  SPI.begin();

  Ethernet.setCsPin(ETH_CS_PIN);
  Ethernet.begin(mac, localIp, subnet, gateway, myDns);
  delay(1500);

  if (Ethernet.link() == 0) {
    stopHere("LINK FAIL", "Check cable");
  }

  lcdShowSafe("TCP Connect...", "");
  client.stop();
  delay(5000);

  int rc = client.connect(serverIp, serverPort);

  if (rc <= 0) {
    stopHere("TCP FAIL", "connect rc<=0");
  }

  lcdShowSafe("TCP Connected", "");
  delay(5000);

  client.println("HELLO");
  lcdShowSafe("HELLO SENT", "");
  delay(5000);

  lcdShowSafe("Wait ACK...", "");

  char rxBuf[64];
  uint8_t idx = 0;
  unsigned long start = millis();

  while (millis() - start < 8000) {
    while (client.available()) {
      char c = client.read();

      if (idx < sizeof(rxBuf) - 1) {
        rxBuf[idx++] = c;
      }

      if (c == '\n') {
        break;
      }
    }

    if (idx > 0 && rxBuf[idx - 1] == '\n') {
      break;
    }

    delay(10);
  }

  rxBuf[idx] = '\0';

  if (idx == 0) {
    lcdShowSafe("NO REPLY", "Sent HELLO");
  } else if (strstr(rxBuf, "ACK") != NULL) {
    lcdShowSafe("ACK OK", rxBuf);
  } else {
    lcdShowSafe("RX DATA", rxBuf);
  }

  client.stop();
}

void loop()
{
}
