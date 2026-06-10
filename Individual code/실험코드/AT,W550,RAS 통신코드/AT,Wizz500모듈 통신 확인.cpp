#include <Arduino.h>
#include <SPI.h>
#include <Ethernet.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x3F, 16, 2);

static const uint8_t ETH_CS_PIN = SS;

byte mac[] = { 0x02, 0x11, 0x22, 0x33, 0x44, 0x55 };

IPAddress localIp(192, 168, 0, 50);
IPAddress dns(8, 8, 8, 8);
IPAddress gateway(192, 168, 0, 1);
IPAddress subnet(255, 255, 255, 0);

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

void ipToChar(IPAddress ip, char* out, size_t outSize)
{
  snprintf(out, outSize, "%u.%u.%u.%u", ip[0], ip[1], ip[2], ip[3]);
}

void stopHere(const char* line1, const char* line2 = "")
{
  lcdShowSafe(line1, line2);
  while (1) delay(100);
}

void setup()
{
  char ipBuf[17];

  lcd.init();
  lcd.backlight();

  lcdShowSafe("Booting...", "");
  delay(1000);

  pinMode(SS, OUTPUT);
  digitalWrite(SS, HIGH);
  SPI.begin();

  lcdShowSafe("SPI Ready", "");
  delay(500);

  Ethernet.init(ETH_CS_PIN);

  ipToChar(localIp, ipBuf, sizeof(ipBuf));
  lcdShowSafe("Set Static IP", ipBuf);
  delay(500);

  Ethernet.begin(mac, localIp, dns, gateway, subnet);
  delay(1200);

  // 링크는 참고만
  EthernetLinkStatus st = Ethernet.linkStatus();
  if (st == LinkON) {
    lcdShowSafe("LAN Connected", "");
    delay(700);
  } else {
    lcdShowSafe("Link Unknown", "Try ping test");
    delay(1200);
  }

  lcdShowSafe("IP READY", ipBuf);
}

void loop()
{
}