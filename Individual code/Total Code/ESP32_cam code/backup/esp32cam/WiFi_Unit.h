#ifndef WIFI_UNIT_H
#define WIFI_UNIT_H

#include <Arduino.h>
#include <WiFi.h>

// 와이파이 연결 보장(타임아웃 내 연결)
bool WifiUnit_Ensure(const char* ssid, const char* pass, uint32_t timeoutMs = 10000);
// IP 문자열
String WifiUnit_IpStr();

// ===== TCP 브리지 =====
void WifiUnit_BeginBridge(const char* ssid, const char* pass, const char* serverIp,uint16_t serverPort);

void WifiUnit_LoopBridge();

// ATmega에서 받은 1줄을 서버로 전송
bool WifiUnit_SendLine(const String& line);

// 서버에서 온 1줄을 읽음 (있으면 true)
bool WifiUnit_ReadLine(String& outLine);

// 서버 연결 상태
bool WifiUnit_ServerConnected();

#endif 