#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"
#include "Cam_Unit.h"
#include "Wifi_Unit.h"
#include "Json_Mini.h"

#define FLASH_LED 4

// WiFi 설정 (사용자 환경 유지)
static const char* WIFI_SSID = "KMS";
static const char* WIFI_PASS = "kimminsoo";
static const char* SERVER_URL = "http://172.20.10.4:5000/api/predict";

// static const char* WIFI_SSID = "LDH";
// static const char* WIFI_PASS = "dlehdgns2001";
// static const char* SERVER_URL = "http://10.196.244.99:5000/api/predict";

static bool g_inProgress = false;
static String g_line = "";
static String g_lastIp = "0.0.0.0";

// ATmega로 전송 (포맷: ACK [메시지])
static void sendAck(const String& msg) {
  Serial.print("ACK ");
  Serial.println(msg);
  Serial.flush();
}

// ATmega LCD에 현재 진행 단계를 표시 (ACK B ST=...)
static void sendProgress(const char* stage) {
  String msg = "B ST=";
  msg += stage;
  msg += " IP=";
  msg += g_lastIp;
  sendAck(msg);
}

// HTTP 응답 읽기
static bool readHttpJsonQuick(WiFiClient& client, String& outBody, uint32_t maxWaitMs) {
  uint32_t t0 = millis();
  while (!client.available() && client.connected() && (millis() - t0 < maxWaitMs)) {
    delay(10);
  }
  String statusLine = client.readStringUntil('\n');
  if (statusLine.indexOf("200") < 0) return false;

  while (client.connected() && (millis() - t0 < maxWaitMs)) {
    String h = client.readStringUntil('\n');
    if (h == "\r" || h.length() == 0) break;
  }

  outBody = "";
  bool started = false;
  while (millis() - t0 < maxWaitMs) {
    while (client.available()) {
      char c = (char)client.read();
      if (!started) {
        if (c == '{') { started = true; outBody += c; }
      } else {
        outBody += c;
        if (c == '}') return true;
      }
    }
    if (!client.connected()) break;
    delay(1);
  }
  return outBody.length() > 0;
}

static void handleLine(String s) {
  if (s.indexOf("PIC") < 0) return;
  if (g_inProgress) { sendAck("B ST=BUSY IP=" + g_lastIp); return; }
  g_inProgress = true;

  if (!WifiUnit_Ensure(WIFI_SSID, WIFI_PASS)) {
    sendAck("F R=WIFI IP=0.0.0.0");
    g_inProgress = false; return;
  }
  g_lastIp = WiFi.localIP().toString();

  // [1단계] 촬영 시작 보고 (LCD에 ST:CAM 표시)
  sendProgress("CAM");

  camera_fb_t* fb = CamUnit_Capture(FLASH_LED);
  delay(1500);
  
  if (!fb) {
    sendAck("F R=CAM IP=" + g_lastIp);
    g_inProgress = false; return;
  }

  // [2단계] 서버 전송 보고 (LCD에 ST:HTTP 표시)
  sendProgress("HTTP");

  // ATmega에서 보낸 무게(W) 파싱
  int wPos = s.indexOf("W=");
  String wVal = (wPos >= 0) ? s.substring(wPos + 2, s.indexOf(';', wPos)) : "0";

  WiFiClient client;
  if (client.connect("172.20.10.4", 5000)) {
    String boundary = "----ESP32";
    String head = "--" + boundary + "\r\nContent-Disposition: form-data; name=\"image\"; filename=\"a.jpg\"\r\nContent-Type: image/jpeg\r\n\r\n";
    String tail = "\r\n--" + boundary + "--\r\n";
    uint32_t len = head.length() + fb->len + tail.length();

    client.print("POST /api/predict HTTP/1.1\r\nHost: 172.20.10.4\r\n");
    client.print("Content-Type: multipart/form-data; boundary=" + boundary + "\r\n");
    client.print("Content-Length: " + String(len) + "\r\nConnection: close\r\n\r\n");
    client.print(head);
    client.write(fb->buf, fb->len);
    client.print(tail);

    delay(500);

    String body;
    if (readHttpJsonQuick(client, body, 40000)) {
      // ✅ ATmega 프로토콜 1단계: 등급 정보 전송
      String grade = JsonPickString(body, "grade");
      String mt = JsonPickString(body, "meat_type");
      int conf = JsonCombinedToPct(JsonPickCombined(body));
      sendAck("OK G=" + grade + " T=" + mt + " C=" + String(conf));
      
      delay(300); // ATmega가 첫 번째 ACK를 처리할 시간 확보

      // ✅ ATmega 프로토콜 2단계: 무게 정보 전송 (이게 와야 화면에 출력됨)
      sendAck("W=" + wVal + " IP=" + g_lastIp);
    } else {
      sendAck("F R=SERVER IP=" + g_lastIp);
    }
  } else {
    sendAck("F R=CONN IP=" + g_lastIp);
  }

  CamUnit_Release(fb);
  Serial.flush();
  g_inProgress = false;
}

void setup() {
  Serial.begin(115200);
  pinMode(FLASH_LED, OUTPUT);
  CamUnit_Begin();
  WifiUnit_Ensure(WIFI_SSID, WIFI_PASS);
  g_lastIp = WiFi.localIP().toString();
  sendAck("OK IP=" + g_lastIp);
}

void loop() {
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      handleLine(g_line);
      g_line = "";
    } else if (c != '\r') {
      g_line += c;
    }
  }
}