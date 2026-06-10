#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include <HTTPClient.h>
#include "esp_camera.h"

#define FLASH_LED 4

// ===== WiFi =====
static const char* WIFI_SSID = "KMS";
static const char* WIFI_PASS = "kimminsoo";

// ⚠️ FastAPI 서버 POST 엔드포인트 (docs ❌)
// static const char* SERVER_URL = "http://10.225.130.99:8000/upload";
static const char* SERVER_URL = "http://10.225.130.99:8000/docs";

// ===== AI Thinker ESP32-CAM 핀맵 =====
#define PWDN_GPIO_NUM     32
#define RESET_GPIO_NUM    -1
#define XCLK_GPIO_NUM      0
#define SIOD_GPIO_NUM     26
#define SIOC_GPIO_NUM     27
#define Y9_GPIO_NUM       35
#define Y8_GPIO_NUM       34
#define Y7_GPIO_NUM       39
#define Y6_GPIO_NUM       36
#define Y5_GPIO_NUM       21
#define Y4_GPIO_NUM       19
#define Y3_GPIO_NUM       18
#define Y2_GPIO_NUM        5
#define VSYNC_GPIO_NUM    25
#define HREF_GPIO_NUM     23
#define PCLK_GPIO_NUM     22

WebServer server(80);
static bool cameraReady = false;
String line;

// 마지막 사진 저장
static uint8_t* g_lastJpg = nullptr;
static size_t   g_lastJpgLen = 0;

// ================= 유틸 =================
static void freeLastJpg() {
  if (g_lastJpg) {
    free(g_lastJpg);
    g_lastJpg = nullptr;
    g_lastJpgLen = 0;
  }
}

static void ackToAtmega(const String& msg) {
  Serial.print("ACK ");
  Serial.println(msg);
}

// ================= Camera =================
static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;
  config.frame_size   = FRAMESIZE_QVGA;
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  return (esp_camera_init(&config) == ESP_OK);
}

// ================= WiFi =================
static bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return false;
}

// ================= 서버 전송 (핵심) =================
static bool sendToServer(const String& weight) {
  if (!g_lastJpg || g_lastJpgLen == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  HTTPClient http;
  WiFiClient client;

  String boundary = "----ESP32Boundary";
  String head =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"weight\"\r\n\r\n" +
    weight + "\r\n"
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"file\"; filename=\"image.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";
  int totalLen = head.length() + g_lastJpgLen + tail.length();

  http.begin(client, SERVER_URL);
  http.addHeader("Content-Type", "multipart/form-data; boundary=" + boundary);
  http.addHeader("Content-Length", String(totalLen));

  int code = http.sendRequest("POST");
  if (code <= 0) {
    http.end();
    return false;
  }

  client.print(head);
  client.write(g_lastJpg, g_lastJpgLen);
  client.print(tail);

  int httpCode = http.GET();
  http.end();

  return (httpCode == 200);
}

// ================= Serial 처리 =================
static String parseWeight(const String& s) {
  int wpos = s.indexOf("W=");
  if (wpos < 0) return "";
  int start = wpos + 2;
  int end = s.indexOf(';', start);
  if (end < 0) end = s.length();
  return s.substring(start, end);
}

static void handleLine(const String& s) {
  String weight = parseWeight(s);
  if (!s.endsWith("PIC")) return;

  digitalWrite(FLASH_LED, HIGH);
  delay(80);
  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED, LOW);

  if (!fb) {
    ackToAtmega("PIC=FAIL");
    return;
  }

  freeLastJpg();
  g_lastJpgLen = fb->len;
  g_lastJpg = (uint8_t*)malloc(fb->len);
  memcpy(g_lastJpg, fb->buf, fb->len);
  esp_camera_fb_return(fb);

  bool ok = sendToServer(weight);
  ackToAtmega(ok ? "PIC=OK SEND=OK" : "PIC=OK SEND=FAIL");
}

// ================= Arduino =================
void setup() {
  pinMode(FLASH_LED, OUTPUT);
  Serial.begin(9600);

  cameraReady = initCamera();
  ensureWifiConnected();

  ackToAtmega("IP=" + WiFi.localIP().toString());
}

void loop() {
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length()) handleLine(line);
      line = "";
    } else if (c != '\r') {
      line += c;
    }
  }
}
