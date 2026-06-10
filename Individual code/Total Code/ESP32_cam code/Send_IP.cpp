#include <Arduino.h>
#include <WiFi.h>
#include <WebServer.h>
#include "esp_camera.h"

#define FLASH_LED 4

static const char* WIFI_SSID = "KMS";
static const char* WIFI_PASS = "kimminsoo";

// ====== AI Thinker ESP32-CAM 핀맵 ======
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

static bool cameraReady = false;
String line;

WebServer server(80);

// 마지막 촬영 JPEG 보관(1장)
static uint8_t* g_lastJpg = nullptr;
static size_t   g_lastJpgLen = 0;

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

static void handleRoot() {
  String html =
    "<html><body>"
    "<h3>ESP32-CAM</h3>"
    "<p><a href=\"/photo\">View last photo</a></p>"
    "<p><a href=\"/shot\">Take photo now</a> (manual test)</p>"
    "</body></html>";
  server.send(200, "text/html", html);
}

// 마지막 사진을 브라우저로 반환
static void handlePhoto() {
  if (!g_lastJpg || g_lastJpgLen == 0) {
    server.send(404, "text/plain", "No photo yet. Press SEND on ATmega.");
    return;
  }
  server.sendHeader("Cache-Control", "no-store");
  server.send_P(200, "image/jpeg", (const char*)g_lastJpg, g_lastJpgLen);
}

// 브라우저에서 강제로 한 번 찍어보는 테스트용
static void handleShot() {
  // 촬영 후 g_lastJpg 갱신
  if (!cameraReady) {
    server.send(500, "text/plain", "Camera not ready");
    return;
  }

  digitalWrite(FLASH_LED, HIGH);
  delay(80);
  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED, LOW);

  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    server.send(500, "text/plain", "Capture failed");
    return;
  }

  freeLastJpg();
  g_lastJpgLen = fb->len;
  g_lastJpg = (uint8_t*)malloc(g_lastJpgLen);
  if (!g_lastJpg) {
    esp_camera_fb_return(fb);
    server.send(500, "text/plain", "Malloc failed");
    return;
  }
  memcpy(g_lastJpg, fb->buf, g_lastJpgLen);
  esp_camera_fb_return(fb);

  server.sendHeader("Location", "/photo");
  server.send(302, "text/plain", "OK");
}

static bool initCamera() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer   = LEDC_TIMER_0;
  config.pin_d0       = Y2_GPIO_NUM;
  config.pin_d1       = Y3_GPIO_NUM;
  config.pin_d2       = Y4_GPIO_NUM;
  config.pin_d3       = Y5_GPIO_NUM;
  config.pin_d4       = Y6_GPIO_NUM;
  config.pin_d5       = Y7_GPIO_NUM;
  config.pin_d6       = Y8_GPIO_NUM;
  config.pin_d7       = Y9_GPIO_NUM;
  config.pin_xclk     = XCLK_GPIO_NUM;
  config.pin_pclk     = PCLK_GPIO_NUM;
  config.pin_vsync    = VSYNC_GPIO_NUM;
  config.pin_href     = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn     = PWDN_GPIO_NUM;
  config.pin_reset    = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // 용량 줄여서 메모리 안정 (처음엔 QVGA 추천!)
  config.frame_size   = FRAMESIZE_QVGA; // VGA면 메모리 부족 위험↑
  config.jpeg_quality = 12;
  config.fb_count     = 1;

  return (esp_camera_init(&config) == ESP_OK);
}

static bool ensureWifiConnected() {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);
  uint32_t t0 = millis();
  while (millis() - t0 < 10000) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return false;
}

// "W=123.4;PIC"에서 weight만 추출
static String parseWeight(const String& s) {
  int wpos = s.indexOf("W=");
  if (wpos < 0) return "";
  int start = wpos + 2;
  int end = s.indexOf(';', start);
  if (end < 0) end = s.length();
  String w = s.substring(start, end);
  w.trim();
  return w;
}

static bool hasPIC(const String& s) {
  return (s.indexOf("PIC") >= 0);
}

static void handleLine(const String& s) {
  String weight = parseWeight(s);
  bool pic = hasPIC(s);

  if (!pic) {
    ackToAtmega(s);
    return;
  }

  if (!cameraReady) {
    ackToAtmega("PIC=FAIL CAM_NOT_READY");
    return;
  }

  // 촬영
  digitalWrite(FLASH_LED, HIGH);
  delay(80);
  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED, LOW);

  if (!fb || fb->format != PIXFORMAT_JPEG) {
    if (fb) esp_camera_fb_return(fb);
    ackToAtmega("PIC=FAIL FB_NULL");
    return;
  }

  // 마지막 사진 저장
  freeLastJpg();
  g_lastJpgLen = fb->len;
  g_lastJpg = (uint8_t*)malloc(g_lastJpgLen);
  if (!g_lastJpg) {
    esp_camera_fb_return(fb);
    ackToAtmega("PIC=FAIL MALLOC");
    return;
  }
  memcpy(g_lastJpg, fb->buf, g_lastJpgLen);
  esp_camera_fb_return(fb);

  // 성공 ACK + (ESP32 IP도 같이 알려주면 편함)
  ackToAtmega(String("W=") + weight + " PIC=OK IP=" + WiFi.localIP().toString());
}

void setup() {
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  Serial.begin(9600);
  delay(200);

  cameraReady = initCamera();
  bool wifiOk = ensureWifiConnected();

  // 웹서버 라우팅
  server.on("/", handleRoot);
  server.on("/photo", handlePhoto);
  server.on("/shot", handleShot);
  server.begin();

  // ackToAtmega(String("BOOT CAM=") + (cameraReady ? "OK" : "FAIL") +
  //             " WIFI=" + (wifiOk ? "OK" : "FAIL") +
  //             " IP=" + WiFi.localIP().toString());
  ackToAtmega(String("IP=") + WiFi.localIP().toString());
}

void loop() {
  server.handleClient(); // 웹 요청 처리

  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      line.trim();
      if (line.length()) handleLine(line);
      line = "";
    } else if (c != '\r') {
      line += c;
      if (line.length() > 120) line = "";
    }
  }
}
