/*
  ESP32-CAM <-> ATmega UART + Flask /api/predict 업로드 (개선판)
  ------------------------------------------------------------
  ✅ ATmega에서 "W=123.4;T=beef;PIC" 같은 라인 받으면 동작
  ✅ 즉시 "ACK PIC=BUSY" 먼저 보내서 ATmega의 RX waiting 방지
  ✅ malloc 복사 제거: fb->buf 그대로 업로드 (안정성↑, OOM↓)
  ✅ 응답 body readString()로 멈추는 문제 해결: 제한 읽기(readBodyLimited)
  ✅ 응답에서 JSON만 분리({부터)해서 grade/meat_type/success 파싱
  ✅ 카메라 VGA는 불안정할 수 있어 기본 QVGA 권장(필요시 VGA로 변경 가능)

  Serial(ATmega UART): 9600
*/

#include <Arduino.h>
#include <WiFi.h>
#include "esp_camera.h"

// ================== 설정 ==================
#define FLASH_LED 4

// WiFi
static const char* WIFI_SSID = "LDH";
static const char* WIFI_PASS = "dlehdgns2001";

// Flask 서버 (상대방 코드: /api/predict)
static const char* SERVER_URL = "http://10.196.244.99:5000/api/predict";

// multipart 필드명 (서버 코드에 고정)
static const char* FIELD_FILE_IMAGE = "image";      // request.files['image']
static const char* FIELD_MEAT_TYPE  = "meat_type";  // request.form.get('meat_type')

// 응답 읽기 제한(멈춤 방지)
static const uint32_t RESP_MAX_MS    = 8000;   // 마지막 데이터 이후 최대 8초 기다림
static const size_t   RESP_MAX_BYTES = 1400;   // 바디는 앞부분 최대 1400바이트만 저장

// 서버 예측이 느리면 status line 대기 시간을 늘리세요(YOLO/U-Net이면 길 수 있음)
static const uint32_t STATUS_WAIT_MS = 120000; // 120초

// AI Thinker ESP32-CAM 핀맵
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

// ================== 전역 상태 ==================
static bool cameraReady = false;
static String g_line;
static bool g_busy = false;

// ================== 유틸 ==================
static void ackToAtmega(const String& msg) {
  Serial.print("ACK ");
  Serial.println(msg);
}

// ================== Camera ==================
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

  config.pin_xclk  = XCLK_GPIO_NUM;
  config.pin_pclk  = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href  = HREF_GPIO_NUM;
  config.pin_sccb_sda = SIOD_GPIO_NUM;
  config.pin_sccb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn  = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;

  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // ⚠️ 안정화 추천: QVGA (VGA는 전원/메모리/카메라 품질에 따라 불안정해질 수 있음)
  config.frame_size   = FRAMESIZE_QVGA;   // 필요하면 FRAMESIZE_VGA로 변경
  config.jpeg_quality = 12;               // 10~15 권장
  config.fb_count     = 1;

  esp_err_t err = esp_camera_init(&config);
  return (err == ESP_OK);
}

// ================== WiFi ==================
static bool ensureWifiConnected(uint32_t timeoutMs = 12000) {
  if (WiFi.status() == WL_CONNECTED) return true;

  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASS);

  uint32_t t0 = millis();
  while (millis() - t0 < timeoutMs) {
    if (WiFi.status() == WL_CONNECTED) return true;
    delay(200);
  }
  return false;
}

// ================== URL 파싱 ==================
static bool parseHttpUrl(const char* url,
                         String& outHost, uint16_t& outPort, String& outPath) {
  String u(url);
  if (!u.startsWith("http://")) return false;
  u.remove(0, 7); // remove "http://"

  int slash = u.indexOf('/');
  String hostport = (slash >= 0) ? u.substring(0, slash) : u;
  outPath         = (slash >= 0) ? u.substring(slash) : "/";

  outHost = hostport;
  outPort = 80;

  int colon = hostport.indexOf(':');
  if (colon >= 0) {
    outHost = hostport.substring(0, colon);
    String portStr = hostport.substring(colon + 1);
    uint32_t p = (uint32_t)portStr.toInt();
    if (p == 0 || p > 65535) return false;
    outPort = (uint16_t)p;
  }
  return true;
}

// ================== 응답 바디 제한 읽기(멈춤 방지) ==================
static String readBodyLimited(WiFiClient& client, uint32_t maxMs, size_t maxBytes) {
  String body;
  uint32_t lastDataMs = millis();

  while ((millis() - lastDataMs) < maxMs) {
    while (client.available()) {
      char c = (char)client.read();
      if (body.length() < (int)maxBytes) body += c;
      lastDataMs = millis();
    }
    if (!client.connected()) break;
    delay(5);
  }
  return body;
}

// 바디에서 JSON만 분리: '{'부터 잘라서 반환
static String extractJson(const String& s) {
  int j = s.indexOf('{');
  if (j < 0) return "";
  return s.substring(j);
}

// ================== 간단 JSON 파싱 ==================
static String jsonPickString(const String& json, const char* key) {
  String k = String("\"") + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return "";
  p += k.length();
  while (p < (int)json.length() && (json[p] == ' ')) p++;
  if (p >= (int)json.length() || json[p] != '\"') return "";
  p++;
  int e = json.indexOf('\"', p);
  if (e < 0) return "";
  return json.substring(p, e);
}

static bool jsonPickBoolean(const String& json, const char* key, bool& out) {
  String k = String("\"") + key + "\":";
  int p = json.indexOf(k);
  if (p < 0) return false;
  p += k.length();
  while (p < (int)json.length() && (json[p] == ' ')) p++;

  if (json.startsWith("true", p))  { out = true;  return true; }
  if (json.startsWith("false", p)) { out = false; return true; }
  return false;
}

// ================== 서버 전송 (multipart/form-data) ==================
static bool sendToServerMultipart(const String& meatType,
                                  const uint8_t* jpg, size_t jpgLen,
                                  String& outStatusLine, String& outBody) {
  outStatusLine = "";
  outBody = "";

  if (!jpg || jpgLen == 0) return false;
  if (WiFi.status() != WL_CONNECTED) return false;

  String host, path;
  uint16_t port = 80;
  if (!parseHttpUrl(SERVER_URL, host, port, path)) return false;

  WiFiClient client;
  client.setTimeout(120000); // 내부 readStringUntil용
  if (!client.connect(host.c_str(), port)) return false;

  String boundary = "----ESP32Boundary7MA4YWxkTrZu0gW";

  // (선택) meat_type 파트: beef/pork만 전송, 아니면 안 보냄(=auto)
  String mt = meatType;
  mt.toLowerCase();

  String meatPart = "";
  if (mt == "beef" || mt == "pork") {
    meatPart =
      "--" + boundary + "\r\n"
      "Content-Disposition: form-data; name=\"" + String(FIELD_MEAT_TYPE) + "\"\r\n\r\n" +
      mt + "\r\n";
  }

  // 파일 파트 (필수)
  String fileHead =
    "--" + boundary + "\r\n"
    "Content-Disposition: form-data; name=\"" + String(FIELD_FILE_IMAGE) + "\"; filename=\"image.jpg\"\r\n"
    "Content-Type: image/jpeg\r\n\r\n";

  String tail = "\r\n--" + boundary + "--\r\n";

  size_t contentLen = meatPart.length() + fileHead.length() + jpgLen + tail.length();

  // ---- HTTP Header ----
  client.print(String("POST ") + path + " HTTP/1.1\r\n");
  client.print(String("Host: ") + host + ":" + port + "\r\n");
  client.print("Connection: close\r\n");
  client.print(String("Content-Type: multipart/form-data; boundary=") + boundary + "\r\n");
  client.print(String("Content-Length: ") + contentLen + "\r\n\r\n");

  // ---- Body ----
  if (meatPart.length()) client.print(meatPart);

  client.print(fileHead);
  client.write(jpg, jpgLen);
  client.print(tail);

  // ---- status line 대기 (서버 예측 느릴 수 있음) ----
  uint32_t t0 = millis();
  while (!client.available() && client.connected() && (millis() - t0 < STATUS_WAIT_MS)) {
    delay(10);
  }
  if (!client.available()) {
    client.stop();
    return false;
  }

  outStatusLine = client.readStringUntil('\n');
  outStatusLine.trim();

  // ---- 헤더 스킵 ----
  while (client.connected()) {
    String h = client.readStringUntil('\n');
    if (h == "\r" || h.length() == 0) break;
  }

  // ---- 바디 제한 읽기(멈춤 방지) ----
  outBody = readBodyLimited(client, RESP_MAX_MS, RESP_MAX_BYTES);

  client.stop();

  return outStatusLine.startsWith("HTTP/1.1 200") || outStatusLine.startsWith("HTTP/1.0 200");
}

// ================== Serial 메시지 파싱 ==================
static String parseField(const String& s, const char* key) {
  String k = String(key) + "=";
  int pos = s.indexOf(k);
  if (pos < 0) return "";
  int start = pos + k.length();
  int end = s.indexOf(';', start);
  if (end < 0) end = s.length();
  String v = s.substring(start, end);
  v.trim();
  return v;
}

static String parseWeightFromLine(const String& s) { return parseField(s, "W"); }
static String parseTypeFromLine(const String& s) {
  String t = parseField(s, "T");
  t.toLowerCase();
  if (t == "beef" || t == "pork") return t;
  return "";
}

// ================== 라인 처리 ==================
static void handleLine(const String& s) {
  if (s.indexOf("PIC") < 0) return;

  if (!cameraReady) { ackToAtmega("PIC=FAIL CAM=NOTREADY"); return; }
  if (g_busy)       { ackToAtmega("PIC=BUSY"); return; }  // 중복 요청 방지

  g_busy = true;

  // ✅ ATmega RX waiting 방지: 먼저 BUSY ACK
  ackToAtmega("PIC=BUSY");

  String weight = parseWeightFromLine(s);
  if (weight.length() == 0) weight = "0";

  String meatType = parseTypeFromLine(s); // ""면 auto

  if (!ensureWifiConnected()) {
    ackToAtmega("PIC=FAIL WIFI=FAIL");
    g_busy = false;
    return;
  }

  // 촬영
  digitalWrite(FLASH_LED, HIGH);
  delay(80);
  camera_fb_t* fb = esp_camera_fb_get();
  digitalWrite(FLASH_LED, LOW);

  if (!fb) {
    ackToAtmega("PIC=FAIL FB=NULL");
    g_busy = false;
    return;
  }

  // ✅ malloc 복사 제거: fb->buf 그대로 전송
  String statusLine, rawBody;
  bool httpOk = sendToServerMultipart(meatType, fb->buf, fb->len, statusLine, rawBody);

  esp_camera_fb_return(fb);

  // 바디에서 JSON만 추출
  String json = extractJson(rawBody);

  bool success = false;
  bool hasSuccess = jsonPickBoolean(json, "success", success);
  String grade = jsonPickString(json, "grade");
  String mtype = jsonPickString(json, "meat_type");

  // ACK 생성
  if (httpOk && hasSuccess && success) {
    String msg = "PIC=OK SEND=OK ";
    msg += "W=" + weight + " ";
    if (meatType.length()) msg += "T=" + meatType + " ";
    if (mtype.length())    msg += "MT=" + mtype + " ";
    if (grade.length())    msg += "GRADE=" + grade + " ";
    msg += "IP=" + WiFi.localIP().toString();
    ackToAtmega(msg);
  } else {
    String msg = "PIC=OK SEND=FAIL ";
    msg += "W=" + weight + " ";
    if (meatType.length()) msg += "T=" + meatType + " ";
    msg += "IP=" + WiFi.localIP().toString();
    ackToAtmega(msg);
  }

  g_busy = false;
}

// ================== Arduino ==================
void setup() {
  pinMode(FLASH_LED, OUTPUT);
  digitalWrite(FLASH_LED, LOW);

  Serial.begin(9600);
  delay(200);

  cameraReady = initCamera();
  bool wifiOk = ensureWifiConnected();

  String msg = "BOOT ";
  msg += (cameraReady ? "CAM=OK " : "CAM=FAIL ");
  msg += (wifiOk ? "WIFI=OK " : "WIFI=FAIL ");
  if (wifiOk) msg += "IP=" + WiFi.localIP().toString();
  ackToAtmega(msg);
}

void loop() {
  // ATmega에서 라인 단위 수신
  while (Serial.available()) {
    char c = (char)Serial.read();
    if (c == '\n') {
      g_line.trim();
      if (g_line.length()) handleLine(g_line);
      g_line = "";
    } else if (c != '\r') {
      g_line += c;
      if (g_line.length() > 250) g_line = "";
    }
  }
}

