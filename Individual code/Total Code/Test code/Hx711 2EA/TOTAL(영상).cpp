// 1. 전원 ON → DT L/R 상태 + Wiring check… (3초)
// -> DT = 'L'나오면 정상
// 2. 자동으로 LNET/RNET 화면으로 넘어감 → “신호 들어오는 거 확인”
// -> offset = 0이라 NET = RAW
// 3. NEXT → TARE 단계 → ACT 눌러 영점
// -> tare잡고 NET 잡음 0 근처(빈상태)
// 4. NEXT → CAL L → 기준추 올리고 ACT
// 5. NEXT → CAL R → 기준추 옮기고 ACT
// -> 0.00xx정도 나옴
// 6. NEXT → MEASURE → 실제 무게/합계 시연

#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ================= LCD =================
LiquidCrystal_I2C lcd(0x27, 16, 2);

// ================= ATmega128 핀 매핑 =================
// HX711 (왼쪽)
#define DT_L   PD3
#define SCK_L  PD2

// HX711 (오른쪽)
#define DT_R   PD5
#define SCK_R  PD4

// 버튼 (GND로 누르는 방식: INPUT_PULLUP)
#define ACT_BTN   PE4   // 실행(=RAW보기/TARE/CAL)
#define NEXT_BTN  PE5   // 단계 이동

// ================= HX711 =================
HX711 scaleL;
HX711 scaleR;

// ================= 캘리브 기준 무게 =================
const float CAL_WEIGHT = 155.0f;

// ================= 보정 값 =================
long offsetL = 0, offsetR = 0;
float factorL = 0.0f, factorR = 0.0f;

// ================= EEPROM 저장 =================
struct CalData {
  long offsetL;
  long offsetR;
  float factorL;
  float factorR;
  uint32_t magic;
};

// EEPROM에 저장된 게 "우리 포맷"인지 확인하는 서명(매직 넘버)
const uint32_t CAL_MAGIC = 0xC0A1B00B;
const int EEPROM_ADDR = 0;

// ================= 유틸 =================
static void lcd2(const char* a, const char* b) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print(b);
}

static bool btnPressed(uint8_t pin) {
  if (digitalRead(pin) == LOW) {
    delay(20);
    if (digitalRead(pin) == LOW) {
      while (digitalRead(pin) == LOW) {}
      return true;
    }
  }
  return false;
}

static void saveCal() {
  CalData d{offsetL, offsetR, factorL, factorR, CAL_MAGIC};
  EEPROM.put(EEPROM_ADDR, d);
}

static bool loadCal() {
  CalData d;
  EEPROM.get(EEPROM_ADDR, d);
  if (d.magic != CAL_MAGIC) return false;
  offsetL = d.offsetL; offsetR = d.offsetR;
  factorL = d.factorL; factorR = d.factorR;
  return true;
}

// bogde/HX711 제공: wait_ready_timeout(timeout, delay_ms)
// -> 블로킹 방지용. (timeout 내 준비 안 되면 false)
static bool readyBoth(uint16_t timeout_ms = 120, uint8_t poll_delay_ms = 2) {
  bool okL = scaleL.wait_ready_timeout(timeout_ms, poll_delay_ms);
  bool okR = scaleR.wait_ready_timeout(timeout_ms, poll_delay_ms);
  return okL && okR;
}

// ================= 단계 =================
enum Stage {
  ST_WIRING_SHOW = 0,  // DT 상태/간단 클럭펄스 "보여주기"만 하고 자동 전환
  ST_RAW_NET_VIEW,     // RAW/NET 표시
  ST_TARE,             // 영점
  ST_CAL_L,            // 왼쪽 캘리브
  ST_CAL_R,            // 오른쪽 캘리브
  ST_MEASURE           // 측정
};
Stage stage = ST_WIRING_SHOW;

// UI 타이머
uint32_t lastUI = 0;

// WIRING 화면 자동 전환용
const uint32_t WIRING_SHOW_MS = 3000; // 3초 보여주고 RAW/NET로 자동
uint32_t wiringStartMs = 0;

// ================= 동작 =================
static void doTareBoth(uint8_t avg = 20) {
  lcd2("TARE...", "Wait");

  if (!readyBoth(800, 2)) {
    lcd2("HX711 NOT READY", "Check Wiring");
    delay(1200);
    return;
  }

  // read_average()는 내부에서 read()를 호출하므로, 준비 안 되면 read()에서 대기할 수 있음.
  // 그래서 위에서 readyBoth로 먼저 확인해 "멈춤" 가능성을 줄임.
  offsetL = scaleL.read_average(avg);
  offsetR = scaleR.read_average(avg);
  saveCal();

  lcd2("TARE OK", "Offsets Saved");
  delay(900);
}

static void doCalOne(HX711 &s, long offset, float &factorOut, const char* tag) {
  char line1[17];
  snprintf(line1, sizeof(line1), "Put %dg on %s", (int)CAL_WEIGHT, tag);
  lcd2(line1, "ACT=Cal NEXT=Skip");

  while (true) {
    if (btnPressed(NEXT_BTN)) return; // 캘리브 스킵
    if (btnPressed(ACT_BTN)) break;   // 캘리브 진행
    delay(10);
  }

  if (!s.wait_ready_timeout(800, 2)) {
    lcd2("NOT READY", "Try Again");
    delay(900);
    return;
  }

  long raw = s.read_average(30);
  long net = raw - offset;

  if (net == 0) {
    lcd2("CAL FAIL", "net=0");
    delay(1000);
    return;
  }

  factorOut = CAL_WEIGHT / (float)net; // count -> gram 변환 계수
  saveCal();

  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(tag); lcd.print(" Cal Done");
  lcd.setCursor(0, 1); lcd.print("f="); lcd.print(factorOut, 6);
  delay(1500);
}

// ================= setup =================
void setup() {
  pinMode(ACT_BTN, INPUT_PULLUP);
  pinMode(NEXT_BTN, INPUT_PULLUP);

  lcd.init();
  lcd.backlight();
  lcd2("Dual Scale Demo", "ATmega128 PIO");
  delay(900);

  // HX711 시작 + gain 명시(128)
  scaleL.begin(DT_L, SCK_L, 128);
  scaleR.begin(DT_R, SCK_R, 128);

  if (loadCal()) lcd2("EEPROM Cal", "Loaded");
  else           lcd2("EEPROM Cal", "Empty");
  delay(900);

  // WIRING 단계 시작 시간 기록
  wiringStartMs = millis();
  stage = ST_WIRING_SHOW;
  lcd.clear();
}

// ================= loop =================
void loop() {
  // 수동 단계 이동
  if (btnPressed(NEXT_BTN)) {
    stage = (Stage)((int)stage + 1);
    if (stage > ST_MEASURE) stage = ST_WIRING_SHOW;

    // WIRING 단계로 돌아오면 타이머 리셋
    if (stage == ST_WIRING_SHOW) wiringStartMs = millis();

    lcd.clear();
    delay(50);
  }

  switch (stage) {
    // 0) DT 상태 "보여주기" -> 몇 초 후 자동 RAW/NET로 이동
    case ST_WIRING_SHOW: {
      // 화면: DT 상태 표시 + "Checking..." (토글은 최소화)
      if (millis() - lastUI > 250) {
        lastUI = millis();
        bool dtL = digitalRead(DT_L);
        bool dtR = digitalRead(DT_R);

        lcd.setCursor(0, 0);
        lcd.print("DT L:");
        lcd.print(dtL ? "H" : "L");
        lcd.print(" R:");
        lcd.print(dtR ? "H" : "L");
        lcd.print("  ");

        lcd.setCursor(0, 1);
        lcd.print("Wiring check...");
      }

      // "연결 확인용" 최소 펄스만 (게인/채널 꼬임 최소화 목적)
      // 1~2회 정도의 짧은 펄스는 시연용으로만.
      digitalWrite(SCK_L, HIGH); delayMicroseconds(2); digitalWrite(SCK_L, LOW);
      digitalWrite(SCK_R, HIGH); delayMicroseconds(2); digitalWrite(SCK_R, LOW);
      delay(5);

      // 3초 지나면 자동으로 RAW/NET로 이동
      if (millis() - wiringStartMs >= WIRING_SHOW_MS) {
        stage = ST_RAW_NET_VIEW;
        lcd.clear();
        delay(50);
      }
    } break;

    // 1) RAW/NET 보기 (ACT 누르면 RAW 잠깐 표시)
    case ST_RAW_NET_VIEW: {
      if (millis() - lastUI > 220) {
        lastUI = millis();

        if (!readyBoth(120, 2)) {
          lcd2("Waiting HX711", "...");
          break;
        }

        long rawL = scaleL.read_average(8);
        long rawR = scaleR.read_average(8);
        long netL = rawL - offsetL;
        long netR = rawR - offsetR;

        char l1[17], l2[17];
        snprintf(l1, sizeof(l1), "LNET:%-10ld", netL);
        snprintf(l2, sizeof(l2), "RNET:%-10ld", netR);
        lcd.setCursor(0, 0); lcd.print(l1);
        lcd.setCursor(0, 1); lcd.print(l2);
      }

      if (btnPressed(ACT_BTN)) {
        // RAW 값도 잠깐 보여주기(영상용)
        if (readyBoth(300, 2)) {
          long rawL = scaleL.read_average(5);
          long rawR = scaleR.read_average(5);

          char l1[17], l2[17];
          snprintf(l1, sizeof(l1), "LRAW:%-10ld", rawL);
          snprintf(l2, sizeof(l2), "RRAW:%-10ld", rawR);

          lcd.setCursor(0, 0); lcd.print(l1);
          lcd.setCursor(0, 1); lcd.print(l2);
          delay(1200);
        }
      }
    } break;

    // 2) TARE
    case ST_TARE: {
      lcd2("Stage:TARE", "ACT=Do Tare");
      if (btnPressed(ACT_BTN)) doTareBoth();
    } break;

    // 3) CAL L
    case ST_CAL_L: {
      lcd2("Stage:CAL L", "ACT=Cal NEXT=Skip");
      doCalOne(scaleL, offsetL, factorL, "L");
    } break;

    // 4) CAL R
    case ST_CAL_R: {
      lcd2("Stage:CAL R", "ACT=Cal NEXT=Skip");
      doCalOne(scaleR, offsetR, factorR, "R");
    } break;

    // 5) MEASURE
    case ST_MEASURE: {
      if (factorL == 0.0f || factorR == 0.0f) {
        lcd2("Need CAL first", "Go CAL L/R");
        delay(200);
        break;
      }

      if (millis() - lastUI > 180) {
        lastUI = millis();

        if (!readyBoth(120, 2)) {
          lcd2("Waiting HX711", "...");
          break;
        }

        long rawL = scaleL.read_average(10);
        long rawR = scaleR.read_average(10);
        long netL = rawL - offsetL;
        long netR = rawR - offsetR;

        float wL = (float)netL * factorL;
        float wR = (float)netR * factorR;
        float tot = wL + wR;

        lcd.setCursor(0, 0);
        lcd.print("L:"); lcd.print(wL, 1);
        lcd.print(" R:"); lcd.print(wR, 1);
        lcd.print("  ");

        lcd.setCursor(0, 1);
        lcd.print("TOT:"); lcd.print(tot, 1);
        lcd.print(" g     ");
      }

      // 측정 중 ACT 누르면 즉시 영점(영상에서 “사용 중 영점 재설정” 시연 가능)
      if (btnPressed(ACT_BTN)) doTareBoth();
    } break;
  }
}