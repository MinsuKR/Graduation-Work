#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ===================== 모드 선택(둘 중 하나만 1) =====================
#define CAL_LEFT   0
#define CAL_RIGHT  1
// ====================================================================

// ====== 핀(너 프로젝트에 맞게 유지/수정) ======
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

// ====== LCD ======
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

// ====== 기준 무게 ======
static const float REF_G = 155.0f;

// ====== HX711 ======
HX711 scaleL;
HX711 scaleR;

// ---------- LCD 유틸: 줄 전체 지우고 출력(잔상 제거) ----------
static void lcdPrintLine(uint8_t row, const String &text) {
  lcd.setCursor(0, row);
  for (int i = 0; i < LCD_COLS; i++) lcd.print(' ');
  lcd.setCursor(0, row);

  String t = text;
  if (t.length() > LCD_COLS) t = t.substring(0, LCD_COLS);
  lcd.print(t);
}

// ---------- raw 평균 ----------
static long readRawAvg(HX711 &s, int n = 20) {
  long sum = 0;
  for (int i = 0; i < n; i++) {
    while (!s.is_ready()) { delay(1); }
    sum += s.read();
    delay(5);
  }
  return sum / n;
}

// ===================== 선택된 채널 바인딩 =====================
#if (CAL_LEFT + CAL_RIGHT) != 1
  #error "CAL_LEFT 또는 CAL_RIGHT 중 하나만 1로 설정하세요."
#endif

#if CAL_LEFT
  #define MODE_NAME "LEFT"
  HX711 &scale = scaleL;
#else
  #define MODE_NAME "RIGHT"
  HX711 &scale = scaleR;
#endif
// =============================================================

// 상태
enum CalState { ST_TARE, ST_WAIT_155, ST_DONE };
CalState st = ST_TARE;

long raw0 = 0;
float scale_factor = 0.0f;    // set_scale에 넣을 값 (diff/155)  -> -185.xxx 같은 형태
float inv_factor   = 0.0f;    // LCD 표시용 역수 (155/diff)       -> -0.005xxx 같은 형태

void setup() {
_bugfix_init:
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // 두 채널 다 begin(안정적으로)
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  lcdPrintLine(0, "CAL 155g MODE");
  lcdPrintLine(1, String(MODE_NAME));
  delay(1200);
}

void loop() {
  switch (st) {
    case ST_TARE: {
      // ✅ 1) 빈 상태에서 raw0 측정 + tare
      lcdPrintLine(0, String(MODE_NAME) + " EMPTY");
      lcdPrintLine(1, "Tare...");

      // raw0는 "캘리브레이션 diff 계산용 기준"이라 tare 전에 잡아도 되고,
      // 여기서는 raw0를 잡고 -> tare도 같이 수행(이후 get_units 편하게)
      raw0 = readRawAvg(scale, 30);

      // 라이브러리 tare (빈 상태에서!)
      scale.tare(15);

      lcdPrintLine(0, "Tare done");
      lcdPrintLine(1, "Place 155g");
      delay(1000);

      st = ST_WAIT_155;
      break;
    }

    case ST_WAIT_155: {
      // ✅ 2) 155g 올리면 diff로 factor 계산
      long rawW = readRawAvg(scale, 15);
      long diff = rawW - raw0;

      // 올려졌는지 감지 임계값(환경 따라 조절)
      const long TH = 2000;

      lcdPrintLine(0, "Put 155g...");
      lcdPrintLine(1, "diff:" + String(diff));

      if (labs(diff) > TH) {
        // 안정화 후 다시 측정
        delay(800);
        rawW = readRawAvg(scale, 30);
        diff = rawW - raw0;

        if (diff == 0) {
          lcdPrintLine(0, "ERROR diff=0");
          lcdPrintLine(1, "Check wiring");
          delay(1500);
          st = ST_TARE;
          break;
        }

        // ✅ 핵심:
        // set_scale()에는 "diff/155"를 넣어야 정상 동작
        scale_factor = (float)diff / REF_G;     // -185.xxx 형태 가능(정상)
        inv_factor   = REF_G / (float)diff;     // -0.005xxx 형태(표시용)

        scale.set_scale(scale_factor);

        // ✅ factor 적용 후에는 "빈 상태"가 아니므로 tare 다시 하면 안됨.
        // (이미 빈 상태에서 tare 했고, 지금은 155g 올린 상태니까)

        lcdPrintLine(0, "Factor computed");
        lcdPrintLine(1, "Applied");
        delay(900);

        st = ST_DONE;
      }
      break;
    }

    case ST_DONE: {
      // ✅ 3) 현재 읽는 무게 + 역수 factor 표시
      // 현재는 tare가 빈 상태 기준이므로, 155g 올린 상태면 g가 155 근처로 나와야 정상
      float g = scale.get_units(10);

      // 1행: 무게
      // 예) "LEFT W:155.0g"
      lcdPrintLine(0, String(MODE_NAME) + " W:" + String(g, 1) + "g");

      // 2행: 역수 factor 표시(-0.005xxx)
      // 예) "inv:-0.005346"
      // 16칸 제한이라 소수 6자리 정도가 적당
      lcdPrintLine(1, "inv:" + String(inv_factor, 6));

      delay(300);
      break;
    }
  }
}
