#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ===================== 모드 선택(둘 중 하나만 1) =====================
#define USE_LEFT   0
#define USE_RIGHT  1
// ====================================================================

// ===================== 여기! 테스트할 factor를 직접 입력 =====================
// ⚠️ HX711(bogde) set_scale()에는 "counts per gram" 값을 넣어야 정상!
// 예: -185.684f 같은 큰 값(정상)
// (네가 갖고 있는 -0.005346 값은 '역수 표시용'이라 set_scale에 넣으면 이상해짐)
#define TEST_FACTOR   (-199.44f)
// ============================================================================

// ====== 핀(너 프로젝트 값으로 유지/수정) ======
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

// ====== LCD ======
#define LCD_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2
LiquidCrystal_I2C lcd(LCD_ADDR, LCD_COLS, LCD_ROWS);

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

// ===================== 선택된 채널 바인딩 =====================
#if (USE_LEFT + USE_RIGHT) != 1
  #error "USE_LEFT 또는 USE_RIGHT 중 하나만 1로 설정하세요."
#endif

#if USE_LEFT
  #define MODE_NAME "LEFT"
  HX711 &scale = scaleL;
#else
  #define MODE_NAME "RIGHT"
  HX711 &scale = scaleR;
#endif
// =============================================================

void setup() {
  Wire.begin();
  lcd.init();
  lcd.backlight();

  // 두 채널 다 begin(안정성)
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  // 안내
  lcdPrintLine(0, "FACTOR TEST");
  lcdPrintLine(1, String(MODE_NAME));
  delay(800);

  // ✅ 여기서 factor 적용 + tare(빈 상태에서!)
  scale.set_scale(TEST_FACTOR);

  lcdPrintLine(0, "Keep EMPTY");
  lcdPrintLine(1, "Tare...");
  delay(300);

  scale.tare(20);  // 빈 상태에서 영점

  lcdPrintLine(0, "Tare done");
  lcdPrintLine(1, "Put weight");
  delay(700);
}

void loop() {
  // 현재 무게(g) 읽기
  float g = scale.get_units(10);

  // 1행: 무게
  // 예: "LEFT W:155.0g"
  String line0 = String(MODE_NAME) + " W:" + String(g, 1) + "g";
  lcdPrintLine(0, line0);

  // 2행: factor(입력한 값) 표시
  // 예: "f:-185.684"
  String line1 = "f:" + String(TEST_FACTOR, 3);
  lcdPrintLine(1, line1);

  delay(250);
}
