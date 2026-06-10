// RAW : 로드셀에서 들어오는 가공되지 않은 순수 전기 신호를 디지털 숫자로 변환한 값
// 즉, RAW 데이터는 HX711 24비트 AD 컨버터가 로드셀의 미세한 전압 변화를 읽어들인 순수 출력값
// NET : RAW 데이터에서 '영점(Tare)'을 빼서 실제 물체의 무게만 나타낸 데이터
// 즉, 소프트웨어적으로 영점을 보정한 결과로 센서의 초기 오차를 제거하고 실제 투입된 무게 정보만을 추출

#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>

// ATmega128 핀 정의
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

HX711 scaleL, scaleR;
LiquidCrystal_I2C lcd(0x27, 16, 2);

long offsetL = 0;
long offsetR = 0;
bool showRaw = false; 
unsigned long lastSwitchTime = 0;

void setup() {
  lcd.init();
  lcd.backlight();
  
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  lcd.setCursor(0, 0);
  lcd.print("HX711 Monitoring");
  lcd.setCursor(0, 1);
  lcd.print("Setting Tare...");
  
  // 영점 설정
  scaleL.wait_ready(1000);
  scaleR.wait_ready(1000);
  if (scaleL.is_ready()) offsetL = scaleL.read_average(10);
  if (scaleR.is_ready()) offsetR = scaleR.read_average(10);

  lcd.clear();
  lastSwitchTime = millis();
}

void loop() {
  // 5초마다 화면 모드 전환
  if (millis() - lastSwitchTime > 5000) {
    showRaw = !showRaw;
    lastSwitchTime = millis();
    lcd.clear(); // 모드 바뀔 때 잔상 제거
  }

  if (scaleL.is_ready() && scaleR.is_ready()) {
    long rawL = scaleL.read();
    long rawR = scaleR.read();

    long netL = rawL - offsetL;
    long netR = rawR - offsetR;

    char line1[17], line2[17];

    if (showRaw) {
      // --- NET 모드 시연 (1열: L_NET, 2열: R_NET) ---
      sprintf(line1, "L_NET:%4ld   ", netL);
      sprintf(line2, "R_NET:%4ld   ", netR);

    } else {
      // --- RAW 모드 시연 (1열: L_RAW, 2열: R_RAW) ---
      sprintf(line1, "L_RAW:%-8ld ", rawL);
      sprintf(line2, "R_RAW:%-8ld ", rawR);
    }

    lcd.setCursor(0, 0); lcd.print(line1);
    lcd.setCursor(0, 1); lcd.print(line2);
  } else {
    // 센서 연결 확인용
    lcd.setCursor(0, 0);
    lcd.print("Connecting...   ");
  }

  delay(200);
}