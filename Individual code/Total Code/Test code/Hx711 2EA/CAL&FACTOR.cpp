// CAL하기 위해서는 처음에 영점 조절을 해야함
// 센서에서 20번의 데이터를 읽어 평균을 낸 뒤, 이 값을 영점(offset) 변수에 저장
// Factor란, 로드셀마다 전압을 숫자로 바꾸는 특성이 다름
// Factor는 센서가 출력하는 'RAW 데이터'와 우리가 쓰는 'g(그램) 단위' 사이의 비례 상수
// Factor = 기준무게 / RAW - OFFSET 사용하여 센서의 개별 편차를 스스로 학습하여 보정설계
// 산출된 보정 계수는 EEPROM에 영구 저장되어 매번 보정할 필요없이 즉시 정확한 무게 측정이 가능함

#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ATmega128 핀 정의
#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

HX711 scaleL, scaleR;
LiquidCrystal_I2C lcd(0x27, 16, 2);

struct CalData {
  long offsetL;
  long offsetR;
  float factorL;
  float factorR;
  uint32_t magic;
};

CalData d;
const uint32_t CAL_MAGIC = 0xC0A1B00B;
const float CAL_WEIGHT = 155.0f;

void lcd2(const char* a, const char* b) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print(b);
}

void setup() {
  lcd.init();
  lcd.backlight();
  
  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);

  // --- [1단계: TARE (영점)] ---
  lcd2("STEP 1: TARE", "Keep Empty!!");
  delay(3000); // 촬영 준비 시간
  
  if (scaleL.wait_ready_timeout(1000) && scaleR.wait_ready_timeout(1000)) {
    d.offsetL = scaleL.read_average(20);
    d.offsetR = scaleR.read_average(20);
    lcd2("TARE Done!", "Prepare 155g");
    delay(2000);
  }

  // --- [2단계: LEFT Calibration] ---
  lcd2("STEP 2: LEFT", "Put 155g on L");
  delay(5000); // 물체 올리는 시간
  
  lcd2("L: Measuring...", "Hold still");
  long netL = scaleL.read_average(30) - d.offsetL;
  if (netL != 0) {
    d.factorL = CAL_WEIGHT / (float)netL;
  }
  
  lcd2("LEFT CAL OK!", "Next: RIGHT");
  delay(3000);

  // --- [3단계: RIGHT Calibration] ---
  lcd2("STEP 3: RIGHT", "Put 155g on R");
  delay(5000);
  
  lcd2("R: Measuring...", "Hold still");
  long netR = scaleR.read_average(30) - d.offsetR;
  if (netR != 0) {
    d.factorR = CAL_WEIGHT / (float)netR;
  }

  // --- [4단계: 결과 표시 및 저장] ---
  d.magic = CAL_MAGIC;
  EEPROM.put(0, d);
  
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print("CALIBRATION DONE");
  lcd.setCursor(0, 1); lcd.print("Saving to ROM...");
  delay(2000);
}

void loop() {
  // 결과 확인 화면 (영상 촬영의 핵심)
  char fL_str[17], fR_str[17];
  
  // 소수점 6자리까지 표시하여 정밀도 강조
  dtostrf(d.factorL, 10, 6, fL_str);
  dtostrf(d.factorR, 10, 6, fR_str);

  lcd.setCursor(0, 0);
  lcd.print("L_FAC:"); lcd.print(fL_str);
  lcd.setCursor(0, 1);
  lcd.print("R_FAC:"); lcd.print(fR_str);
  
  delay(4000);

  lcd2("CAL Complete", "Ready to Measure");
  delay(4000);
}