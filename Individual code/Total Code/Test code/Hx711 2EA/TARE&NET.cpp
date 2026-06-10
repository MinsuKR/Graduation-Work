// TARE (영점 조절) : 로드셀은 주변 온도나 미세한 위치 변화에 따라 초기값이 달라지기 때문에 TARE 과정을 통해 현재 상태를 0점(기준)으로 잡음
// EEPROM 저장 : 0으로 저장 후 측정된 영점 데이터(Offset)는 전원이 꺼져도 보존되도록 EEPROM에 저장함
// TARE가 완료된 후 출력되는 NET 값은 영점이 완벽히 보정 (크게 오차 없음)

// -50 ~ 50 까지 차이가 나는 이유  
// 24비트 고해상도로 아주 미세한 진동이나 공기 흐름조차 수치가 보임
// 전기적 노이즈로 주변 전자기기나 ATmega128의 전원부에서 발생하는 미세한 전기적 간섭이 수치에 반영될 수 있음
// 하드웨어 특성으로 로드셀은 금속의 미세한 변형을 측정하므로 온도 변화나 브레드보드의 접촉 저항 등으로 인해 값이 조금씩 흔들리는 '드리프트(Drift)' 현상이 발생

// 로드셀을 눌렀다가 뗐을 때 값이 즉시 처음 상태로 돌아오지 않는 이유
// 탄성 이력 현상 및 크리프 현상으로 금속 몸체가 압력을 받아 휘어졌다가 원래 상태로 돌아올 때, 미세하게 시간이 걸리는 성질이 있음
// 전기적 충방전으로 압력이 급격히 변할 때 전기적 신호가 완전히 안정화될 때까지 아주 짧은 과도 상태(Transient state)가 발생하여 숫자가 출렁일 수 있음

#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>

// ATmega128 전용 핀 정의 사용
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

const uint32_t CAL_MAGIC = 0xC0A1B00B;
const int EEPROM_ADDR = 0;

long offsetL = 0, offsetR = 0;

void lcd2(const char* a, const char* b) {
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(a);
  lcd.setCursor(0, 1); lcd.print(b);
}

void saveToEEPROM() {
  // 현재 잡은 영점을 EEPROM에 안전하게 저장
  CalData d = {offsetL, offsetR, 1.0f, 1.0f, CAL_MAGIC};
  EEPROM.put(EEPROM_ADDR, d);
}

void setup() {
  lcd.init();
  lcd.backlight();
  lcd2("Smart Scale v1.0", "Initializing...");

  scaleL.begin(DT_L, SCK_L);
  scaleR.begin(DT_R, SCK_R);
  delay(1000);

  // 1. TARE 과정 (시연 핵심)
  lcd2("TARE: Emptying..", "Don't touch!");
  
  // 센서가 준비될 때까지 대기
  if (scaleL.wait_ready_timeout(1000) && scaleR.wait_ready_timeout(1000)) {
    // 20번 읽어 평균을 내어 정밀한 영점(Offset) 획득
    offsetL = scaleL.read_average(20);
    offsetR = scaleR.read_average(20);
    
    // 2. EEPROM 저장
    saveToEEPROM();
    
    lcd2("TARE Complete!", "Saved to EEPROM");
    delay(1500);
  } else {
    lcd2("HX711 ERROR!", "Check Wiring");
    while(1);
  }
  lcd.clear();
}

void loop() {
  // 실시간 NET 값 측정
  if (scaleL.is_ready() && scaleR.is_ready()) {
    long netL = scaleL.read_average(5) - offsetL;
    long netR = scaleR.read_average(5) - offsetR;

    char l1[17], l2[17];
    // 영상에서 잘 보이도록 좌측 정렬 및 단위 표시 시늉
    sprintf(l1, "L_NET:%9ld", netL);
    sprintf(l2, "R_NET:%9ld", netR);
    
    lcd.setCursor(0, 0); lcd.print(l1);
    lcd.setCursor(0, 1); lcd.print(l2);
  }
  delay(100);
}