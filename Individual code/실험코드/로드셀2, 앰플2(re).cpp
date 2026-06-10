#include <Arduino.h>
#include <Wire.h>
#include <HX711.h>
#include <LiquidCrystal_I2C.h>
#include <EEPROM.h>
#include <math.h>

#include "LED.h"
#include "Buzzer.h"
#include "Button.h"

/* =========================
   실험 모드 선택
   ========================= */
#define EXP_POS_ERROR   1
#define EXP_REPEAT      2
#define EXP_CAL_STAB    3
#define EXP_DRIFT       4

// ★ 테스트하고 싶은 모드를 여기서 선택하세요 ★
#define EXPERIMENT_MODE EXP_REPEAT

/* =========================
   하드웨어 및 설정 (Dual Scale)
   ========================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);
HX711 scaleL, scaleR;

#define DT_L   PIN_PD3
#define SCK_L  PIN_PD2
#define DT_R   PIN_PD5
#define SCK_R  PIN_PD4

const float CAL_WEIGHT = 155.0f;

// 보정 변수
float factorL = 1.0f;
float factorR = 1.0f;
long offsetL = 0;
long offsetR = 0;

/* ==============================================
   핵심 측정 함수
   ============================================== */

long readFilteredRaw(HX711 &target, int samples) {
    long sum = 0;
    long minVal = 2147483647;
    long maxVal = -2147483648;
    for (int i = 0; i < samples; i++) {
        while(!target.is_ready());
        long raw = target.read();
        if (raw < minVal) minVal = raw;
        if (raw > maxVal) maxVal = raw;
        sum += raw;
        delay(5);
    }
    return (sum - minVal - maxVal) / (samples - 2);
}

float readTotalWeight(uint8_t samples) {
    long rawL = readFilteredRaw(scaleL, samples);
    long rawR = readFilteredRaw(scaleR, samples);
    float wL = (float)(rawL - offsetL) * factorL;
    float wR = (float)(rawR - offsetR) * factorR;
    return wL + wR;
}

/* =========================
   실험 모드 함수들
   ========================= */

void experimentPositionError() {
    const char* posName[7] = {"LU", "L", "LD", "C", "RU", "R", "RD"};
    Serial.println("Position,Weight(g)");
    for (int i = 0; i < 7; i++) {
        lcd.clear(); lcd.print("Pos: "); lcd.print(posName[i]);
        lcd.setCursor(0, 1); lcd.print("Put 155g (5s)");
        delay(5000); 
        float w = readTotalWeight(40);
        Serial.print(posName[i]); Serial.print(","); Serial.println(w, 2);
        lcd.setCursor(0, 1); lcd.print("W: "); lcd.print(w, 2); lcd.print("g   ");
        Buzzer_Click();
        delay(3000);
    }
}

void experimentRepeatability() {
    Serial.println("Trial,Weight(g)");
    for (int i = 1; i <= 10; i++) {
        lcd.clear(); lcd.print("Trial: "); lcd.print(i);
        lcd.setCursor(0, 1); lcd.print("Put 155g (5s)");
        delay(5000);
        float w = readTotalWeight(40);
        Serial.print(i); Serial.print(","); Serial.println(w, 2);
        lcd.setCursor(0, 1); lcd.print("W: "); lcd.print(w, 2); lcd.print("g   ");
        Buzzer_Click();
        delay(2000);
        lcd.clear(); lcd.print("Remove Weight");
        delay(4000);
    }
}

void experimentDrift() {
    Serial.println("Time(min),Weight(g)");
    unsigned long start = millis();
    unsigned long duration = 600000UL; 
    while (true) { 
        unsigned long elapsed = millis() - start;
        unsigned long t = elapsed / 60000UL; 
        if (t > 10) t = 10; 
        float w = readTotalWeight(30);
        Serial.print(t); Serial.print(","); Serial.println(w, 3);
        lcd.setCursor(0, 0); lcd.print("Drift: "); lcd.print(t); lcd.print("min   "); 
        lcd.setCursor(0, 1); lcd.print("W: "); lcd.print(w, 3); lcd.print("g     ");
        if (elapsed >= duration) break;
        delay(30000); 
    }
}

/* =========================
   자동 실행 시퀀스 (Setup)
   ========================= */

void setup() {
    Serial.begin(9600);
    lcd.init(); lcd.backlight();
    LED_Init(); Buzzer_Init(); Button_Init();
    
    scaleL.begin(DT_L, SCK_L);
    scaleR.begin(DT_R, SCK_R);

    // 1단계: 자동 영점 (Tare)
    lcd.clear(); lcd.print("1. AUTO TARE");
    lcd.setCursor(0, 1); lcd.print("EMPTY SCALE...");
    delay(3000); 
    offsetL = readFilteredRaw(scaleL, 40);
    offsetR = readFilteredRaw(scaleR, 40);
    Buzzer_Click();

    // 2단계: 자동 보정 (Calibration) - 왼쪽, 오른쪽, 가운데
    // [2-1. 왼쪽]
    lcd.clear(); lcd.print("2-1. CAL LEFT");
    lcd.setCursor(0, 1); lcd.print("PUT 155g on L");
    delay(5000);
    long rawL_l = readFilteredRaw(scaleL, 40);
    factorL = CAL_WEIGHT / (float)(rawL_l - offsetL);
    Buzzer_Click();

    // [2-2. 오른쪽]
    lcd.clear(); lcd.print("2-2. CAL RIGHT");
    lcd.setCursor(0, 1); lcd.print("MOVE 155g to R");
    delay(5000);
    long rawR_r = readFilteredRaw(scaleR, 40);
    factorR = CAL_WEIGHT / (float)(rawR_r - offsetR);
    Buzzer_Click();

    // [2-3. 가운데 - 확인 단계]
    lcd.clear(); lcd.print("2-3. CHK CENTER");
    lcd.setCursor(0, 1); lcd.print("MOVE 155g to C");
    delay(5000);
    float wCenter = readTotalWeight(40);
    Serial.print("Calibration Check (Center): "); Serial.println(wCenter, 2);
    Buzzer_Success();

    // 3단계: 무게추 제거 안내
    lcd.clear(); lcd.print("3. REMOVE WEIGHT");
    lcd.setCursor(0, 1); lcd.print("PREPARE EXP...");
    delay(5000);
    
    // 4단계: 실험 시작
    lcd.clear(); lcd.print("4. START EXP");
    lcd.setCursor(0, 1); lcd.print("MODE: "); lcd.print(EXPERIMENT_MODE);
    delay(2000);
    Buzzer_Success();

    #if EXPERIMENT_MODE == EXP_POS_ERROR
        experimentPositionError();
    #elif EXPERIMENT_MODE == EXP_REPEAT
        experimentRepeatability();
    #elif EXPERIMENT_MODE == EXP_DRIFT
        experimentDrift();
    #endif

    lcd.clear(); lcd.print("ALL TEST DONE!");
    Buzzer_Success();
}

void loop() {
    static uint32_t last = 0;
    if (millis() - last > 500) {
        last = millis();
        float w = readTotalWeight(8);
        lcd.setCursor(0, 0); lcd.print("Weight: "); lcd.print(w, 1); lcd.print("g   ");
    }
}