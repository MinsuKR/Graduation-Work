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
#define EXP_POS_ERROR   1 // 위치 편차
#define EXP_REPEAT      2 // 반복 측정
#define EXP_CAL_STAB    3 // 캘 안정성 (추가됨)
#define EXP_DRIFT       4 // 드리프트 측정

// ★ 테스트하고 싶은 모드를 선택하세요 ★
#define EXPERIMENT_MODE EXP_CAL_STAB

/* =========================
   하드웨어 및 설정
   ========================= */
LiquidCrystal_I2C lcd(0x27, 16, 2);
HX711 scale;

#define DT_PIN   PIN_PD3
#define SCK_PIN  PIN_PD2
#define EEPROM_ADDR 0

const float CAL_WEIGHT = 155.0f;
const long  STABLE_DIFF_MAX = 200;
const uint8_t STABLE_SAMPLING = 10;

long  tare_offset = 0;
float calibration_factor = -0.095f; 

/* ==============================================
   핵심 측정 함수 (기존 코드의 정밀도 유지)
   ============================================== */

bool waitReady(uint16_t timeout_ms = 100) {
    uint32_t t0 = millis();
    while (millis() - t0 < timeout_ms) {
        if (scale.is_ready()) return true;
    }
    return false;
}

// 필터링된 Raw값 읽기 (최대/최소 제외 평균)
long readFilteredRaw(int samples) {
    long sum = 0;
    long minVal = 2147483647;
    long maxVal = -2147483648;
    for (int i = 0; i < samples; i++) {
        while(!scale.is_ready());
        long raw = scale.read();
        if (raw < minVal) minVal = raw;
        if (raw > maxVal) maxVal = raw;
        sum += raw;
        delay(5);
    }
    return (sum - minVal - maxVal) / (samples - 2);
}

float readTotalWeight(uint8_t samples) {
    long raw = readFilteredRaw(samples);
    return (float)(raw - tare_offset) * calibration_factor;
}

// 안정도 측정 함수
bool isStable(long* avg_out = nullptr) {
    long samples[STABLE_SAMPLING];
    for (int i = 0; i < STABLE_SAMPLING; i++) {
        if (!waitReady(300)) return false;
        samples[i] = scale.read();
    }
    long minV = samples[0], maxV = samples[0], sum = 0;
    for (int i = 0; i < STABLE_SAMPLING; i++) {
        if (samples[i] < minV) minV = samples[i];
        if (samples[i] > maxV) maxV = samples[i];
        sum += samples[i];
    }
    if (avg_out) *avg_out = sum / STABLE_SAMPLING;
    return (maxV - minV < STABLE_DIFF_MAX);
}

/* =========================
   실험 모드 함수들
   ========================= */

void experimentPositionError() {
    const char* posName[7] = {"C", "L", "LU","LD", "RU", "R", "RD"};
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

void experimentCalStability() {
    // 시리얼 헤더 출력 (엑셀 분석용)
    Serial.println("Cycle,Factor,Measured_Weight(g)");

    for (int i = 1; i <= 5; i++) {
        // --- 1단계: 진행 알림 화면 ---
        lcd.clear();
        delay(100); // LCD 컨트롤러가 화면을 완전히 비울 시간을 줍니다.
        
        lcd.setCursor(0, 0);
        lcd.print("Cal Cycle: "); lcd.print(i);
        lcd.setCursor(0, 1);
        lcd.print("Stabilizing...");
        
        // 센서 안정화 대기
        delay(3000); 
        
        // 2단계: 새로운 데이터 측정 및 계산
        long raw = readFilteredRaw(40);
        long net = raw - tare_offset;
        
        // 분모가 0이 되는 것을 방지
        if (abs(net) > 100) {
            calibration_factor = CAL_WEIGHT / (float)net;
        }
        
        // 방금 구한 Factor로 즉시 무게 측정
        float w = readTotalWeight(20);

        // 시리얼 전송
        Serial.print(i); Serial.print(","); 
        Serial.print(calibration_factor, 6); Serial.print(","); 
        Serial.println(w, 3);

        // --- 3단계: 결과 출력 화면 (잔상 완벽 제거) ---
        lcd.clear();
        delay(100); // 화면 전환을 위한 충분한 시간 확보

        // 첫 번째 줄: Factor 출력 (남은 칸은 공백으로 덮어씀)
        lcd.setCursor(0, 0);
        lcd.print("F:"); lcd.print(calibration_factor, 6);
        lcd.print("      "); 

        // 두 번째 줄: 측정 무게 출력
        lcd.setCursor(0, 1);
        lcd.print("W:"); lcd.print(w, 3); lcd.print("g");
        lcd.print("       "); 
        
        Buzzer_Click();
        
        // 결과를 확인할 수 있도록 4초 대기
        delay(4000); 
    }
    
    lcd.clear();
    lcd.print("Stability Done!");
    Buzzer_Success();
}

void experimentDrift() {
    Serial.println("Time(min),Weight(g)");
    unsigned long start = millis();
    unsigned long duration = 600000UL; 
    while (true) { 
        unsigned long elapsed = millis() - start;
        unsigned long t = elapsed / 60000UL; 
        if (t > 11) t = 11; 
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
    
    scale.begin(DT_PIN, SCK_PIN);
    scale.set_gain(128);
    LED_Set(LEDSTATE_IDLE);
    Buzzer_Click();

    // 1단계: 3초간 정밀 자동 영점 (Tare)
    lcd.clear(); lcd.print("1. PRECISION TARE");
    lcd.setCursor(0, 1); lcd.print("Stabilizing 3s..");
    
    // 1-1. 일단 값이 안정될 때까지 대기 (기존 로직 유지)
    long dummyRaw = 0;
    while (!isStable(&dummyRaw)) {
        lcd.setCursor(14, 1); lcd.print("..");
        delay(500);
    }

    // 1-2. [핵심] 3초 동안 정밀하게 샘플 수집하여 평균 내기
    long long tareSum = 0;
    int sampleCount = 0;
    uint32_t tareStartTime = millis();

    while (millis() - tareStartTime < 3000) { // 정확히 3초 동안 반복
        if (scale.is_ready()) {
            tareSum += scale.read();
            sampleCount++;
        }
        // 진행률 표시 (시각적 피드백)
        if (sampleCount % 10 == 0) {
            lcd.setCursor(13, 1); 
            lcd.print((millis() - tareStartTime) / 300); // 0~10 표시
        }
        delay(10); 
    }

    if (sampleCount > 0) {
        tare_offset = tareSum / sampleCount; // 3초간의 산술 평균
    } else {
        tare_offset = scale.read(); // 만약 실패 시 단일 값이라도 할당
    }

    Buzzer_Click();
    lcd.clear(); lcd.print("Tare OK!");
    lcd.setCursor(0, 1); lcd.print("Raw Offs:"); lcd.print(tare_offset);
    delay(1500);

    // 2단계: 자동 보정 (Calibration)
    lcd.clear(); lcd.print("2. AUTO CAL");
    lcd.setCursor(0, 1); lcd.print("PUT 155g (5s)");
    delay(5000);
    long raw = readFilteredRaw(40);
    long net = raw - tare_offset;
    if (abs(net) > 100) {
        calibration_factor = CAL_WEIGHT / (float)net;
        EEPROM.put(EEPROM_ADDR, calibration_factor);
        Buzzer_Success();
        lcd.clear(); lcd.print("CAL DONE!");
        lcd.setCursor(0, 1); lcd.print("F:"); lcd.print(calibration_factor, 6);
        delay(2000);
    } else {
        lcd.clear(); lcd.print("CAL ERROR!");
        Buzzer_Error();
        while(1); 
    }

    // 3단계: 무게추 제거 안내
    lcd.clear(); lcd.print("3. REMOVE WEIGHT");
    lcd.setCursor(0, 1); lcd.print("Wait 5s...");
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
    #elif EXPERIMENT_MODE == EXP_CAL_STAB
        experimentCalStability();
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
        lcd.setCursor(0, 0); lcd.print("Weight: "); 
        lcd.print(w, 1); lcd.print("g   ");
    }
}