#include <HX711.h>

HX711 scale;

#define DOUT 6   // HX711 DT
#define CLK  7   // HX711 SCK

void setup() {
  Serial.begin(9600);
  scale.begin(DOUT, CLK);

  // 첫 영점조정 (빈 저울에서 실행)
  scale.tare();

  // 보정값 설정 (추후 조정)
  scale.set_scale(-7050.0);
}

void loop() {
  if (scale.is_ready()) {
    long reading = scale.get_units(5); // 평균값 5회
    Serial.print("Weight: ");
    Serial.println(reading);
  } else {
    Serial.println("HX711 not ready");
  }
  delay(500);
}