/* Test complete */
// SDA = PD1 , SCL = PD0
#include <Wire.h>
#include <LiquidCrystal_I2C.h>

LiquidCrystal_I2C lcd(0x27, 16, 2);

char name[] = "Kim min su";

void setup() {
    lcd.init();
    lcd.backlight();
    lcd.clear();
    lcd.setCursor(0, 0);
    lcd.print("LCD Test");
}

void loop() {
    lcd.setCursor(0, 1);
    lcd.print(name);
}