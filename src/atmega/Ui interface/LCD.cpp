#include "LCD.h"

void LCD_Init(LiquidCrystal_I2C& lcdWeight, LiquidCrystal_I2C& lcdStatus)
{
  lcdWeight.init();
  lcdWeight.backlight();

  lcdStatus.init();
  lcdStatus.backlight();
}

void LCD_Msg(LiquidCrystal_I2C& lcd, const char* line1,const char* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}

void LCD_Msg(LiquidCrystal_I2C& lcd, const __FlashStringHelper* line1, const __FlashStringHelper* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}

void LCD_Msg(LiquidCrystal_I2C& lcd, const __FlashStringHelper* line1, const char* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}

void LCD_Msg(LiquidCrystal_I2C& lcd, const char* line1, const __FlashStringHelper* line2)
{
  lcd.clear();
  lcd.setCursor(0, 0); lcd.print(line1);
  lcd.setCursor(0, 1); lcd.print(line2);
}
