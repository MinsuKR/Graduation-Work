#ifndef LCD_H
#define LCD_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

void LCD_Init(LiquidCrystal_I2C& lcdWeight, LiquidCrystal_I2C& lcdStatus);

void LCD_Msg(LiquidCrystal_I2C& lcd, const char* line1,const char* line2);
void LCD_Msg(LiquidCrystal_I2C& lcd, const __FlashStringHelper* line1, const __FlashStringHelper* line2);
void LCD_Msg(LiquidCrystal_I2C& lcd, const __FlashStringHelper* line1, const char* line2);
void LCD_Msg(LiquidCrystal_I2C& lcd, const char* line1, const __FlashStringHelper* line2);

#endif
