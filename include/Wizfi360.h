#ifndef WIZFI360_H
#define WIZFI360_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

bool Wizfi360_Begin(LiquidCrystal_I2C& lcd);
bool Wizfi360_IsReady();
void Wizfi360_ShowIp(LiquidCrystal_I2C& lcd);
bool Wizfi360_SendRequest(const char* request,
                          LiquidCrystal_I2C& lcd,
                          const char* title,
                          char* reply,
                          size_t replySize,
                          uint32_t timeoutMs = 12000);
#endif
