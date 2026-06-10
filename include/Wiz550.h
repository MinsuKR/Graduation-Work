#ifndef WIZ550_H
#define WIZ550_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

bool Wiz550_Begin();
bool Wiz550_IsReady();
void Wiz550_ShowIp(LiquidCrystal_I2C& lcd);
bool Wiz550_SendRequest(const char* request,
                        LiquidCrystal_I2C& lcd,
                        const char* title,
                        char* reply,
                        size_t replySize,
                        uint32_t timeoutMs = 12000);
#endif
