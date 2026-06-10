#ifndef PIN_MAP_H
#define PIN_MAP_H

#include <Arduino.h>

// ATmega328P-AU TQFP on Arduino Uno pin mapping.
#define PIN_BUZZER      8   // PB0

#define PIN_WIZ_RST     9   // PB1
#define PIN_WIZ_CS      10  // PB2 / SS

#define PIN_WIFI_RX     0   // PD0 / RXD, MCU RX <- WizFi360-C TX
#define PIN_WIFI_TX     1   // PD1 / TXD, MCU TX -> WizFi360-C RX

#define PIN_SW_SEND     2   // PD2 / INT0

// PCB is mounted 180 degrees rotated, so physical L/R load-cell inputs are swapped.
// These names follow the physical scale position used by firmware and LCD display.
#define PIN_HX_L_SCK    A0  // PC0 / ADC0, physical left load cell via PCB R input
#define PIN_HX_L_DT     A1  // PC1 / ADC1, physical left load cell via PCB R input

#define PIN_LED_B       5   // PD5, PCB LED_B
#define PIN_LED_G       6   // PD6, PCB LED_G
#define PIN_LED_R       7   // PD7, PCB LED_R

#define PIN_HX_R_SCK    3   // PD3, physical right load cell via PCB L input
#define PIN_HX_R_DT     4   // PD4, physical right load cell via PCB L input

#define PIN_SW_TARE     A2  // PC2 / ADC2
#define PIN_SW_CAL      A3  // PC3 / ADC3

#define PIN_LCD_SDA     A4  // PC4 / SDA
#define PIN_LCD_SCL     A5  // PC5 / SCL

#endif
