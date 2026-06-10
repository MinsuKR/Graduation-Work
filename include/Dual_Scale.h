#ifndef DUAL_SCALE_H
#define DUAL_SCALE_H

#include <Arduino.h>
#include <HX711.h>

#include "Config.h"

struct ScaleReading {
  float wL;
  float wR;
  float total;
  bool valid;
};

class DualScale {
public:
  // HX711 init and gain setup.
  void begin(uint8_t dtL, uint8_t sckL, uint8_t dtR, uint8_t sckR);

  // Wait until both HX711 modules are ready.
  bool waitReady(uint16_t timeout_ms = 100);

  // Stability check based on rawL + rawR.
  bool isStable(int32_t* avg_out = nullptr);

  // Save current raw values as zero offsets.
  bool tare(uint8_t avg);

  // Calibration helper: averaged net raw values.
  bool measureNetLR(uint8_t samples, int32_t& outNetL, int32_t& outNetR);

  // One measurement for the main loop.
  ScaleReading readOnce(uint8_t samples);

  // Calibration/state values saved to EEPROM.
  float factorL = DEFAULT_FACTOR_L;
  float factorR = DEFAULT_FACTOR_R;
  int32_t offsetL = 0;
  int32_t offsetR = 0;
  float a_coef = 1.0f;
  float b_coef = 1.0f;

private:
  HX711 _L;
  HX711 _R;
};

#endif
