#include <math.h>

#include "Dual_Scale.h"

void DualScale::begin(uint8_t dtL, uint8_t sckL, uint8_t dtR, uint8_t sckR)
{
  _L.begin(dtL, sckL);
  _R.begin(dtR, sckR);
  _L.set_gain(128);
  _R.set_gain(128);
}

bool DualScale::waitReady(uint16_t timeout_ms)
{
  uint32_t t0 = millis();
  while (millis() - t0 < timeout_ms) {
    if (_L.is_ready() && _R.is_ready()) return true;
    delay(1);
  }
  return false;
}

bool DualScale::isStable(int32_t* avg_out)
{
  int32_t samples[STABLE_SAMPLING];

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (!waitReady(300)) return false;
    int32_t rawL = (int32_t)_L.read();
    int32_t rawR = (int32_t)_R.read();
    samples[i] = rawL + rawR;
  }

  int32_t minV = samples[0];
  int32_t maxV = samples[0];
  int32_t sum = 0;

  for (int i = 0; i < STABLE_SAMPLING; i++) {
    if (samples[i] < minV) minV = samples[i];
    if (samples[i] > maxV) maxV = samples[i];
    sum += samples[i];
  }

  if (avg_out) *avg_out = sum / STABLE_SAMPLING;
  return ((maxV - minV) < STABLE_DIFF_MAX);
}

bool DualScale::tare(uint8_t avg)
{
  if (!waitReady(800)) return false;

  offsetL = _L.read_average(avg);
  offsetR = _R.read_average(avg);
  return true;
}

bool DualScale::measureNetLR(uint8_t samples, int32_t& outNetL, int32_t& outNetR)
{
  int64_t sumL = 0;
  int64_t sumR = 0;
  uint8_t got = 0;
  uint32_t t0 = millis();

  while (got < samples) {
    if ((uint32_t)(millis() - t0) > 3500UL) break;
    if (!waitReady(200)) continue;

    int32_t rawL = (int32_t)_L.read();
    int32_t rawR = (int32_t)_R.read();
    sumL += rawL;
    sumR += rawR;
    got++;
    delay(5);
  }

  if (got == 0) return false;

  int32_t avgRawL = (int32_t)(sumL / got);
  int32_t avgRawR = (int32_t)(sumR / got);

  // A+/A- swapped load-cell wiring now increases raw counts under load.
  // Keep factors positive by using raw - offset as the positive load direction.
  outNetL = avgRawL - offsetL;
  outNetR = avgRawR - offsetR;
  return true;
}

ScaleReading DualScale::readOnce(uint8_t samples)
{
  ScaleReading value{};
  value.valid = false;

  if (!waitReady(100)) return value;

  int32_t rawL = (int32_t)_L.read_average(samples);
  int32_t rawR = (int32_t)_R.read_average(samples);

  // A+/A- swapped load-cell wiring now increases raw counts under load.
  // Keep factors positive by using raw - offset as the positive load direction.
  int32_t netL = rawL - offsetL;
  int32_t netR = rawR - offsetR;

  float wL = (float)netL * factorL;
  float wR = (float)netR * factorR;

  float total = a_coef * wL + b_coef * wR;
  if (isnan(total) || isinf(total)) total = 0.0f;

  value.wL = wL;
  value.wR = wR;
  value.total = total;
  value.valid = true;
  return value;
}
