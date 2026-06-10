#ifndef CALIB_H
#define CALIB_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>
#include "Dual_Scale.h"

bool DoCalibration(DualScale& scale, LiquidCrystal_I2C& lcd);

#endif