#ifndef CONFIG_H
#define CONFIG_H

#include <Arduino.h>
#include <stdint.h>

#include "PinMap.h"

// LCD
#define LCD_WEIGHT_ADDR 0x3F
#define LCD_STATUS_ADDR 0x27
#define LCD_COLS 16
#define LCD_ROWS 2

// HX711 pins. PinMap.h already maps physical L/R after PCB rotation.
#define DT_L  PIN_HX_L_DT
#define SCK_L PIN_HX_L_SCK
#define DT_R  PIN_HX_R_DT
#define SCK_R PIN_HX_R_SCK

// Measurement constants
#define STABLE_SAMPLING 10
#define AVG_TARE_SAMPLES 25
#define AVG_CAL_SAMPLES 25
#define AVG_DISPLAY_SAMPLES 1
#define AVG_MEAS_SAMPLES 5
#define BUTTON_WEIGHT_IGNORE_MS 500UL
#define BUTTON_RECOVERY_SAMPLES 10
#define STABLE_DIFF_MAX 500L

// Display/calibration constants
#define ZERO_SNAP_G 2.0f
#define DISPLAY_HOLD_BAND_G 1.0f
#define CAL_WEIGHT 155.0f
#define CAL_DET_REL_EPS 0.02f
#define OFFSET_LIMIT 10000000L
#define DEFAULT_FACTOR_L 0.004932f
#define DEFAULT_FACTOR_R 0.005406f
#define FACTOR_ABS_MIN 1e-6f
#define FACTOR_ABS_MAX 1.0f

// Auto zero tracking
#define AUTO_ZERO_BAND_G 3
#define AUTO_ZERO_NO_MOVE_MS 10000UL
#define AUTO_ZERO_MOVE_THRESHOLD_G 2
#define AUTO_ZERO_MIN_INTERVAL_MS 15000UL
#define AUTO_ZERO_STABLE_CHECK_MS 1000UL
#define AUTO_ZERO_UI_RETURN_MS 800UL

#endif
