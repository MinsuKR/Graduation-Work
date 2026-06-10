#ifndef MEAT_ANALYSIS_H
#define MEAT_ANALYSIS_H

#include <Arduino.h>
#include <LiquidCrystal_I2C.h>

struct MeatAnalysisResult {
  char grade[8];
  char meatType[16];
  char cut[16];
  bool valid;
};

void MeatAnalysis_Clear(MeatAnalysisResult& result);
void MeatAnalysis_BuildCaptureRequest(char* out, size_t outSize, float weight_g);
bool MeatAnalysis_ParseResponse(const char* reply, MeatAnalysisResult& result);
void MeatAnalysis_ShowResult(LiquidCrystal_I2C& lcd, const MeatAnalysisResult& result, float weight_g);
void MeatAnalysis_GetTypeCode(const MeatAnalysisResult& result, char* out, size_t outSize);

#endif
