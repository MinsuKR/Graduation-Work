#include "MeatAnalysis.h"

#include <ctype.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

static void trimToken(char* text)
{
  if (!text || !text[0]) return;

  size_t start = 0;
  while (text[start] == ' ' || text[start] == '\t') {
    start++;
  }

  size_t len = strlen(text + start);
  memmove(text, text + start, len + 1);

  while (len > 0) {
    char c = text[len - 1];
    if (c == ' ' || c == '\t' || c == '\r' || c == '\n') {
      text[len - 1] = '\0';
      len--;
    } else {
      break;
    }
  }
}

static bool extractValue(const char* src, const char* key, char* out, size_t outSize)
{
  if (!src || !key || !out || outSize == 0) return false;

  const char* found = strstr(src, key);
  if (!found) return false;

  found += strlen(key);
  if (*found == '=') {
    found++;
  }

  size_t idx = 0;
  while (*found && *found != ',' && *found != ';' && *found != '\r' && *found != '\n') {
    if (idx < outSize - 1) {
      out[idx++] = *found;
    }
    found++;
  }
  out[idx] = '\0';
  trimToken(out);
  return out[0] != '\0';
}

static bool extractValueAnyCase(const char* src, char key, char* out, size_t outSize)
{
  char upperKey[3] = { key, '=', '\0' };
  char lowerKey[3] = { (char)tolower(key), '=', '\0' };

  if (extractValue(src, upperKey, out, outSize)) {
    return true;
  }

  return extractValue(src, lowerKey, out, outSize);
}

static void copyText(char* out, size_t outSize, const char* text)
{
  if (!out || outSize == 0) return;

  if (!text) {
    out[0] = '\0';
    return;
  }

  strncpy(out, text, outSize - 1);
  out[outSize - 1] = '\0';
}

void MeatAnalysis_Clear(MeatAnalysisResult& result)
{
  memset(&result, 0, sizeof(result));
  strncpy(result.grade, "?", sizeof(result.grade) - 1);
  strncpy(result.meatType, "?", sizeof(result.meatType) - 1);
  strncpy(result.cut, "?", sizeof(result.cut) - 1);
  result.valid = false;
}

void MeatAnalysis_BuildCaptureRequest(char* out, size_t outSize, float weight_g)
{
  if (!out || outSize == 0) return;

  int weightInt = (int)lroundf(weight_g);
  snprintf(out, outSize, "CAPTURE,W=%d\n", weightInt);
}

bool MeatAnalysis_ParseResponse(const char* reply, MeatAnalysisResult& result)
{
  MeatAnalysis_Clear(result);
  if (!reply || !reply[0]) return false;

  char grade[8];
  char meatType[16];
  char cut[16];
  grade[0] = '\0';
  meatType[0] = '\0';
  cut[0] = '\0';

  bool hasGrade = extractValueAnyCase(reply, 'G', grade, sizeof(grade));
  bool hasMeatType = extractValueAnyCase(reply, 'M', meatType, sizeof(meatType));
  bool hasCut = extractValueAnyCase(reply, 'C', cut, sizeof(cut));

  if (!hasGrade || !hasMeatType || !hasCut) {
    return false;
  }

  strncpy(result.grade, grade, sizeof(result.grade) - 1);
  strncpy(result.meatType, meatType, sizeof(result.meatType) - 1);
  strncpy(result.cut, cut, sizeof(result.cut) - 1);
  result.valid = true;
  return true;
}

void MeatAnalysis_GetTypeCode(const MeatAnalysisResult& result, char* out, size_t outSize)
{
  if (!out || outSize == 0) return;

  if (strncasecmp(result.meatType, "BEEF", 4) == 0) {
    copyText(out, outSize, "BF");
    return;
  }

  if (strncasecmp(result.meatType, "PORK", 4) == 0) {
    copyText(out, outSize, "PK");
    return;
  }

  copyText(out, outSize, result.meatType);
}

void MeatAnalysis_ShowResult(LiquidCrystal_I2C& lcd, const MeatAnalysisResult& result, float weight_g)
{
  char line1[17];
  char line2[17];
  char typeCode[4];
  int weightInt = (int)lroundf(weight_g);

  MeatAnalysis_GetTypeCode(result, typeCode, sizeof(typeCode));
  snprintf(line1, sizeof(line1), "W:%d G:%s", weightInt, result.grade);
  snprintf(line2, sizeof(line2), "M:%s C:%s", typeCode, result.cut);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(line1);
  lcd.setCursor(0, 1);
  lcd.print(line2);
}
