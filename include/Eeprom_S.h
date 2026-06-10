#ifndef EEPROM_S_H
#define EEPROM_S_H

#include <Arduino.h>
#include "Dual_Scale.h"
#include "Config.h"

// namespace 쓰는 이유?
// 함수들이 뭐에 대한 Load/Save인지 명확하게 하기 위해 + 이름 충돌 위험
namespace EepromStore {

  // === EEPROM에서 저장해둔 보정값을 읽어오는 함수 ===
  void LoadAll(DualScale& scale);

  // === 현재 장치의 보정값들을 EEPROM(영구메모리)에 저장하는 함수 ===
  void SaveAll(const DualScale& scale);

}

#endif