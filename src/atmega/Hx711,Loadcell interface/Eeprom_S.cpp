#include "Eeprom_S.h"
#include <math.h>
#include <EEPROM.h>

// EEPROM 주소(원 코드 그대로)
// #define으로 사용해도 되나, 타입x, 텍스트로 치환되어 버그 발생 + 디버거에서 확인 불가
// C에선 #define & C++에선 const
static const int EE_FACTOR_L = 0;
static const int EE_FACTOR_R = 4;
static const int EE_A_COEF   = 8;
static const int EE_B_COEF   = 12;
static const int EE_TARE_L   = 16;
static const int EE_TARE_R   = 20;

namespace EepromStore {

    void LoadAll(DualScale& scale)
    {
        EEPROM.get(EE_FACTOR_L, scale.factorL);
        EEPROM.get(EE_FACTOR_R, scale.factorR);
        EEPROM.get(EE_A_COEF,   scale.a_coef);
        EEPROM.get(EE_B_COEF,   scale.b_coef);
        EEPROM.get(EE_TARE_L,   scale.offsetL);
        EEPROM.get(EE_TARE_R,   scale.offsetR);

        // 유효범위 체크 -> 이상 시 기본 값 복구
        if (!isfinite(scale.factorL) || fabs(scale.factorL) < 1e-6) {
            scale.factorL = DEFAULT_FACTOR_L;
        } else {
            scale.factorL = fabsf(scale.factorL);
        }

        if (!isfinite(scale.factorR) || fabs(scale.factorR) < 1e-6) {
            scale.factorR = DEFAULT_FACTOR_R;
        } else {
            scale.factorR = fabsf(scale.factorR);
        }
        if (!isfinite(scale.a_coef)) scale.a_coef = 1.0f;
        if (!isfinite(scale.b_coef)) scale.b_coef = 1.0f;
        if (scale.offsetL < -OFFSET_LIMIT || scale.offsetL > OFFSET_LIMIT) scale.offsetL = 0;
        if (scale.offsetR < -OFFSET_LIMIT || scale.offsetR > OFFSET_LIMIT) scale.offsetR = 0;
    
        scale.a_coef = 1.0f;
        scale.b_coef = 1.0f;
    }
    
    void SaveAll(const DualScale& scale)
    {
        
        EEPROM.put(EE_FACTOR_L, scale.factorL);
        EEPROM.put(EE_FACTOR_R, scale.factorR);
        EEPROM.put(EE_A_COEF,   scale.a_coef);
        EEPROM.put(EE_B_COEF,   scale.b_coef);
        EEPROM.put(EE_TARE_L,   scale.offsetL);
        EEPROM.put(EE_TARE_R,   scale.offsetR);
    }

}
