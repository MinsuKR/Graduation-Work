# ATmega328P-AU 포팅 및 네트워크 통신 작업 로그

작성일: 2026-05-18  
프로젝트: Graduation-Work  
대상 MCU: ATmega328P-AU, Arduino Uno 핀맵 기준  

## 1. 파일 구조 확인

처음 확인한 주요 구조는 다음과 같다.

- `src/atmega`: ATmega 펌웨어 본체
- `include`: ATmega 쪽 헤더 파일
- `src/esp32cam`: ESP32-CAM 관련 코드
- `raspberry_pi`: 라즈베리파이 TCP 서버 및 카메라 코드
- `platformio.ini`: PlatformIO 환경 설정
- `Individual code`: 실험 코드 및 백업 코드

현재 ATmega 주요 파일:

- `src/atmega/main.cpp`
- `src/atmega/Wiz550.cpp`
- `src/atmega/Wizfi360.cpp`
- `src/atmega/MeatAnalysis.cpp`
- `include/PinMap.h`
- `include/Config.h`
- `include/Wiz550.h`
- `include/Wizfi360.h`

## 2. ATmega128에서 ATmega328P-AU로 포팅

기존 ATmega128/MegaCore 기반 설정을 ATmega328P-AU 새 PCB 기준으로 변경했다.

### platformio.ini 변경

기존:

```ini
[env:ATmega128]
platform = atmelavr
board = ATmega128
framework = arduino
upload_protocol = usbasp
board_build.f_cpu = 8000000L
board_build.core = megacore
```

변경 후:

```ini
[env:uno_usbasp]
platform = atmelavr
board = uno
framework = arduino
upload_protocol = usbasp
upload_flags = -e
board_build.f_cpu = 16000000L
monitor_speed = 115200
```

주의 사항:

- 새 PCB는 ATmega328P-AU TQFP이지만 Arduino Uno 핀맵 기준으로 사용한다.
- `board_build.core = megacore`는 제거했다.
- `build_src_filter`는 기존처럼 `+<atmega/>`만 컴파일하도록 유지했다.
- `Ethernet3`는 PlatformIO 레지스트리에 `1.5.3`이 없어서 GitHub 태그로 고정했다.

```ini
https://github.com/sstaub/Ethernet3.git#1.5.3
```

## 3. 새 PCB 핀맵 정리

핀 정의가 여러 헤더와 소스에 흩어져 있어서 `include/PinMap.h`를 새로 만들고 ATmega328P-AU / Arduino Uno 기준 핀맵을 모았다.

주요 핀맵:

```cpp
#define PIN_BUZZER      8

#define PIN_WIZ_RST     9
#define PIN_WIZ_CS      10

#define PIN_WIFI_RX     0
#define PIN_WIFI_TX     1

#define PIN_SW_SEND     2

#define PIN_HX_L_SCK    3
#define PIN_HX_L_DT     4

#define PIN_LED_R       5
#define PIN_LED_G       6
#define PIN_LED_B       7

#define PIN_HX_R_SCK    A0
#define PIN_HX_R_DT     A1

#define PIN_SW_TARE     A2
#define PIN_SW_CAL      A3

#define PIN_LCD_SDA     A4
#define PIN_LCD_SCL     A5
```

수정한 헤더:

- `include/Config.h`
- `include/Button.h`
- `include/LED.h`
- `include/Buzzer.h`

## 4. Serial / UART 구조 정리

ATmega328P는 하드웨어 UART가 1개뿐이므로 D0/D1은 WizFi360-C 전용으로 유지했다.

수정 내용:

- `main.cpp`의 디버그용 `Serial.begin()` 제거
- PC 시리얼 모니터 출력용 `Serial.print()` 추가하지 않음
- `Wizfi360.cpp`에서만 `Serial`을 AT 명령 통신용으로 사용

현재 구조:

```cpp
#define WIFI_SERIAL Serial
```

## 5. WIZ550io / Ethernet3 포팅

WIZ550io 핀:

- CS: D10
- RST: D9
- MOSI: D11
- MISO: D12
- SCLK: D13

`Wiz550.cpp`에서 `PIN_WIZ_CS`, `PIN_WIZ_RST`를 사용하도록 변경했다.

초기화 시 W5500 reset pulse를 추가했다.

```cpp
pinMode(ETH_RST_PIN, OUTPUT);
digitalWrite(ETH_RST_PIN, LOW);
delay(20);
digitalWrite(ETH_RST_PIN, HIGH);
delay(150);
```

Ethernet3 기준 `begin()` 인자 순서는 다음으로 유지했다.

```cpp
Ethernet.begin(g_mac, g_localIp, g_subnet, g_gateway, g_dns);
```

주의:

- Arduino 공식 `Ethernet.h`와 인자 순서가 다르다.
- `Ethernet3`에서는 현재 순서가 맞다.

## 6. SRAM / Flash 용량 대응

ATmega328P는 SRAM 2KB, Flash 약 32KB라서 메모리 여유가 매우 작았다.

조치:

- LCD 고정 문자열을 `F()` 매크로로 이동
- `LCD_Msg()`에 `__FlashStringHelper` overload 추가
- Wi-Fi 수신 버퍼 크기 축소
- 사용하지 않는 보조 전송 함수 일부 제거
- 파싱 실패 시 `W=...g` 보조 표시 제거

빌드 결과 예:

```text
RAM:   약 1515 / 2048 bytes
Flash: 약 32142 / 32256 bytes
```

Flash 여유가 매우 적으므로 이후 기능 추가 시 주의가 필요하다.

## 7. W5500 통신 문제 분석

초기 증상:

- W5500은 한 번 통신되다가 다음 SEND에서 TCP 실패
- 실패 후 Wi-Fi fallback으로 넘어감
- 리셋하면 W5500이 다시 한 번 됨

분석:

- W5500이 완전히 고장난 문제가 아니라 TCP 연결 관리 문제로 판단했다.
- 성공 후 `client.stop()`으로 닫으면 다음 재접속이 불안정해지는 현상이 있었다.
- 라즈베리 서버도 정상 응답 후 연결을 닫고 있어서 W5500 지속 연결 구조와 맞지 않았다.

최종 방향:

- W5500은 성공하면 TCP 연결을 닫지 않고 유지
- 다음 SEND에서도 연결이 살아 있으면 W5500을 계속 사용
- W5500 링크/전송/응답 실패 시에만 `client.stop()`
- 실패한 경우 Wi-Fi fallback 시도
- 랜선이 다시 연결되면 다음 SEND에서 W5500을 다시 우선 시도

현재 의도한 흐름:

```text
SEND 버튼
→ W5500 링크 확인
→ W5500 연결이 살아 있으면 그대로 사용
→ W5500 성공 시 연결 유지
→ W5500 실패 시 client.stop()
→ Wi-Fi fallback
→ 이후 랜선 재연결 시 다음 SEND에서 W5500 우선 재시도
```

## 8. Wi-Fi / WizFi360-C 통신 문제 분석

초기 증상:

- W5500이 실패한 뒤 Wi-Fi도 안 되는 경우가 있었다.
- Wi-Fi가 되는 경우에도 W5500과 번갈아 쓰면 연결이 꼬이는 느낌이 있었다.

수정한 주요 내용:

- `AT+CIPSTART` 결과 판정에서 단순 `OK`를 성공으로 보지 않도록 수정

기존:

```cpp
waitForAny("CONNECT", "ALREADY", "OK", 10000);
```

변경:

```cpp
waitForAny("CONNECT", "ALREADY", nullptr, 10000);
```

이유:

- `OK`만 보고 TCP 연결 성공으로 판단하면 실제 연결 전 `AT+CIPSEND`가 실패할 수 있다.

## 9. W5500 / Wi-Fi 우선순위

현재 메인 흐름은 W5500 우선이다.

```text
SEND 버튼
→ W5500 먼저 시도
→ W5500 성공 시 결과 표시
→ W5500 실패 시 Wi-Fi 시도
→ Wi-Fi 성공 시 결과 표시
→ 둘 다 실패 시 SEND/RX FAIL
```

즉 W5500과 Wi-Fi가 둘 다 가능한 상태라면 W5500이 우선이다.

## 10. 라즈베리 TCP 서버 수정

파일:

- `raspberry_pi/meat_ai_server.py`

수정 이유:

- W5500 연결을 유지하려면 라즈베리 서버도 정상 응답 후 연결을 바로 닫으면 안 된다.
- 이전에는 정상 응답 후 `conn.close()`를 호출해서 W5500 지속 연결이 불가능했다.

최종 방향:

- 정상 응답 후에는 연결 유지
- 클라이언트가 끊기거나 새 연결이 들어오면 기존 연결 정리
- 에러 발생 시에는 연결 정리

주의:

- 라즈베리 서버 코드를 수정한 뒤에는 반드시 서버 스크립트를 재시작해야 적용된다.

## 11. 전원 인가 직후 SEND 문제

전원 인가 직후 바로 SEND를 누르면 W5500과 Wi-Fi가 모두 실패할 수 있다.

이유:

- W5500 PHY/link up 시간 필요
- 공유기 포트 링크 협상 시간 필요
- WizFi360-C 모듈 부팅 시간 필요
- AP 접속 및 IP 할당 시간 필요

실사용 권장:

```text
전원 인가 후 5~10초 정도 대기
LCD에 기본 대기 화면 또는 네트워크 준비 화면 확인
그 뒤 SEND 버튼 사용
```

## 12. 이전 동작 코드 백업

요청에 따라 TCP 닫기/재연결 실험 전 상태를 별도 백업했다.

위치:

```text
Individual code/Gradu260303/before_tcp_close_changes
```

저장한 파일:

- `main.cpp`
- `Wiz550.cpp`
- `Wiz550.h`
- `Wizfi360.cpp`
- `Wizfi360.h`
- `README.txt`

## 13. 앞으로 로그 작성 방식

앞으로 추가 질문이나 수정이 있을 때 이 폴더에 계속 한글 로그를 남긴다.

기본 규칙:

- 어떤 문제가 있었는지 기록
- 어떤 파일을 수정했는지 기록
- 수정한 이유 기록
- 최종 동작 흐름 기록
- 빌드 결과가 있으면 RAM/Flash 사용량 기록

로그 위치:

```text
Individual code/logfile
```

## 14. 2026-05-21 로드셀 좌/우 표시 반전 수정

문제:

- 물리적 좌측 로드셀을 누르면 LCD의 `R` 값이 변했다.
- 물리적 우측 로드셀을 누르면 LCD의 `L` 값이 변했다.

원인:

- PCB가 계획과 다르게 180도 회전되어 장착되었다.
- 물리적 좌측 로드셀 신호가 PCB의 R 입력으로 들어가고, 물리적 우측 로드셀 신호가 PCB의 L 입력으로 들어가는 상태였다.

수정 파일:

- `include/Config.h`

수정 방식:

- `DualScale` 내부 계산, EEPROM 저장 구조, LCD 표시 코드는 그대로 유지했다.
- 소프트웨어 기준 `L` 채널이 PCB의 R HX711 핀을 읽도록 변경했다.
- 소프트웨어 기준 `R` 채널이 PCB의 L HX711 핀을 읽도록 변경했다.

변경 후 매핑:

```cpp
#define DT_L   PIN_HX_R_DT
#define SCK_L  PIN_HX_R_SCK

#define DT_R   PIN_HX_L_DT
#define SCK_R  PIN_HX_L_SCK
```

의도한 최종 동작:

```text
물리적 좌측 로드셀 누름
→ PCB R 입력으로 들어옴
→ 소프트웨어 L 값으로 처리
→ LCD L 값 변경

물리적 우측 로드셀 누름
→ PCB L 입력으로 들어옴
→ 소프트웨어 R 값으로 처리
→ LCD R 값 변경
```

빌드 결과:

```text
SUCCESS
RAM:   1515 / 2048 bytes, 74.0%
Flash: 32142 / 32256 bytes, 99.6%
```

추가 정리:

- 처음에는 `Config.h`에서 `DT_L = PIN_HX_R_DT`처럼 직접 swap했다.
- 하지만 코드 가독성이 떨어져 헷갈릴 수 있으므로 swap 위치를 `PinMap.h`로 옮겼다.
- 이제 `Config.h`는 다시 직관적인 형태를 사용한다.

```cpp
#define DT_L   PIN_HX_L_DT
#define SCK_L  PIN_HX_L_SCK

#define DT_R   PIN_HX_R_DT
#define SCK_R  PIN_HX_R_SCK
```

- 실제 물리 좌/우 보정은 `PinMap.h`의 `PIN_HX_L_*`, `PIN_HX_R_*` 정의에서 처리한다.

최종 의미:

```text
PIN_HX_L_* = 물리적 좌측 로드셀
PIN_HX_R_* = 물리적 우측 로드셀
```

재빌드 결과:

```text
SUCCESS
RAM:   1515 / 2048 bytes, 74.0%
Flash: 32142 / 32256 bytes, 99.6%
```

## 15. 2026-05-21 LED RGB 핀맵 수정

문제:

- 코드상 LED 핀 정의는 다음처럼 되어 있었다.

```text
PD5(D5) = LED_R
PD6(D6) = LED_G
PD7(D7) = LED_B
```

- 하지만 실제 PCB 배선은 다음과 같았다.

```text
PD5(D5) = LED_B
PD6(D6) = LED_G
PD7(D7) = LED_R
```

수정 파일:

- `include/PinMap.h`
- `src/atmega/main.cpp`

수정 후 핀맵:

```cpp
#define PIN_LED_B       5
#define PIN_LED_G       6
#define PIN_LED_R       7
```

LED 상태 의미:

```text
LEDSTATE_PROCESSING = LED_B, 초기화/연결/전송/분석 등 동작 중
LEDSTATE_ERROR      = LED_R, 연결 실패/오류/전송 실패 등 실패 상태
LEDSTATE_IDLE       = LED_G, 정상/대기 상태
```

추가 보강:

- SEND 버튼 처리 중 무게 읽기 실패 시 `LEDSTATE_ERROR`를 켜도록 수정했다.
- W5500/Wi-Fi 모두 실패해 최종 `SEND/RX FAIL`이 되는 경우 `LEDSTATE_ERROR`를 켜도록 수정했다.

빌드 결과:

```text
SUCCESS
RAM:   1515 / 2048 bytes, 74.0%
Flash: 32172 / 32256 bytes, 99.7%
```

## 16. 2026-05-21 로드셀 표시 반응 속도 개선

문제:

- 물건을 올렸을 때 LCD 무게값이 즉시 올라가지 않고 천천히 따라가는 느낌이 있었다.
- 물건을 제거했을 때도 값이 서서히 내려가는 반응 지연이 있었다.

확인한 원인:

- 별도의 큰 이동평균 버퍼는 없었다.
- 다만 표시 측정에서 `AVG_MEAS_SAMPLES = 3`으로 HX711 값을 3회 평균내고 있었다.
- HX711이 10SPS 모드이면 3회 평균 자체가 체감 지연을 만들 수 있다.
- LCD 표시값에는 별도 동적 필터가 없어서, 샘플 평균과 표시 주기가 반응성을 결정하고 있었다.

수정 파일:

- `src/atmega/Config.cpp`
- `src/atmega/main.cpp`
- `platformio.ini`

수정 내용:

1. 표시용 측정 샘플 수를 줄였다.

```cpp
const uint8_t AVG_MEAS_SAMPLES = 1;
```

2. LCD 표시 단계에 동적 필터를 추가했다.

동작 방식:

```text
큰 무게 변화가 감지되면 즉시 현재값으로 표시
작은 변화만 있을 때는 0.1g 단위 정수 필터로 천천히 안정화
0.2g 이하의 미세 변화는 표시값 고정
```

적용 기준:

```text
5.0g 이상 변화 → 즉시 반영
0.2g 이하 변화 → 표시 고정
그 사이 변화 → diff / 3 만큼 보정
```

3. Tare 또는 Calibration 후에는 이전 표시 필터값이 남지 않도록 `displayFilterReady = false`로 초기화했다.

4. ATmega328P Flash 용량 초과가 발생해서 AVR linker relaxation을 켰다.

`platformio.ini`:

```ini
build_flags =
  -DSERIAL_TX_BUFFER_SIZE=16
  -DTWI_BUFFER_LENGTH=16
  -Wl,--relax
```

빌드 결과:

```text
SUCCESS
RAM:   1522 / 2048 bytes, 74.3%
Flash: 31778 / 32256 bytes, 98.5%
```

## 17. 2026-05-21 부팅 초기화 단계 LED 상태 흐름 수정

문제:

- 부팅 초반 WIZ550io 연결 실패 시 `LED_R`이 켜진 뒤 Wi-Fi 연결 시도 중에도 계속 빨강이 유지되었다.
- 사용자가 보기에는 Wi-Fi를 시도 중인지, 실패 상태인지 구분이 어려웠다.

수정 파일:

- `src/atmega/main.cpp`

수정한 LED 흐름:

```text
WIZ550io 연결 시도 중
→ LED_B

WIZ550io 연결 성공
→ LED_B 유지하면서 다음 Wi-Fi 초기화 단계 진행

WIZ550io 연결 실패
→ LED_R
→ 이후 Wi-Fi 연결 시도 직전 LED_B로 변경

Wi-Fi 연결 시도 중
→ LED_B

Wi-Fi 연결 성공
→ LED_B 유지

Wi-Fi 연결 실패
→ LED_R

무게 영점조절 / Auto Tare 시작
→ LED_B

전체 초기화 완료 후 대기 상태
→ LED_G
```

빌드 결과:

```text
SUCCESS
RAM:   1522 / 2048 bytes, 74.3%
Flash: 31822 / 32256 bytes, 98.7%
```

## 2026-05-21 - 불필요 코드 정리

요청 내용:

- 현재 코드에서 사용하지 않는 코드와 중복 정의를 제거.
- 동작 로직은 유지하고, 컴파일에 영향 없는 범위에서만 정리.

수정 파일:

- `include/Config.h`
- `src/atmega/Config.cpp`
- `include/Dual_Scale.h`
- `src/atmega/Dual_Scale.cpp`

정리 내용:

- 사용되지 않는 `SNAP_RANGE` 상수 선언/정의 제거.
- 사용되지 않는 `UiState` enum 제거.
- 사용되지 않는 `DualScale::measureLR()` 제거.
- 사용되지 않는 `DualScale::readNetAvg()` 제거.
- `Config.h`의 중복 `LCD_COLS`, `LCD_ROWS` 정의 제거.
- 깨져 보이던 주석을 간단한 ASCII 주석으로 정리.
- 보정에서 실제 사용하는 `measureNetLR()`와 일반 측정에서 사용하는 `readOnce()`는 유지.

빌드 결과:

```text
SUCCESS
RAM:   1522 / 2048 bytes, 74.3%
Flash: 31862 / 32256 bytes, 98.8%
```

## 2026-05-21 - 보정 factor 표시 부호 변경

요청 내용:

- `Calib.cpp` 보정 완료 화면에서 `factorL`, `factorR` 값이 `-`로 표시되어 헷갈림.
- LCD에는 양수처럼 보이도록 표시 변경.

수정 파일:

- `src/atmega/Hx711,Loadcell interface/Calib.cpp`

수정 내용:

- 실제 저장되는 `scale.factorL`, `scale.factorR` 값은 그대로 유지.
- 무게 계산 부호가 바뀌지 않도록 저장값은 건드리지 않고, 보정 완료 LCD 표시에서만 `-scale.factorL`, `-scale.factorR`를 출력.

빌드 결과:

```text
SUCCESS
RAM:   1522 / 2048 bytes, 74.3%
Flash: 31866 / 32256 bytes, 98.8%
```

## 2026-05-29 - 로드셀 A+/A- 신호선 반전 대응

상황:

- 로드셀 4선식 배선에서 A+ / A- 신호선이 기존과 반대로 연결됨.
- 기존: A+ = White, A- = Green
- 현재: A+ = Green, A- = White
- 무게를 올렸을 때 LCD 무게값이 음수로 표시됨.

원인:

- HX711의 DT/SCK MCU 핀 문제가 아니라, 로드셀 차동 입력 A+ / A- 극성이 바뀌어 raw 증가/감소 방향이 달라진 문제.
- 현재 기본 factor와 EEPROM factor는 양수로 관리되고 있으므로, 측정 raw 방향을 `raw - offset` 기준으로 맞춰야 함.

수정 파일:

- `src/atmega/Hx711,Loadcell interface/Dual_Scale.cpp`
- `platformio.ini`

수정 내용:

- 일반 측정 `readOnce()`에서 net raw 계산을 `offset - raw`에서 `raw - offset`으로 변경.
- 보정 측정 `measureNetLR()`에서도 동일하게 `raw - offset` 방향으로 변경.
- DT/SCK 핀 정의나 HX711 라이브러리 설정은 변경하지 않음. A+/A- 반전은 MCU 디지털 핀 문제가 아니라 HX711 차동 입력 극성 문제이기 때문.
- 플래시 용량 초과 방지를 위해 AVR 링크 최적화 옵션 `-Wl,--relax`를 `platformio.ini`에 추가.

빌드 결과:

```text
SUCCESS
RAM:   1515 / 2048 bytes, 74.0%
Flash: 31496 / 32256 bytes, 97.6%
```

## 2026-05-21 - LCD Weight/Status 위치 교체

요청 내용:

- PCB가 회전되어 LCD 두 개의 물리 위치도 서로 바뀜.
- 로드셀 L/R을 소프트웨어에서 맞춘 것처럼, LCD도 무게 표시용과 상태 표시용 역할을 서로 바꿈.

수정 파일:

- `include/Config.h`

수정 내용:

- LCD 표시 로직과 `main.cpp` 흐름은 그대로 유지.
- I2C 주소 매핑만 서로 교체.

```cpp
#define LCD_WEIGHT_ADDR 0x3F
#define LCD_STATUS_ADDR 0x27
```

결과:

- `lcdWeight`에 출력되는 L/R/TOT 무게 화면이 반대쪽 LCD로 표시됨.
- `lcdStatus`에 출력되는 W5500, Wi-Fi, Tare, Cal, Send 상태 화면이 반대쪽 LCD로 표시됨.

빌드 결과:

```text
SUCCESS
RAM:   1522 / 2048 bytes, 74.3%
Flash: 31862 / 32256 bytes, 98.8%
```
## 2026-06-06 LED NORMAL 상태명 변경 및 부저 용량 최적화

- 평상시 초록 LED 상태명을 `LEDSTATE_IDLE`에서 `LEDSTATE_NORMAL`로 변경했다.
- `include/LED.h`, `src/atmega/Ui interface/LED.cpp`, `src/atmega/main.cpp`의 호출부를 모두 `LEDSTATE_NORMAL` 기준으로 맞췄다.
- 변경 후 빌드 시 Flash 용량을 초과하여, 수동 부저 소리는 유지하되 Arduino `tone()/noTone()` 대신 D8(PB0)을 직접 약 2kHz로 토글하도록 `Buzzer.cpp`를 최적화했다.
- D8은 현재 PCB의 부저 핀이므로 `DDRB`, `PORTB`, `PB0` 직접 제어를 사용했다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1521/2048 bytes, Flash 32228/32256 bytes.

## 2026-06-01 ZERO SNAP 2g 변경 및 EMA 제거

- 빈 저울에서 내부값이 1g 근처로 흔들려 `TOT:1g`이 표시될 수 있어 `ZERO_SNAP_G`를 1.0f에서 2.0f로 변경했다.
- `TOT` 표시가 서서히 따라가는 원인이 되었던 EMA 필터는 현재 표시 로직에서 불필요하므로 제거했다.
- 현재 `TOT` 표시는 이전 표시 정수값 기준 ±1.0g 이내에서는 유지하고, 그 범위를 벗어나는 실제 무게 변화는 즉시 새 정수값으로 갱신한다.
- EMA 제거로 RAM/Flash 사용량이 조금 줄었다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1523/2048 bytes, Flash 30264/32256 bytes.

## 2026-06-01 LCD TOT 큰 무게 변화 즉시 반영 수정

- 이전 `TOT` 표시 안정화에서 EMA 필터가 먼저 적용되어 물건을 올릴 때 총 무게 표시가 서서히 따라가는 문제가 있었다.
- 표시값 기준 ±1.0g 이내의 작은 흔들림은 계속 고정하되, ±1.0g을 벗어나는 실제 무게 변화는 EMA를 기다리지 않고 즉시 새 정수값으로 갱신하도록 수정했다.
- 0g에서 155g 물체를 올리는 것처럼 큰 변화가 생기면 LCD `TOT`가 바로 155g 근처로 바뀌고, 이후 154~156g 범위의 소수점 흔들림은 기존 표시값을 유지한다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1527/2048 bytes, Flash 30394/32256 bytes.

## 2026-06-01 SEND 무게값 정수 g 전송으로 변경

- 버튼 외압 때문에 SEND 직후 순간값이 튈 수 있으므로 기존처럼 500ms 안정화 후 10샘플 평균을 먼저 구하는 흐름은 유지했다.
- 평균으로 얻은 `sendWeight`는 내부적으로 float이지만, 라즈베리파이 요청 문자열을 만들 때 정수 g로 반올림해 `CAPTURE,W=155` 형태로 전송하도록 수정했다.
- 라즈베리파이 응답 결과를 상태 LCD에 표시할 때도 `W:155`처럼 정수 g 기준으로 표시되도록 수정했다.
- LCD 실시간 `TOT` 표시, SEND 전송값, 결과 표시가 모두 정수 g 단위로 보여 시연 중 값이 서로 다르게 보이는 혼란을 줄였다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1527/2048 bytes, Flash 30380/32256 bytes.

## 2026-06-01 LCD TOT 정수 표시 히스테리시스 추가

- 졸업작품 시연 중 총 무게 `TOT` 값의 소수점 흔들림이 눈에 띄는 문제를 줄이기 위해 LCD 표시용 안정화 로직을 추가했다.
- 내부 무게 계산값(`ScaleReading.total`), CAL 보정, TARE 영점, SEND 전송값은 기존 float 계산 흐름을 유지했다.
- `Config.h`에 `DISPLAY_HOLD_BAND_G`, `DISPLAY_EMA_ALPHA`를 추가했다.
- `main.cpp`에 `displayFilteredTotal`, `displayTotalInt`, `displayTotalInitialized` 표시 전용 상태 변수를 추가했다.
- `GetStableDisplayTotal()` 함수에서 EMA 필터를 적용하고, 현재 LCD 표시 정수값 기준 ±1.0g 이내에서는 기존 표시값을 유지하도록 했다.
- LCD 무게 화면은 L/R 값은 기존처럼 소수점 1자리, TOT 값만 `TOT:155g`처럼 정수 g 단위로 표시되도록 수정했다.
- TARE/CAL 이후 이전 무게값의 표시 필터 잔상이 남지 않도록 표시용 필터만 초기화하는 `ResetStableDisplayTotal()`을 호출하도록 했다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1527/2048 bytes, Flash 31854/32256 bytes.

## 2026-06-01 버튼 입력 후 로드셀 표시/전송 안정화 처리

- 전면 버튼을 누를 때 저울 외관에 전달되는 순간 힘 때문에 LCD 무게값이나 SEND 전송값이 튀는 문제를 줄이기 위해 `BUTTON_WEIGHT_IGNORE_MS`, `BUTTON_RECOVERY_SAMPLES` 상수를 추가했다.
- `main.cpp`에 `weightIgnoreUntil`, `needWeightRecoveryAverage` 상태 변수를 추가했다.
- TARE, CAL, SEND 버튼 이벤트가 확정되면 `StartButtonWeightIgnore()`를 호출해서 약 500ms 동안 LCD 무게 표시 갱신을 멈추도록 했다.
- 500ms 안정화 시간이 끝난 뒤에는 기존 1샘플 표시 대신 10샘플 평균을 한 번 읽고 LCD 무게 표시를 재개하도록 했다.
- SEND 버튼은 누른 직후의 튄 무게를 보내지 않도록 500ms 대기 후 `BUTTON_RECOVERY_SAMPLES`만큼 평균 측정한 값을 `sendWeight`로 사용하게 했다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1520/2048 bytes, Flash 31662/32256 bytes.

## 2026-06-01 SEND 버튼 상태 LCD 즉시 반응 수정

- 버튼 안정화 대기 기능 추가 후 SEND 버튼을 눌렀을 때 상태 LCD가 500ms 뒤에 바뀌는 현상이 있었다.
- 원인은 SEND 처리에서 500ms 안정화 대기를 먼저 수행하고, 이후 W5500 전송 함수에 들어가면서 상태 LCD가 갱신되는 순서였기 때문이다.
- SEND 버튼 이벤트가 확정되자마자 `LCD_Msg(lcdStatus, F("SEND PRESSED"), F("Stabilizing..."))`를 먼저 표시하도록 수정했다.
- 이후 기존처럼 500ms 안정화 대기, 10샘플 평균 측정, W5500 우선 전송, Wi-Fi fallback 흐름은 유지했다.
- PlatformIO `uno_usbasp` 빌드 확인 결과 성공했다. RAM 1520/2048 bytes, Flash 31708/32256 bytes.
