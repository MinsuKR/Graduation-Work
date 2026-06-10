// const & constexpr 차이?
// const = "값을 바꿀 수 없음" / constexpr = "컴파일 시간에 값이 확정됨" => 둘 다 값은 변하지 않지만 결정 시점이 다름
// constexpr 사용 이유? => 배열 크기, enum 등에 필요 & 메모리/속도 유리 => 절대 변하지 않음 + 컴파일 타임에 확정 가능 + 배열/조건/계산에 쓰임
// const 사용 => 값은 안바꾸지만 캘리브나 EEPROM에서 바뀔 가능성 있음 + 캘리브나 EEPROM에서 바뀔 가능성 있음

// static
// 하나만 존재한다 / 오래 산다 / 범위가 제한 / 값은 바뀜

// extern으로 선언?
// 이 변수(또는 객체)는 다른 파일(.cpp) 에 실제로 만들어져 있으니, 여기서는 존재한다고만 선언
// .cpp 파일은 각각 컴파일돼서 연결을 해줘야함
// extern은 여러 파일에 선언 상관 없지만 정의(즉, 값 설정)은 딱 한번만 해야함
// 선언(이런 이름/타입의 변수가 어딘가 존재)해줌 -> 따라서 main에서 사용 가능, 실제로는 cpp에 존재(값을 넣어줌)

// Config.h 역할
// “설정/상수/주소/핀” 모음 (값 정의)
// 즉, 하드웨어/동작 설정값(변하지 않는 값)만 모아두는 파일

// Scale.h 역할
// “프로그램 상태/공용 자원” 모음 (변하는 것 포함)
// 즉, 여러 모듈이 공유해야 하는 ‘현재 상태’와 ‘공용 함수’의 연결점

// #pragma once vs #ifndef/#define/#endif 차이
// 둘다 헤더를 "한 번만 include"하게 만드는 목적
// #pragma once
// 더 짧고 편함, 매크로 이름 관리 필요 없음
// 대부분 컴파일러에서 지원(Arduino/avr-gcc도 보통 문제 없음)

// #ifndef/#define/#endif 
// = 표준 C/C++ 전통 방식이라 거의 모든 컴파일러에서 확실하게 동작
// = 파일명이 같아도, 매크로 이름만 다르면 안전

// C++ "::"의미
// 범위 지정 연산자(이 이름이 어디에 속해있는지 알려주는 역할)
// 클래스에 속한 함수임을 명시할 때 (ScaleDual::isStable는 ScaleDual 클래스에 속한 isStable 함수라는 뜻)
// ex) class ScaleDual {
// public:
//     bool isStable(int32_t* avg_out);
// }; <= 선언
// bool ScaleDual::isStable(int32_t* avg_out){} <= 구현(.cpp에서는 "클래스명::함수명")

// struct 쓰는 이유
// 여러 개의 값을 하나의 묶음으로 돌려주기 위해 사용(함수 내부에서 사용하기 위한 타입 정의)
// struct(주 public) = 값 묶음
// class(주 private) = 행동(로직)
// namespace = 이름을 묶는 공간(정리)

// private의 의미
// 클래스 내부 구현을 외부로부터 숨기기 위한 접근 제한자(ScaleDual 내부에서만 접근 가능)
// public = 외부에서 “의도적으로 조절하거나 호출해야 하는 것들” 
// private = 내부 구현용 도구(알 필요없을 경우)

// Hx711 _L & Hx711 _R = "_" 이유
// = 의미 표시 네이밍

// if 다음 else 안쓰는 이유(가드 클로즈 스타일)
// if (실패) return;
// 정상 처리

// 객체 선언(asc)& asd 는 main에 있는 asc를 참조로 받아 사용한다는 뜻
// 

// class DualScale {} 👉 타입(설계도) 정의(DualScale이라는 타입은 이런 멤버와 함수로 구성)
// DualScale scale; 👉 실제 객체(실물) 생성
// DualScale& scale 👉 main에서 만든 그 객체를 참조(별명)로 받는 것 = main의 scale
// DualScale::asd 👉 객체 없이도 쓸 수 있는 static 멤버일 때만 가능(DualScale이라는 타입 자체에 소속된 것)