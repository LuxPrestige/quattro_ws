# GIM6010/GDS68 재감사 기록

## 감사 기준

2026-08-19 기준 GDS68 rev2.2 제조사 매뉴얼, 현재 코드, Xacro, bringup, calibration GUI와 테스트를 대조했다. 정적 감사 및 가상 검증 기록이며 실기 검증 완료 기록이 아니다.

## Critical

| 파일 | 문제 | 위험 | 수정 |
|---|---|---|---|
| `quattro_system.cpp` | MIT를 ROS position command 뒤에 숨김 | q/v/Kp/Kd/τ 의미가 소실됨 | MIT는 5개 전용 command interface를 모두 claim해야 하며 기본 gait controller와 연결하지 않음 |
| `quattro_system.cpp` | 초기 command limit 검사를 조건부로 비활성화 | 범위 밖 목표 송신 가능 | 첫 write부터 모든 범위와 finite 값 거부 |
| `can_socket.cpp` | CAN error frame 폐기 | bus-off가 stale feedback처럼 보임 | error filter와 warning/passive/bus-off/ACK/protocol/TRX/TX/RX 진단 추가 |

## High

| 파일 | 문제 | 위험 | 수정 |
|---|---|---|---|
| `gim6010_motor.cpp` | 현재 control mode 미추적 | 잘못된 command family 송신 | mode state와 mode별 command guard 추가 |
| `quattro_system.cpp`, calibration GUI | startup에서 자동 `Clear_Errors` | fault evidence 소실 | 자동 clear 제거, fault capture 후 명시적 recovery만 허용 |
| `quattro_system.cpp` | mode/limit 설정 후 feedback 확인 | 장치 상태 미확인 상태에서 구성 변경 | communication/encoder/heartbeat/fault preflight 후 runtime mode/gain 설정 |
| `quattro_system.cpp` | enable 순간 Direct Position target 미보장 | 저장된 target과 현재 위치 오차로 급동작 | enable 전 current-position target 준비, enable 후 재송신 및 feedback 확인 |
| calibration GUI | enable 결과와 fault 미확인 | 부분 enable 및 fault 상태 이동 | idle/fault preflight와 closed-loop 확인 추가 |

## Medium

| 파일 | 문제 | 위험 | 수정 |
|---|---|---|---|
| `gds68_protocol.*` | MIT 중심 API, 일반 controller gain 미구현 | Position controller를 정확히 구성 불가 | Direct/Filter/Trajectory mode, `0x1A`, `0x1B`, trajectory limit encode 추가 |
| `quattro_system.cpp` | MIT gain 이름을 일반 gain처럼 사용 | 서로 다른 단위/의미 혼동 | `mit_kp/mit_kd`와 `PositionControlGains` 완전 분리 |
| `quattro_system.cpp` | 단일 scheduling delay 100 ms 즉시 fault | Linux/Docker jitter와 hardware fault 혼동 | 50 ms warning, 250 ms fault로 분리; feedback/heartbeat도 별도 timeout |
| `gim6010_motor.cpp` | encoder feedback effort를 0으로 표시 | 실제 측정처럼 오해 | torque 미확보 시 NaN 유지 |
| `motor_manager.cpp` | malformed/unknown frame 관찰 불가 | routing 문제 진단 곤란 | unknown node/command, malformed counters 추가 |

## Low

| 파일 | 문제 | 위험 | 수정 |
|---|---|---|---|
| 좌표 변환 | 식이 여러 위치에 중복 | 부호/offset 회귀 | `JointTransform`과 ±direction round-trip test 추가 |
| 운영 도구 | read-only 단일 모터 확인 도구 없음 | 처음부터 제어 프로그램 사용 유도 | `gim6010_diagnostic` 추가; enable/config/clear/save 없음 |

## 남은 실기 게이트

- 모든 GDS68 firmware가 heartbeat `0.5.13+` 형식인지 확인
- secondary encoder가 실제 firmware에서 선택된 feedback source인지 확인
- RTR `0x009`, `0x017` 응답과 error query를 단일 모터에서 확인
- CAN adapter/kernel 조합의 warning/passive/bus-off/ACK/error counter 확인
- Position 내부 gain의 실제 장치 값과 안정성 측정
- undervoltage/overvoltage/overcurrent/encoder/controller fault injection
- Raspberry Pi 5 + Docker에서 100 Hz jitter와 두 bus 장시간 부하 측정

위 항목 전에 12축 gait 시험으로 확대하지 않는다.
