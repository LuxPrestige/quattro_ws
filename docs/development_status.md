# Quattro 개발 현황

## 목적

이 문서는 현재 구현 상태와 실기에서 확인된 사실, 다음 리팩터링 작업을 기록한다.

## 현재 핵심 전환점

기존 Quattro 하드웨어 코드는 `Direct Position + Closed Loop 전 encoder 읽기 + Set_Input_Pos(current)`를 Safe Start로 사용했다.

실기 검증 결과 이 전제는 폐기한다.

## GIM6010-8 startup 실기 확인 사항

다음은 현재 하드웨어 구현의 공식 전제로 사용한다.

### 1. 설정 순서

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Set_Controller_Mode(Position Control, Pos Filter)
→ Set_Axis_State(Closed Loop Control)
```

### 2. Closed Loop 이전 encoder 위치는 무효

Closed Loop 이전 `EncoderEstimate` 위치값은 startup 현재 위치로 신뢰할 수 없다.

따라서 다음 과거 구현은 제거 대상이다.

```text
pre-Closed-Loop encoder
→ current position 계산
→ Set_Input_Pos(current)
→ Closed Loop
```

### 3. EncoderEstimate 자동 broadcast

`Get_Encoder_Estimates(0x009)`는 약 10 ms 주기로 자동 송신하도록 운용한다.

Quattro 정상 경로에서는 encoder request를 보내지 않는다.

### 4. Closed Loop 자체 Hold

Position Control + Pos Filter 설정 후 Closed Loop 상태가 되면 별도 position command 없이도 모터 축이 현재 위치에서 고정된다.

따라서 startup에서 `Set_Input_Pos(current)`는 필요하지 않다.

### 5. startup 위치 동기화

Closed Loop가 Heartbeat로 확인된 뒤 도착한 새 EncoderEstimate를 최초 유효 위치로 사용한다.

이 값으로 ROS joint state와 첫 command 기준을 동기화한 뒤 controller에 제어권을 넘긴다.

## 기존 코드와의 충돌

현재 main 기준 다음 구현은 새 하드웨어 계약과 충돌한다.

- `quattro_system.cpp`
  - Closed Loop 전에 encoder 사용
  - `InputMode::kDirect`
  - startup `Set_Input_Pos(current)`
- `quattro_system.hpp`
  - pre-activation fresh encoder를 요구하는 helper 구조
- `calibration_gui.cpp`
  - enable 전에 encoder를 읽어 hold target 생성
  - Direct input mode
  - enable 전에 Set_Input_Pos
- `direct_position_tuning_gui.cpp`
  - 동일한 기존 activation sequence
- `quattro_bringup`
  - `OnProcessExit` chain 중심 startup orchestration
- 문서 전반
  - Direct Position이라는 이전 runtime 명칭

## 리팩터링 목표

### `gim6010_driver`

- encoder sequence/generation 제공
- 자동 broadcast 수신 중심 구조 유지
- request API는 범용 진단 기능으로만 유지

### `quattro_hardware`

```text
on_configure
  CAN open
  heartbeat/fault check
  limits/gains
  Position + PosFilter

on_activate
  Closed Loop
  heartbeat closed-loop 확인
  post-Closed-Loop encoder sync
  ROS state/command 기준 동기화
  12축 완료 후 SUCCESS
```

### GUI

Enable 시 command 없이 Closed Loop 자체 Hold를 사용한다.

첫 `Set_Input_Pos`는 사용자가 Jog/Move target을 요청했을 때만 전송한다.

### `quattro_bringup`

`ament_python` 기반으로 재구성하고 launch 파일은 프로세스 실행만 담당한다.

startup manager가 hardware/controller state를 명시적으로 관리한다.

## 목표 bringup 상태

```text
QuattroSystem          ACTIVE
GIM6010 x12            CLOSED LOOP
Encoder                post-Closed-Loop synchronized
joint_state_broadcaster ACTIVE
joint_trajectory_controller ACTIVE
Gait                    OFF
Robot                    HOLD / READY
```

## 안전 정책

아래 조건에서 READY로 진행하지 않는다.

- heartbeat 누락
- axis_error
- Closed Loop 진입 실패
- Closed Loop 이후 encoder timeout
- CAN unhealthy
- controller activation 실패

부분 activation은 허용하지 않는다.

## 테스트 추가 필요

- Closed Loop 이전 encoder 무시
- PosFilter mode 확인
- startup Set_Input_Pos 0회
- post-Closed-Loop encoder만 초기 state로 사용
- encoder timeout → activation fail + all idle
- axis_error → activation fail
- N번째 motor 실패 → 이전 motor 포함 all idle
- first normal command continuity 확인

## 문서 정책

- `docs/packages/*.md`: 현재 설계와 API 계약만 기록
- `docs/development_status.md`: 날짜별 실험 결과와 전환 이력 기록
- `AGENTS.md`: 반드시 지켜야 할 짧은 하드웨어 계약만 기록

세부 구현 목표는 `docs/packages/quattro_hardware.md`, `docs/packages/quattro_bringup.md`, `docs/calibration.md`를 따른다.
