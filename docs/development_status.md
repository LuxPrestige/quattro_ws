# Quattro 개발 현황

## 목적

이 문서는 현재 구현 상태와 실기에서 확인된 사실, 다음 리팩터링 작업을 기록한다.

## 현재 상태 (2026-08-22)

`Position Control + Pos Filter` 리팩터링 완료. 코드/문서/테스트가 아래 실기 확인 사항과 일치한다.

실기 시험은 아직 수행하지 않았다. 검증은 빌드, 자동 테스트, URDF 파싱, mock hardware 기반 bringup까지다.

### 폐기된 전제

기존 Quattro 하드웨어 코드는 `Direct Position + Closed Loop 전 encoder 읽기 + Set_Input_Pos(current)`를 Safe Start로 사용했다.

실기 검증 결과 이 전제는 폐기했다.

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

## 제거된 기존 구현

새 하드웨어 계약과 충돌하던 다음 구현은 모두 삭제했다.

- `quattro_system.cpp`
  - Closed Loop 전 encoder를 초기 위치로 사용
  - `InputMode::kDirect`
  - startup `Set_Input_Pos(current)` (hold target 선전송)
  - 고정 지연 `motor_activation_interval_ms`
- `quattro_system.hpp`
  - pre-activation fresh **encoder**를 요구하던 `wait_for_fresh_feedback_and_no_faults()` / `wait_for_all_motors_fresh_feedback()` 구조
- `calibration_gui.cpp`
  - enable 전에 encoder를 읽어 hold target 생성
  - Direct input mode
  - enable 시 `Set_Input_Pos`
  - enable된 모든 축에 매 tick hold command 전송
- `direct_position_tuning_gui.cpp`
  - 파일/실행파일/클래스 이름과 activation sequence 전체
- `position_control_tuning_gui.cpp`
  - 별도 실행파일 자체를 삭제. 게인 편집/저장 기능은 `calibration_gui`에 통합했다 (2026-08-24)
- `quattro_bringup`
  - `OnProcessExit` chain 중심 startup orchestration
  - `hardware_spawner` + spawner chain
  - `ament_cmake` 빌드
- YAML/Xacro
  - `direct_position` 키
  - `motor_activation_interval_ms` argument/parameter

## 적용된 구조

### `gim6010_driver`

- `Gim6010Motor::encoder_sequence()` 추가 (0x009 수신마다 증가)
- 자동 broadcast 수신 중심 구조 유지
- `request_encoder_estimate()`는 범용 진단 기능으로만 유지
- `CanSocketInterface` + `MotorManager` socket factory 생성자 추가 (테스트 seam)

### `quattro_hardware`

```text
on_configure
  CAN open
  heartbeat 존재 확인 (encoder 미사용)
  axis_error 확인
  limits/gains
  Position + PosFilter

on_activate
  Closed Loop
  heartbeat closed-loop 확인
  확인 시점 encoder_sequence를 baseline으로 저장
  baseline + encoder_sync_frames 대기
  ROS state/command 기준 동기화
  12축 완료 후 SUCCESS
```

helper: `wait_for_all_heartbeats()`, `wait_for_all_fresh_feedback()`, `check_pre_activation_faults()`, `wait_for_closed_loop()`, `wait_for_post_closed_loop_encoder()`, `activate_joint()`, `safe_stop_all()`

### GUI

`calibration_gui` 하나로 offset 캘리브레이션과 position control 게인 튜닝을 모두 수행한다(별도 `position_control_tuning_gui` 실행파일은 삭제). Enable 시 command 없이 Closed Loop 자체 Hold를 사용하는 원칙은 그대로다.

첫 `Set_Input_Pos`는 사용자가 Jog / Go to Target(절대 목표각) / Move to Saved Zero를 요청했을 때만 전송한다. 축별 "target 요청됨" 플래그로 주기 전송을 제어한다.

12축 상태(축 상태/fault, saved/session/target 각도, 추종 오차, 속도, raw motor rev, offset)를 표로 항상 표시한다. 게인은 `position_control` 키 하나로 전역 적용되며, `Apply to Enabled Motors`로 즉시 반영, `Save Gains to YAML`로 파일에 저장한다.

### `quattro_bringup`

`ament_python` 패키지로 재구성했고 launch 파일은 프로세스 실행만 담당한다.

`bringup_manager` 노드가 hardware/controller state를 명시적 상태 머신으로 관리한다.

READY에서 latched `/bringup/ready`(`std_msgs/Bool`)를 발행하고 그 flag를 들고 살아 있는다. gait controller는 `wait_for_bringup_ready: true`로 이 flag를 기다리며, `joint_trajectory_controller` ACTIVE 이전에 initial pose 궤적이 나가 버리는 문제를 노드 안에서 막는다. launch 파일의 `OnProcessExit`은 teardown 용도로만 남는다.

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

## 추가된 테스트

`quattro_hardware/test/test_quattro_system_startup.cpp` (12건). in-memory CAN transport(`test/fake_can_network.hpp`) 위에서 실제 `QuattroSystem`을 구동한다.

- Closed Loop 이전 encoder 무시
- Position Control + PosFilter mode 및 전송 순서 확인
- startup `Set_Input_Pos` 0회
- post-Closed-Loop encoder만 초기 state로 사용 (`encoder_sync_frames` 포함)
- post-Closed-Loop encoder timeout → activation fail + all idle
- enable 시 axis_error → activation fail + all idle
- 기존 axis_error → configure 거부, `Clear_Errors` 미전송
- heartbeat 없음 → configure 실패
- N번째 motor 실패 → 이전 motor 포함 all idle
- first normal command continuity 확인 (command == state, 첫 `write()` 페이로드)
- `read()`가 encoder request를 보내지 않음
- direction/offset 반영

`gim6010_driver/test/test_motor_manager.cpp`에 encoder sequence counter 및 socket factory 검증 4건 추가.

`quattro_bringup`에 `ament_flake8` / `ament_pep257` / `ament_copyright` 테스트 추가.

### 검증 상태

| 항목 | 결과 |
|---|---|
| `colcon build` (8 패키지) | 통과 |
| gtest (driver 40건, hardware 22건) | 통과 |
| `quattro_bringup` flake8/pep257 | 통과 |
| `xacro` + `check_urdf` (`simulation` true/false) | 통과 |
| mock hardware bringup (`mock_components/GenericSystem`) | READY 도달, JSB/JTC active |
| 실기 모터 시험 | **미수행** |

`uncrustify`는 이 작업 이전부터 실패 상태다 (HEAD 기준 14개 파일). 저장소 전반의 `{ return x; }` 표기가 ament 기본 설정과 다르며, 이번 변경으로 고치지 않았다.

## 문서 정책

- `docs/packages/*.md`: 현재 설계와 API 계약만 기록
- `docs/development_status.md`: 날짜별 실험 결과와 전환 이력 기록
- `AGENTS.md`: 반드시 지켜야 할 짧은 하드웨어 계약만 기록

세부 구현 목표는 `docs/packages/quattro_hardware.md`, `docs/packages/quattro_bringup.md`, `docs/calibration.md`를 따른다.
