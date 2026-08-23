# GIM6010 Position + PosFilter Bringup Refactor Prompt

아래 지시를 기준으로 `LuxPrestige/quattro_ws`의 실기 하드웨어 startup 구조를 재설계하고 구현하라.

## 작업 전 필수 확인

먼저 다음 파일을 읽고 현재 구현과 문서의 충돌점을 정리한 뒤 수정하라.

```text
AGENTS.md
docs/README.md
docs/architecture.md
docs/development_status.md
docs/calibration.md
docs/packages/gim6010_driver.md
docs/packages/quattro_hardware.md
docs/packages/quattro_bringup.md
docs/packages/quattro_description.md

src/gim6010_driver/include/gim6010_driver/gim6010_motor.hpp
src/gim6010_driver/src/gim6010_motor.cpp
src/gim6010_driver/include/gim6010_driver/motor_manager.hpp
src/gim6010_driver/include/gim6010_driver/types.hpp

src/quattro_hardware/include/quattro_hardware/quattro_system.hpp
src/quattro_hardware/src/quattro_system.cpp
src/quattro_hardware/src/calibration_gui.cpp
src/quattro_hardware/src/direct_position_tuning_gui.cpp
src/quattro_hardware/CMakeLists.txt

src/quattro_description/urdf/quattro.urdf.xacro
src/quattro_description/config/calibration.yaml

src/quattro_bringup/package.xml
src/quattro_bringup/CMakeLists.txt
src/quattro_bringup/launch/hardware.launch.py
src/quattro_bringup/config/calibration.yaml
src/quattro_bringup/config/calibration.yaml.example
src/quattro_bringup/config/hardware_controllers.yaml
```

## 절대 변경하면 안 되는 실기 확인 사실

다음은 추측이 아니라 실제 GIM6010-8에서 확인된 동작이다. 구현은 반드시 이 사실을 기준으로 한다.

1. startup 설정 순서는 다음과 같다.

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Set_Controller_Mode(
     control_mode = Position Control,
     input_mode   = Pos Filter)
→ Set_Axis_State(Closed Loop Control)
```

2. **Closed Loop 이전 EncoderEstimate 위치값은 쓰레기 값이 들어올 수 있으므로 절대 startup 현재 위치로 사용하지 않는다.**

3. `Get_Encoder_Estimates(0x009)`는 모터가 약 10 ms 주기로 자동 송신한다. Quattro runtime, bringup, calibration/tuning GUI에서는 정상 제어 경로에서 encoder request를 보내지 않는다.

4. `Position Control + Pos Filter` 설정 후 Closed Loop 상태에 들어가면 별도 position command를 보내지 않아도 모터 축이 현재 위치에서 고정된다.

5. 따라서 기존 Safe Start 방식인:

```text
Closed Loop 전 encoder 읽기
→ Set_Input_Pos(current)
→ Closed Loop
```

는 완전히 제거한다.

6. 초기 유효 위치는 **Closed Loop가 Heartbeat로 확인된 이후 도착한 새 EncoderEstimate**만 사용한다.

7. `Get_Error(0x03)`는 실기에서 응답하지 않으므로 startup/runtime safety는 `Heartbeat.axis_error`를 사용한다.

## 목표 제어 방식

기존 `Direct Position`이라는 표현과 `InputMode::kDirect` 기반 runtime을 제거하고 실기 기본 제어를 다음으로 통일한다.

```text
Control Mode = Position Control
Input Mode   = Pos Filter
```

YAML의 기존:

```yaml
direct_position:
```

키는 가능하면:

```yaml
position_control:
```

로 변경하고, 다음 파일 전체에서 일관되게 수정한다.

```text
quattro.urdf.xacro
quattro_bringup calibration YAML
quattro_description calibration YAML
calibration_gui
position tuning GUI
관련 문서
```

## 1. `gim6010_driver` 수정

### 목표

`Gim6010Motor`가 Closed Loop 이후 새 encoder frame인지 상위 계층이 확실하게 판단할 수 있도록 encoder generation counter를 추가하라.

예시 개념:

```cpp
std::uint64_t encoder_sequence_{0};
```

`on_encoder_estimate()` 호출 때마다 증가한다.

getter도 제공하라.

```cpp
std::uint64_t encoder_sequence() const noexcept;
```

기존 timestamp/freshness 기능은 유지한다.

### encoder request

`MotorManager::request_encoder_estimate()` API 자체는 범용 진단/재사용을 위해 남겨도 된다.

하지만 Quattro 정상 runtime에서는 사용하지 않는다.

## 2. `quattro_hardware::QuattroSystem` 전면 수정

### `on_configure()`

다음 순서로 구성하라.

```text
MotorManager 생성
→ can0/can1 open
→ Heartbeat 기반 motor 존재 확인
→ Heartbeat.axis_error 확인
→ 각 모터 Set_Limits
→ 각 모터 Set_Pos_Gain
→ 각 모터 Set_Vel_Gains
→ 각 모터 Set_Controller_Mode(PositionControl, PosFilter)
→ SUCCESS
```

중요:

- Closed Loop 이전 encoder position을 읽어 초기 위치로 사용하지 않는다.
- startup 설정 burst의 CAN TX queue 문제를 막기 위한 기존 bounded retry/pacing은 유지하거나 개선한다.
- Clear_Errors를 자동 호출하지 않는다.

### `on_activate()`

각 motor를 순차적으로 활성화한다.

모터 하나의 목표 sequence:

```text
현재 encoder sequence 저장
→ Set_Axis_State(ClosedLoopControl)
→ poll()
→ Heartbeat.axis_state == ClosedLoopControl 확인
→ Heartbeat.axis_error == 0 확인
→ Closed Loop 확인 이후 도착한 새 EncoderEstimate 대기
→ motor_rev -> joint_rad 변환
→ ROS state.position 동기화
→ 첫 command 기준을 같은 joint_rad로 동기화
→ 다음 motor
```

더 엄격하게 하려면 Closed Loop Heartbeat를 확인한 시점의 encoder sequence를 기준으로 잡고 그 이후 frame을 사용하라.

startup에서는 절대로:

```cpp
send_set_input_pos(current_position)
```

를 호출하지 마라.

12축 모두 성공해야 `on_activate()`가 SUCCESS를 반환한다.

한 축이라도 실패하면:

```text
safe_stop_all()
→ 모든 motor Idle
→ activation ERROR
```

로 처리한다.

### `read()`

- `MotorManager::poll()`만 사용해 자동 broadcast를 읽는다.
- encoder request를 보내지 않는다.
- encoder/heartbeat timeout, axis_error, CAN unhealthy를 검사한다.
- position/velocity state를 업데이트한다.

### `write()`

정상 controller command를 `Set_Input_Pos`로 보낸다.

단 startup Safe Start 목적으로는 `Set_Input_Pos`를 사용하지 않는다.

첫 command가 activation 직후 현재 위치와 불연속이 되지 않도록 command interface 초기값을 post-Closed-Loop encoder 위치와 동기화하라.

기존 연속 write failure 카운터 정책은 유지한다.

## 3. `quattro_system.hpp` 정리

다음과 같은 기존 개념을 제거/교체하라.

```text
pre-activation fresh encoder requirement
wait_for_fresh_feedback_and_no_faults()가 encoder를 요구하는 구조
activate_joint()가 Closed Loop 전 encoder를 읽는 구조
```

대신 책임을 명확히 나눈 helper를 사용하라.

예시:

```text
wait_for_all_heartbeats()
check_pre_activation_faults()
wait_for_closed_loop()
wait_for_post_closed_loop_encoder()
activate_joint()
safe_stop_all()
```

함수명은 코드 스타일에 맞게 조정해도 된다.

## 4. `calibration_gui.cpp` 수정

기존:

```text
Closed Loop 전 fresh encoder 읽기
→ Direct mode
→ Set_Input_Pos(current)
→ Closed Loop
```

를 제거한다.

새 Enable sequence:

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position + PosFilter
→ Closed Loop
→ Heartbeat Closed Loop 확인
→ post-Closed-Loop encoder 대기
→ session position 설정
```

Enable 순간에는 `Set_Input_Pos`를 보내지 않는다.

첫 position command는 다음 사용자 동작에서만 보낸다.

```text
-1 deg
+1 deg
Move to Saved Zero
```

Save Current Position as Zero도 반드시 Closed Loop 이후 유효 encoder를 사용한다.

## 5. tuning GUI 수정

현재 `direct_position_tuning_gui.cpp`는 이름과 activation sequence가 모두 오래된 설계다.

가능하면:

```text
direct_position_tuning_gui.cpp
→ position_control_tuning_gui.cpp
```

로 변경하고 CMake executable 이름도 같이 수정하라.

Apply and Enable sequence는 calibration GUI와 동일하게 한다.

```text
limits/gains
→ Position + PosFilter
→ Closed Loop
→ post-Closed-Loop encoder sync
```

Enable 시 `Set_Input_Pos` 금지.

`Send Relative Target`을 눌렀을 때 처음 `Set_Input_Pos`를 보낸다.

## 6. Xacro / YAML 수정

`quattro.urdf.xacro`와 calibration YAML 계약을 새 제어 방식에 맞춰 정리한다.

가능하면:

```yaml
position_control:
  current_limit: ...
  position_gain: ...
  velocity_gain: ...
  velocity_integrator_gain: ...
```

로 통일한다.

현재 실제 calibration 값은 임의로 변경하지 마라. 키 구조만 바꿀 경우 기존 수치는 보존한다.

기존 `motor_activation_interval_ms`가 단순 sleep만 의미한다면 이를 실제 상태 기반 timeout으로 교체하는 것을 검토한다.

권장 parameter:

```text
closed_loop_timeout_ms
encoder_sync_timeout_ms
encoder_sync_frames
```

`encoder_sync_frames`는 1 이상으로 하고, 안전 마진으로 2~3 frame을 사용할지 설계 이유를 문서화하라.

## 7. `quattro_bringup` 재구성

`quattro_bringup`은 `ament_python` 패키지로 전환하라.

목표 구조:

```text
quattro_bringup/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/quattro_bringup
├── quattro_bringup/
│   ├── __init__.py
│   └── bringup_manager.py
├── launch/
└── config/
```

### launch 역할

`hardware.launch.py`는 다음만 담당하게 단순화하라.

```text
robot_description 생성
robot_state_publisher 시작
controller_manager 시작
bringup_manager 시작
IMU/teleop 조건부 시작
```

복잡한 `OnProcessExit` chain을 startup 상태 머신으로 사용하지 마라.

### bringup manager 목표 상태

```text
WAIT_CONTROLLER_MANAGER
CONFIGURE_HARDWARE
ACTIVATE_HARDWARE
VERIFY_HARDWARE
START_JSB
VERIFY_JOINT_STATES
START_JTC
READY
FAULT
```

READY의 의미:

```text
QuattroSystem active
GIM6010 x12 closed-loop
post-Closed-Loop encoder synchronized
joint_state_broadcaster active
joint_trajectory_controller active
Gait OFF
```

bringup 완료만으로 gait/initial pose를 자동 실행하지 마라.

## 8. 테스트 추가

최소 다음 테스트를 추가하라.

```text
1. Closed Loop 이전 encoder가 startup state로 사용되지 않음
2. controller mode = PositionControl + PosFilter
3. startup Set_Input_Pos 호출 0회
4. Closed Loop 이후 새 encoder만 initial state로 사용
5. encoder timeout -> activation failure + all idle
6. axis_error -> activation failure + all idle
7. N번째 motor activation failure -> 앞서 활성화된 motor도 all idle
8. encoder sequence counter 동작
9. first normal command가 synchronized position에서 연속적으로 시작
```

필요하면 MotorManager transport를 mock/fake할 수 있도록 최소한의 테스트 seam을 추가하되 production 구조를 과도하게 복잡하게 만들지 마라.

## 9. 문서 동기화

코드 구현이 끝나면 다음 문서를 실제 코드와 다시 대조해 수정하라.

```text
AGENTS.md
docs/README.md
docs/architecture.md
docs/development_status.md
docs/calibration.md
docs/packages/gim6010_driver.md
docs/packages/quattro_hardware.md
docs/packages/quattro_bringup.md
docs/packages/quattro_description.md
```

패키지 문서는 현재 설계/API만 설명하고 과거 실험 로그는 `development_status.md`에 둔다.

## 10. 검증

Docker 컨테이너 `/quattro_ws` 기준으로 수행하라.

```bash
cd /quattro_ws
colcon build --symlink-install --event-handlers console_direct+
colcon test --event-handlers console_direct+
colcon test-result --verbose
```

Xacro:

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

가능한 경우 Python syntax/launch import도 검증하라.

실물 모터를 실제로 enable하거나 움직이는 시험은 자동으로 수행하지 마라. 실기 시험 명령과 예상 CAN sequence만 제시하라.

## 11. 최종 보고 형식

작업 완료 후 다음 순서로 요약하라.

1. 변경한 파일 목록
2. 삭제한 잘못된 기존 startup 로직
3. 새 startup sequence
4. `gim6010_driver` 변경점
5. `quattro_hardware` 변경점
6. GUI 변경점
7. bringup 변경점
8. YAML/Xacro 변경점
9. 추가한 테스트
10. build/test 결과
11. 실기에서 사용자가 확인해야 할 항목

중요: 기존 동작을 보존하기 위해 잘못된 pre-Closed-Loop encoder 로직을 남기지 마라. 이 리팩터링의 핵심은 **GIM6010의 실제 동작에 맞춰 startup 기준을 Closed Loop 이후 encoder로 옮기는 것**이다.
