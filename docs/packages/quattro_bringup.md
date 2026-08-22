# `quattro_bringup`

## 역할

`quattro_bringup`은 실제 시스템의 프로세스 실행과 startup orchestration을 담당한다.

핵심 원칙:

- GIM6010 protocol과 실시간 `read()`/`write()`는 `quattro_hardware`가 담당한다.
- bringup은 하드웨어 lifecycle과 controller 순서를 관리한다.
- hardware bringup 완료 직후 gait를 자동 시작하지 않는다.
- 정상 종료 시 안전하게 hardware lifecycle을 종료한다.

## 목표 패키지 구조

이번 리팩터링에서는 `quattro_bringup`을 `ament_python` 패키지로 전환하는 것을 권장한다.

```text
src/quattro_bringup/
├── package.xml
├── setup.py
├── setup.cfg
├── resource/quattro_bringup
├── quattro_bringup/
│   ├── __init__.py
│   └── bringup_manager.py
├── launch/
│   ├── hardware.launch.py
│   ├── gait_visualization.launch.py
│   └── remote_visualization.launch.py
└── config/
    ├── calibration.yaml
    ├── calibration.yaml.example
    └── hardware_controllers.yaml
```

## 실행 전 조건

1. 로봇을 지지대에 올린다.
2. 다른 CAN 송신 프로그램을 종료한다.
3. `can0`, `can1`이 500 kbit/s, ERROR-ACTIVE인지 확인한다.
4. 12개 모터의 Heartbeat와 0x009 자동 broadcast 설정을 확인한다.
5. calibration과 gain/current limit을 확인한다.
6. 즉시 전원을 차단할 수 있는 수단을 준비한다.

## 공식 startup sequence

```text
BOOT
 ↓
robot_state_publisher + controller_manager
 ↓
QuattroSystem configure
   Set_Limits
   Set_Pos_Gain
   Set_Vel_Gains
   Position Control + Pos Filter
 ↓
QuattroSystem activate
   Closed Loop
   motor 자체 Hold
   Closed Loop 이후 EncoderEstimate sync
 ↓
QuattroSystem ACTIVE
 ↓
joint_state_broadcaster ACTIVE
 ↓
/joint_states 확인
 ↓
joint_trajectory_controller ACTIVE
 ↓
READY / HOLD
```

Gait는 이 sequence에 포함하지 않는다.

## 하드웨어 activation 규칙

bringup manager는 GIM6010 command를 직접 보내지 않는다. 다음은 모두 `QuattroSystem` 내부 책임이다.

```text
Limits / Gains
Position + PosFilter
Closed Loop
post-Closed-Loop encoder sync
fault detection
safe stop
```

Closed Loop 이전 encoder 위치를 사용하거나 startup `Set_Input_Pos(current)`를 보내면 안 된다.

## `hardware.launch.py`

launch 파일은 최대한 단순하게 유지한다.

권장 역할:

- launch argument 선언
- Xacro로 `robot_description` 생성
- `robot_state_publisher` 시작
- `controller_manager` 시작
- `bringup_manager` 시작
- 필요 시 IMU/teleop 시작

복잡한 `OnProcessExit -> OnProcessExit` chain으로 startup 상태 머신을 구현하지 않는다.

## `bringup_manager.py`

목표 상태:

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

`QuattroSystem` activation 성공은 12축 모두가 Closed Loop에 들어가고, Closed Loop 이후 encoder sync가 끝났음을 의미해야 한다.

## READY 상태

bringup 완료의 의미:

```text
QuattroSystem          ACTIVE
GIM6010-8 x12          CLOSED LOOP
motor shafts           current-position hold
joint_state_broadcaster ACTIVE
joint_trajectory_controller ACTIVE
joint_states           valid
Gait                    OFF
```

READY는 걷는 상태가 아니다.

## controller 설정

`hardware_controllers.yaml`은 다음을 유지한다.

- `update_rate: 100`
- `QuattroSystem` 초기 lifecycle은 inactive
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- position command interface
- position/velocity state interface

## Position Control 설정 파일

기존 `direct_position` 명칭은 실제 mode와 맞지 않으므로 리팩터링 시 `position_control`로 통일하는 것을 권장한다.

예:

```yaml
position_control:
  current_limit: 10.0
  position_gain: 20.0
  velocity_gain: 0.11
  velocity_integrator_gain: 0.32
```

joint별 CAN mapping/direction/offset은 `joints` 아래에 둔다.

## 확인 명령

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /joint_states
ip -details -statistics link show can0
ip -details -statistics link show can1
```

정상 조건:

- QuattroSystem active
- JSB active
- JTC active
- `/joint_states`가 12축 유효 encoder 기반
- can0/can1 ERROR-ACTIVE
- Gait가 명시적 enable 전에는 동작하지 않음

## fault 정책

다음 중 하나가 발생하면 READY로 진행하지 않는다.

- controller_manager 미응답
- hardware configure 실패
- motor Closed Loop 실패
- post-Closed-Loop encoder timeout
- heartbeat timeout
- axis_error
- CAN unhealthy
- JSB/JTC activation 실패

하드웨어 fault의 실제 motor Idle 처리는 `QuattroSystem`이 책임진다.

## 관련 문서

- `docs/packages/quattro_hardware.md`
- `docs/packages/gim6010_driver.md`
- `docs/calibration.md`
- `docs/development_status.md`
