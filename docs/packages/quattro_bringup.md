# `quattro_bringup`

## 역할

`quattro_bringup`은 실제 시스템의 프로세스 실행과 startup orchestration을 담당한다.

핵심 원칙:

- GIM6010 protocol과 실시간 `read()`/`write()`는 `quattro_hardware`가 담당한다.
- bringup은 하드웨어 lifecycle과 controller 순서를 관리한다.
- hardware bringup 완료 직후 gait를 자동 시작하지 않는다.
- 정상 종료 시 안전하게 hardware lifecycle을 종료한다.

## 패키지 구조

`quattro_bringup`은 `ament_python` 패키지다.

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
├── config/
│   ├── calibration.yaml
│   ├── calibration.yaml.example
│   └── hardware_controllers.yaml
└── test/
    ├── test_copyright.py
    ├── test_flake8.py
    └── test_pep257.py
```

실행 파일:

```bash
ros2 run quattro_bringup bringup_manager
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
 ↓
/bringup/ready latched true
```

Gait는 이 sequence에 포함하지 않는다. gait controller 프로세스는 다른 노드와 함께 시작하되, `/bringup/ready`를 받기 전까지 아무 trajectory도 발행하지 않는다.

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

launch 파일은 프로세스 실행만 담당한다.

역할:

- launch argument 선언
- Xacro로 `robot_description` 생성
- `robot_state_publisher` 시작
- `controller_manager` 시작
- `bringup_manager` 시작
- 조건부로 IMU/teleop/gait 시작

startup 순서는 `bringup_manager`가 소유한다. `OnProcessExit` chain으로 startup 상태 머신을 구현하지 않는다. spawner 프로세스의 종료는 "성공해서 끝났다"와 "포기해서 끝났다"를 구분하지 못하고, 그 사이에 검증 단계를 넣을 자리도 없기 때문이다.

모든 노드는 병렬로 시작하며 gait controller도 예외가 아니다. gait의 대기는 launch event handler가 아니라 노드 안에 있다. `hardware.launch.py`는 gait controller에 `wait_for_bringup_ready: true`를 넘기고, 노드는 `/bringup/ready`를 받을 때까지 발행을 보류한다. inactive 상태의 `joint_trajectory_controller`는 들어온 trajectory를 조용히 버리므로, JTC ACTIVE 이전에 나간 staged initial pose는 그대로 사라지고 로봇은 나중에 gait stance로 튀게 된다.

남아 있는 `OnProcessExit`은 두 개뿐이며 둘 다 순서 제어가 아니라 teardown용이다.

- `controller_manager` 종료 → 전체 shutdown
- `bringup_manager` 종료 → 전체 shutdown (FAULT 시 non-zero exit)

`bringup_manager`는 READY 이후에도 살아 있으므로 그 종료는 언제나 teardown 신호다.

### launch argument

| argument | 기본값 | 의미 |
|---|---|---|
| `calibration_file` | `config/calibration.yaml` | 기체별 calibration YAML |
| `controller_file` | `config/hardware_controllers.yaml` | controller 설정 |
| `use_gait` | `false` | gait controller 프로세스 시작 여부 |
| `start_gait_enabled` | `false` | READY 직후 즉시 initial pose 명령 (`use_gait:=true` 필요) |
| `staged_initial_pose` | `true` | initial pose를 다리 단위로 순차 이동 |
| `initial_pose_duration` | `5.0` | initial pose 궤적 시간(초) |
| `use_imu` | `true` | BNO085 노드 시작 |
| `use_teleop` | `true` | joystick/teleop 시작 |

hardware bringup에는 gait가 필요 없으므로 `use_gait`의 기본값은 `false`다.

## `bringup_manager.py`

상태:

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

각 단계는 `controller_manager` 서비스만 사용한다.

| 상태 | 사용 서비스/토픽 |
|---|---|
| `WAIT_CONTROLLER_MANAGER` | 사용할 서비스 전부의 존재 확인 |
| `CONFIGURE_HARDWARE` | `set_hardware_component_state` → inactive |
| `ACTIVATE_HARDWARE` | `set_hardware_component_state` → active |
| `VERIFY_HARDWARE` | `list_hardware_components` |
| `START_JSB` / `START_JTC` | `load_controller`, `configure_controller`, `switch_controller`(STRICT), `list_controllers` |
| `VERIFY_JOINT_STATES` | `/joint_states` 구독 |
| `READY` | `/bringup/ready` latched 발행 |

`ACTIVATE_HARDWARE` 성공은 12축 모두가 Closed Loop에 들어가고 Closed Loop 이후 encoder sync가 끝났음을 의미한다. 그 판정 자체는 `QuattroSystem::on_activate()`가 하고, bringup manager는 결과만 받는다.

`VERIFY_JOINT_STATES`는 12개 position이 모두 유한한 `/joint_states` 메시지를 기다린다. `QuattroSystem::read()`는 encoder 값이 없는 축을 NaN으로 보고하므로, 메시지 길이만으로는 모든 축이 실제로 보고 중인지 알 수 없다.

### parameter

| parameter | 기본값 | 의미 |
|---|---|---|
| `controller_manager` | `/controller_manager` | 서비스 namespace |
| `service_timeout` | `30.0` | 서비스 대기/응답 한도(초) |
| `joint_state_timeout` | `10.0` | 유효 `/joint_states` 대기 한도(초) |
| `expected_joints` | `12` | 필요한 joint 수 |
| `shutdown_on_fault` | `true` | FAULT 시 non-zero exit |
| `ready_topic` | `bringup/ready` | READY 발행 토픽 |

### `/bringup/ready`

| 항목 | 값 |
|---|---|
| 타입 | `std_msgs/Bool` (`data: true`) |
| QoS | `depth 1`, `transient local`(latched) |
| 발행 시점 | `READY` 진입 직후 1회 |

READY는 event가 아니라 state이므로 latched로 발행한다. bringup이 끝난 뒤 수동으로 다시 띄운 gait controller처럼 늦게 구독한 노드도 즉시 flag를 받는다.

transient local sample은 publisher가 살아 있는 동안만 유지되므로 `bringup_manager`는 READY 이후 종료하지 않고 flag를 들고 남는다. 따라서 이 프로세스의 종료 코드는 더 이상 "bringup 완료" 신호가 아니다. FAULT면 non-zero로 즉시 종료하고, READY 이후에는 launch teardown에서만 종료한다.

FAULT에서 bringup manager는 모터에 명령을 보내지 않는다. 하드웨어 fault는 이미 `QuattroSystem` 내부에서 전 축 Idle로 처리되며, controller 기동 실패 같은 bringup 측 실패를 두 번째 주체가 같은 bus에 axis-state 명령을 보내 "수습"하게 두지 않는다.

## READY 상태

bringup 완료의 의미:

```text
QuattroSystem          ACTIVE
GIM6010-8 x12          CLOSED LOOP
motor shafts           current-position hold
joint_state_broadcaster ACTIVE
joint_trajectory_controller ACTIVE
joint_states           valid
/bringup/ready          latched true
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

설정 키는 `position_control`이다.

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
ros2 topic echo --once /bringup/ready
ip -details -statistics link show can0
ip -details -statistics link show can1
```

정상 조건:

- QuattroSystem active
- JSB active
- JTC active
- `/joint_states`가 12축 유효 encoder 기반
- `/bringup/ready`가 `data: true`로 latched
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
