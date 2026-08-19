# Quattro 실기 Bringup

## 목적

실제 Quattro 하드웨어에서 다음 구성요소를 함께 실행한다.

- GIM6010-8 × 12 / GDS68
- `quattro_hardware::QuattroSystem`
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- BNO085 IMU
- Switch Pro Controller teleop
- gait controller

하드웨어 구조와 CAN 규칙은 `docs/gim6010_hardware.md`를 먼저 확인한다.

## 실행 전 안전 확인

1. 로봇을 지지대에 올려 다리에 하중이 걸리지 않게 한다.
2. calibration GUI와 다른 CAN 송신 프로그램을 종료한다.
3. 실제 `calibration.yaml`이 존재하고 12개 offset을 확인한다.
4. `can0`, `can1`이 모두 500 kbit/s이며 `ERROR-ACTIVE`인지 확인한다.
5. 전원과 E-stop 수단을 준비한다.
6. 초기 시험은 낮은 gain/current 조건에서 수행한다.

CAN 상태:

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

## Docker

Raspberry Pi 하드웨어 장치 전달은 `docs/development_environment.md`를 따른다.

컨테이너 내부에서:

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash
```

## 실행

기본 제어 방식은 MIT control이다.

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml
```

모터는 calibration 파일 순서대로 하나씩 활성화한다. 각 모터가 fresh feedback과
fault 없는 closed-loop 상태에 진입한 것을 확인하고 기본 100 ms 동안 안정 상태를
유지한 뒤 다음 모터를 활성화한다. 이 대기는 closed-loop 확인 자체를 건너뛰는
것이 아니라 그 확인이 끝난 뒤 추가로 두는 안정화 시간이므로, 간격 값을 줄여도
활성화 안전 로직은 그대로 유지된다. 간격은 다음 인자로 조정한다.

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  motor_activation_interval_ms:=100
```

활성화 도중 한 모터라도 timeout, stale feedback, heartbeat fault를 보이면 이미
활성화된 모터를 포함해 전체 safe stop으로 전환한다.

MIT command는 매 frame feedback을 반환하므로 Direct Position용 주기적 `0x009`
요청을 함께 보내지 않는다. `hardware_control_method:=direct_position`으로
전환한 경우에는 encoder estimate(`0x009`)가 요청형 feedback이므로 하드웨어
계층이 기본 50 ms 주기로 모든 활성 모터에 요청한다. 이 주기는 150 ms feedback
timeout보다 짧아야 하며, 응답이 timeout을 넘으면 전체 safe stop으로 전환한다.

기본 제어 방식은 MIT control이며 전용 `mit_trajectory_controller`가 12축의
`position`, `velocity`, `kp`, `kd`, `effort` interface를 모두 claim하고 gait의
`JointTrajectory` 입력을 시간 보간한다. `mit_trajectory_controller`의 `kp`/`kd`는
`calibration_file`의 joint별 값을 launch 시점에 읽어 자동으로 채운다. 이렇게
calibration.yaml을 유일한 gain 소스로 두어, `QuattroSystem`의 활성화
gain-ramp가 끝나고 제어권이 controller로 넘어가는 순간에도 gain이 불연속으로
바뀌지 않는다. `calibration_file`을 읽을 수 없으면 `hardware_controllers_mit.yaml`의
값(`kp=60.0, kd=0.8`)으로 대체하므로 파일 문제로 bringup 자체가 막히지는 않는다.

위 기본 명령이 곧 MIT hold-only bringup이다 (gait trajectory를 자동 전송하지 않음).

전원 용량, 영점, gain을 단일 모터부터 검증한 후에만 gait를 명시적으로 활성화한다.

```bash
ros2 service call /gait/enable std_srvs/srv/SetBool '{data: true}'
```

bringup 직후 초기 자세 trajectory까지 자동 실행하려면 명시적으로
`start_gait_enabled:=true`를 추가한다. 이 옵션은 12축에 동시에 하중을 걸 수 있으므로
기본값은 `false`이다.

새 controller package를 빌드한 셸에서는 plugin index를 반영하도록 실행 전에 반드시
`source /ws/install/setup.bash`를 다시 수행한다.

GDS68 Direct Position으로 전환할 때는 다음과 같이 명시한다.

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  hardware_control_method:=direct_position \
  controller_file:=/ws/src/quattro_bringup/config/hardware_controllers.yaml \
  command_controller_name:=joint_trajectory_controller
```

Direct Velocity/Torque에서는 gait controller를 시작하지 않고 matching forward command controller를 선택한다.

```bash
# Direct Velocity
ros2 launch quattro_bringup hardware.launch.py \
  hardware_control_method:=direct_velocity \
  controller_file:=/ws/src/quattro_bringup/config/hardware_controllers_velocity.yaml \
  command_controller_name:=joint_command_controller

# Direct Torque
ros2 launch quattro_bringup hardware.launch.py \
  hardware_control_method:=direct_torque \
  controller_file:=/ws/src/quattro_bringup/config/hardware_controllers_torque.yaml \
  command_controller_name:=joint_command_controller
```

선택 인자:

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  use_imu:=true \
  use_teleop:=true \
  initial_pose_duration:=5.0
```

현재 launch 기본값은 IMU와 teleop을 모두 실행한다.

## 현재 launch 순서

`hardware.launch.py`의 실행 흐름은 다음과 같다.

```text
robot_state_publisher
controller_manager
        ↓
hardware_spawner --activate QuattroSystem
        ↓
joint_state_broadcaster
        ↓
joint_trajectory_controller
        ↓
gait_controller
```

IMU와 joystick/teleop 노드는 조건에 따라 병렬로 시작한다.

`controller_manager`가 종료되면 전체 launch도 종료한다.

## QuattroSystem 활성화

현재 하드웨어 계층은 활성화 과정에서 다음 원칙을 사용한다.

1. CAN motor mapping 구성
2. encoder와 heartbeat 확인
3. 기존 fault와 상세 오류 capture
4. 현재 output position 확보
5. Direct Position mode와 limit 설정
6. 현재 위치 target을 enable 전에 준비
7. 모터 하나를 enable하고 현재 위치 target을 계속 유지
8. 해당 모터의 fresh feedback과 closed-loop 확인
9. 설정된 안정화 간격 동안 이미 활성화된 모든 모터의 상태 확인
10. 다음 모터에 6~9를 반복
11. 상위 position controller 사용 가능 상태로 전환

한 모터라도 startup 조건을 만족하지 못하면 safe stop으로 전환한다.

## 초기 자세

`gait_controller`는 `start_gait_enabled:=true`이거나 `/gait/enable` 서비스로 활성화된
후 `initial_pose_duration`을 사용해 초기 자세 trajectory를 처리한다.

현재 `hardware.launch.py` 기본값:

```text
initial_pose_duration = 5.0 s
start_gait_enabled = false
```

초기 자세 transition 로직을 변경할 때는 gait controller와 하드웨어 Safe Start를 별개의 단계로 유지한다.

실기 launch는 staged_initial_pose 기본값을 true로 사용한다. 초기 자세 활성화 시
앞왼쪽, 앞오른쪽, 뒤왼쪽, 뒤오른쪽 순서로 한 다리(3축)씩 이동하며 각 단계는
initial_pose_duration만큼 진행된다. 기본 5초 설정에서는 전체 전환에 20초가 걸린다.
이 시험은 로봇을 지지한 상태에서 수행한다.

Quattro는 방식 A, 즉 `JointTrajectoryController`가 시간 기반 trajectory를 만들고 GDS68은 Direct Position target을 수행하는 구성을 사용한다. GDS68 internal trapezoidal trajectory를 동시에 사용하지 않아 두 계층의 shaping 중복을 피한다.

## Read-only 단일 모터 확인

다음 도구는 mode, gain, limit을 바꾸지 않고 enable, clear error, save configuration도 호출하지 않는다.

```bash
ros2 run gim6010_driver gim6010_diagnostic can0 0
```

heartbeat, rotor encoder를 8:1로 환산한 output position/velocity, encoder count(진단 전용, `shadow_count`/`count_in_cpr`), detailed errors, bus voltage/current와 CAN error/drop 상태를 출력한다. 이 read-only 결과를 먼저 저장한 뒤에만 별도 수동 Position hold 시험을 계획한다.

GIM6010-8은 온보드 인코더가 1개뿐이라(`docs/gim6010_hardware.md` 11절) 이 도구가 보여주는 위치는 항상 그 하나의 센서 값이다. 전원을 재인가한 직후에는 로터 단일회전 절대각만 즉시 확인되고, 관절이 실제로 가동범위 내 어느 45° 구간에 있는지는 자동으로 검증되지 않으므로, 재부팅 후 첫 시험에서는 이 출력을 반드시 관절의 실제 육안 위치와 대조한다.

## 단일 모터 Position 시험 순서

자동 movement 도구는 제공하지 않는다. 실제 시험은 로봇 지지와 전원 차단 수단을 준비하고 다음 단계마다 결과를 확인한다.

1. `ip -details -statistics link show can0`
2. read-only diagnostic으로 heartbeat/encoder/error/bus voltage 확인
3. 대상 node만 연결했는지 확인
4. Direct Position mode와 검증된 낮은 current/velocity limit 설정
5. 현재 output position을 읽어 같은 위치를 target으로 준비
6. closed-loop 진입 후 current-position hold 확인
7. 사용자 명시 하에 작은 output angle step
8. feedback 방향, current, fault 확인
9. idle/disable

단일 모터 hold와 작은 step이 검증되기 전에는 12축 시험을 하지 않는다.

## Teleop 시작 상태

현재 `hardware.launch.py`는 `quattro_teleop`에 다음 값을 전달한다.

```text
start_stepping = true
```

따라서 문서나 설정을 변경할 때 실제 launch 코드와 일치시킨다.

## 제어 주기

현재 controller manager 설정:

```text
update_rate = 100 Hz
```

`quattro_hardware::read()` / `write()`는 controller manager update마다 실행된다.

실제 주기가 안정적인지는 다음과 같이 확인한다.

```bash
ros2 topic hz /joint_states
ros2 topic hz /joint_trajectory_controller/joint_trajectory
ros2 topic hz /imu/data
```

## 정상 상태 확인

```bash
ros2 control list_hardware_components
ros2 control list_controllers
```

최소 정상 조건:

- `QuattroSystem`: active
- `joint_state_broadcaster`: active
- `joint_trajectory_controller`: active
- `/joint_states`: 실제 motor feedback 기반
- CAN bus: `ERROR-ACTIVE`

## 하드웨어 fault 확인

다음 로그는 의미가 다르므로 구분한다.

```text
Hardware command watchdog expired
Stale feedback from ...
Stale heartbeat from ...
Motor left closed-loop control ...
Motor fault details ...
```

CAN 통신 자체를 조사할 때는 동시에 다음을 확인한다.

```bash
candump -e -tz can0
candump -e -tz can1

ip -details -statistics link show can0
ip -details -statistics link show can1
```

단순 stale feedback과 GDS68 자체 fault를 동일한 원인으로 단정하지 않는다.

## 종료

`Ctrl+C` 또는 controller manager 종료 시 하드웨어 lifecycle이 종료되며 safe stop을 수행한다.

fault/timeout이 발생했을 때도 정의된 안전 정책에 따라 모터를 비활성화한다.

실제 fault 분석 중에는 로그와 CAN 상태를 먼저 저장하고, 원인 정보를 읽기 전에 무조건 error clear를 반복하지 않는다.
