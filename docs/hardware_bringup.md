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

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml
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
2. encoder feedback 확인
3. motor enable
4. 최신 motor position에서 hold
5. Kp를 점진적으로 적용
6. heartbeat와 closed-loop state 확인
7. 상위 controller 사용 가능 상태로 전환

한 모터라도 startup 조건을 만족하지 못하면 safe stop으로 전환한다.

## 초기 자세

`gait_controller`는 controller 활성화 후 `initial_pose_duration`을 사용해 초기 자세 trajectory를 처리한다.

현재 `hardware.launch.py` 기본값:

```text
initial_pose_duration = 5.0 s
```

초기 자세 transition 로직을 변경할 때는 gait controller와 하드웨어 Safe Start를 별개의 단계로 유지한다.

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
