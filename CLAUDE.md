# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

이 저장소에는 이미 상세한 `AGENTS.md`가 루트에 있으니 함께 읽는다. 패키지 경계, 코딩 규칙, 의존 방향을 이 파일보다 더 자세히 다룬다. 이 파일과 `AGENTS.md` 내용이 겹치는 부분은 서로 일치해야 하므로, 한쪽을 수정하면 다른 쪽도 확인한다.

## 프로젝트 범위

ROS 2 Jazzy 기반 워크스페이스(`lgh_ws`). 12축 4족 보행 로봇 **Quattro**의 제어 소프트웨어.

- OS: Ubuntu 24.04, SBC: Raspberry Pi 5
- 액추에이터: SteadyWin GIM6010-8 × 12, GDS68 드라이버로 구동 (온보드 인코더는 MA732 14-bit single-turn absolute 1개뿐 — secondary encoder 없음, `docs/gim6010_hardware.md` 11절)
- 모터 통신: Linux SocketCAN, CAN Simple, 500 kbit/s, `can0` = joint 0~5, `can1` = joint 6~11
- 실행 시 선택 가능한 제어 방식: GDS68 Direct Position/Velocity/Torque 또는 MIT Control
- IMU: BNO085. 시뮬레이션: Gazebo Harmonic + `gz_ros2_control`
- 개발은 기본적으로 Docker 컨테이너 내부(`/ws` = 저장소 루트)에서 수행

## 빌드, 실행, 테스트

ROS 2 빌드/테스트는 기본적으로 dev 컨테이너 내부에서 수행한다.

```bash
# host
cd ~/lgh_ws
docker compose up -d dev
docker compose exec dev bash

# 컨테이너 내부
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --event-handlers console_direct+
source /ws/install/setup.bash
```

특정 패키지만 빌드:

```bash
colcon build --symlink-install --packages-select <package_name> --event-handlers console_direct+
```

테스트 실행:

```bash
colcon test --packages-select <package_name>
colcon test-result --verbose
```

빌드 후 C++ gtest 바이너리를 직접 실행할 수도 있다. 예:

```bash
./build/quattro_hardware/test_joint_transform
```

Python 테스트 파일 하나만 실행 (ament pytest 패키지: `quattro`, `quattro_sensors`, `quattro_teleop`):

```bash
python3 -m pytest src/quattro/test/test_gait.py -v
```

Xacro/URDF 검사:

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
# 시뮬레이션용
xacro src/quattro_description/urdf/quattro.urdf.xacro simulation:=true > /tmp/quattro_sim.urdf
```

### 장치 전달용 compose 변형

추가 compose 파일은 `compose.yaml` 위에 겹쳐서 사용한다(대체하지 않는다).

```bash
# Raspberry Pi: I2C(BNO085) + joystick
INPUT_GID=$(getent group input | cut -d: -f3) \
docker compose -f compose.yaml -f compose.hardware.yaml up -d --force-recreate dev

# X11 GUI (calibration GUI, RViz) — 반드시 `ssh -Y` 세션에서 실행 (VS Code Remote-SSH 단독 사용 X)
docker compose -f compose.yaml -f compose.x11.yaml up -d --force-recreate dev

# NVIDIA 데스크톱 GPU 가속 (Pi에서는 사용하지 않음)
docker compose -f compose.yaml -f compose.x11.yaml -f compose.nvidia.yaml up -d --force-recreate dev
```

`can0`/`can1`은 Linux network interface이며 `network_mode: host`로 전달된다.

### 실기 bringup

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml
```

기본 제어 방식은 `direct_position`이며 표준 `joint_trajectory_controller`를 사용한다. MIT로 전환하려면 control-method 플래그와 그에 맞는 controller 설정을 함께 지정해야 한다. 예:

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  hardware_control_method:=mit \
  controller_file:=/ws/src/quattro_bringup/config/hardware_controllers_mit.yaml \
  command_controller_name:=mit_trajectory_controller
```

새/수정된 controller 패키지를 빌드한 셸에서는 재실행 전에 반드시 `source /ws/install/setup.bash`를 다시 수행해야 `pluginlib` index가 반영된다. 전체 안전 점검 목록, 방식별 launch 인자, fault 진단 절차는 `docs/hardware_bringup.md`에 있으므로 hardware bringup 코드를 수정하거나 실제로 모터를 활성화하기 전에 먼저 읽는다.

## 아키텍처

패키지 의존 방향은 단방향이며 역방향으로 만들지 않는다.

```text
quattro  ->  ros2_control controller interface  ->  quattro_hardware  ->  gim6010_driver  ->  SocketCAN  ->  GDS68 / GIM6010-8
```

`gim6010_driver`는 `quattro_hardware`나 `quattro`에 의존해서는 안 된다. `quattro`는 raw CAN을 직접 다루지 않는다.

| 패키지 | 책임 | 넣지 않는 내용 |
|---|---|---|
| `quattro` | FK/IK, gait 생성, 몸체 자세/balance 제어, 상위 상태 (Python) | raw CAN, GDS68 패킷 |
| `quattro_description` | URDF/Xacro (원본 기준 — 생성된 URDF를 직접 수정하지 않음), mesh, TF, `ros2_control` `<ros2_control>` description | 제어 알고리즘, 드라이버 구현 |
| `quattro_bringup` | launch 파일, controller YAML 선택, 실기 launch 구성 | 드라이버/알고리즘 구현 |
| `quattro_gazebo` | Gazebo Harmonic world, `gz_ros2_control`, 시뮬레이션 controller 설정 | 실제 하드웨어 제어 |
| `quattro_hardware` | `hardware_interface::SystemInterface`(`QuattroSystem`), joint↔motor 좌표 변환, lifecycle, 안전 정책(watchdog/timeout/fault → safe stop), calibration GUI | raw MIT/CAN 프레임 생성, gait 로직 |
| `quattro_controllers` | 커스텀 `ros2_control` controller. 특히 `MitTrajectoryController`는 `position`/`velocity`/`kp`/`kd`/`effort`를 함께 claim하고 `JointTrajectory`를 MIT 명령으로 시간 보간한다 | 드라이버/CAN 내부 구현 |
| `gim6010_driver` | SocketCAN RAII wrapper, CAN error/bus-off 진단, CAN Simple arbitration ID, GDS68 Direct Position/Velocity/Torque 및 MIT encode/decode, 단일/다중 모터(`MotorManager`) 추상화 | Quattro joint 이름, URDF, IK/FK |
| `quattro_sensors` | BNO085 취득, 표준 `sensor_msgs/Imu` 발행 | balance 제어 |
| `quattro_teleop` | joystick(Switch Pro Controller)/keyboard 입력 → 상위 명령 topic 변환 | 모터 직접 제어 |

참고: 현재 working tree에서 `gim6010_driver`가 개편 중이다(`git status`상 파일들이 삭제로 표시되고 `quattro_controllers`는 새로 추가되어 untracked 상태). `docs/gim6010_hardware.md`의 드라이버 구조 설명이 현재 working tree와 정확히 일치한다고 가정하지 말고, 작업 전 `git status`/`git diff`로 실제 상태를 확인한다.

### 좌표/단위 규칙

- ROS 외부 인터페이스는 SI 단위와 REP-103 좌표계(+X 전방, +Y 좌측, +Z 위쪽, 오른손 좌표계)를 사용한다. calibration GUI는 사용자 표시용으로 degree를 쓸 수 있지만 저장값과 ROS 경계는 radian이다.
- Joint 변환: `joint_position = direction * motor_position - offset`; `motor_position = direction * (joint_command + offset)`, offset 단위는 rad.
- GDS68 Direct Position/Velocity payload는 motor rotor 단위(rev, rev/s)이며, 드라이버가 8:1 감속비로 변환한다(`output = rotor * 2π / 8`). MIT payload는 output축 SI 값을 그대로 사용하며 상위 비트부터 배치하는 bit-packed 형식이다(다른 CAN Simple payload의 little-endian 규칙과 다름).
- `Set_Limits`의 velocity/current는 ROS joint rad/s가 아니라 motor rotor rev/s와 A이다.

### 제어 방식 선택이 아키텍처에 미치는 영향

`hardware_control_method`(기본값 `direct_position`, 그 외 `direct_velocity`, `direct_torque`, `mit`)는 GDS68 mode와 claim할 `ros2_control` command interface를 함께 결정한다. MIT는 전용 `MitTrajectoryController`가 다섯 interface를 모두 함께 claim해야만 유효하며, 일반 `JointTrajectoryController`가 MIT position만 claim하는 구성은 설계상 거부된다. bringup launch 인자에서 `hardware_control_method`, `controller_file`, `command_controller_name`은 함께 짝을 이루므로 하나만 바꾸면 mode switch가 깨진다.

CAN ID/bus 매핑(`can0` = 0~5, `can1` = 6~11)과 전체 joint별 표는 `docs/gim6010_hardware.md`에 있다.

## 프로젝트 규칙 (자세한 내용은 AGENTS.md 참고)

- 기존 파일을 먼저 읽고 수정한다. 요청 없이 파일을 전면 재작성하지 않는다.
- ROS 2 표준 메시지와 `ros2_control` 표준 인터페이스를 우선 사용한다. 커스텀 메시지는 표준으로 표현할 수 없을 때만 추가한다.
- 잘못된 설정이나 명령은 조용히 clamp하지 말고 명시적으로 거부하거나 보고한다.
- 하드웨어 제어 코드는 안전한 기본값, 입력 검증, timeout, stale feedback/fault 처리를 우선한다.
- C++: C++17, RAII 사용, 소유권을 가진 raw pointer 지양, ROS 계층과 CAN/프로토콜 계층 분리, encode/decode 함수는 ROS 없이 unit test 가능해야 함.
- Python: class 기반 노드, 가능하면 type hint 사용, PEP 8 준수.
- 소스 코드 주석에는 이모지를 사용하지 않는다.
- 머신별 파일(`.env`, 실제 `calibration.yaml`, `build/`, `install/`, `log/`)은 gitignore 대상이며, `.env.example`과 `calibration.yaml.example`만 버전 관리한다.

## 문서 색인

작업 주제에 맞는 문서를 먼저 읽는다. 이 파일보다 더 자세하고 최신 상태로 유지된다.

| 주제 | 문서 |
|---|---|
| 문서 전체 색인 | `docs/README.md` |
| 패키지 구조, ROS 인터페이스, 좌표계·이름 규칙 | `docs/architecture.md` |
| Docker/X11/GPU/빌드/Git 개발 환경 | `docs/development_environment.md` |
| Gazebo 시뮬레이션 | `docs/gazebo.md` |
| GIM6010-8/GDS68/CAN/`ros2_control` 하드웨어 상세 | `docs/gim6010_hardware.md` |
| 관절 영점 캘리브레이션 | `docs/calibration.md` |
| 실기 bringup과 안전 절차 | `docs/hardware_bringup.md` |
| 구현 상태 / 실기 검증 여부 | `docs/development_status.md` |
| 제조사 매뉴얼 한국어 번역본 | `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` |

문서와 코드가 충돌하면 먼저 현재 코드와 실제 하드웨어 동작을 확인한 뒤 문서를 함께 수정한다.
