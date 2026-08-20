# Quattro 문서 색인

이 저장소(`quattro_ws`)는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어 워크스페이스다.

## 전체 구조

| 문서 | 내용 |
|---|---|
| [`architecture.md`](architecture.md) | 패키지 책임, 의존 방향, ROS 인터페이스, 좌표계·이름 규칙, 12관절 표준 순서 |
| [`development_status.md`](development_status.md) | 현재 구현된 기능, 미구현/재작성 중인 패키지, 실기 검증이 필요한 이슈 |

## 패키지별 세부 구현

`docs/packages/`에 패키지마다 파일 구조, ROS 토픽/서비스/파라미터, 동작 원칙을 정리했다. 아래 패키지는 모두 구현되어 있다.

| 패키지 | 문서 |
|---|---|
| `quattro` (FK/IK, gait, gait_controller) | [`packages/quattro.md`](packages/quattro.md) |
| `quattro_description` (URDF/Xacro, mesh, RViz) | [`packages/quattro_description.md`](packages/quattro_description.md) |
| `quattro_bringup` (실기 launch, controller 구성) | [`packages/quattro_bringup.md`](packages/quattro_bringup.md) |
| `quattro_gazebo` (Gazebo Harmonic 시뮬레이션) | [`packages/quattro_gazebo.md`](packages/quattro_gazebo.md) |
| `quattro_sensors` (BNO085 IMU) | [`packages/quattro_sensors.md`](packages/quattro_sensors.md) |
| `quattro_teleop` (joystick/keyboard teleop) | [`packages/quattro_teleop.md`](packages/quattro_teleop.md) |
| `gim6010_driver` (CAN 드라이버) | [`packages/gim6010_driver.md`](packages/gim6010_driver.md) |
| `quattro_hardware` (`ros2_control` 하드웨어 계층, `QuattroSystem`) | [`packages/quattro_hardware.md`](packages/quattro_hardware.md) |

## 실행 환경

| 문서 | 내용 |
|---|---|
| [`development_environment.md`](development_environment.md) | Docker, X11, GPU, 빌드, Git 개발 환경 |
| [`gazebo.md`](gazebo.md) | Gazebo Harmonic 시뮬레이션 실행 |
| [`calibration.md`](calibration.md) | 관절 영점 캘리브레이션 |

## 도커 실행 시나리오

기본 원칙과 항목별 상세는 [`development_environment.md`](development_environment.md)가 1차 출처다. 아래는 목적별로 자주 쓰는 조합만 정리한 요약이다. 모든 명령은 저장소 루트(`~/lgh_ws`)에서 실행한다.

### A. Raspberry Pi 5 — 실기 (X11 + 하드웨어)

실물 로봇(CAN, IMU, joystick)을 다루거나 캘리브레이션 GUI를 띄울 때 사용한다.

```bash
ssh -Y <user>@<raspberry-pi-ip>
echo "$DISPLAY"   # 비어 있으면 X11 forwarding부터 확인
```

```bash
cd ~/lgh_ws
INPUT_GID=$(getent group input | cut -d: -f3) \
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.hardware.yaml \
  up -d --force-recreate dev
```

```bash
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.hardware.yaml \
  exec dev bash
```

- 캘리브레이션 GUI만 필요하고 IMU/joystick 장치 전달이 필요 없으면 `-f compose.hardware.yaml`을 빼고 `compose.yaml` + `compose.x11.yaml`만 사용해도 된다.
- `can0`, `can1`은 `compose.yaml`의 `network_mode: host`로 이미 접근 가능하므로 별도 device 매핑이 필요 없다.
- VS Code Remote SSH 터미널은 `ssh -Y` 세션과 X11 환경이 다를 수 있으므로, 컨테이너 재생성(`up -d --force-recreate`)은 `ssh -Y` 세션에서 수행한다.

### B. 데스크톱 — GPU 개발 (NVIDIA)

Gazebo/RViz를 하드웨어 가속으로 돌리며 개발할 때 사용한다. 실기 장치는 다루지 않는다.

```bash
cd ~/lgh_ws
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.nvidia.yaml \
  up -d --force-recreate dev
```

```bash
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.nvidia.yaml \
  exec dev bash
```

```bash
docker compose exec dev nvidia-smi   # GPU 인식 확인
```

Raspberry Pi에서는 `compose.nvidia.yaml`을 사용하지 않는다.

### 공통 진입 후

```bash
source /opt/ros/jazzy/setup.bash
cd /ws
colcon build --symlink-install --event-handlers console_direct+
source /ws/install/setup.bash
```

## 실행 명령어 요약

컨테이너 진입 후 자주 쓰는 실행 명령어만 모았다. 안전 절차·인자 설명 등 전체 내용은 각 문서(`packages/quattro_bringup.md`, `calibration.md`, `gazebo.md`)를 따른다.

### bringup (실기, 시나리오 A)

```bash
ros2 launch quattro_bringup hardware.launch.py
```

단일 모터 시험과 영점/gain 확인이 끝난 뒤에만 gait를 켠다.

```bash
ros2 service call /gait/enable std_srvs/srv/SetBool '{data: true}'
```

상태 확인:

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /imu/data
```

### 캘리브레이션 (실기, 시나리오 A)

```bash
colcon build --symlink-install --packages-select gim6010_driver quattro_hardware
source /ws/install/setup.bash

ros2 run quattro_hardware calibration_gui \
  --calibration-file /ws/src/quattro_bringup/config/calibration.yaml
```

캘리브레이션 GUI와 `hardware.launch.py`는 동시에 실행하지 않는다.

### Gazebo 시뮬레이션 (시나리오 B 권장)

```bash
ros2 launch quattro_gazebo simulation.launch.py
```

headless:

```bash
ros2 launch quattro_gazebo simulation.launch.py \
  headless:=true \
  use_rviz:=false
```

속도 명령 예:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

## 하드웨어 참고 자료 (GIM6010-8 / GDS68)

CAN 프로토콜은 `packages/gim6010_driver.md`, 안전 정책·활성화 절차는 `packages/quattro_hardware.md`, 실행 절차는 `packages/quattro_bringup.md`가 1차 출처다. 아래는 그 문서들의 근거 자료.

| 자료 | 내용 |
|---|---|
| `GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` | 제조사 번역 매뉴얼 (CAN Simple/오류 코드 원본) |
| `ros_odrive/`, `Steadywin-RS485-CAN-Connector/` | `gim6010_driver`/`quattro_hardware` 재설계를 위해 아키텍처만 참고한 외부 레퍼런스 프로젝트. 워크스페이스 의존성이 아니다 |

