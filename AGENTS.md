# AGENTS.md

## 1. 프로젝트 개요

이 저장소는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어를 개발하기 위한 워크스페이스이다.

프로젝트는 기존 코드를 그대로 이전하는 방식이 아니라, ROS 2 표준 구조와 `ros2_control`을 기준으로 처음부터 다시 설계한다.

### 개발 환경

- 워크스페이스 이름: `lgh_ws`
- ROS 2 배포판: Jazzy
- OS: Ubuntu Server 24.04
- SBC: Raspberry Pi 5
- 개발 환경: Docker
- 컨테이너 내부 워크스페이스: `/ws`
- 모터: GIM6010-8 액추에이터 12개
- 모터 통신: SocketCAN / CAN Simple / MIT Control
- 기본 CAN bitrate: `500000`
- IMU: BNO085
- 시각화: RViz2

---

## 2. 기본 개발 원칙

1. 모든 ROS 2 개발과 빌드는 Docker 컨테이너 내부에서 수행한다.
2. 기존 파일이 존재하는 경우 먼저 읽고 분석한 뒤 수정한다.
3. 사용자의 명시적인 요청 없이 기존 파일을 전면 재작성하지 않는다.
4. ROS 2 표준 메시지와 표준 인터페이스를 우선 사용한다.
5. 커스텀 메시지는 표준 메시지 또는 `ros2_control`로 표현할 수 없는 경우에만 추가한다.
6. ROS 외부 인터페이스는 SI 단위를 사용한다.
7. URDF/Xacro 변경 후 반드시 Xacro 변환과 URDF 검사를 수행한다.
8. 하드웨어 제어에서는 안전 동작을 최우선으로 한다.
9. 소스 코드 주석에는 이모지를 사용하지 않는다.
10. Python 노드는 객체지향 구조로 작성하고, C++ 하드웨어 계층은 ROS 로직과 프로토콜 로직을 분리한다.

---

## 3. 저장소 구조

목표 패키지 구조는 다음과 같다.

```text
lgh_ws/
├── AGENTS.md
├── README.md
├── compose.yaml
├── compose.hardware.yaml
├── docker/
│   ├── Dockerfile
│   └── entrypoint.sh
├── docs/
├── scripts/
└── src/
    ├── quattro/
    ├── quattro_description/
    ├── quattro_bringup/
    ├── quattro_hardware/
    ├── quattro_sensors/
    └── quattro_teleop/
```

### `quattro`

Python 기반 상위 제어 패키지.

담당 기능:

- Forward Kinematics
- Inverse Kinematics
- 자세 제어
- 보행 궤적 생성
- gait generator
- balance control
- 로봇 상위 상태 머신

CAN 통신이나 모터 드라이버 코드는 이 패키지에 넣지 않는다.

### `quattro_description`

`ament_cmake` 기반 로봇 모델 패키지.

담당 기능:

- URDF
- Xacro
- STL mesh
- TF 구조
- RViz 설정
- 추후 `ros2_control` robot description

### `quattro_bringup`

전체 시스템 실행 담당.

담당 기능:

- launch 파일
- 시스템 전체 파라미터
- 실제 로봇 실행 조합

### `quattro_hardware`

C++ 기반 하드웨어 계층.

담당 기능:

- SocketCAN
- GIM6010-8 CAN Simple 프로토콜
- MIT Control 패킷 인코딩/디코딩
- 모터 피드백
- `ros2_control HardwareInterface`
- 하드웨어 안전 로직

상위 보행 알고리즘과 IK를 이 패키지에 넣지 않는다.

### `quattro_sensors`

센서 드라이버 패키지.

현재 계획:

- BNO085 IMU

IMU 출력은 다음 표준 메시지를 사용한다.

```text
sensor_msgs/msg/Imu
```

권장 토픽:

```text
/imu/data
```

### `quattro_teleop`

사용자 입력 담당.

담당 기능:

- Nintendo Switch Pro Controller 입력
- `sensor_msgs/msg/Joy` 처리
- `/cmd_vel` 생성
- 모드 전환
- 소프트웨어 E-stop 입력

---

## 4. ROS 2 인터페이스 규칙

가능하면 다음 표준 메시지를 사용한다.

| 용도 | ROS 2 인터페이스 |
|---|---|
| 로봇 이동 속도 명령 | `geometry_msgs/msg/Twist` |
| IMU | `sensor_msgs/msg/Imu` |
| 실제 관절 상태 | `sensor_msgs/msg/JointState` |
| 관절 궤적 | `trajectory_msgs/msg/JointTrajectory` |
| 오도메트리 | `nav_msgs/msg/Odometry` |
| 하드웨어 진단 | `diagnostic_msgs/msg/DiagnosticArray` |
| 조이스틱 | `sensor_msgs/msg/Joy` |
| TF | `tf2_ros` |
| 좌표 변환 | `geometry_msgs/msg/TransformStamped` |

### 단위

ROS 외부 인터페이스에서는 다음 단위를 사용한다.

- 위치 각도: `rad`
- 각속도: `rad/s`
- 토크: `N·m`
- 길이: `m`
- 선속도: `m/s`
- 선가속도: `m/s²`

표준 ROS 토픽에 degree 값을 노출하지 않는다.

사람이 직접 조정하는 캘리브레이션 툴 내부에서는 필요 시 degree를 사용할 수 있지만, 하드웨어 및 ROS 표준 인터페이스 경계에서는 radian으로 변환한다.

---

## 5. 좌표계 규칙

ROS REP-103 관례를 따른다.

로봇 body frame 기준:

```text
+X : 전방
+Y : 좌측
+Z : 위쪽
```

모든 좌표계는 오른손 좌표계를 사용한다.

권장 TF 구조:

```text
base_footprint
└── base_link
    ├── imu_link
    ├── front_left_...
    ├── front_right_...
    ├── rear_left_...
    └── rear_right_...
```

실제 localization 또는 state estimation이 구현되기 전에는 임의의 `map` 또는 `odom` TF를 만들지 않는다.

---

## 6. 이름 규칙

ROS 내부 이름은 다음 규칙으로 통일한다.

### 다리 위치

```text
front_left
front_right
rear_left
rear_right
```

새 코드에서는 `back_*`과 `rear_*`를 섞지 않는다.

현재 CAD에서 내보낸 STL 파일 중 `back_left_hip.stl`, `back_right_hip.stl`은 파일 이름을 그대로 유지할 수 있지만, ROS link/joint 이름은 앞으로 `rear_*`를 사용한다.

### Link 이름

반드시 `_link`로 끝낸다.

예:

```text
front_left_hip_link
front_left_upper_leg_link
front_left_lower_leg_link
```

### Joint 이름

반드시 `_joint`로 끝낸다.

예:

```text
front_left_hip_joint
front_left_upper_leg_joint
front_left_lower_leg_joint
```

모터 ID를 joint 이름에 넣지 않는다.

모터 ID와 joint 대응은 하드웨어 설정 파일 또는 `ros2_control` 하드웨어 파라미터에서 관리한다.

---

## 7. `quattro_description` 현재 구조

현재 우선 개발 대상은 `quattro_description`이다.

현재 mesh 경로:

```text
src/quattro_description/meshes/stl/
```

현재 STL 파일:

```text
back_left_hip.stl
back_right_hip.stl
body_frame.stl
front_left_hip.stl
front_right_hip.stl
left_lower_leg.stl
left_upper_leg.stl
right_lower_leg.stl
right_upper_leg.stl
```

메인 Xacro 파일:

```text
src/quattro_description/urdf/quattro.urdf.xacro
```

STL은 다음 형식으로 참조한다.

```text
package://quattro_description/meshes/stl/<파일명>.stl
```

현재는 같은 STL을 `<visual>`과 `<collision>`에 모두 사용한다.

예:

```xml
<visual>
  <geometry>
    <mesh filename="package://quattro_description/meshes/stl/front_left_hip.stl"/>
  </geometry>
</visual>

<collision>
  <geometry>
    <mesh filename="package://quattro_description/meshes/stl/front_left_hip.stl"/>
  </geometry>
</collision>
```

사용자의 요청 없이 collision을 box, cylinder, sphere 등의 primitive로 변경하지 않는다.

추후 시뮬레이션 성능 문제가 발생하면 별도의 단순 collision mesh를 검토한다.

---

## 8. URDF/Xacro 규칙

Xacro 파일을 소스 원본으로 사용한다.

```text
*.urdf.xacro
```

Xacro에서 생성된 `.urdf` 파일은 검사 결과물이며 직접 수정하지 않는다.

`quattro_description`을 수정한 후 반드시 다음 명령을 실행한다.

```bash
cd /ws

xacro \
  src/quattro_description/urdf/quattro.urdf.xacro \
  > /tmp/quattro.urdf

check_urdf /tmp/quattro.urdf
```

두 명령 중 하나라도 실패하면 다음 단계로 진행하지 않는다.

---

## 9. RViz2 현재 상태

현재 launch 파일:

```text
src/quattro_description/launch/display.launch.py
```

실행 대상:

```text
robot_state_publisher
joint_state_publisher_gui
rviz2
```

데이터 흐름:

```text
joint_state_publisher_gui
        │
        │ /joint_states
        ▼
robot_state_publisher
        │
        ├── /tf
        ├── /tf_static
        └── /robot_description
                │
                ▼
              RViz2
```

현재 `robot_state_publisher`는 Xacro를 정상적으로 파싱하며 다음 로그까지 확인했다.

```text
Robot initialized
```

현재 문제는 URDF가 아니라 Docker GUI 전달이다.

RViz2와 `joint_state_publisher_gui` 실행 시 다음 오류가 발생한다.

```text
qt.qpa.xcb: could not connect to display
```

따라서 해당 오류를 해결할 때 URDF를 임의로 수정하지 않는다.

우선 Docker의 `DISPLAY`, X11 forwarding, Xauthority 설정을 확인한다.

### 현재 추가 경고

다음 KDL 경고도 발생한다.

```text
The root link base_link has an inertia specified in the URDF,
but KDL does not support a root link with an inertia.
```

향후 관성이 없는 dummy root link인 `base_footprint`를 추가하고 `base_link`와 fixed joint로 연결하는 방향을 사용한다.

---

## 10. Docker 개발 방법

호스트에서 개발 컨테이너 실행:

```bash
cd ~/lgh_ws

docker compose up -d dev
docker compose exec dev bash
```

컨테이너 내부 기본 위치:

```text
/ws
```

ROS 환경 확인:

```bash
source /opt/ros/jazzy/setup.bash
```

빌드 후:

```bash
source /ws/install/setup.bash
```

전체 빌드:

```bash
cd /ws

colcon build \
  --symlink-install \
  --event-handlers console_direct+
```

특정 패키지만 빌드:

```bash
cd /ws

colcon build \
  --symlink-install \
  --packages-select <package_name> \
  --event-handlers console_direct+
```

`quattro_description` 빌드:

```bash
cd /ws

colcon build \
  --symlink-install \
  --packages-select quattro_description \
  --event-handlers console_direct+

source /ws/install/setup.bash
```

GUI 없이 description 테스트:

```bash
ros2 launch quattro_description display.launch.py \
  use_rviz:=false \
  use_gui:=false
```

확인할 토픽:

```text
/joint_states
/robot_description
/tf
/tf_static
```

---

## 11. Raspberry Pi와 GitHub를 이용한 개발 방식

이 저장소를 여러 개발 PC와 Raspberry Pi에서 동일하게 사용하기 위해 GitHub를 기준 저장소로 사용한다.

원격 저장소:

```text
https://github.com/LuxPrestige/lgh_ws.git
```

### 다른 PC에서 작업을 끝낸 뒤

현재 작업 내용을 먼저 커밋하고 GitHub에 push한다.

```bash
cd ~/lgh_ws

git status
git add -A
git commit -m "작업 내용 요약"
git push origin main
```

### Raspberry Pi에서 처음 가져올 때

```bash
cd ~

git clone https://github.com/LuxPrestige/lgh_ws.git
cd ~/lgh_ws
```

저장소가 이미 존재한다면 다시 clone하지 않는다.

대신 다음을 사용한다.

```bash
cd ~/lgh_ws

git status
git pull origin main
```

로컬에 아직 커밋하지 않은 수정 사항이 있다면 `git pull` 전에 먼저 커밋하거나 별도로 보관한다.

### Raspberry Pi에서 수정 후 다시 GitHub에 반영

```bash
cd ~/lgh_ws

git status
git add -A
git commit -m "feat: 변경 내용"
git push origin main
```

다른 PC에서는 이후 다음 명령으로 최신 상태를 가져온다.

```bash
cd ~/lgh_ws
git pull origin main
```

### 머신별 파일

다음과 같은 로컬 전용 값은 Git에 직접 저장하지 않는다.

- `.env`
- 실제 모터 캘리브레이션 오프셋
- 머신별 CAN 설정
- 로그
- `build/`
- `install/`
- `log/`

필요한 경우 다음처럼 예제 파일만 Git에 저장한다.

```text
.env.example
calibration.yaml.example
```

---

## 12. GIM6010-8 하드웨어 규칙

로봇은 GIM6010-8 액추에이터 12개를 사용한다.

통신:

- SocketCAN
- CAN Simple
- 기본 bitrate `500000`
- MIT Control 사용 예정

저수준 구현 위치:

```text
src/quattro_hardware/
```

MIT 제어 입력:

- 목표 위치
- 목표 속도
- Kp
- Kd
- feed-forward torque

하드웨어 제어 내부 단위는 SI 단위를 사용한다.

실제 `/joint_states`는 명령값이 아니라 실제 모터 피드백을 나타내야 한다.

---

## 13. `ros2_control` 방향

최종 하드웨어 계층은 `ros2_control`을 사용한다.

`quattro_hardware`는 최종적으로 다음 인터페이스를 구현하는 방향으로 개발한다.

```text
hardware_interface::SystemInterface
```

목표 state interface:

- position
- velocity
- effort

초기 command interface:

- position

MIT 고유의 Kp, Kd, feed-forward torque는 표준 `JointState`에 억지로 넣지 않는다.

`/joint_states`를 명령 토픽으로 사용하지 않는다.

---

## 14. 하드웨어 안전 요구사항

GIM6010-8 제어를 구현할 때 다음을 반드시 지킨다.

- 시작 시 임의의 목표 위치를 바로 전송하지 않는다.
- actuator 활성화 전 현재 위치를 먼저 읽는다.
- 현재 위치에서 hold한 뒤 제어를 시작한다.
- command timeout을 구현한다.
- stale feedback을 감지한다.
- Safe Start를 구현한다.
- 정상 종료 시 안전한 상태로 전환한다.
- node ID를 검증한 뒤 전송한다.
- position, velocity, torque, Kp, Kd 입력 범위를 검증한다.
- 잘못된 설정을 무조건 clamp해서 숨기지 않는다.
- CAN 오류를 진단 메시지 또는 명확한 로그로 보고한다.

하드웨어 시험 순서:

1. SocketCAN 인터페이스 확인
2. heartbeat 수신 확인
3. encoder feedback 확인
4. 단일 모터 상태 전환 확인
5. 현재 위치 hold
6. 작은 각도 범위에서 단일 모터 이동
7. timeout 및 종료 동작 검증
8. 여러 모터로 확대
9. 12축 전체 구성

단일 모터가 충분히 검증되기 전에 12개 모터를 동시에 제어하지 않는다.

---

## 15. 코딩 스타일

### Python

- Python 3 사용
- ROS 노드는 class 기반으로 작성
- 가능한 경우 type hint 사용
- PEP 8 준수
- 코드 주석에 이모지 사용 금지

### C++

- 기본 C++17
- RAII 사용
- 소유권을 가진 raw pointer 지양
- ROS 코드와 CAN 프로토콜 코드를 분리
- 패킷 변환 함수는 ROS 없이 unit test 가능하도록 작성

### ROS 2

- parameter는 사용 전에 반드시 선언
- 로봇 설정값은 가능한 YAML로 분리
- 절대 파일 경로를 하드코딩하지 않는다.
- 패키지 리소스는 package share 경로를 사용한다.
- 표준 ROS 토픽 이름을 우선한다.

---

## 16. 현재 개발 순서

명시적인 지시가 없다면 다음 순서로 진행한다.

1. `quattro_description` 완성
2. STL 모델 시각 검증
3. 12개 joint origin 검증
4. 12개 joint axis 검증
5. joint limit 검증
6. TF tree 검증
7. `base_footprint` 구조 정리
8. `ros2_control` description 추가
9. mock hardware 테스트
10. `quattro_hardware` 구현
11. GIM6010-8 단일 모터 테스트
12. 12축 하드웨어로 확대
13. BNO085 `quattro_sensors` 구현
14. FK/IK 구현
15. 자세 제어 구현
16. gait generator 구현
17. teleop 구현
18. 전체 bringup 구성

`quattro_description`과 하드웨어 인터페이스가 검증되기 전에 보행 알고리즘 구현으로 건너뛰지 않는다.

---

## 17. 현재 바로 이어서 할 작업


작업을 시작하기 전에 현재 Git 상태와 관련 파일을 확인한다.

```bash
git status
```

파일 수정 후에는 가능한 범위에서 빌드와 검증 명령을 실행하고 결과를 보고한다.
