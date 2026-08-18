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
    ├── quattro_gazebo/
    ├── quattro_hardware/
    ├── quattro_sensors/
    ├── quattro_teleop/
    └── gim6010_driver/
```

### `quattro`

Python 기반 상위 제어 패키지.

담당 기능:

* Forward Kinematics
* Inverse Kinematics
* 자세 제어
* 보행 궤적 생성
* gait generator
* balance control
* 로봇 상위 상태 머신
* 목표 joint position / velocity / effort 생성

CAN 통신, 모터 프로토콜, SocketCAN 관련 코드는 이 패키지에 넣지 않는다.

---

### `quattro_description`

`ament_cmake` 기반 로봇 모델 패키지.

담당 기능:

* URDF
* Xacro
* STL mesh
* TF 구조
* joint/link 정의
* collision 및 inertial 정보
* RViz 설정
* `ros2_control` robot description

필요한 경우 `<ros2_control>` 설정을 통해 다음 하드웨어 파라미터를 정의할 수 있다.

```text
CAN ID
joint direction
joint offset
joint limit
```

---

### `quattro_bringup`

전체 시스템 실행 담당.

담당 기능:

* launch 파일
* 시스템 전체 파라미터
* 실제 로봇 실행 조합
* `robot_state_publisher`
* `controller_manager`
* ros2_control controller 로딩
* 센서 노드 실행
* teleop 실행
* 상위 제어 노드 실행

드라이버 구현이나 제어 알고리즘은 포함하지 않는다.

---

### `quattro_gazebo`

Gazebo Harmonic 기반 Quattro 시뮬레이션 패키지.

담당 기능:

* Gazebo world
* `gz_ros2_control` controller 설정
* Gazebo 로봇 생성
* ROS-Gazebo clock bridge
* 시뮬레이션용 launch 파일
* Gazebo, gait controller, RViz2 실행 조합

실제 로봇 bringup, 하드웨어 드라이버 또는 보행 알고리즘은 포함하지 않는다.
시뮬레이션 전용 world, controller YAML 및 launch 파일은
`quattro_bringup`이 아니라 이 패키지에서 관리한다.

기본 실행 명령:

```bash
ros2 launch quattro_gazebo simulation.launch.py
```

GUI 없이 실행하려면 다음을 사용한다.

```bash
ros2 launch quattro_gazebo simulation.launch.py \
  headless:=true \
  use_rviz:=false
```

#### 현재 Gazebo 구현 구조

시뮬레이션 관련 파일은 다음 위치에서 관리한다.

```text
src/quattro_gazebo/
├── CMakeLists.txt
├── package.xml
├── config/
│   └── gazebo_controllers.yaml
├── launch/
│   └── simulation.launch.py
└── worlds/
    └── flat.world.sdf
```

ROS 2 Jazzy의 기본 조합인 Gazebo Harmonic과 `gz_ros2_control`을 사용한다.
Docker 이미지에는 다음 패키지를 설치한다.

```text
ros-jazzy-ros-gz-sim
ros-jazzy-ros-gz-bridge
ros-jazzy-gz-ros2-control
```

`quattro_description/urdf/quattro.urdf.xacro`는 `simulation` 인자로 실제
하드웨어와 Gazebo 하드웨어를 분기한다.

```text
simulation:=false
    → quattro_hardware/QuattroSystem

simulation:=true
    → gz_ros2_control/GazeboSimSystem
```

시뮬레이션 controller YAML은 Xacro의 `simulation_controllers` 인자로
전달한다. 기본 소스 경로는 다음과 같다.

```text
src/quattro_gazebo/config/gazebo_controllers.yaml
```

시뮬레이션에서도 기존 Xacro의 visual과 collision 정의를 사용한다. 기존에
STL을 사용하는 link의 collision을 임의로 primitive 또는 단순 mesh로
변경하지 않는다. 성능이나 물리 안정성 문제가 실제로 확인된 경우에만 별도
collision 형상을 검토한다.

Gazebo용 12축 초기 관절값은 기존 IK의 nominal stance와 일치시켜 시작 순간
급격한 관절 이동을 방지한다.

```text
hip joint        :  0.0 rad
upper leg joint  :  0.63356747785 rad
lower leg joint  : -1.53223802029 rad
```

`gazebo_controllers.yaml`에는 다음 controller를 정의한다.

- `joint_state_broadcaster`: Gazebo의 실제 관절 상태를 `/joint_states`로 발행한다.
- `joint_trajectory_controller`: 12축 position command를 받고 position과 velocity state를 사용한다.

controller manager update rate는 `100 Hz`이며 모든 시뮬레이션 ROS 노드는
`use_sim_time:=true`를 사용한다. Gazebo Transport의 `/clock`은
`ros_gz_bridge`를 통해 ROS의 `/clock`으로 전달한다.

현재 launch 실행 흐름은 다음과 같다.

```text
flat.world.sdf 로딩
    ↓
robot_state_publisher 및 /clock bridge 시작
    ↓
robot_description 토픽으로 quattro 모델 생성
    ↓
joint_state_broadcaster 로딩
joint_trajectory_controller 로딩
    ↓
quattro/gait_controller 시작
    ↓
선택적으로 RViz2 시작
```

로봇은 world 기준 `z=0.325 m`에서 생성한다. world의 physics step은
`0.001 s`, real-time factor는 `1.0`, 지면 마찰 계수는 `mu=1.0`,
`mu2=1.0`으로 설정한다.

기존 gait controller는 다음 토픽으로 시뮬레이션 controller에 12축 궤적을
전달한다.

```text
/joint_trajectory_controller/joint_trajectory
```

보행 속도 명령 예시는 다음과 같다.

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

Gazebo 관련 변경 후에는 Docker 컨테이너 안에서 다음 검사를 수행한다.

```bash
cd /ws

xacro src/quattro_description/urdf/quattro.urdf.xacro \
  simulation:=true > /tmp/quattro_sim.urdf
check_urdf /tmp/quattro_sim.urdf

gz sdf -k src/quattro_gazebo/worlds/flat.world.sdf

colcon build --symlink-install \
  --packages-select quattro_description quattro quattro_gazebo \
  --event-handlers console_direct+

ros2 launch quattro_gazebo simulation.launch.py \
  headless:=true \
  use_rviz:=false
```

실행 후 최소한 다음 항목을 확인한다.

- Gazebo에 `quattro` 모델이 생성되는지 확인한다.
- `joint_state_broadcaster`와 `joint_trajectory_controller`가 `active`인지 확인한다.
- `/clock` 값이 증가하는지 확인한다.
- `/joint_states`에 12개 관절의 실제 Gazebo 상태가 발행되는지 확인한다.
- gait controller가 12축 `JointTrajectory`를 발행하는지 확인한다.

현재 RViz2는 관절 상태와 TF를 표시한다. Gazebo world에서 이동하는
`base_link` 위치까지 RViz2에서 추적하려면 향후 odometry 또는 Gazebo pose를
기반으로 한 world/odom TF가 추가로 필요하다.

---

### `gim6010_driver`

C++ 기반 GIM6010-8 전용 저수준 모터 드라이버 패키지.

이 패키지는 Quattro 로봇 구조를 알지 않으며, GIM6010-8 액추에이터와 CAN 통신하는 기능만 담당한다.

전체 구조:

```text
GIM6010 API
    ↓
Motor abstraction
    ↓
GIM6010 protocol
    ↓
MIT Control protocol
    ↓
SocketCAN
    ↓
Linux can0
    ↓
GIM6010-8
```

담당 기능:

* Linux SocketCAN 인터페이스
* CAN socket 생성 및 종료
* CAN frame 송수신
* CAN ID 처리
* GIM6010-8 프로토콜 구현
* MIT Control 패킷 인코딩
* MIT Control 피드백 디코딩
* float ↔ integer quantization
* motor enable
* motor disable
* motor zero 설정
* position command
* velocity command
* Kp command
* Kd command
* torque command
* position feedback
* velocity feedback
* torque feedback
* 다중 모터 CAN feedback routing
* 통신 timeout 검출
* CAN 오류 검출

권장 내부 구조:

```text
gim6010_driver/
├── include/
│   └── gim6010_driver/
│       ├── can_socket.hpp
│       ├── mit_protocol.hpp
│       ├── gim6010_motor.hpp
│       └── motor_manager.hpp
├── src/
│   ├── can_socket.cpp
│   ├── mit_protocol.cpp
│   ├── gim6010_motor.cpp
│   └── motor_manager.cpp
├── test/
├── CMakeLists.txt
└── package.xml
```

#### `CanSocket`

Linux SocketCAN과 직접 통신한다.

다음 정보만 다룬다.

```text
CAN ID
DLC
CAN DATA
```

position, velocity, torque와 같은 모터 제어 의미는 알지 않는다.

#### `MitProtocol`

MIT Control packet의 직렬화 및 역직렬화를 담당한다.

처리 대상:

```text
position
velocity
Kp
Kd
torque
```

예상 인터페이스:

```cpp
MitCommand
encodeCommand(...)

MitFeedback
decodeFeedback(...)
```

#### `Gim6010Motor`

GIM6010-8 모터 한 개를 추상화한다.

예상 인터페이스:

```text
enable()
disable()
setZero()
sendCommand()
position()
velocity()
torque()
```

#### `MotorManager`

하나의 CAN bus에 연결된 여러 GIM6010-8을 관리한다.

담당 기능:

* CAN ID별 모터 관리
* feedback frame routing
* command 전송
* 모터 상태 관리

`gim6010_driver`에는 다음 내용을 넣지 않는다.

```text
FL_HIP
FR_HIP
Quattro joint 이름
URDF
IK
FK
gait generator
balance control
ros2_control SystemInterface
```

---

### `quattro_hardware`

C++ 기반 Quattro 전용 하드웨어 계층.

ROS 2의 joint 개념과 실제 GIM6010-8 모터를 연결한다.

`gim6010_driver`에 의존하며, 직접 MIT packet이나 raw CAN frame을 생성하지 않는다.

전체 구조:

```text
ros2_control
      ↓
QuattroSystem
      ↓
quattro_hardware
      ↓
gim6010_driver
      ↓
SocketCAN
      ↓
GIM6010-8
```

담당 기능:

* `hardware_interface::SystemInterface` 구현
* `read()` 구현
* `write()` 구현
* ROS joint와 motor CAN ID 매핑
* joint direction 변환
* encoder/joint zero offset 적용
* joint position limit
* joint velocity limit
* joint effort/torque limit
* ROS command → motor command 변환
* motor feedback → ROS joint state 변환
* ros2_control lifecycle 처리
* hardware watchdog
* 통신 timeout 발생 시 안전 동작
* hardware fault 처리
* 안전 정지

권장 구조:

```text
quattro_hardware/
├── include/
│   └── quattro_hardware/
│       └── quattro_system.hpp
├── src/
│   └── quattro_system.cpp
├── config/
├── quattro_hardware.xml
├── CMakeLists.txt
└── package.xml
```

`quattro_hardware`는 다음 정보를 알고 있다.

```text
front_left_hip_joint → 관절 번호 0 → CAN ID 0 → can0
front_left_upper_leg_joint → 관절 번호 1 → CAN ID 1 → can0
...
```

전체 관절 번호, CAN ID 및 CAN 인터페이스 매핑은
`12. GIM6010-8 하드웨어 규칙`의 기준표를 따른다.

예를 들어 모터 피드백은 다음 경로로 전달된다.

```text
GIM6010-8
    ↓
CAN feedback
    ↓
gim6010_driver
    ↓
position / velocity / torque
    ↓
quattro_hardware::read()
    ↓
ros2_control state interface
```

명령은 반대 방향이다.

```text
ros2_control command
    ↓
quattro_hardware::write()
    ↓
joint direction / offset 적용
    ↓
gim6010_driver
    ↓
MIT CAN command
    ↓
GIM6010-8
```

상위 보행 알고리즘, FK, IK, gait generation, balance control은 이 패키지에 넣지 않는다.

---

### `quattro_sensors`

센서 드라이버 패키지.

현재 계획:

* BNO085 IMU

IMU 출력은 ROS 2 표준 메시지를 사용한다.

```text
sensor_msgs/msg/Imu
```

권장 토픽:

```text
/imu/data
```

자세 보정이나 balance control은 이 패키지가 아니라 `quattro`에서 수행한다.

---

### `quattro_teleop`

사용자 입력 담당.

담당 기능:

* Nintendo Switch Pro Controller 입력
* `sensor_msgs/msg/Joy` 처리
* `/cmd_vel` 생성
* 모드 전환
* 보행 활성화/비활성화
* 소프트웨어 E-stop 입력

모터나 CAN을 직접 제어하지 않는다.

---

## 패키지 간 의존 관계

권장 의존 관계는 다음과 같다.

```text
quattro
   ↓
ROS 2 command interface
   ↓
ros2_control controller
   ↓
quattro_hardware
   ↓
gim6010_driver
   ↓
SocketCAN
   ↓
GIM6010-8
```

각 패키지의 책임은 다음 기준으로 구분한다.

```text
로봇을 어떻게 움직일 것인가?
        ↓
      quattro


로봇의 구조는 무엇인가?
        ↓
quattro_description


시스템을 어떻게 실행할 것인가?
        ↓
quattro_bringup


Quattro joint와 실제 모터를 어떻게 연결할 것인가?
        ↓
quattro_hardware


GIM6010-8과 어떻게 CAN 통신할 것인가?
        ↓
gim6010_driver


센서 데이터를 어떻게 가져올 것인가?
        ↓
quattro_sensors


사용자 입력을 어떻게 받을 것인가?
        ↓
quattro_teleop
```

특히 다음 의존 방향을 유지한다.

```text
quattro_hardware
        ↓
gim6010_driver
```

반대로 `gim6010_driver`가 `quattro_hardware`에 의존해서는 안 된다.

이렇게 구성하면 GIM6010 드라이버를 향후 다른 로봇에서도 재사용할 수 있다.



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
    ├── back_left_...
    └── back_right_...
```

실제 localization 또는 state estimation이 구현되기 전에는 임의의 `map` 또는 `odom` TF를 만들지 않는다.

---

## 6. 이름 규칙

ROS 내부 이름은 다음 규칙으로 통일한다.

### 다리 위치

```text
front_left
front_right
back_left
back_right
```


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

### IMU 및 조이스틱 장치 전달

BNO085 IMU의 `/dev/i2c-1`과 조이스틱의 `/dev/input`을 컨테이너에서
사용할 때는 `compose.hardware.yaml`을 함께 적용한다. 호스트의 `input`
그룹 GID를 전달한 뒤 컨테이너를 생성하거나 다시 생성한다.

```bash
cd ~/lgh_ws

INPUT_GID=$(getent group input | cut -d: -f3) \
docker compose \
  -f compose.yaml \
  -f compose.hardware.yaml \
  up -d --force-recreate dev
```

하드웨어 설정이 적용된 컨테이너에는 다음 명령으로 진입한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.hardware.yaml \
  exec dev bash
```

컨테이너 내부에서 장치 전달 여부를 확인한다.

```bash
ls -l /dev/i2c-1
ls -l /dev/input/js0
```

`docker compose up -d dev`처럼 기본 `compose.yaml`만 사용하면 IMU와
조이스틱 장치가 컨테이너에 전달되지 않는다. 실제 하드웨어 작업에서는
항상 `compose.hardware.yaml`을 함께 지정한다.

### 데스크톱 NVIDIA GPU 사용

Gazebo와 RViz2를 데스크톱 Linux에서 실행할 때는 NVIDIA GPU 하드웨어
가속을 사용한다. 호스트의 NVIDIA 드라이버와 NVIDIA Container Toolkit이
정상적으로 설치되어 있어야 한다.

`compose.yaml`의 `dev` 서비스는 다음 설정으로 모든 NVIDIA GPU와 그래픽
기능을 컨테이너에 전달한다.

```yaml
gpus: all

environment:
  NVIDIA_VISIBLE_DEVICES: all
  NVIDIA_DRIVER_CAPABILITIES: compute,utility,graphics,display
```

`LIBGL_ALWAYS_SOFTWARE=1`은 Mesa의 CPU 렌더러를 강제하므로 Gazebo 또는
RViz2를 실행하는 개발 컨테이너에 설정하지 않는다.

Ubuntu 기본 APT 저장소에서 `nvidia-container-toolkit`을 찾지 못하는 경우
NVIDIA 공식 저장소를 먼저 추가한다.

```bash
sudo apt-get update
sudo apt-get install -y --no-install-recommends \
  ca-certificates curl gnupg2

curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey \
  | sudo gpg --dearmor \
    -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg

curl -sL https://nvidia.github.io/libnvidia-container/stable/deb/nvidia-container-toolkit.list \
  | sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' \
  | sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo nvidia-ctk runtime configure --runtime=docker
sudo systemctl restart docker
```

Docker를 재시작하면 기존 컨테이너가 정지할 수 있으므로 개발 컨테이너를
다시 생성하고 GPU 인식을 확인한다.

```bash
cd ~/lgh_ws

docker compose up -d dev
docker compose exec dev nvidia-smi
```

컨테이너의 `nvidia-smi` 출력에 호스트 GPU가 표시되어야 한다. Gazebo 실행
중에는 다른 호스트 터미널에서 다음 명령으로 GPU 사용률과 Gazebo 프로세스를
확인한다.

```bash
watch -n 1 nvidia-smi
```

다음 오류가 발생하면 NVIDIA Container Toolkit 또는 Docker runtime 설정이
완료되지 않은 상태이다.

```text
failed to discover GPU vendor from CDI: no known GPU vendor found
```

이 경우 `nvidia-ctk runtime configure --runtime=docker`와 Docker 재시작을
다시 확인한다. 이 NVIDIA GPU 설정은 NVIDIA GPU가 장착된 데스크톱용이다.
Raspberry Pi처럼 NVIDIA GPU가 없는 장치에서는 `gpus: all`을 요구하는
구성으로 컨테이너를 실행하지 않는다.

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

### 관절 번호와 CAN 채널 매핑

관절 번호는 `quattro` 내부의 12축 순서이자 모터 CAN ID와 동일하게
사용한다. CAN ID `0~5`는 `can0`, CAN ID `6~11`은 `can1`에 연결한다.

| 관절 번호 | ROS 관절 이름 | CAN ID | CAN 인터페이스 |
|---:|---|---:|---|
| 0 | `front_left_hip_joint` | 0 | `can0` |
| 1 | `front_left_upper_leg_joint` | 1 | `can0` |
| 2 | `front_left_lower_leg_joint` | 2 | `can0` |
| 3 | `front_right_hip_joint` | 3 | `can0` |
| 4 | `front_right_upper_leg_joint` | 4 | `can0` |
| 5 | `front_right_lower_leg_joint` | 5 | `can0` |
| 6 | `back_left_hip_joint` | 6 | `can1` |
| 7 | `back_left_upper_leg_joint` | 7 | `can1` |
| 8 | `back_left_lower_leg_joint` | 8 | `can1` |
| 9 | `back_right_hip_joint` | 9 | `can1` |
| 10 | `back_right_upper_leg_joint` | 10 | `can1` |
| 11 | `back_right_lower_leg_joint` | 11 | `can1` |

이 매핑은 `quattro_description`의 `<ros2_control>` 설정, 머신별
`calibration.yaml`, 캘리브레이션 GUI에서 동일하게 유지한다. 관절 이름에
CAN ID를 포함하지 않는다.

### 관절 캘리브레이션 GUI

관절 영점 캘리브레이션 프로그램은 `quattro_hardware` 패키지의
`src/quattro_hardware/src/calibration_gui.cpp`에 구현한다. 실행 파일 이름은
`calibration_gui`이며 Docker 컨테이너 내부에서 다음과 같이 실행한다.

```bash
ros2 run quattro_hardware calibration_gui \
  --calibration-file /ws/src/quattro_bringup/config/calibration.yaml
```

캘리브레이션 설정 파일의 역할은 다음과 같이 구분한다.

- `/ws/src/quattro_bringup/config/calibration.yaml`: 실제 로봇에서 사용하는 머신별 캘리브레이션 값이다. Git에 저장하지 않는다.
- `src/quattro_bringup/config/calibration.yaml.example`: 실제 설정 파일을 만들기 위한 Git 관리 예제이다.
- `src/quattro_description/config/calibration.yaml`: Xacro 변환, URDF 검사, 시각화를 위한 영점 오프셋 기본값이다. 실제 로봇의 캘리브레이션 파일로 사용하지 않는다.

GUI는 Qt5로 구현하며 다음 방식으로 동작한다.

- 관절 선택 버튼에는 관절 이름과 관절 번호만 표시한다. 예: `front_left_hip_joint (0번)`
- 선택한 관절의 CAN 채널, CAN ID, direction과 불러온 offset을 별도 정보 영역에 표시한다.
- offset은 radian과 degree를 함께 표시하지만 YAML에는 radian으로 저장한다.
- `+1도`, `-1도` 버튼으로 선택한 관절만 ROS joint 좌표계 기준 1도씩 움직인다.
- 동시에 하나의 모터만 활성화하며, 다른 관절을 선택하면 기존 모터를 먼저 비활성화한다.
- 모터 활성화 전 확인 대화상자를 표시하고, 현재 encoder 위치를 읽은 뒤 그 위치를 hold한 상태에서 제어를 시작한다.
- 저장 시 목표값이 아니라 최신 실제 encoder feedback을 사용해 offset을 계산한다.
- 저장, 오류, 창 닫기, 명시적 비활성화 시 활성 모터를 비활성화한다.
- GUI에서 CAN ID `0~5`는 `can0`, CAN ID `6~11`은 `can1`을 사용하며, 중복 주소와 잘못된 설정을 거부한다.

저장되는 영점 offset 계산식은 다음과 같다.

```text
offset = direction × current_motor_position
```

`quattro_hardware`에서 사용하는 좌표 변환은 다음과 같다.

```text
joint_position = direction × motor_position - offset
motor_command = direction × (joint_command + offset)
```

관절 direction은 기존 레퍼런스 코드의 값을 그대로 사용한다.

```text
[-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]
```

캘리브레이션 실행 전에는 반드시 로봇을 지지대에 올려 다리에 하중이 걸리지
않게 하고, `controller_manager`를 포함한 다른 모든 CAN 송신 프로그램을
종료한다. 단일 모터 동작을 확인한 뒤 다음 관절로 진행한다.

Xacro의 `calibration_file` 인자는 선택한 YAML을 읽어 각 관절의
`can_interface`, `can_id`, `direction`, `offset`, `kp`, `kd`를
`<ros2_control>` 하드웨어 파라미터로 전달한다. 실제 bringup에서는 머신별
파일을 명시적으로 지정해야 한다.

GUI 실행에는 Docker의 `DISPLAY`와 X11 전달 설정이 필요하다. 한글 표시를
위해 Docker 이미지에 `fonts-noto-cjk`를 설치하며, 이미지 변경 후에는
컨테이너를 다시 빌드해야 한다.

상세 사용 절차와 문제 해결 방법은 `docs/calibration.md`를 따른다.

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
