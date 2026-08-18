# Quattro 소프트웨어 아키텍처

## 1. 목적

이 문서는 Quattro ROS 2 워크스페이스의 패키지 책임, 의존 방향, ROS 인터페이스, 좌표계와 이름 규칙을 정의한다.

## 2. 패키지 구성

```text
src/
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

상위 로봇 제어 패키지.

- Forward / Inverse Kinematics
- gait generation
- 몸체 자세 목표
- IMU 기반 balance 제어
- 상위 상태와 trajectory 생성

CAN 프레임, SocketCAN, GDS68 프로토콜은 넣지 않는다.

### `quattro_description`

로봇 모델 패키지.

- URDF/Xacro
- mesh
- TF 구조
- joint/link 정의
- inertial/collision
- RViz 설정
- 실제/시뮬레이션 `ros2_control` description

Xacro가 소스 원본이며 생성된 URDF를 직접 수정하지 않는다.

### `quattro_bringup`

실제 시스템 실행 조합.

- `robot_state_publisher`
- `controller_manager`
- controller/hardware spawner
- IMU
- teleop
- gait controller
- 실기 launch 파라미터

드라이버와 제어 알고리즘 구현은 포함하지 않는다.

### `quattro_gazebo`

Gazebo Harmonic 시뮬레이션.

- world
- `gz_ros2_control`
- simulation controller YAML
- simulation launch
- Gazebo clock bridge

실제 하드웨어 제어를 포함하지 않는다.

### `gim6010_driver`

GIM6010-8 + GDS68 전용 저수준 C++ 드라이버.

```text
Gim6010Motor / MotorManager
        ↓
CAN Simple direct/MIT command routing
        ↓
Direct Position/Velocity/Torque 및 MIT encode/decode
        ↓
SocketCAN
```

Quattro joint 이름, IK/FK, URDF를 알지 않는다.

### `quattro_hardware`

Quattro joint와 실제 GIM6010-8을 연결하는 `ros2_control` 계층.

```text
ros2_control
    ↓
QuattroSystem
    ↓
joint direction / offset / safety
    ↓
gim6010_driver
```

raw CAN 프레임과 MIT bit packing은 이 패키지에서 구현하지 않는다.

### `quattro_sensors`

BNO085 등 센서 취득 계층.

센서 데이터를 표준 ROS 메시지로 발행하며 balance 제어는 수행하지 않는다.

### `quattro_teleop`

Switch Pro Controller와 keyboard 입력을 상위 ROS 명령으로 변환한다.

모터를 직접 제어하지 않는다.

## 3. 의존 방향

핵심 의존 방향은 다음과 같다.

```text
quattro
   ↓
ROS command / trajectory
   ↓
ros2_control controller
   ↓
quattro_hardware
   ↓
gim6010_driver
   ↓
SocketCAN
   ↓
GDS68 / GIM6010-8
```

역방향 의존을 만들지 않는다.

특히:

- `gim6010_driver -> quattro_hardware` 금지
- `gim6010_driver -> quattro` 금지
- `quattro -> raw CAN` 금지

## 4. ROS 2 인터페이스

표준 메시지를 우선한다.

| 기능 | 인터페이스 |
|---|---|
| 이동 명령 | `geometry_msgs/msg/Twist` |
| 몸체 자세 목표 | `geometry_msgs/msg/PoseStamped` |
| IMU | `sensor_msgs/msg/Imu` |
| 실제 관절 상태 | `sensor_msgs/msg/JointState` |
| 관절 궤적 | `trajectory_msgs/msg/JointTrajectory` |
| joystick | `sensor_msgs/msg/Joy` |
| E-stop | `std_msgs/msg/Bool` |
| 하드웨어 진단 | `diagnostic_msgs/msg/DiagnosticArray` |
| TF | `tf2_ros` / `geometry_msgs/msg/TransformStamped` |

MIT 전용 Kp/Kd/feed-forward torque 값을 `JointState` 필드에 억지로 넣지 않는다.

실기 기본 경로는 표준 `position` command interface와 GDS68 Direct Position이다. MIT를 선택할 때는 `position`, `velocity`, `kp`, `kd`, `effort` command interface를 모두 claim하는 전용 controller가 필요하며 일반 position controller 뒤에 숨기지 않는다.

## 5. 단위 규칙

ROS 외부 인터페이스는 SI 단위를 사용한다.

- 길이: `m`
- 선속도: `m/s`
- 선가속도: `m/s²`
- 관절 각도: `rad`
- 각속도: `rad/s`
- 토크: `N·m`

캘리브레이션 GUI에서 사용자 표시용 degree를 사용할 수 있지만 저장과 ROS 경계는 radian이다.

## 6. 좌표계

REP-103을 따른다.

```text
+X: 전방
+Y: 좌측
+Z: 위쪽
```

오른손 좌표계를 사용한다.

기본 TF 방향:

```text
base_footprint
└── base_link
    ├── imu_link
    ├── front_left_...
    ├── front_right_...
    ├── back_left_...
    └── back_right_...
```

localization/state estimation이 구현되기 전에는 임의의 `map`/`odom` TF를 만들지 않는다.

## 7. 이름 규칙

다리 순서와 이름:

```text
front_left
front_right
back_left
back_right
```

link는 `_link`, joint는 `_joint`로 끝낸다.

예:

```text
front_left_hip_link
front_left_hip_joint
```

CAN ID를 ROS joint 이름에 포함하지 않는다.

## 8. 관절 순서

12축 표준 순서는 다음과 같다.

```text
0  front_left_hip_joint
1  front_left_upper_leg_joint
2  front_left_lower_leg_joint
3  front_right_hip_joint
4  front_right_upper_leg_joint
5  front_right_lower_leg_joint
6  back_left_hip_joint
7  back_left_upper_leg_joint
8  back_left_lower_leg_joint
9  back_right_hip_joint
10 back_right_upper_leg_joint
11 back_right_lower_leg_joint
```

이 순서는 IK 출력, controller joint 목록, calibration과 하드웨어 매핑에서 동일하게 유지한다.

## 9. URDF/Xacro 규칙

메인 파일:

```text
src/quattro_description/urdf/quattro.urdf.xacro
```

변경 후 최소 검사:

```bash
cd /ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

실제 하드웨어와 시뮬레이션은 Xacro의 `simulation` 인자로 분리한다.

```text
simulation:=false -> quattro_hardware/QuattroSystem
simulation:=true  -> gz_ros2_control/GazeboSimSystem
```

현재 STL collision을 임의로 primitive로 바꾸지 않는다. 성능 문제가 실제로 확인된 경우에만 별도 collision 형상을 검토한다.
