# Quattro 소프트웨어 아키텍처

## 1. 목적

이 문서는 Quattro ROS 2 워크스페이스의 패키지 책임, 의존 방향, 실기 startup 경계와 ROS 인터페이스를 정의한다.

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

FK/IK, gait generation, body pose, IMU balance, 상위 상태와 `JointTrajectory` 생성을 담당한다. raw CAN이나 GDS68 protocol을 직접 다루지 않는다.

### `quattro_description`

URDF/Xacro, mesh, TF, joint/link, inertial/collision, 실제/시뮬레이션 `ros2_control` description을 소유한다.

### `quattro_bringup`

실제 시스템의 프로세스 실행과 startup orchestration을 담당한다. 모터 프로토콜이나 실시간 `read()`/`write()`는 구현하지 않는다.

### `quattro_hardware`

`hardware_interface::SystemInterface` 구현인 `QuattroSystem`을 소유한다. joint 좌표 변환, GIM6010 활성화 시퀀스, encoder 동기화, runtime watchdog과 safe stop을 담당한다.

### `gim6010_driver`

ROS 비의존 저수준 C++ 드라이버다. SocketCAN, CAN Simple/MIT encode/decode, 모터별 최신 heartbeat/encoder 상태와 다중 bus routing을 담당한다. Quattro joint 이름과 URDF를 알지 않는다.

### 기타

- `quattro_gazebo`: Gazebo Harmonic simulation
- `quattro_sensors`: BNO085 등 센서 취득
- `quattro_teleop`: Switch Pro Controller/keyboard 입력 변환

## 3. 의존 방향

```text
quattro
   ↓
JointTrajectory / ROS command
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

역방향 의존은 금지한다.

## 4. 실기 Position Control 계약

Quattro 실기는 `Position Control + Pos Filter`를 사용한다.

실기에서 확인된 startup 특성은 다음과 같다.

```text
Set_Limits
  ↓
Set_Pos_Gain
  ↓
Set_Vel_Gains
  ↓
Set_Controller_Mode
  control_mode = Position Control
  input_mode   = Pos Filter
  ↓
Set_Axis_State
  requested_state = Closed Loop Control
  ↓
GIM6010 자체 현재 축 Hold
  ↓
Closed Loop 이후 새 EncoderEstimate 수신
  ↓
ROS joint state/command 기준 동기화
```

중요한 제약:

- Closed Loop 이전 EncoderEstimate 위치값은 startup 위치로 사용하지 않는다.
- `0x009 Get_Encoder_Estimates`는 약 10 ms 주기로 자동 수신하며 정상 runtime에서 요청하지 않는다.
- Closed Loop 진입 자체로 축이 고정되므로 startup `Set_Input_Pos(current)`는 사용하지 않는다.
- Closed Loop 이후 수신된 새 encoder를 ROS 위치 기준으로 사용한다.

## 5. lifecycle 책임

### `QuattroSystem::on_configure()`

목표 역할:

```text
CAN open
→ static mapping 확인
→ heartbeat 기반 motor 존재/fault 확인
→ Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position Control + Pos Filter 설정
```

Closed Loop 전 encoder 위치를 초기 위치로 사용하지 않는다.

### `QuattroSystem::on_activate()`

목표 역할:

```text
모터별 Closed Loop 요청
→ Heartbeat로 Closed Loop 확인
→ 그 이후 도착한 새 EncoderEstimate 대기
→ motor coordinate -> ROS joint coordinate 변환
→ state.position 및 command 기준 동기화
→ 12축 성공 후 ACTIVE
```

한 축이라도 실패하면 partial activation을 유지하지 않고 전체를 안전 상태로 전환한다.

### `read()` / `write()`

- `read()`: 자동 broadcast된 encoder/heartbeat를 `poll()`로 수신하고 state interface를 갱신한다.
- `write()`: 상위 controller의 position command를 `Set_Input_Pos`로 전송한다.
- startup 중에는 `Set_Input_Pos`를 Safe Start 수단으로 사용하지 않는다.

## 6. Bringup 경계

`quattro_bringup`은 프로세스/컨트롤러 순서를 관리하고, 실제 GIM6010 lifecycle은 `QuattroSystem`이 책임진다.

목표 startup:

```text
robot_state_publisher + controller_manager
  ↓
QuattroSystem configure/activate
  ↓
post-Closed-Loop encoder sync 완료
  ↓
joint_state_broadcaster
  ↓
joint_trajectory_controller
  ↓
READY / HOLD
```

Gait는 기본 OFF이며 hardware bringup과 분리한다.

## 7. ROS 2 인터페이스

| 기능 | 인터페이스 |
|---|---|
| 이동 명령 | `geometry_msgs/msg/Twist` |
| 몸체 자세 목표 | `geometry_msgs/msg/PoseStamped` |
| IMU | `sensor_msgs/msg/Imu` |
| 실제 관절 상태 | `sensor_msgs/msg/JointState` |
| 관절 궤적 | `trajectory_msgs/msg/JointTrajectory` |
| joystick | `sensor_msgs/msg/Joy` |
| E-stop | `std_msgs/msg/Bool` |
| TF | `tf2_ros` |

## 8. 단위와 좌표계

ROS 경계는 SI 단위를 사용하며 REP-103을 따른다.

```text
+X: 전방
+Y: 좌측
+Z: 위쪽
```

관절 각도는 rad, 각속도는 rad/s, 토크는 N·m이다.

## 9. 관절 순서

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

CAN mapping, calibration, controller joint list에서 이 순서를 유지한다.

## 10. URDF/Xacro 검증

```bash
cd /ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

```text
simulation:=false -> quattro_hardware/QuattroSystem
simulation:=true  -> gz_ros2_control/GazeboSimSystem
```
