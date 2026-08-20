# Quattro Gazebo 시뮬레이션

## 1. 구성

ROS 2 Jazzy와 Gazebo Harmonic을 사용한다.

핵심 패키지:

```text
quattro_gazebo
quattro_description
quattro
quattro_controllers   # hardware_control_method=mit일 때만
quattro_core_ros      # hardware_control_method=mit일 때만
```

주요 파일:

```text
src/quattro_gazebo/
├── config/gazebo_controllers.yaml       # direct_position(기본)
├── config/gazebo_controllers_mit.yaml   # mit
├── config/gain_scheduler.yaml           # mit에서 gain_scheduler_node 파라미터
├── launch/simulation.launch.py
└── worlds/flat.world.sdf
```

## 2. 하드웨어 분기

`quattro_description/urdf/quattro.urdf.xacro`의 `simulation` 인자로 실제 하드웨어와 Gazebo를 분리한다.

```text
simulation:=false
  -> quattro_hardware/QuattroSystem

simulation:=true
  -> gz_ros2_control/GazeboSimSystem
```

시뮬레이션 controller YAML은 Xacro의 `simulation_controllers` 인자로 전달한다.

### `hardware_control_method`가 시뮬레이션에도 영향을 준다

`simulation.launch.py`의 `hardware_control_method` 인자(기본 `direct_position`, 기존 동작과 완전히 동일)가 `simulation:=true`일 때 Xacro에 그대로 전달된다.

```text
hardware_control_method:=direct_position (기본)
  -> quattro_simulation_joint 매크로가 position command interface 하나만 export
  -> joint_trajectory_controller/JointTrajectoryController (표준)

hardware_control_method:=mit
  -> quattro_simulation_joint 매크로가 effort command interface 하나만 export
     (gz_ros2_control 표준 GazeboSimSystem은 GIM6010 MIT의 kp/kd 필드를
     모른다 -- position/velocity/effort 표준 interface만 이해한다)
  -> quattro_controllers/MitTrajectoryController를
     command_mode=effort_emulation으로 구동:
     tau = clamp(kp*(q_des-q) + kd*(qd_des-qd) + effort_ff, ±max_emulated_torque_Nm)
     를 host에서 계산해 Gazebo의 effort interface로 그대로 쓴다 -- 실제
     GDS68 MIT 펌웨어가 계산하는 것과 동일한 식이다(docs/packages/
     quattro_controllers.md, docs/control/gain_tuning.md).
```

`direct_velocity`/`direct_torque`는 시뮬레이션에서 지원하지 않는다(실기 전용).

### `mit` 모드에서만 추가로 실행되는 노드

`hardware_control_method:=mit`일 때만(`use_gain_scheduler:=true`, 기본값):

- `quattro_core_ros/gain_scheduler_node` — `quattro_core::FsmGainTable`(swing/stance gain profile)을 `mit_trajectory_controller`의 런타임 `kp`/`kd` 파라미터로 밀어넣는다. `docs/packages/quattro_core_ros.md`, `docs/control/gain_tuning.md`.
- `quattro/gait_controller`가 다리별 swing 상태를 `swing/<leg>`(`std_msgs/Bool`)로 발행 — `gain_scheduler_node`가 구독한다.

## 3. 기본 초기 자세

Gazebo 초기 관절값은 nominal stance와 맞춘다.

```text
hip joint       :  0.0 rad
upper leg joint :  0.63356747785 rad
lower leg joint : -1.53223802029 rad
```

초기 자세가 실제 IK nominal stance와 달라지면 모델 생성 직후 큰 trajectory가 발생할 수 있으므로 함께 갱신한다.

## 4. controller

시뮬레이션에서 기본(`hardware_control_method=direct_position`)으로 다음 controller를 사용한다.

- `joint_state_broadcaster`
- `joint_trajectory_controller`

`hardware_control_method=mit`이면 대신:

- `joint_state_broadcaster`
- `mit_trajectory_controller`(`quattro_controllers/MitTrajectoryController`, `command_mode=effort_emulation`)

controller manager update rate는 `100 Hz`이다.

모든 simulation ROS 노드는 `use_sim_time:=true`를 사용하며 Gazebo `/clock`을 ROS `/clock`으로 bridge한다.

## 5. 실행

GUI 포함:

```bash
ros2 launch quattro_gazebo simulation.launch.py
```

headless:

```bash
ros2 launch quattro_gazebo simulation.launch.py \
  headless:=true \
  use_rviz:=false
```

MIT 모드(gain scheduling 포함):

```bash
ros2 launch quattro_gazebo simulation.launch.py \
  headless:=true \
  use_rviz:=false \
  hardware_control_method:=mit
```

속도 명령 예:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

## 6. 데이터 흐름

`direct_position`(기본):

```text
flat.world.sdf
    ↓
robot_state_publisher + /clock bridge
    ↓
Gazebo에 quattro 생성
    ↓
gz_ros2_control
    ↓
joint_state_broadcaster
joint_trajectory_controller
    ↓
quattro gait_controller
```

Gait controller 출력은 다음 controller 토픽으로 전달된다.

```text
/joint_trajectory_controller/joint_trajectory
```

`mit`:

```text
flat.world.sdf
    ↓
robot_state_publisher + /clock bridge
    ↓
Gazebo에 quattro 생성 (effort command interface)
    ↓
gz_ros2_control
    ↓
joint_state_broadcaster
mit_trajectory_controller (command_mode=effort_emulation)
    ↓                                    ↑
quattro gait_controller ---- swing/<leg> -+
    ↓                                    |
mit_trajectory_controller/joint_trajectory   quattro_core_ros/gain_scheduler_node
                                              (kp/kd via set_parameters)
```

## 6a. 실기 검증 없이 확인된 사항 (2026-08-20, 이 개발 환경에서 실행)

GPU/렌더링 없는 컨테이너에서 `gz sim -s`(headless, physics-only)로 실제 실행해 확인했다 — 정적 Xacro/URDF 검사가 아니라 라이브 실행 결과다.

- `hardware_control_method:=mit`: `GazeboSimSystem`이 12관절 모두 `effort` command interface로 로드, `mit_trajectory_controller` 활성화 성공, `/joint_states`가 ~80~90 Hz로 발행됨.
- 초기 자세 전환 후 `front_left_hip_joint` 위치가 목표값(0.0 rad) 근처로 수렴 — effort_emulation MIT PD가 Gazebo 물리엔진을 통해 실제로 관절을 제어하고 있음을 확인.
- `/cmd_vel`로 전진 명령을 주면 `gain_scheduler_node`가 `/swing/<leg>`를 구독해 대각선 트롯 패턴(`front_left`+`back_right` 쌍 vs `front_right`+`back_left` 쌍)에 맞춰 `mit_trajectory_controller`의 `kp`를 실시간으로 `[10,10,10, 20,20,20, 20,20,20, 10,10,10]`처럼 전환하는 것을 `ros2 param get`으로 직접 관찰함.
- `hardware_control_method:=direct_position`(기본값, 변경 전 동작): 동일 환경에서 `joint_trajectory_controller`가 정상 활성화되어 회귀가 없음을 확인.

GUI 렌더링(RViz, Gazebo GUI)은 이 환경에 GPU가 없어 검증하지 않았다 — physics-only headless 실행만 확인했다.

## 7. 검증

Gazebo 관련 변경 후 최소한 다음을 수행한다.

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

실행 후 확인:

```bash
ros2 control list_controllers
ros2 topic hz /clock
ros2 topic hz /joint_states
ros2 topic hz /joint_trajectory_controller/joint_trajectory
```

최소 정상 조건:

- Gazebo에 `quattro` 모델 생성
- 두 controller가 `active`
- `/clock` 증가
- `/joint_states` 발행
- gait controller의 12축 trajectory 발행

`mit` 모드 추가 확인:

```bash
ros2 control list_controllers            # mit_trajectory_controller가 active
ros2 topic hz /mit_trajectory_controller/joint_trajectory
ros2 topic echo /swing/front_left --once
ros2 param get /mit_trajectory_controller kp   # gain_scheduler_node가 push한 값
```

## 8. collision 원칙

현재 URDF의 STL visual/collision 구조를 기본으로 유지한다.

성능이나 접촉 안정성 문제가 실제로 확인되지 않은 상태에서 collision을 box/cylinder/sphere로 임의 단순화하지 않는다. 단순 collision이 필요하면 원본 visual geometry와 분리하여 명시적으로 관리한다.

## 9. RViz

RViz의 관절 상태와 TF는 확인할 수 있지만 Gazebo world에서 이동하는 `base_link`의 전역 위치를 RViz에서 추적하려면 별도 odometry 또는 world/odom TF가 필요하다.
