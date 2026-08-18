# Quattro Gazebo 시뮬레이션

## 1. 구성

ROS 2 Jazzy와 Gazebo Harmonic을 사용한다.

핵심 패키지:

```text
quattro_gazebo
quattro_description
quattro
```

주요 파일:

```text
src/quattro_gazebo/
├── config/gazebo_controllers.yaml
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

## 3. 기본 초기 자세

Gazebo 초기 관절값은 nominal stance와 맞춘다.

```text
hip joint       :  0.0 rad
upper leg joint :  0.63356747785 rad
lower leg joint : -1.53223802029 rad
```

초기 자세가 실제 IK nominal stance와 달라지면 모델 생성 직후 큰 trajectory가 발생할 수 있으므로 함께 갱신한다.

## 4. controller

시뮬레이션에서 기본적으로 다음 controller를 사용한다.

- `joint_state_broadcaster`
- `joint_trajectory_controller`

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

속도 명령 예:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}" -r 10
```

## 6. 데이터 흐름

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

## 8. collision 원칙

현재 URDF의 STL visual/collision 구조를 기본으로 유지한다.

성능이나 접촉 안정성 문제가 실제로 확인되지 않은 상태에서 collision을 box/cylinder/sphere로 임의 단순화하지 않는다. 단순 collision이 필요하면 원본 visual geometry와 분리하여 명시적으로 관리한다.

## 9. RViz

RViz의 관절 상태와 TF는 확인할 수 있지만 Gazebo world에서 이동하는 `base_link`의 전역 위치를 RViz에서 추적하려면 별도 odometry 또는 world/odom TF가 필요하다.
