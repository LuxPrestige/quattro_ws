# `quattro_gazebo`

## 역할

Gazebo Harmonic 시뮬레이션 패키지(`ament_cmake`, launch/config/world만 설치). `gz_ros2_control`을 통해 실제 하드웨어 없이 `quattro`/`quattro_description`을 그대로 검증한다. 실제 하드웨어 제어는 포함하지 않는다.

ROS 2 Jazzy + Gazebo Harmonic 조합을 사용한다.

핵심 패키지:

```text
quattro_gazebo
quattro_description
quattro
```

```text
src/quattro_gazebo/
├── worlds/flat.world.sdf         # 평평한 바닥 월드
├── config/gazebo_controllers.yaml # joint_state_broadcaster + joint_trajectory_controller
└── launch/simulation.launch.py
```

## 하드웨어/시뮬레이션 분기

`quattro_description/urdf/quattro.urdf.xacro`의 `simulation` 인자로 실제 하드웨어와 Gazebo를 분리한다.

```text
simulation:=false
  -> quattro_hardware/QuattroSystem

simulation:=true
  -> gz_ros2_control/GazeboSimSystem
```

`quattro.urdf.xacro`가 `simulation:=true`일 때 `quattro_simulation_joint` 매크로(`gz_ros2_control/GazeboSimSystem`)를 사용하도록 분기한다. 이 매크로는 `quattro_hardware_joint`(실기)와 동일하게 `position` command interface 하나와 `position`/`velocity`/`effort` state interface를 export하며, `command_interface`의 `min`/`max`는 실기 쪽과 같은 관절 한계 값을 사용한다(`docs/packages/quattro_description.md` 참고).

시뮬레이션 controller YAML은 Xacro의 `simulation_controllers` 인자로 전달한다. 관절마다 `position` command interface 하나만 export하며, 표준 `joint_trajectory_controller/JointTrajectoryController`가 이를 소비한다.

## 기본 초기 자세

Gazebo 초기 관절값(`quattro_simulation_joint`의 `initial_value`)은 nominal stance와 맞춘다.

```text
hip joint       :  0.0 rad
upper leg joint :  0.63356747785 rad
lower leg joint : -1.53223802029 rad
```

초기 자세가 실제 IK nominal stance(`quattro/config/kinematics.yaml`의 `nominal_height`/leg length 등)와 달라지면 모델 생성 직후 큰 trajectory가 발생할 수 있으므로 함께 갱신한다.

## `simulation.launch.py`

**Launch 인자**: `use_rviz`(기본 `true`), `headless`(기본 `false`, 참이면 GUI 없이 서버만).

**흐름**:

1. `GZ_SIM_RESOURCE_PATH`에 `quattro_description`의 상위 디렉터리를 추가(mesh 리소스 탐색용).
2. `ros_gz_sim`의 `gz_sim.launch.py`를 `-r`(즉시 재생) 옵션으로 include — GUI(`headless=false`) 또는 서버 전용(`headless=true`) 중 하나만 조건부 실행.
3. `/clock` 브리지(`ros_gz_bridge parameter_bridge`)로 Gazebo 시뮬레이션 시각을 ROS에 전달.
4. `robot_state_publisher`가 `simulation:=true simulation_controllers:=<gazebo_controllers.yaml>`로 변환한 URDF를 발행.
5. 2초 지연 후 `ros_gz_sim create`로 `robot_description` 토픽 기준 로봇을 `z=0.325`에 스폰.
6. 4초 지연 후 `joint_state_broadcaster` → `joint_trajectory_controller` spawner.
7. 6초 지연 후 `quattro/gait_controller`를 `use_sim_time:=true`, `trajectory_controller_name:=joint_trajectory_controller`으로 시작(파라미터는 `quattro/config/kinematics.yaml`).
8. `use_rviz`가 참이면 RViz(`quattro_description/rviz/quattro.rviz`).

지연 시간(2/4/6초)은 Gazebo 리소스 로딩과 `ros2_control` 하드웨어 초기화가 끝나길 기다리는 고정 대기이며, `hardware.launch.py`(실기)처럼 프로세스 종료 이벤트 기반 순차 실행은 아니다.

모든 simulation ROS 노드는 `use_sim_time:=true`를 사용하며 Gazebo `/clock`을 ROS `/clock`으로 bridge한다. controller manager update rate는 `100 Hz`이다.

## `gazebo_controllers.yaml`

`quattro_bringup/hardware_controllers.yaml`(실기 구성)과 동일한 형태: `joint_state_broadcaster` + 표준 `joint_trajectory_controller/JointTrajectoryController`(`position` command, `position`/`velocity` state).

- `joint_state_broadcaster`
- `joint_trajectory_controller`

## 실행

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

## 데이터 흐름

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

Gait controller는 `cmd_vel` -> `GaitGenerator`(발끝 목표 궤적) -> IK(관절 목표 각도) 순서로 계산하고, 결과를 다음 controller 토픽에 위치(`position`)만 담아 발행한다.

```text
/joint_trajectory_controller/joint_trajectory
```

## collision 원칙

Visual은 기존 STL mesh를 유지하고 collision은 명시적인 primitive geometry로 분리한다. Base와 upper leg는 box, hip과 lower leg는 cylinder, foot은 sphere를 사용한다.

## RViz

RViz의 관절 상태와 TF는 확인할 수 있지만 Gazebo world에서 이동하는 `base_link`의 전역 위치를 RViz에서 추적하려면 별도 odometry 또는 world/odom TF가 필요하다.

## 검증

Gazebo 관련 변경 후 최소한 다음을 수행한다.

```bash
cd /quattro_ws

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

## 관련 문서

- URDF 시뮬레이션/실기 분기, 관절 한계: `docs/packages/quattro_description.md`
- Docker/X11/GPU 실행 환경: `docs/development_environment.md`
