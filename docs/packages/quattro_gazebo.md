# `quattro_gazebo`

## 역할

Gazebo Harmonic 시뮬레이션 패키지(`ament_cmake`, launch/config/world만 설치). `gz_ros2_control`을 통해 실제 하드웨어 없이 `quattro`/`quattro_description`/`quattro_controllers`를 그대로 검증한다. 실제 하드웨어 제어는 포함하지 않는다.

```text
src/quattro_gazebo/
├── worlds/flat.world.sdf              # 평평한 바닥 월드
├── config/gazebo_controllers.yaml      # direct_position(기본): joint_state_broadcaster + joint_trajectory_controller
├── config/gazebo_controllers_mit.yaml  # mit: joint_state_broadcaster + mit_trajectory_controller(effort_emulation)
├── config/gain_scheduler.yaml          # mit에서 quattro_core_ros/gain_scheduler_node 파라미터
└── launch/simulation.launch.py
```

## `simulation.launch.py`

**Launch 인자**: `use_rviz`(기본 `true`), `headless`(기본 `false`, 참이면 GUI 없이 서버만), `hardware_control_method`(기본 `direct_position`, `mit`도 가능), `use_gain_scheduler`(기본 `true`, `mit`일 때만 의미 있음).

**흐름**:

1. `GZ_SIM_RESOURCE_PATH`에 `quattro_description`의 상위 디렉터리를 추가(mesh 리소스 탐색용).
2. `ros_gz_sim`의 `gz_sim.launch.py`를 `-r`(즉시 재생) 옵션으로 include — GUI(`headless=false`) 또는 서버 전용(`headless=true`) 중 하나만 조건부 실행.
3. `/clock` 브리지(`ros_gz_bridge parameter_bridge`)로 Gazebo 시뮬레이션 시각을 ROS에 전달.
4. `robot_state_publisher`가 `simulation:=true hardware_control_method:=<arg> simulation_controllers:=<controller_file>`로 변환한 URDF를 발행 — `controller_file`은 `hardware_control_method`에 따라 `gazebo_controllers.yaml`(기본) 또는 `gazebo_controllers_mit.yaml`(mit)로 갈린다.
5. 2초 지연 후 `ros_gz_sim create`로 `robot_description` 토픽 기준 로봇을 `z=0.325`에 스폰.
6. 4초 지연 후 `joint_state_broadcaster` → (`joint_trajectory_controller` 또는 `mit_trajectory_controller`) spawner.
7. `mit`이고 `use_gain_scheduler=true`이면 5초 지연 후 `quattro_core_ros/gain_scheduler_node` 시작(`config/gain_scheduler.yaml`).
8. 6초 지연 후 `quattro/gait_controller`를 `use_sim_time:=true`, `trajectory_controller_name:=<command_controller_name>`으로 시작(파라미터는 `quattro/config/kinematics.yaml`).
9. `use_rviz`가 참이면 RViz(`quattro_description/rviz/quattro.rviz`).

지연 시간(2/4/5/6초)은 Gazebo 리소스 로딩과 `ros2_control` 하드웨어 초기화가 끝나길 기다리는 고정 대기이며, `hardware.launch.py`(실기)처럼 프로세스 종료 이벤트 기반 순차 실행은 아니다.

## `gazebo_controllers.yaml` / `gazebo_controllers_mit.yaml`

`gazebo_controllers.yaml`(기본, `direct_position`)은 `quattro_bringup/hardware_controllers.yaml`(direct_position 실기 구성)과 동일한 형태: `joint_state_broadcaster` + 표준 `joint_trajectory_controller/JointTrajectoryController`(`position` command, `position`/`velocity` state).

`gazebo_controllers_mit.yaml`(`mit`)은 `mit_trajectory_controller`(`quattro_controllers/MitTrajectoryController`)를 `command_mode: effort_emulation`으로 로드한다 — `kp`/`kd`/`position`/`velocity`/`effort`를 그대로 5개 command interface로 내보내는 실기 경로 대신, `gz_ros2_control`의 표준 `effort` interface 하나에 host에서 계산한 MIT PD 토크를 쓴다(`docs/packages/quattro_controllers.md`). 두 컨트롤러 모두 같은 `joints`/`kp`/`kd`/`command_timeout` 파라미터 이름을 쓰므로 `docs/control/gain_tuning.md`의 gain profile 설계가 실기·시뮬레이션 양쪽에서 동일하게 적용된다.

## Xacro 분기

`quattro.urdf.xacro`가 `simulation:=true`일 때 `quattro_simulation_joint` 매크로(`gz_ros2_control/GazeboSimSystem`)를 사용하도록 분기한다. 이 매크로는 이제 `hardware_control_method`(실기와 같은 Xacro 인자)도 함께 확인한다:

- `hardware_control_method != mit`(기본 `direct_position`): `position` command interface 하나만 export — 이전과 완전히 동일한 동작.
- `hardware_control_method == mit`: `effort` command interface 하나만 export(`gz_ros2_control`의 표준 `GazeboSimSystem`은 GIM6010 MIT의 kp/kd 필드를 모르므로 position/velocity/kp/kd/effort 5개를 실기처럼 그대로 내보낼 수 없다).

모든 분기에서 state interface(`position`/`velocity`/`effort`)는 동일하다.

## 관련 문서

- 시뮬레이션 실행 세부 절차: `docs/gazebo.md`
- URDF 시뮬레이션/실기 분기: `docs/packages/quattro_description.md`
- MIT effort_emulation 모드: `docs/packages/quattro_controllers.md`
- gain scheduling: `docs/packages/quattro_core_ros.md`, `docs/control/gain_tuning.md`
