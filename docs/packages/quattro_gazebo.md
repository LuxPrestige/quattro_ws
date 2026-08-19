# `quattro_gazebo`

## 역할

Gazebo Harmonic 시뮬레이션 패키지(`ament_cmake`, launch/config/world만 설치). `gz_ros2_control`을 통해 실제 하드웨어 없이 `quattro`/`quattro_description`/`quattro_controllers`를 그대로 검증한다. 실제 하드웨어 제어는 포함하지 않는다.

```text
src/quattro_gazebo/
├── worlds/flat.world.sdf          # 평평한 바닥 월드
├── config/gazebo_controllers.yaml  # joint_state_broadcaster + joint_trajectory_controller
└── launch/simulation.launch.py
```

## `simulation.launch.py`

**Launch 인자**: `use_rviz`(기본 `true`), `headless`(기본 `false`, 참이면 GUI 없이 서버만).

**흐름**:

1. `GZ_SIM_RESOURCE_PATH`에 `quattro_description`의 상위 디렉터리를 추가(mesh 리소스 탐색용).
2. `ros_gz_sim`의 `gz_sim.launch.py`를 `-r`(즉시 재생) 옵션으로 include — GUI(`headless=false`) 또는 서버 전용(`headless=true`) 중 하나만 조건부 실행.
3. `/clock` 브리지(`ros_gz_bridge parameter_bridge`)로 Gazebo 시뮬레이션 시각을 ROS에 전달.
4. `robot_state_publisher`가 `simulation:=true simulation_controllers:=<gazebo_controllers.yaml>`로 변환한 URDF를 발행.
5. 2초 지연 후 `ros_gz_sim create`로 `robot_description` 토픽 기준 로봇을 `z=0.325`에 스폰.
6. 4초 지연 후 `joint_state_broadcaster` → `joint_trajectory_controller` spawner.
7. 6초 지연 후 `quattro/gait_controller`를 `use_sim_time:=true`로 시작(파라미터는 `quattro/config/kinematics.yaml`).
8. `use_rviz`가 참이면 RViz(`quattro_description/rviz/quattro.rviz`).

지연 시간(2/4/6초)은 Gazebo 리소스 로딩과 `ros2_control` 하드웨어 초기화가 끝나길 기다리는 고정 대기이며, `hardware.launch.py`(실기)처럼 프로세스 종료 이벤트 기반 순차 실행은 아니다.

## `gazebo_controllers.yaml`

`quattro_bringup/hardware_controllers.yaml`(direct_position 실기 구성)과 동일한 형태: `joint_state_broadcaster` + 표준 `joint_trajectory_controller/JointTrajectoryController`(`position` command, `position`/`velocity` state). MIT나 direct_velocity/torque용 시뮬레이션 구성은 없다 — 시뮬레이션은 항상 `direct_position`과 동등한 인터페이스로 검증한다.

## Xacro 분기

`quattro.urdf.xacro`가 `simulation:=true`일 때 `quattro_simulation_joint` 매크로(`gz_ros2_control/GazeboSimSystem`, `position` command만)를 사용하도록 분기한다. 실물 하드웨어 대상 `hardware_control_method`(mit 등)는 시뮬레이션에 영향을 주지 않는다.

## 관련 문서

- 시뮬레이션 실행 세부 절차: `docs/gazebo.md`
- URDF 시뮬레이션/실기 분기: `docs/packages/quattro_description.md`
