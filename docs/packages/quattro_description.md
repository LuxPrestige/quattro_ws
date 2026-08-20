# `quattro_description`

## 역할

Quattro의 로봇 모델 패키지(`ament_cmake`). URDF/Xacro, mesh, TF 구조, joint/link/inertial/collision 정의, RViz 설정, 실제/시뮬레이션 `ros2_control` description을 소유한다. 제어 알고리즘과 드라이버 구현은 포함하지 않는다.

```text
src/quattro_description/
├── urdf/quattro.urdf.xacro     # 소스 원본 (생성된 URDF를 직접 수정하지 않음)
├── meshes/stl/                  # 몸체/다리 STL (visual과 collision이 동일 mesh 사용)
├── rviz/quattro.rviz            # 기본 RViz 설정
├── rviz/hardware_remote.rviz    # 원격 하드웨어 시각화용 RViz 설정
├── launch/display.launch.py     # joint_state_publisher_gui 기반 단독 모델 확인
├── scripts/trajectory_to_joint_state.py  # 시각화 전용 trajectory→joint_states 브리지
└── config/calibration.yaml      # Xacro 검사용 기본 calibration (머신별 파일과 별개)
```

## `quattro.urdf.xacro`

**Xacro 인자**

| 인자 | 기본값 | 용도 |
|---|---|---|
| `calibration_file` | `../config/calibration.yaml` | 관절별 `can_interface`/`can_id`/`direction`/`offset`/`kp`/`kd`(+선택적 `current_limit`) YAML |
| `simulation` | `false` | `true`면 `gz_ros2_control/GazeboSimSystem`, `false`면 `quattro_hardware/QuattroSystem` |
| `apply_position_gains`, `position_gain`, `velocity_gain`, `velocity_integrator_gain` | `false`, `0.0`×3 | GDS68 runtime position/velocity gain을 configure 단계에서 덮어쓸지 여부 |
| `motor_activation_interval_ms` | `100` | 모터 순차 활성화 안정화 간격 |
| `simulation_controllers` | `../../quattro_gazebo/config/gazebo_controllers.yaml` | Gazebo `ros2_control` 플러그인에 전달되는 controller yaml 경로 |

**`quattro_hardware_joint` 매크로**: joint당 `position` command interface 하나와 `position`/`velocity`/`effort` state interface 3개를 선언한다. joint당 추가 `<param>`으로 `can_interface`, `can_id`, `direction`, `offset`, `gear_ratio`(`8.0` 고정), `current_limit`을 넘긴다 — 이 값들은 `quattro_hardware/QuattroSystem` 구현이 소비하는 계약이다(`docs/packages/quattro_hardware.md` 참고).

**`quattro_simulation_joint` 매크로**: Gazebo용 구성. joint당 `position` command interface 하나와 `position`(초기값 지정)/`velocity`/`effort` state interface 3개를 선언한다 — `quattro_hardware_joint`와 동일한 command interface 구성이라 `joint_trajectory_controller/JointTrajectoryController`가 실기·시뮬레이션 양쪽에서 그대로 쓰인다.

**`<ros2_control name="QuattroSystem" type="system">`**: 실제 하드웨어 플러그인 파라미터(gain 관련, `feedback_timeout_ms`, `feedback_request_period_ms`, `heartbeat_timeout_ms`, `startup_timeout_ms`, `motor_activation_interval_ms`, `command_timeout_ms`, `scheduling_warning_ms`, `rotor_velocity_limit_rev_s`, `motor_current_limit_a`, `telemetry_period_ms`)를 12관절 매크로 호출과 함께 선언한다. 이 파라미터 이름 자체가 `quattro_hardware::QuattroSystem`이 `on_init`에서 읽어야 할 계약이다.

## 검증

```bash
cd /ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

`simulation:=true`/`false` 양쪽 모두 별도로 검사한다.

## `display.launch.py`

`robot_state_publisher` + `joint_state_publisher_gui`(또는 `use_gui:=false`면 `joint_state_publisher`) + RViz만 실행하는 모델 단독 확인용 launch. 실제/시뮬레이션 하드웨어와 무관하게 URDF의 링크/조인트/mesh를 눈으로 검사할 때 사용한다.

## `trajectory_to_joint_state.py`

`<controller>/joint_trajectory`를 구독해 **마지막 포인트의 목표 위치**를 그대로 `sensor_msgs/JointState`로 재발행하는 시각화 전용 노드(`quattro_bringup/gait_visualization.launch.py`, `remote_visualization.launch.py`에서 사용). 실제 하드웨어 feedback이 아니므로 노드 시작 시 경고 로그를 남긴다. 이름/위치 개수 불일치, 비유한(non-finite) 값이 있는 trajectory는 무시한다.

## 좌표계와 이름 규칙

`docs/architecture.md` 6~8절(REP-103, 다리/관절 이름, 12관절 표준 순서)을 그대로 따른다. CAN ID는 ROS joint 이름에 포함하지 않는다.
