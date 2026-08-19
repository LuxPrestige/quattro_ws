# `quattro_bringup`

## 역할

실제 시스템 실행 조합 패키지(`ament_cmake`, 코드 없이 launch/config만 설치). `robot_state_publisher`, `controller_manager`, hardware/controller spawner, IMU, teleop, gait controller를 하나의 launch로 묶는다. 드라이버와 제어 알고리즘 구현은 포함하지 않는다.

```text
src/quattro_bringup/
├── launch/hardware.launch.py           # 실기 전체 bringup (핵심)
├── launch/gait_visualization.launch.py  # 하드웨어 없이 gait/IK만 RViz로 확인
├── launch/remote_visualization.launch.py # 실기 목표/실제 joint를 원격 데스크톱에서 비교
├── config/calibration.yaml.example      # 버전관리 템플릿 (실제 파일은 .gitignore)
├── config/calibration.yaml              # 머신별 실제 값 (Git 제외)
├── config/hardware_controllers_mit.yaml       # 기본: mit_trajectory_controller
├── config/hardware_controllers.yaml           # direct_position: joint_trajectory_controller
├── config/hardware_controllers_velocity.yaml  # direct_velocity: forward_command_controller(velocity)
└── config/hardware_controllers_torque.yaml    # direct_torque: forward_command_controller(effort)
```

## 실행 전 안전 확인

1. 로봇을 지지대에 올려 다리에 하중이 걸리지 않게 한다.
2. 다른 CAN 송신 프로그램(캘리브레이션 도구 등)을 모두 종료한다.
3. 실제 `calibration.yaml`이 존재하고 12개 관절의 `can_interface`/`can_id`/`direction`/`offset`/`kp`/`kd`를 확인한다.
4. `can0`, `can1`이 모두 500 kbit/s이며 `ERROR-ACTIVE`인지 확인한다(`ip -details -statistics link show can0`).
5. 전원과 E-stop 수단을 준비한다.
6. 초기 시험은 낮은 gain/current 조건에서 수행한다. 로봇을 처음 켜거나 새 calibration을 적용한 직후에는 `docs/packages/quattro_hardware.md` 6절의 단일 모터 시험부터 거친다 — `hardware.launch.py`로 곧바로 12축을 동시에 활성화하지 않는다.

## `hardware.launch.py`

**Launch 인자**

| 인자 | 기본값 | 설명 |
|---|---|---|
| `calibration_file` | `<share>/config/calibration.yaml` | 머신별 모터 calibration |
| `hardware_control_method` | `mit` | `direct_position` / `direct_velocity` / `direct_torque` / `mit` |
| `apply_position_gains` | `false` | GDS68 runtime position/velocity gain 덮어쓰기 여부 |
| `position_gain`, `velocity_gain`, `velocity_integrator_gain` | `0.0` | `apply_position_gains=true`일 때만 사용 |
| `motor_activation_interval_ms` | `100` | 모터 순차 활성화 안정화 간격 |
| `command_controller_name` | `mit_trajectory_controller` | `controller_file`에 실제로 존재하는 controller 이름과 일치해야 함 |
| `controller_file` | `config/hardware_controllers_mit.yaml` | ros2_control 컨트롤러 구성 |
| `initial_pose_duration` | `5.0` | 초기 자세 전환 궤적 시간(초) |
| `start_gait_enabled` | `false` | 활성화 직후 초기 자세를 바로 명령할지 여부(12축 동시 하중이므로 기본 비활성) |
| `staged_initial_pose` | `true` | 초기 자세를 다리 단위(3관절씩) 순차 전환 |
| `use_imu`, `use_teleop` | `true`, `true` | BNO085/joystick teleop 노드 시작 여부 |

**흐름**: `robot_state_publisher` + `controller_manager` 시작 → `hardware_spawner --activate QuattroSystem` → 성공 시 `joint_state_broadcaster` → 성공 시 `command_controller_name` spawner → 성공 시(`hardware_control_method`가 `direct_position`/`mit`일 때만) `gait_controller` 시작(각 단계는 이전 단계 프로세스 종료 이벤트로 트리거). IMU(`use_imu`)와 teleop(`use_teleop`, joystick 노드 + `teleop_node`, `start_stepping:=true` 고정)은 조건부로 병렬 시작한다. `controller_manager`가 종료되면 전체 launch를 종료한다(`Shutdown` 이벤트) — fault/timeout으로 하드웨어 lifecycle이 안전 상태로 전환된 뒤에도 동일하게 적용된다.

`calibration_file`을 launch 시점에 파이썬으로 직접 파싱해(`_mit_gains_from_calibration`) `mit_trajectory_controller`의 `kp`/`kd` 파라미터에 주입한다. 읽기 실패 시 `hardware_controllers_mit.yaml`의 기본값(`kp=60.0, kd=0.8`)으로 대체해 bringup 자체가 막히지 않게 한다. 이 오버라이드는 `mit_trajectory_controller`가 실제로 로드된 경우에만 의미가 있다. 이렇게 `calibration.yaml`을 유일한 gain 소스로 두면, 활성화 중 하드웨어 계층이 쓰는 gain과 활성화 이후 컨트롤러가 쓰는 gain이 같은 값에서 이어진다.

기본 명령은 MIT hold-only bringup이다 — gait trajectory를 자동으로 보내지 않는다. 전원 용량, 영점, gain을 단일 모터부터 검증한 뒤에만 `ros2 service call /gait/enable std_srvs/srv/SetBool '{data: true}'`로 gait를 명시적으로 켠다. `start_gait_enabled:=true`(또는 위 서비스 호출)로 시작되는 초기 자세 전환은 `initial_pose_duration`(기본 5.0초)짜리 궤적이며, `staged_initial_pose`(기본 `true`)이면 앞왼쪽→앞오른쪽→뒤왼쪽→뒤오른쪽 순서로 한 다리(3관절)씩 이동한다(`docs/packages/quattro.md`의 `staged_joint_targets`) — 기본 설정에서 전체 전환에 약 20초가 걸린다. 이 시험은 로봇을 지지한 상태에서 수행한다.

Quattro는 `JointTrajectoryController`/`MitTrajectoryController`가 시간 기반 궤적을 만들고 GDS68은 그 목표를 그대로 따라가는 구성을 쓴다. GDS68 내부 trapezoidal trajectory 기능(`0x11`/`0x12`)을 동시에 쓰면 두 계층의 shaping이 중복되므로 함께 켜지 않는다.

새 controller 패키지를 빌드한 셸에서는 plugin index를 반영하기 위해 실행 전 `source /ws/install/setup.bash`를 다시 수행한다.

## Controller 파일 4종

`hardware_control_method`와 `controller_file`/`command_controller_name`은 항상 짝으로 지정한다.

| `hardware_control_method` | `controller_file` | `command_controller_name` | controller 타입 | command interface |
|---|---|---|---|---|
| `mit` (기본) | `hardware_controllers_mit.yaml` | `mit_trajectory_controller` | `quattro_controllers/MitTrajectoryController` | position/velocity/kp/kd/effort (5) |
| `direct_position` | `hardware_controllers.yaml` | `joint_trajectory_controller` | `joint_trajectory_controller/JointTrajectoryController` | position |
| `direct_velocity` | `hardware_controllers_velocity.yaml` | `joint_command_controller` | `forward_command_controller/ForwardCommandController` | velocity |
| `direct_torque` | `hardware_controllers_torque.yaml` | `joint_command_controller` | `forward_command_controller/ForwardCommandController` | effort |

4개 yaml 모두 `update_rate: 100`, `hardware_components_initial_state.inactive: [QuattroSystem]`, `joint_state_broadcaster`를 공통으로 갖는다.

## `calibration.yaml`

`joints.<joint_name>`마다 `can_interface`(`can0`/`can1`), `can_id`(0~11), `direction`(±1), `offset`(rad), `kp`/`kd`(MIT hold gain), 선택적 `current_limit`을 정의한다. 실제 파일은 Git에서 제외하며 `calibration.yaml.example`을 복사해 만든다. 상세 절차는 `docs/calibration.md`, CAN ID/bus 기준 매핑은 `docs/packages/quattro_hardware.md` 0절.

## `gait_visualization.launch.py` / `remote_visualization.launch.py`

두 launch 모두 실제 CAN/모터 없이 `robot_state_publisher` + `gait_controller`(선택) + `trajectory_to_joint_state.py`(`quattro_description`) + RViz만 실행해 IK/gait 출력이나 실기 목표 자세를 확인한다. `remote_visualization.launch.py`는 `target/` frame prefix로 실제 로봇과 겹치지 않게 원격 표시한다.

## 실행 후 확인

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /mit_trajectory_controller/joint_trajectory
ros2 topic hz /imu/data
```

최소 정상 조건: `QuattroSystem` active, `joint_state_broadcaster` active, 선택된 command controller active, `/joint_states`가 실제 motor feedback 기반, CAN bus `ERROR-ACTIVE`. `controller_manager`의 `update_rate`는 `100 Hz`이며 `quattro_hardware::read()`/`write()`는 이 주기마다 호출된다.

## 하드웨어 fault 조사

```bash
candump -e -tz can0
candump -e -tz can1
ip -details -statistics link show can0
ip -details -statistics link show can1
```

단순 stale feedback(일시적 프레임 누락)과 GDS68 자체 fault(`Get_Error` 응답)를 같은 원인으로 단정하지 않는다(`docs/packages/quattro_hardware.md` 4절). 로그와 CAN 상태를 먼저 저장하고, 원인을 확인하기 전에 무조건 error clear를 반복하지 않는다.

## 종료

`Ctrl+C` 또는 `controller_manager` 종료 시 하드웨어 lifecycle이 종료되며 안전 정책에 따라 모터를 비활성화한다. fault/timeout 발생 시에도 동일하게 안전 상태로 전환한다.

## 관련 문서

- 캘리브레이션: `docs/calibration.md`
- 하드웨어/CAN 설계, 활성화 절차, 단일 모터 시험: `docs/packages/quattro_hardware.md`
- 사용하는 드라이버: `docs/packages/gim6010_driver.md`
- Docker/원격 실행 환경: `docs/development_environment.md`
