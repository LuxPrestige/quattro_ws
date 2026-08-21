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
└── config/hardware_controllers.yaml     # joint_state_broadcaster + joint_trajectory_controller
```

## 실행 전 안전 확인

1. 로봇을 지지대에 올려 다리에 하중이 걸리지 않게 한다.
2. 다른 CAN 송신 프로그램(캘리브레이션 도구 등)을 모두 종료한다.
3. 실제 `calibration.yaml`이 존재하고 공통 Direct Position 설정 및 12개 관절의 CAN 매핑·방향·offset을 확인한다.
4. `can0`, `can1`이 모두 500 kbit/s이며 `ERROR-ACTIVE`인지 확인한다(`ip -details -statistics link show can0`).
5. 전원과 E-stop 수단을 준비한다.
6. 초기 시험은 낮은 gain/current 조건에서 수행한다. 로봇을 처음 켜거나 새 calibration을 적용한 직후에는 `docs/packages/quattro_hardware.md` 6절의 단일 모터 시험부터 거친다 — `hardware.launch.py`로 곧바로 12축을 동시에 활성화하지 않는다.

## `hardware.launch.py`

**Launch 인자**

| 인자 | 기본값 | 설명 |
|---|---|---|
| `calibration_file` | `<share>/config/calibration.yaml` | 머신별 모터 calibration |
| `motor_activation_interval_ms` | `100` | 모터 순차 활성화 안정화 간격 |
| `controller_file` | `config/hardware_controllers.yaml` | ros2_control 컨트롤러 구성 |
| `initial_pose_duration` | `5.0` | 초기 자세 전환 궤적 시간(초) |
| `start_gait_enabled` | `false` | 활성화 직후 초기 자세를 바로 명령할지 여부(12축 동시 하중이므로 기본 비활성) |
| `staged_initial_pose` | `true` | 초기 자세를 다리 단위(3관절씩) 순차 전환 |
| `use_imu`, `use_teleop` | `true`, `true` | BNO085/joystick teleop 노드 시작 여부 |

**흐름**: `robot_state_publisher` + `controller_manager` 시작 → `hardware_spawner --activate QuattroSystem` → 성공 시 `joint_state_broadcaster` → 성공 시 `joint_trajectory_controller` spawner → 성공 시 `gait_controller` 시작(각 단계는 이전 단계 프로세스 종료 이벤트로 트리거). IMU(`use_imu`)와 teleop(`use_teleop`, joystick 노드 + `teleop_node`, `start_stepping:=true` 고정)은 조건부로 병렬 시작한다. `controller_manager`가 종료되면 전체 launch를 종료한다(`Shutdown` 이벤트) — fault/timeout으로 하드웨어 lifecycle이 안전 상태로 전환된 뒤에도 동일하게 적용된다.

기본 명령은 hold-only bringup이다 — gait trajectory를 자동으로 보내지 않는다. 전원 용량, 영점, gain을 단일 모터부터 검증한 뒤에만 `ros2 service call /gait/enable std_srvs/srv/SetBool '{data: true}'`로 gait를 명시적으로 켠다. `start_gait_enabled:=true`(또는 위 서비스 호출)로 시작되는 초기 자세 전환은 `initial_pose_duration`(기본 5.0초)짜리 궤적이며, `staged_initial_pose`(기본 `true`)이면 앞왼쪽→앞오른쪽→뒤왼쪽→뒤오른쪽 순서로 한 다리(3관절)씩 이동한다(`docs/packages/quattro.md`의 `staged_joint_targets`) — 기본 설정에서 전체 전환에 약 20초가 걸린다. 이 시험은 로봇을 지지한 상태에서 수행한다.

Quattro는 `JointTrajectoryController`가 시간 기반 궤적을 만들고 GDS68은 그 목표를 그대로 따라가는 구성을 쓴다. GDS68 내부 trapezoidal trajectory 기능(`0x11`/`0x12`)을 동시에 쓰면 두 계층의 shaping이 중복되므로 함께 켜지 않는다.

새 controller 패키지를 빌드한 셸에서는 plugin index를 반영하기 위해 실행 전 `source /ws/install/setup.bash`를 다시 수행한다.

## `hardware_controllers.yaml`

`update_rate: 100`, `hardware_components_initial_state.inactive: [QuattroSystem]`, `joint_state_broadcaster` + 표준 `joint_trajectory_controller/JointTrajectoryController`(`position` command interface)를 선언한다.

## `calibration.yaml`

`direct_position`에는 12축 공통 `current_limit`, `position_gain`, `velocity_gain`, `velocity_integrator_gain`을 정의한다. 현재 gain `20.0/0.16/0.32`는 제조사 튜닝 예시이며 검증된 factory default가 아니다. `joints.<joint_name>`에는 `can_interface`, `can_id`, `direction`, `offset`을 정의한다. 실제 파일은 Git에서 제외하며 `calibration.yaml.example`을 복사해 만든다.

bringup configure는 이 공통 설정을 모터마다 한 번 적용하지만 `Save_Configuration`으로 flash에 저장하지 않는다. 활성화 이후 정상 제어 루프에서는 `Set_Input_Pos`만 전송하고 encoder/error/telemetry는 별도 읽기 요청으로 확인한다.

## `gait_visualization.launch.py` / `remote_visualization.launch.py`

두 launch 모두 시각화 프로세스 자체는 CAN/모터에 접근하지 않고 `robot_state_publisher` + `gait_controller`(선택) + `trajectory_to_joint_state.py`(`quattro_description`) + RViz로 IK/gait 출력이나 실기 상태를 확인한다. `remote_visualization.launch.py`는 `/joint_states`의 실제 엔코더 자세를 검정색 `JointStale`, trajectory 명령 자세를 주황색 `JointAngle` STL 모델로 같은 위치에 겹쳐 표시한다. 각 RViz RobotModel의 `Collision Enabled`를 켜면 해당 자세의 primitive 충돌 범위를 추가로 표시할 수 있다.

`remote_visualization.launch.py`의 `show_target`(기본 `true`)를 `false`로 주면 주황색 목표(`JointAngle`) 오버레이 없이 검정색 실제 인코더 위치(`JointStale`)만 표시한다 — 목표와 겹쳐 보면 헷갈리는, 실제 모터가 예상과 다른 방향/거리로 움직이는지만 순수하게 눈으로 확인하고 싶을 때 쓴다:

```bash
ros2 launch quattro_bringup remote_visualization.launch.py show_target:=false
```

## 실행 후 확인

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /joint_trajectory_controller/joint_trajectory
ros2 topic hz /imu/data
```

최소 정상 조건: `QuattroSystem` active, `joint_state_broadcaster` active, `joint_trajectory_controller` active, `/joint_states`가 실제 motor feedback 기반, CAN bus `ERROR-ACTIVE`. `controller_manager`의 `update_rate`는 `100 Hz`이며 `quattro_hardware::read()`/`write()`는 이 주기마다 호출된다.

## 하드웨어 fault 조사

```bash
candump -e -tz can0
candump -e -tz can1
ip -details -statistics link show can0
ip -details -statistics link show can1
```

단순 stale feedback(일시적 프레임 누락)과 GDS68 자체 fault(`Heartbeat.axis_error` — `Get_Error`는 실기에서 응답하지 않아 쓰지 않는다, `docs/packages/gim6010_driver.md` 0절)를 같은 원인으로 단정하지 않는다(`docs/packages/quattro_hardware.md` 4절). 로그와 CAN 상태를 먼저 저장하고, 원인을 확인하기 전에 무조건 error clear를 반복하지 않는다.

## 종료

`Ctrl+C` 또는 `controller_manager` 종료 시 하드웨어 lifecycle이 종료되며 안전 정책에 따라 모터를 비활성화한다. fault/timeout 발생 시에도 동일하게 안전 상태로 전환한다.

## 관련 문서

- 캘리브레이션: `docs/calibration.md`
- 하드웨어/CAN 설계, 활성화 절차, 단일 모터 시험: `docs/packages/quattro_hardware.md`
- 사용하는 드라이버: `docs/packages/gim6010_driver.md`
- Docker/원격 실행 환경: `docs/development_environment.md`
