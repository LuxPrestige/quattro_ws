# `quattro`

## 역할

Quattro의 상위 로봇 제어 패키지(Python, `ament_python`). FK/IK, gait 생성, 몸체 자세 명령을 표준 ROS 2 메시지 기반으로 처리해 `JointTrajectory`를 만든다. CAN 프레임, SocketCAN, GDS68 프로토콜은 알지 못한다.

```text
src/quattro/quattro/
├── kinematics.py       # ROS 비의존 analytic FK/IK
├── gait.py              # ROS 비의존 Bézier/sine trot gait 생성기
├── gait_controller.py    # gait_controller 노드 (실행 파일)
└── pose_controller.py   # pose_controller 노드 (실행 파일, 독립 실행용)
```

`kinematics.py`와 `gait.py`는 rclpy에 의존하지 않아 시뮬레이션과 unit test에서 그대로 재사용한다(`test/test_kinematics.py`, `test/test_gait.py`).

## `kinematics.py`

- `RobotGeometry`: 로봇 치수 dataclass (hip_offset_y, upper/lower_leg_length, hip/foot spacing, nominal_height, center_of_mass_offset_x). 모든 길이는 양수여야 한다(`__post_init__` 검증).
- `LegKinematics`: 힙 프레임 기준 3관절(hip/upper/lower) analytic inverse/forward. 도달 불가능한 목표는 `UnreachableTargetError`를 던진다.
  - `jacobian(joints)`: analytic 3x3 hip-frame Jacobian(`forward`의 닫힌형 미분, 수치미분 아님).
  - `foot_velocity(joints, joint_velocities)`: `v = J(q) qdot`.
  - `joint_velocity_from_foot_velocity(joints, foot_velocity, damping=1e-3)`: `qdot = J^T (J J^T + damping^2 I)^-1 v` (damped least-squares pseudoinverse) — Jacobian singularity 근처에서도 발산하지 않는다.
  - `force_to_joint_torque(joints, force)`: `tau = J(q)^T F`.
- `QuadrupedKinematics`: 4다리 IK/FK를 몸체 rpy + translation + world-frame foot position(또는 nominal stance)으로 계산. `joint_names`가 12관절 표준 순서(`docs/architecture.md` 8절)를 반환한다.
  - `joint_velocities(rpy, joint_angles, foot_velocities, damping=1e-3)`: world/body-frame foot velocity(`inverse`의 `foot_positions`와 같은 프레임)를 몸체 회전만 이용해 각 힙 프레임으로 옮긴 뒤, 다리별 damped pseudoinverse로 4x3 joint velocity를 반환한다.
- `rotation_matrix_from_rpy`: 고정축 roll-pitch-yaw 회전행렬.

## `gait.py`

레퍼런스 사족보행 코드의 12-control-point Bézier swing 곡선과 sine stance 곡선, diagonal trot phasing(0.5 위상차 대각선 쌍), yaw-circle 회전 이동, touchdown 기반 위상 재동기화를 그대로 구현한다.

- `GaitParameters`: swing/stance duration, clearance/penetration height, 최대 선속도·yaw rate, 정지 복귀 속도. `stance_duration <= 1.3 * swing_duration` 등 물리적으로 불가능한 조합은 생성 시점에 거부한다.
- `GaitGenerator.update(dt, linear_velocity, yaw_rate, contacts, complete_cycle_on_stop)`: 매 제어 주기 목표 foot position(nominal 좌표계)을 반환한다(`update_states`의 얇은 wrapper, position만 추출). 정지 명령이 들어오면 현재 진행 중인 stride를 끝까지 마치거나(`complete_cycle_on_stop=True`) 즉시 nominal 위치로 복귀한다.
- `GaitGenerator.update_states(...)`: `update`와 같은 인자·gait clock 동작이지만, 다리마다 `FootTrajectoryState(position, velocity, acceleration, phase, in_swing)`를 반환한다. velocity/acceleration은 Bézier/sine 곡선의 **analytic** phase-derivative(Bernstein 다항식 미분 항등식)를 segment duration(swing/stance)으로 스케일링해 얻는다 — finite difference를 쓰지 않는다.
- 앞왼쪽 다리(`front_left`)의 swing phase가 0.9 이상이고 contact가 감지되면 위상을 재동기화한다(다리별 개별 접촉 센서가 없는 현재 구성에서는 `contacts/<leg>` 토픽 기본값 `False`로 동작).

## `gait_controller` 노드

`geometry_msgs/Twist`(`/cmd_vel`)와 `geometry_msgs/PoseStamped`(`/body_pose`)를 받아 100 Hz(기본) 주기로 `trajectory_msgs/JointTrajectory`를 발행하는 실시간 상위 제어 루프.

**구독**

| 토픽 | 타입 | 용도 |
|---|---|---|
| `cmd_vel` | `geometry_msgs/Twist` | 이동 속도 명령 |
| `body_pose` | `geometry_msgs/PoseStamped` | 몸체 자세 명령(정지 상태에서 사용) |
| `joint_states` | `sensor_msgs/JointState` | 초기 자세 전환 시작점 |
| `imu/data` | `sensor_msgs/Imu` (`qos_profile_sensor_data`) | roll/pitch balance 입력 |
| `estop` | `std_msgs/Bool` | 즉시 정지 |
| `imu_auto` | `std_msgs/Bool` | IMU balance PID on/off |
| `gait/clearance_height`, `gait/penetration_depth`, `gait/swing_duration` | `std_msgs/Float64` | 실행 중 gait 파라미터 조정(검증 실패 시 로그만 남기고 무시) |
| `contacts/<leg>` (4개) | `std_msgs/Bool` | 다리별 접지 상태(현재 실제 센서 미연결, 기본 `False`) |
| `bringup_ready_topic` (기본 `/bringup/ready`) | `std_msgs/Bool` (latched) | `wait_for_bringup_ready: true`일 때만 구독 |

**발행**: `<trajectory_controller_name>/joint_trajectory` (기본 `joint_trajectory_controller/joint_trajectory`). 매 제어 주기 `GaitGenerator.update`가 계산한 발끝 목표 위치를 IK로 관절 각도(`q_des = IK(p_des)`)로 변환해 `positions`만 채운 `JointTrajectory`를 발행한다(속도 feed-forward 없이 순수 위치 제어).

**서비스**: `gait/enable` (`std_srvs/SetBool`, stepping ↔ 정지-시야 전환), `balance/enable` (`std_srvs/SetBool`, IMU PID on/off).

**동작 원칙**

- `estop=true`면 속도를 즉시 0으로 램프하고 이후 아무 것도 발행하지 않는다.
- 명령 나이(`command_timeout`, 기본 0.5 s)를 넘으면 즉시 정지 램프를 사용한다(정상 정지 대비 더 빠른 `stop_ramp_rate`).
- `_controlled_body_rpy`는 `balance_enabled`일 때만 IMU roll/pitch 오차에 대해 PID(P/I/D, integral clamp)를 적용해 목표 body rpy를 보정한다. 비활성 시 적분 항을 초기화한다.
- `start_enabled`가 참이거나 `/gait/enable`이 처음 호출될 때 `staged_initial_pose`가 참이면 네 다리의 hip 관절 4개를 먼저 동시에 옮기고 이어서 나머지 8관절을 동시에 옮기는 2단계 `JointTrajectory`를 만들어 급격한 동시 하중을 피한다. 두 단계는 `initial_pose_duration`을 절반씩 나눠 쓰므로 전체 전환 시간은 `initial_pose_duration`과 같다.
- IK 실패(`UnreachableTargetError`/`ValueError`)는 명령을 거부하고 에러 로그만 남긴다(이전 목표를 그대로 유지).
- `wait_for_bringup_ready`(기본 `false`)가 참이면 `/bringup/ready`에서 `data: true`를 받기 전까지 아무 것도 발행하지 않는다. inactive 상태의 `joint_trajectory_controller`는 trajectory를 조용히 버리므로, 첫 궤적인 staged initial pose가 JTC ACTIVE 이전에 나가면 그대로 사라진다. flag는 latched라 이 노드가 bringup보다 먼저 시작하든 나중에 시작하든 동작한다. 대기 중 도착한 `cmd_vel`은 ready 시점에 stale로 처리해 즉시 걷기 시작하지 않는다.
- 실기 launch(`hardware.launch.py`)는 이 파라미터를 `true`로 넘긴다. bringup manager가 없는 Gazebo/시각화 launch는 기본 `false`를 그대로 쓴다.

## `pose_controller` 노드

`gait_controller`와 별개의 경량 대안: `/body_pose`만 구독해 nominal foot position 기준 정지 자세 `JointTrajectory`를 발행한다. gait/balance/estop 로직이 없으므로 `gait_controller`가 이미 `/body_pose`를 처리하는 일반 bringup에서는 사용하지 않고, kinematics만 단독 검증할 때 쓴다.

## 설정

기본 파라미터는 `config/kinematics.yaml`에 `gait_controller`/`pose_controller` 두 노드 이름으로 정의되어 있으며, `RobotGeometry`/`GaitParameters`의 모든 필드를 ROS 파라미터로 노출한다(패키지 share의 `config/kinematics.yaml`로 설치).

## 테스트

`test/test_kinematics.py`, `test/test_gait.py`가 순수 Python 레벨에서 FK/IK round-trip과 gait 궤적 성질을 검증한다. `test_flake8.py`/`test_pep257.py`/`test_copyright.py`는 표준 ament lint.
