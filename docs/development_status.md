# Quattro 개발 현황과 향후 개발 순서

## 1. 문서 목적

이 문서는 현재 Quattro ROS 2 워크스페이스에 구현된 기능, 아직 검증되지
않은 기능, 앞으로의 개발 순서를 기록한다.

기존 `lgh_ws-old` 코드를 그대로 복사하는 대신 동작을 분석한 후 ROS 2
Jazzy 표준 인터페이스와 `ros2_control` 구조에 맞게 재구현하는 것을
목표로 한다.

모든 ROS 외부 인터페이스는 SI 단위를 사용한다.

- 길이: `m`
- 선속도: `m/s`
- 각도: `rad`
- 각속도: `rad/s`
- 토크: `N·m`

다리 이름은 다음 순서와 표기를 사용한다.

1. `front_left`
2. `front_right`
3. `back_left`
4. `back_right`

---

## 2. 현재 패키지 구성

```text
src/
├── quattro/              상위 제어, FK/IK, gait, 자세 제어
├── quattro_description/  URDF/Xacro, STL, RViz 설정
├── quattro_bringup/      시스템 조합 launch
├── quattro_gazebo/       Gazebo Harmonic 시뮬레이션
├── quattro_hardware/     ros2_control 및 GIM6010 하드웨어 계층 예정
├── quattro_sensors/      BNO085 IMU 드라이버
└── quattro_teleop/       조이스틱 및 키보드 입력
```

---

## 3. 구현된 기능

### 3.1 `quattro_description`

- Quattro Xacro 모델
- 몸체와 네 다리의 link/joint 구성
- `front_*`, `back_*` 이름 체계
- STL visual 및 collision geometry
- inertial 정보
- revolute joint axis 및 limit
- 네 발의 fixed foot link
- RViz2 설정
- `robot_state_publisher` 기반 TF 발행

현재 Xacro는 다음 검사를 통과한다.

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

### 3.2 Kinematics

구현 파일: `src/quattro/quattro/kinematics.py`

- 한 다리의 3-DOF analytic IK
- 한 다리의 FK
- 네 다리 전체 IK
- 네 다리 전체 FK
- roll, pitch, yaw 몸체 회전 변환
- 몸체 translation 반영
- center of mass X offset 반영
- 좌우 다리 관절 부호 처리
- 도달할 수 없는 발 목표 검출
- FK와 IK 왕복 단위 테스트
- URDF와 동일한 12개 joint name 출력

IK 결과는 다음 순서를 사용한다.

```text
front_left_hip_joint
front_left_upper_leg_joint
front_left_lower_leg_joint
front_right_hip_joint
front_right_upper_leg_joint
front_right_lower_leg_joint
back_left_hip_joint
back_left_upper_leg_joint
back_left_lower_leg_joint
back_right_hip_joint
back_right_upper_leg_joint
back_right_lower_leg_joint
```

### 3.3 Bézier gait generator

구현 파일: `src/quattro/quattro/gait.py`

초기에 작성했던 단순 smoothstep gait는 폐기했다. 현재 구현은
`lgh_ws-old`의 `BezierGait` 동작을 기준으로 재작성했다.

- 12개 control point
- degree-11 Bernstein polynomial
- Bézier swing trajectory
- sine stance trajectory
- 대각선 trot 위상
  - `front_left` + `back_right`
  - `front_right` + `back_left`
- clearance height
- penetration depth
- 선형 보행 방향 계산
- yaw-circle 접선 방향 계산
- 이전 발 offset 기반 yaw 연속성
- `front_left` touchdown 기반 gait clock 재동기화
- 선속도 및 yaw rate 제한
- 정지 명령 시 nominal stance 복귀
- 생성된 발 위치에 대한 IK 도달성 테스트

현재 기본값은 다음과 같다.

| 파라미터 | 기본값 |
|---|---:|
| Swing duration | `0.25 s` |
| Stance duration | `0.30 s` |
| Clearance height | `0.040 m` |
| Penetration depth | `0.008 m` |
| Maximum linear speed | `0.30 m/s` |
| Maximum yaw rate | `1.0 rad/s` |

### 3.4 Gait controller와 자세 제어

구현 파일: `src/quattro/quattro/gait_controller.py`

- `/cmd_vel` 입력
- 100 Hz 기본 제어 주기
- command timeout
- E-stop 즉시 정지
- 일반 속도 명령 ramp
- Stepping/Viewing 전환
- `PoseStamped` 기반 몸체 자세와 높이 목표
- `sensor_msgs/msg/Imu` 입력
- roll/pitch PID 자세 보정
- gyro 기반 derivative 항
- PID integral limit
- 네 발 contact 입력
- clearance, penetration, swing duration 실시간 입력
- `JointTrajectory` 출력

현재 PID 기본값은 다음과 같다.

| 파라미터 | 기본값 |
|---|---:|
| `pose_pid.kp` | `1.5` |
| `pose_pid.ki` | `0.1` |
| `pose_pid.kd` | `0.05` |
| `pose_pid.integral_limit` | `0.5` |

### 3.5 ROS 2 표준 인터페이스

레퍼런스의 커스텀 메시지는 새 구현에서 사용하지 않는다.

| 기능 | 인터페이스 |
|---|---|
| 이동 명령 | `/cmd_vel` — `geometry_msgs/msg/Twist` |
| 몸체 자세 목표 | `/body_pose` — `geometry_msgs/msg/PoseStamped` |
| IMU | `/imu/data` — `sensor_msgs/msg/Imu` |
| E-stop | `/estop` — `std_msgs/msg/Bool` |
| IMU 자동 자세 | `/imu_auto` — `std_msgs/msg/Bool` |
| 발 접촉 | `/contacts/<leg>` — `std_msgs/msg/Bool` |
| Clearance | `/gait/clearance_height` — `std_msgs/msg/Float64` |
| Penetration | `/gait/penetration_depth` — `std_msgs/msg/Float64` |
| Swing duration | `/gait/swing_duration` — `std_msgs/msg/Float64` |
| 관절 목표 | `trajectory_msgs/msg/JointTrajectory` |
| Gait 활성화 | `/gait/enable` — `std_srvs/srv/SetBool` |
| Balance 활성화 | `/balance/enable` — `std_srvs/srv/SetBool` |

대체된 레퍼런스 커스텀 메시지는 다음과 같다.

```text
MiniCmd       -> Twist, PoseStamped, SetBool
JointAngles   -> JointTrajectory
IMUdata       -> sensor_msgs/Imu
ContactData   -> 다리별 std_msgs/Bool
JoyButtons    -> 표준 토픽과 서비스
```

### 3.6 Switch Pro Controller teleop

구현 파일:

- `src/quattro_teleop/quattro_teleop/teleop_node.py`
- `src/quattro_teleop/config/switch_pro.yaml`

레퍼런스의 축과 버튼 번호를 유지한다.

| 입력 | Stepping | Viewing |
|---|---|---|
| 왼쪽 스틱 상하 | 전진/후진 | pitch |
| 왼쪽 스틱 좌우 | 좌우 이동 | roll |
| 오른쪽 스틱 상하 | 몸체 높이 | 몸체 높이 |
| 오른쪽 스틱 좌우 | yaw rate | yaw 자세 |
| B | Viewing으로 전환 | Stepping으로 전환 |
| A | E-stop 토글 | E-stop 토글 |
| Y | IMU 자동 자세 토글 | IMU 자동 자세 토글 |
| D-pad 상하 | clearance 조절 | clearance 조절 |
| D-pad 좌우 | penetration 조절 | penetration 조절 |
| LB 또는 RB | gait 조절값 초기화 | gait 조절값 초기화 |

조이스틱 연결 후 실제 SDL axis/button 번호는 다음 명령으로 확인해야 한다.

```bash
ros2 topic echo /joy
```

### 3.7 데스크톱 키보드 teleop

구현 파일: `src/quattro_teleop/quattro_teleop/keyboard_teleop.py`

| 키 | 기능 |
|---|---|
| W/S | 전진/후진 |
| A/D | 좌우 이동 |
| Q/E | 좌우 회전 |
| Space | 정지 |
| Ctrl-C | 종료 |

키 입력이 `0.25 s` 동안 갱신되지 않으면 0 속도를 발행한다.

### 3.8 BNO085

`quattro_sensors`에 다음 기반 구현이 존재한다.

- BNO085 장치 접근 계층
- ROS 2 class 기반 센서 노드
- `sensor_msgs/msg/Imu` 출력
- YAML 설정
- launch 파일
- 드라이버 단위 테스트

실제 Raspberry Pi I2C 하드웨어 검증은 아직 필요하다.

### 3.9 RViz gait 시각화

구현 파일:

- `src/quattro_bringup/launch/gait_visualization.launch.py`
- `src/quattro_description/scripts/trajectory_to_joint_state.py`

시각화 데이터 흐름은 다음과 같다.

```text
/cmd_vel
    -> gait_controller
    -> JointTrajectory
    -> visualization-only trajectory bridge
    -> /joint_states
    -> robot_state_publisher
    -> /tf
    -> RViz2
```

시각화 브리지의 `/joint_states`는 명령 위치이며 실제 모터 피드백이 아니다.
실제 하드웨어 실행 구성에서는 이 브리지를 사용하면 안 된다.

실행 명령:

```bash
ros2 launch quattro_bringup gait_visualization.launch.py
```

키보드 입력은 별도 터미널에서 실행한다.

```bash
ros2 run quattro_teleop keyboard_teleop
```

---

## 4. 현재 검증 결과

- `quattro` 빌드 성공
- `quattro_teleop` 빌드 성공
- Xacro 변환 성공
- `check_urdf` 성공
- FK/IK 왕복 테스트 성공
- Bézier swing endpoint 테스트 성공
- sine stance 테스트 성공
- touchdown 위상 동기화 테스트 성공
- 100주기 gait IK 도달성 테스트 성공
- 관련 누적 테스트 결과: `33 tests, 0 failures, 2 skipped`
- `git diff --check` 성공

---

## 5. 부분 구현 또는 미검증 항목

다음 항목은 코드가 존재하더라도 실제 로봇에 사용할 수준으로 검증되지 않았다.

- 모든 gait 조합에서 URDF joint limit 준수 여부
- 관절 목표 velocity 및 acceleration 제한
- 정지 시 현재 gait phase에서 nominal stance로 복귀하는 전환 궤적
- Stepping/Viewing 전환의 실제 조이스틱 통합 시험
- IMU PID 방향과 gain의 실물 검증
- contact sensor 실제 입력 장치
- contact 신호 debounce 및 stale timeout
- body pose와 gait 동시 명령 정책
- 실제 `joint_trajectory_controller`
- `ros2_control` mock hardware
- 실제 GIM6010 motor feedback
- 실제 `/joint_states`
- 하드웨어 진단과 fault 처리

현재 구현은 실제 모터에 연결하지 않는다.

---

## 6. 앞으로의 개발 순서

### 1단계: Description 최종 검증

1. 12개 joint origin 시각 검증
2. 12개 joint axis 방향 검증
3. joint lower/upper limit 검증
4. FK 결과와 URDF TF 결과 비교
5. 발 위치와 nominal stance 실측 비교
6. `base_footprint` dummy root 추가
7. KDL root inertia 경고 제거
8. TF tree 최종 검증

완료 조건:

- RViz에서 각 관절을 하나씩 움직였을 때 실제 로봇과 같은 방향으로 회전한다.
- FK 발 위치와 TF 발 위치가 허용 오차 안에서 일치한다.

### 2단계: Gait 안전성과 연속성

1. IK 결과에 URDF joint limit 적용
2. 잘못된 목표는 clamp하지 않고 거부 및 보고
3. joint velocity limit 구현
4. joint acceleration limit 구현
5. gait 시작 전 nominal stance 확인
6. gait 시작 transition 구현
7. 현재 phase 기반 안전 정지 transition 구현
8. E-stop과 일반 정지 동작 분리
9. contact stale timeout 구현
10. 모든 속도/회전 조합 sweep test

완료 조건:

- 최대 설정 범위에서 joint limit 위반이 없다.
- 시작과 정지 시 관절 위치가 불연속적으로 변하지 않는다.

### 3단계: Joystick 및 RViz 통합 검증

1. 실제 Switch Pro `/joy` axis/button 확인
2. 설정 파일의 axis/button 번호 보정
3. Stepping 조작 확인
4. Viewing 조작 확인
5. E-stop latch 확인
6. IMU auto toggle 확인
7. D-pad gait 조절 확인
8. bumper reset 확인
9. joystick timeout 확인

완료 조건:

- 모든 입력이 레퍼런스와 동일한 물리 조작에 대응한다.
- 연결 해제 시 0 속도 명령으로 전환한다.

### 4단계: IMU 및 자세 제어 검증

1. BNO085 실제 I2C 연결
2. orientation frame 확인
3. REP-103 축 방향 확인
4. covariance 설정
5. roll/pitch 부호 검증
6. 정지 상태에서 낮은 PID gain으로 시험
7. integral windup 검증
8. IMU timeout 시 balance 해제
9. 수동 자세와 자동 자세 전환 검증

완료 조건:

- IMU 입력이 끊기면 자동 보정 명령을 계속 만들지 않는다.
- 로봇을 기울였을 때 보정 방향이 실제 자세를 복원하는 방향이다.

### 5단계: `ros2_control` description

1. Xacro에 `<ros2_control>` 추가
2. 12개 joint state interface 추가
   - position
   - velocity
   - effort
3. 초기 position command interface 추가
4. joint와 motor parameter 분리
5. controller YAML 작성
6. `joint_state_broadcaster` 구성
7. `joint_trajectory_controller` 또는 적합한 position controller 구성

### 6단계: Mock hardware

1. mock `SystemInterface` 연결
2. command와 feedback 분리 확인
3. 실제 `/joint_states` 의미 검증
4. controller lifecycle 시험
5. command timeout 시험
6. launch 종료 동작 시험
7. gait와 mock controller 통합 시험

mock hardware 검증 전 실제 모터에 연결하지 않는다.

### 7단계: `quattro_hardware` 프로토콜 계층

구현 순서는 다음 계층을 유지한다.

```text
SocketCAN
    -> MIT/GIM6010 protocol
    -> GIM6010 motor abstraction
    -> Motor manager
    -> ros2_control SystemInterface
```

세부 순서:

1. SocketCAN RAII 클래스
2. CAN frame 송수신 테스트
3. MIT encode/decode와 경계값 단위 테스트
4. node ID 검증
5. 단일 motor abstraction
6. enable/disable/set-zero
7. feedback decode
8. Motor manager
9. direction 및 offset 변환
10. hardware `read()`/`write()`
11. diagnostics
12. watchdog와 stale feedback
13. Safe Start와 안전 종료

### 8단계: 단일 GIM6010 실물 시험

1. CAN interface와 bitrate 확인
2. heartbeat 확인
3. encoder feedback만 수신
4. 현재 위치 읽기
5. 현재 위치 hold
6. 작은 각도 범위 이동
7. position/velocity/torque limit 확인
8. command timeout 확인
9. 프로세스 종료 시 disable 또는 안전 상태 확인

단일 모터 검증 전 여러 모터를 동시에 enable하지 않는다.

### 9단계: 12축 확대

1. 한 다리 3축
2. 좌우 두 다리
3. 네 다리 12축
4. CAN bus load 측정
5. feedback 주기 측정
6. stale motor 개별 감지
7. 전체 E-stop
8. nominal stance hold

### 10단계: 실제 보행 시험

1. 로봇을 지면에서 들어 올린 상태로 trajectory 확인
2. 낮은 gain과 낮은 속도로 단일 다리 swing
3. 대각 다리 swing
4. 지지대와 safety tether를 사용한 제자리 trot
5. 낮은 속도 전진
6. 후진과 횡이동
7. yaw 회전
8. IMU balance 결합
9. contact touchdown 결합
10. 장시간 watchdog 및 열 상태 시험

---

## 7. 당장 다음 작업

다음 작업은 실제 모터 구현이 아니라 description과 gait 안전성 검증이다.

1. RViz에서 12개 joint axis를 하나씩 검증한다.
2. FK 발 위치와 URDF TF 발 위치를 자동 비교하는 테스트를 추가한다.
3. URDF joint limit을 상위 trajectory 검증기에 연결한다.
4. gait 시작 및 정지 transition을 구현한다.
5. 최대 조이스틱 입력 범위 전체를 sweep test한다.
6. `base_footprint`를 추가하고 KDL root inertia 경고를 제거한다.

이 항목이 완료된 후 `ros2_control` description과 mock hardware로 진행한다.
