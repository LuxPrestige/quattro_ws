# Quattro 개발 현황

## 목적

이 문서는 현재 저장소에 구현된 기능과 아직 실기 검증이 필요한 항목을 기록한다.

세부 아키텍처와 실행법은 각각 다음 문서를 따른다.

- `docs/architecture.md`
- `docs/gazebo.md`
- `docs/gim6010_hardware.md`
- `docs/calibration.md`
- `docs/hardware_bringup.md`

## 현재 패키지

```text
src/
├── quattro/
├── quattro_description/
├── quattro_bringup/
├── quattro_gazebo/
├── quattro_hardware/
├── quattro_sensors/
├── quattro_teleop/
└── gim6010_driver/
```

## 구현된 주요 기능

### `quattro`

- 3-DOF leg FK/IK
- 4-leg FK/IK
- Bézier swing / stance gait
- diagonal trot phase
- `/cmd_vel` 기반 gait controller
- body pose command
- IMU roll/pitch balance PID
- `JointTrajectory` 출력

### `quattro_description`

- 12축 Quattro Xacro
- link/joint/inertial/collision
- STL mesh
- 실제 하드웨어와 Gazebo용 `ros2_control` 분기
- RViz 관련 설정

### `quattro_gazebo`

- Gazebo Harmonic world
- `gz_ros2_control`
- simulation controller 설정
- headless/GUI launch
- gait controller 연동

### `quattro_teleop`

- Switch Pro Controller 입력
- keyboard teleop
- gait/pose 관련 표준 ROS 명령 발행

### `quattro_sensors`

- BNO085 ROS 2 node
- `sensor_msgs/msg/Imu` 출력 구조

실제 Raspberry Pi I2C 조건에서 계속 검증이 필요하다.

### `gim6010_driver`

- Linux SocketCAN RAII wrapper
- standard CAN frame 송수신
- CAN error frame 및 warning/passive/bus-off 진단
- CAN Simple arbitration ID
- Direct Position/Velocity/Torque 및 MIT command encode/decode
- Position Filter/Trapezoidal input mode와 runtime controller gain 설정
- heartbeat decode
- error/bus voltage/q-axis current diagnostics
- encoder estimates
- 단일 GIM6010 motor abstraction
- 다중 motor routing

### `quattro_hardware`

- `hardware_interface::SystemInterface`
- 12축 joint ↔ motor mapping
- direction / offset 변환
- Safe Start
- 현재 위치 hold
- Direct Position enable 전 current-position target 준비
- MIT 전용 5-command-interface와 Kp engagement
- `read()` / `write()`
- feedback timeout
- heartbeat timeout
- command watchdog
- fault diagnostics
- safe stop
- calibration GUI
- read-only 단일 모터 diagnostic CLI

### `quattro_bringup`

- 실제 하드웨어 launch
- controller manager
- hardware spawner
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- IMU/teleop/gait controller 실행 조합
- remote visualization 관련 launch

## 현재 하드웨어 구성

- SteadyWin GIM6010-8 × 12
- GDS68 + secondary encoder
- CAN Simple / Direct Position·Velocity·Torque / MIT Control
- `can0`: CAN ID 0~5
- `can1`: CAN ID 6~11
- 500 kbit/s

상세 매핑은 `docs/gim6010_hardware.md`를 따른다.

## 현재 중요한 실기 이슈

### 1. 모터가 운전 중 비활성화되는 원인 분리

현재 `QuattroSystem`은 다음 조건에서 safe stop을 수행한다.

- command watchdog timeout
- stale feedback
- stale heartbeat
- motor가 closed-loop를 이탈
- axis fault
- 잘못된 joint command

따라서 실제 GDS68 fault와 소프트웨어가 의도적으로 전체 motor를 disable한 경우를 구분해야 한다.

필요한 진단:

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
candump -e -tz can0
candump -e -tz can1
```

그리고 ROS 로그의 다음 항목을 함께 확인한다.

```text
Hardware command watchdog expired
Stale feedback
Stale heartbeat
Motor left closed-loop control
Motor fault details
```

### 2. CAN error frame 실기 검증

SocketCAN 계층과 `/diagnostics` 전달은 구현됐지만 실제 두 CAN bus에서 fault injection 검증이 필요하다.

필요 항목:

- error-warning / error-passive / bus-off fault injection
- kernel 및 CAN adapter별 error frame 지원 확인
- stale feedback과 CAN physical error 구분

### 3. gain / current limit 실기 검증

현재 gain과 current limit은 코드에 존재하지만 최종 안전값으로 확정된 상태가 아니다.

일반 Position gain 자동 적용은 기본적으로 꺼져 있어 장치 값을 보존한다. 문서의 `20.0/0.16/0.32`는 제조사 tuning 예시이며 factory default로 확인된 값이 아니다.

필요 항목:

- 모터 전압 버전과 GDS68 사양 재확인
- 단일 모터 저전류/저gain 시험
- offset/direction 오설정 시 보호 동작 확인
- 한 다리, 12축 순으로 확대

### 4. calibration 검증

실제 머신별 `calibration.yaml`은 Git에 저장하지 않는다.

실기에서 확인할 항목:

- 12개 CAN ID와 bus
- direction
- offset
- encoder zero와 ROS joint zero 일치 여부

## 아직 필요한 검증

- 모든 gait 범위에서 URDF joint limit 준수
- joint velocity / acceleration 제한
- gait 시작/정지 transition 연속성
- 실제 joystick axis/button 검증
- IMU orientation과 PID 부호 실기 검증
- contact sensor 실제 연결 및 stale/debounce
- `/joint_states` 100 Hz 안정성
- controller manager 100 Hz scheduling 안정성
- 두 CAN bus의 장시간 부하 측정
- timeout/fault 시 원인별 diagnostics
- 로봇 지지 상태에서 12축 nominal stance
- 실제 지면 보행

## 권장 개발 순서

현재는 기본 소프트웨어 구조가 이미 구현되어 있으므로 과거의 “description부터 처음 구현” 계획은 더 이상 기준으로 사용하지 않는다.

다음 순서를 권장한다.

1. 단일 GIM6010/GDS68 CAN 진단 안정화
2. CAN error frame과 fault diagnostics 강화
3. 단일 모터 gain/current/timeout 검증
4. 머신별 12축 calibration 재검증
5. 한 다리 3축 hold 및 작은 trajectory
6. 12축 nominal stance hold
7. controller 100 Hz 장시간 안정성 시험
8. 지지대 상태 gait 시험
9. IMU balance 결합
10. 실제 지면 저속 gait
11. contact feedback 결합
12. 장시간 열/전원/CAN fault 시험

## 문서 유지 규칙

새 기능이 실제 코드에 들어가면 이 문서의 “구현됨” 항목을 갱신한다.

코드가 존재하지만 실물 검증이 되지 않은 기능은 “구현 완료”와 “실기 검증 완료”를 구분해서 기록한다.
