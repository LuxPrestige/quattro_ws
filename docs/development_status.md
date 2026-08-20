# Quattro 개발 현황

## 목적

이 문서는 현재 저장소에 구현된 기능과 아직 실기 검증이 필요한 항목을 기록한다.

세부 아키텍처와 실행법은 각각 다음 문서를 따른다.

- `docs/architecture.md`
- `docs/packages/*.md`
- `docs/gazebo.md`
- `docs/calibration.md`

## 현재 패키지

```text
src/
├── quattro/
├── quattro_description/
├── quattro_bringup/
├── quattro_gazebo/
├── quattro_hardware/    # 구현됨
├── quattro_sensors/
├── quattro_teleop/
└── gim6010_driver/       # 구현됨
```

패키지별 세부 구현은 `docs/packages/<패키지명>.md`를 따른다.

## 구현된 주요 기능

### `quattro`

- 3-DOF leg FK/IK
- Jacobian(analytic), foot velocity, damped pseudoinverse 기반 joint velocity, force→torque(`LegKinematics.jacobian`/`foot_velocity`/`joint_velocity_from_foot_velocity`/`force_to_joint_torque`)
- 4-leg FK/IK
- Bézier swing / stance gait(`GaitGenerator.update`) — diagonal trot phase
- `/cmd_vel` 기반 gait controller — 발끝 목표 궤적을 IK로 관절 각도(`positions`)로 변환해 `JointTrajectory`로 발행
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
- simulation controller 설정: `joint_trajectory_controller`(관절당 `position` command interface 하나)
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

- Linux SocketCAN RAII wrapper, non-blocking, error frame(warning/passive/bus-off) 파싱
- CAN Simple 전체 명령(axis 상태, Direct Position/Velocity/Torque, 리밋/게인, 텔레메트리, Get_Error) encode/decode
- MIT motion control encode/decode(범위 밖 입력은 clamp 없이 거부) — `calibration_gui`의 관절 영점 조깅 절차가 사용
- 다중 bus/다중 모터 라우팅(`MotorManager`), 특정 관절 구성에 하드코딩되지 않아 다른 프로젝트에서도 재사용 가능
- 단일 모터 read-only 진단 CLI(`gim6010_diagnostic`)
- gtest 36개 통과(실제 CAN 버스 없이 encode/decode/routing 검증, `docs/packages/gim6010_driver.md` 6절). `quattro_hardware`가 이 라이브러리를 링크하지만, 실물 CAN이 없어 실기 연동은 미검증.

세부 내용은 `docs/packages/gim6010_driver.md`.

### `quattro_hardware`

**구현됨.** `QuattroSystem`(`hardware_interface::SystemInterface`)이 `gim6010_driver`를 라이브러리로 링크해 joint 방향/오프셋/기어비 변환, 순차 활성화, Direct Position 명령 변환을 수행한다. `calibration_gui`도 함께 구현되어 있다. 소비 측 계약(`ros2_control` 파라미터 이름, command/state interface 목록)은 `quattro_description`의 Xacro와 `quattro_bringup`에 고정되어 있다. 실제 CAN bus·모터 연동(`on_activate`/`read`/`write`)은 이 개발 환경에 실물 CAN이 없어 미검증이다. 상세는 `docs/packages/quattro_hardware.md`.

### `quattro_bringup`

- 실제 하드웨어 launch
- controller manager
- hardware spawner
- `joint_state_broadcaster`
- `joint_trajectory_controller`
- IMU/teleop/gait controller 실행 조합
- remote visualization 관련 launch
- `motor_activation_interval_ms` 기본값 `100`

## 현재 하드웨어 구성

- SteadyWin GIM6010-8 × 12 (온보드 인코더 1개, secondary encoder 없음 — `docs/packages/quattro_hardware.md` 0절)
- GDS68
- CAN Simple / Direct Position
- `can0`: CAN ID 0~5
- `can1`: CAN ID 6~11
- 500 kbit/s

상세 매핑은 `docs/packages/quattro_hardware.md`(0절)를 따른다.

## 현재 중요한 실기 이슈

`gim6010_driver`와 `quattro_hardware`는 모두 구현되어 있지만 실제 CAN 버스·모터 연동(`on_activate`/`read`/`write`)은 이 개발 환경에 실물 CAN이 없어 아직 검증되지 않았다. 아래는 코드 검증이 아니라 하드웨어/매뉴얼 사실에 근거한, 실기 연동 시 반드시 다뤄야 할 위험이다. 상세 근거는 `docs/packages/gim6010_driver.md`, `docs/packages/quattro_hardware.md`.

- **전원 재인가 후 멀티턴 위치 모호성 — 상충하는 증거, 실기 확인 필요**: 매뉴얼 BOM은 인코더 칩을 로터측 `MA732` 1개로만 기재하지만(이 경우 감속비 8:1로 인해 로터 절대각이 출력축 위치를 45° 간격으로만 구분), 실제 분해 사진 기반 제3자 보고(비공식)는 출력축 쪽에 별도 홀센서 기반 2차 encoder 보드가 있고 전원 재인가 후에도 회전수가 유지된다고 관찰했다(`docs/packages/quattro_hardware.md` 0절). 어느 쪽이 맞는지, 2차 encoder를 CAN으로 어떻게 읽는지 문서만으로 확정할 수 없어 6절의 벤치 시험(손으로 여러 바퀴 돌린 뒤 전원 재인가, `0x009`/`0x00A` 값 대조)으로 확인해야 한다.
- **heartbeat 형식 firmware 의존성**: heartbeat decoder는 firmware `0.5.13+` 형식을 전제한다. 실제 12개 장치의 firmware version을 확인하기 전에는 검증 완료로 볼 수 없다.
- **`Get_Error (0x03)` 응답 형식 가정 미검증**: `gim6010_driver`는 ODrive CAN Simple 표준 형식(active_errors/disarm_reason, 각 uint32, 요청 없이도 자기완결적으로 decode 가능)으로 구현했다(`docs/packages/gim6010_driver.md` 0절). 실기에서 다른 폭이나 카테고리별 응답이 관찰되면 그 즉시 이 가정과 `can_simple_messages.cpp`를 함께 수정해야 한다.
- **RTR/주기 조회 미검증**: `0x009`/`0x00A`/`0x017`과 보정된 `0x003` 조회를 단일 모터에서 실기로 확인한 적이 없다.
- **CAN adapter/kernel 조합의 에러 카운터 미검증**: warning/passive/bus-off/ACK/protocol/TRX/TX/RX 에러 카운터가 실제 CAN 어댑터·커널 조합에서 기대대로 잡히는지 확인이 필요하다.
- **gain/limit 실측값 없음**: Position gain, MIT Kp/Kd, current limit은 로봇 실제 하중으로 결정해야 하며 매뉴얼 예시값(`20.0/0.16/0.32`)을 factory default로 가정하지 않는다.
- **Raspberry Pi 5 + Docker 타이밍 미측정**: `update_rate=100 Hz`에서 feedback/heartbeat/scheduling watchdog 값(`docs/packages/quattro_hardware.md` 7절 — 원칙만 정의, 구체적 ms 값은 실측 후 확정)이 실제 jitter 대비 적절한지 실측이 필요하다.
- **`quattro_sensors`(BNO085)**: Raspberry Pi 5 실제 I2C 조건에서 계속 검증이 필요하다.

12축 gait 시험으로 범위를 넓히기 전에 `docs/packages/quattro_hardware.md` 6절의 단일 모터부터 확대하는 시험 순서를 따른다.

## 문서 유지 규칙

새 기능이 실제 코드에 들어가면 이 문서의 “구현됨” 항목을 갱신한다.

코드가 존재하지만 실물 검증이 되지 않은 기능은 “구현 완료”와 “실기 검증 완료”를 구분해서 기록한다.
