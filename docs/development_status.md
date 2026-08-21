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

`gim6010_driver`와 `quattro_hardware`는 모두 구현되어 있고, 2026-08-21부터 실물 12축 하드웨어(`can0`/`can1`)로 첫 실기 검증(`direct_position_tuning_gui`, `ros2 launch quattro_bringup hardware.launch.py`)을 진행 중이다. 아래 항목 중 "검증 완료"로 표시된 것은 이 실기 검증에서 확인된 사실이고, 나머지는 여전히 코드 검증이 아니라 하드웨어/매뉴얼 사실에 근거한 추정이므로 실기 연동을 계속 넓혀가며 확인해야 한다. 상세 근거는 `docs/packages/gim6010_driver.md`, `docs/packages/quattro_hardware.md`.

- **전원 재인가 후 멀티턴 위치 모호성 — 상충하는 증거, 실기 확인 필요**: 매뉴얼 BOM은 인코더 칩을 로터측 `MA732` 1개로만 기재하지만(이 경우 감속비 8:1로 인해 로터 절대각이 출력축 위치를 45° 간격으로만 구분), 실제 분해 사진 기반 제3자 보고(비공식)는 출력축 쪽에 별도 홀센서 기반 2차 encoder 보드가 있고 전원 재인가 후에도 회전수가 유지된다고 관찰했다(`docs/packages/quattro_hardware.md` 0절). 어느 쪽이 맞는지, 2차 encoder를 CAN으로 어떻게 읽는지 문서만으로 확정할 수 없어 6절의 벤치 시험(손으로 여러 바퀴 돌린 뒤 전원 재인가, `0x009`/`0x00A` 값 대조)으로 확인해야 한다.
- **heartbeat 형식 firmware 의존성**: heartbeat decoder는 firmware `0.5.13+` 형식을 전제한다. 실제 12개 장치의 firmware version을 확인하기 전에는 검증 완료로 볼 수 없다.
- **`Get_Error (0x03)`는 실기에서 응답하지 않음 — 검증 완료(2026-08-21)**: 응답 페이로드 형식(active_errors/disarm_reason uint32) 가정 자체가 아니라, `can0` node 0/1/2에 RTR·일반 프레임 양쪽으로 여러 차례 직접 요청했으나 **응답이 한 번도 오지 않음**을 `candump`/`cansend`로 확인했다(같은 조건에서 `Get_Encoder_Estimates(0x09)`는 매번 응답, `Heartbeat(0x01)`도 12개 노드 전부 100ms 주기로 정상 도착). `direct_position_tuning_gui`가 이 응답을 기다리는 조건 때문에 enable이 항상 실패했고, `QuattroSystem::on_activate`/`read()`도 같은 이유로 동일하게 막혀 있었다(`on_activate`는 이 문제로 실기에서 한 번도 성공할 수 없는 상태였다). 세 곳 모두 `Heartbeat.axis_error`로 대체해 수정했다 — 상세는 `docs/packages/gim6010_driver.md` 0절, `docs/packages/quattro_hardware.md` 2/4/5절.
- **RTR/주기 조회 부분 검증**: `0x009`(Get_Encoder_Estimates)와 `0x001`(Heartbeat)은 위 확인 과정에서 실기 응답을 확인했다. `0x00A`(Get_Encoder_Count)/`0x017`(Get_Bus_Voltage_Current)은 아직 실기로 확인한 적이 없다. `0x003`(Get_Error)은 위 항목대로 "응답 없음"으로 확인 완료.
- **`on_configure`의 무-pacing 프레임 전송이 커널 CAN TX 큐를 넘침 — 검증 완료(2026-08-21)**: `hardware.launch.py`로 첫 12축 bringup을 실행하자 `QuattroSystem::on_configure`가 관절당 3프레임(Set_Limits/Set_Pos_Gain/Set_Vel_Gains) × 12관절을 텀 없이 보내면서 각 bus의 5~6번째 관절(`front_right_upper/lower_leg_joint`, `back_right_upper/lower_leg_joint`)에서 `CanSocket::send()`(non-blocking `write()`)가 커널 TX 큐(기본 `qlen 10`) 초과로 실패했고, `resource_manager`가 이를 흡수하지 못해 `ros2_control_node`가 abort됐다. `on_configure`의 전송을 최대 20회·2ms 간격 재시도(`send_with_retry`)로 고쳤다 — 상세는 `docs/packages/quattro_hardware.md` 2절. 아래 두 항목대로 `write()` 실시간 경로에서도 결국 같은 큐 한계가 원인인 문제가 재현됐다.
- **node 3/5/11의 자발적 encoder estimate 브로드캐스트(100Hz) + `write()`가 CAN TX 큐 실패 1회에 바로 전체 safe stop — 검증 완료(2026-08-21)**: 12관절 정상 bringup(configure→activate→controller 전부 active) 후 약 52초 뒤 `front_right_lower_leg_joint`에서 "position command rejected" → `write()`가 즉시 12관절 전체 safe stop시켰다. 조사 결과 `front_right_hip_joint`(node 3)/`front_right_lower_leg_joint`(node 5, `can0`)/`back_right_lower_leg_joint`(node 11, `can1`) 세 노드가 **아무 프로세스도 안 떠 있는 상태에서도** 요청 없이 자기 encoder estimate를 100Hz로 계속 브로드캐스트하고 있음을 확인했다(`docs/packages/gim6010_driver.md` 0절, 원인/끄는 방법 미확정) — 이 추가 트래픽이 커널 CAN TX 큐(`qlen 10`)를 이따금 채워 같은 cycle의 다른 전송을 실패시키는 것으로 추정된다. `write()`가 단 1회 실패에 전체를 세우는 건 4절의 "일시적 프레임 누락과 실제 fault를 구분한다" 원칙에 어긋나므로, 관절별 연속 실패 카운터를 두어 3 cycle(30ms) 연속 실패해야만 그 관절을 fault로 보고 safe stop하도록 고쳤다(`consecutive_write_failures_`) — 상세는 `docs/packages/quattro_hardware.md` 2절.
- **`RxSdo/TxSdo(0x04/0x05)` 자체가 응답 없음 — 검증 완료(2026-08-21), 이 자발적 브로드캐스트를 끄거나 나머지 9개에 켤 방법 없음으로 결론**: `axis0.config.can.encoder_rate_ms` 같은 ODrive 표준 설정으로 브로드캐스트를 켜거나 끌 수 있을 것으로 보고, raw SocketCAN으로 node 3(endpoint 0~1999)/node 0(0~299)에 `RxSdo` Read를 전수 조사했으나 **`TxSdo` 응답이 한 건도 없었다**(같은 소켓의 `Get_Encoder_Estimates` RTR 요청은 정상 응답해 조사 방법 자체는 검증됨). Write도 무응답. `Get_Error`에 이어 **RxSdo/TxSdo 전체가 이 펌웨어에서 미구현**이라는 뜻이라, 이름 붙지 않은 파라미터(극쌍수, 토크상수, `encoder_rate_ms` 등)에 CAN으로 접근할 방법이 이 로봇의 모터들에는 없다. 대신 `QuattroSystem::read()`가 이미 fresh한 feedback은 다시 요청하지 않도록 고쳐서(하드코딩된 node 목록 없이 "이미 fresh하면 skip"이라는 일반 규칙), 자발적으로 도는 노드에 대한 우리 쪽 중복 요청만이라도 없앴다 — 상세는 `docs/packages/gim6010_driver.md` 0절, `docs/packages/quattro_hardware.md` 2절.
- **순차 활성화 직후 stale-feedback 오탐으로 즉시 safe stop — 검증 완료(2026-08-21)**: `on_configure` 수정 후 처음으로 `on_activate`(2단계 순차 활성화)까지 성공했으나, 활성화 직후 첫 `read()`에서 12관절 중 8개가 "stale feedback"으로 즉시 실패해 활성화된 지 수백 ms 만에 전체가 safe stop됐다(`joint_state_broadcaster`/`joint_trajectory_controller` activate도 이 때문에 연쇄로 실패). 원인: 2단계가 12관절을 `motor_activation_interval_ms`(100ms)씩 순차 활성화하는 동안 이미 활성화된 관절의 encoder estimate를 다시 요청하지 않아(`motor_manager_->poll()`은 수신만 드레인) 총 소요시간(1.2초+)이 `feedback_timeout_ms`(150ms)를 훌쩍 넘겼고, `read()`가 재개되자마자 먼저 활성화된 관절부터 stale로 잡혔다. `on_activate`가 `active_ = true`로 넘어가기 전에 전체 관절의 feedback을 한 번 더 refresh하고 fresh 응답을 기다리도록(`wait_for_all_motors_fresh_feedback`) 고쳤다 — 상세는 `docs/packages/quattro_hardware.md` 2절.
- **활성화 순간 반대 방향으로 크게 회전 — 검증 완료, 원인은 encoder 모호성이 아니라 명령 순서(2026-08-21)**: 실기에서 관절을 활성화(현재 위치를 target으로 hold)하는 바로 그 순간, 이미 반대 방향으로 180° 이상 틀어지는 현상이 보고됐다. target은 그 순간 읽은 현재 위치 그대로라 위 "멀티턴 위치 모호성" 항목과는 무관하다(그 경우라면 hold 자체가 틀린 위치에서 조용히 정지할 뿐, "hold하려는 순간 반대로 튄다"는 식으로 나타나지 않는다). 실제 원인은 `activate_joint()`/`calibration_gui`/`direct_position_tuning_gui`의 enable 순서였다 — `Set_Axis_State(closed-loop)`를 먼저 보내고 `Set_Input_Pos`(hold 목표)를 나중에 보냈는데, `Input_Pos`는 axis state와 무관하게 마지막으로 쓴 값(이전 세션에서 명령했던, 몇 바퀴씩 떨어진 값일 수 있음)을 그대로 들고 있는 레지스터라, 그 사이 창에서 axis가 이미 그 stale한 값을 향해 실제로 구동을 시작했다. 세 곳 모두 `Set_Input_Pos`를 `Set_Axis_State(closed-loop)`보다 먼저 보내도록 순서를 바꿔 이 창을 없앴다 — 상세는 `docs/packages/quattro_hardware.md` 2/5절.
- **CAN adapter/kernel 조합의 에러 카운터 미검증**: warning/passive/bus-off/ACK/protocol/TRX/TX/RX 에러 카운터가 실제 CAN 어댑터·커널 조합에서 기대대로 잡히는지 확인이 필요하다.
- **gain/limit 실측값 없음**: Position gain, MIT Kp/Kd, current limit은 로봇 실제 하중으로 결정해야 하며 매뉴얼 예시값(`20.0/0.16/0.32`)을 factory default로 가정하지 않는다.
- **Raspberry Pi 5 + Docker 타이밍 미측정**: `update_rate=100 Hz`에서 feedback/heartbeat/scheduling watchdog 값(`docs/packages/quattro_hardware.md` 7절 — 원칙만 정의, 구체적 ms 값은 실측 후 확정)이 실제 jitter 대비 적절한지 실측이 필요하다.
- **`quattro_sensors`(BNO085)**: Raspberry Pi 5 실제 I2C 조건에서 계속 검증이 필요하다.

12축 gait 시험으로 범위를 넓히기 전에 `docs/packages/quattro_hardware.md` 6절의 단일 모터부터 확대하는 시험 순서를 따른다.

## 문서 유지 규칙

새 기능이 실제 코드에 들어가면 이 문서의 “구현됨” 항목을 갱신한다.

코드가 존재하지만 실물 검증이 되지 않은 기능은 “구현 완료”와 “실기 검증 완료”를 구분해서 기록한다.
