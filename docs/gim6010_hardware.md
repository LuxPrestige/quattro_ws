# GIM6010-8 / GDS68 하드웨어 구조

## 1. 대상 하드웨어

Quattro는 다음 조합을 사용한다.

- SteadyWin GIM6010-8 × 12 (온보드 인코더는 MA732 14-bit single-turn absolute 1개뿐, 물리적으로 분리된 secondary encoder 없음 — 11절 참고)
- GDS68 driver
- Linux SocketCAN
- CAN Simple
- Direct Position / Velocity / Torque 또는 MIT Control (실행 시 선택)
- 기본 bitrate: `500000 bit/s`

세부 CAN 명령과 하드웨어 사양은 `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` 및 실제 검증된 firmware를 기준으로 한다.

## 2. 소프트웨어 계층

```text
ros2_control controller
        ↓
quattro_hardware::QuattroSystem
        ↓
joint direction / offset / limits / safety
        ↓
gim6010_driver::Gim6010Motor
        ↓
CAN Simple direct/MIT command routing
        ↓
Direct/MIT encode/decode
        ↓
SocketCAN
        ↓
can0 / can1
        ↓
GDS68
```

`quattro_hardware`는 raw CAN payload를 직접 만들지 않는다.

`gim6010_driver`는 Quattro joint 이름이나 URDF를 알지 않는다.

## 3. `gim6010_driver`

주요 구성:

```text
include/gim6010_driver/
├── can_frame.hpp        원시 CAN frame 구조체 (전송 계층 비의존)
├── can_socket.hpp        SocketCAN RAII wrapper (유일하게 OS를 건드리는 클래스)
├── can_error.hpp          커널 에러 frame 순수 decode (ROS/소켓 비의존, unit test 가능)
├── can_diagnostics.hpp   버스별 에러/드롭 카운터 집계
├── types.hpp              공용 enum/struct (MotorControlMode, ErrorType 등)
├── gds68_protocol.hpp     CAN Simple/GDS68 encode·decode 순수 함수
├── mit_protocol.hpp       MIT bit-pack encode·decode 순수 함수
├── gim6010_motor.hpp       단일 모터 상태 추상화
└── motor_manager.hpp        bus당 1개, arbitration id 기반 프레임 라우팅
```

encode/decode 함수(`gds68_protocol.*`, `mit_protocol.*`)와 에러 frame 분류(`can_error.*`)는 CAN 소켓이나 ROS에 의존하지 않는 순수 함수이며 `test/`의 gtest로 검증한다. 모터별 `MotorControlMode` 상태를 `Gim6010Motor`가 직접 들고 있어, configure하지 않은 mode의 명령(예: MIT configure 없이 `sendMitCommand`)을 보내면 즉시 예외를 던진다.

책임:

- standard CAN frame 송수신
- Linux CAN error frame 수집과 warning/passive/bus-off 분류
- CAN Simple arbitration ID 생성
- Direct Position (`0x00C`), Velocity (`0x00D`), Torque (`0x00E`) encode
- MIT command encode/decode
- heartbeat / encoder / q-axis current / bus telemetry decode
- 주기적인 q-axis current와 bus telemetry 조회는 CAN burst를 피하도록 모터별로 분산
- GIM6010 motor abstraction
- 한 CAN bus의 다중 motor routing

현재 arbitration ID 형식:

```text
(node_id << 5) | command_id
```

GDS68 매뉴얼 rev2.2 기준 CAN Simple은 11-bit standard frame이며 일반 payload는 little-endian이다. 단, MIT Control의 bit-packed payload는 표에 정의된 순서대로 상위 비트부터 배치하므로 일반 payload의 byte order 규칙을 그대로 적용하지 않는다.

| 메시지 | 좌표/단위 | 드라이버 처리 |
|---|---|---|
| `0x008` MIT feedback | 출력축, rad / rad/s / N·m | 출력축 SI 값으로 사용 (runtime 위치/속도 source) |
| `0x009` Get_Encoder_Estimates | motor rotor, rev / rev/s | `2π / 8`을 곱해 출력축 rad / rad/s로 변환 (runtime 위치/속도 source) |
| `0x00A` Get_Encoder_Count | rotor shadow count(다중회전) / count_in_cpr(단일회전), 정수 | startup 진단 전용. `0x009`와 같은 센서의 원시 카운트이며 runtime feedback에 섞지 않는다 |
| `0x003` Get_Error | 카테고리별 uint32/uint64 | fault 진단 전용 — 11절 참고 |
| `0x014` Get_Iq | motor current, A | setpoint과 measured q-axis current를 진단 항목으로 사용 |

`Get_Error` (`0x003`)는 요청 payload 1바이트로 카테고리를 고르고(`0`=motor, `1`=encoder, `3`=controller, `4`=system — `2`는 매뉴얼에 정의되지 않음), 응답에는 카테고리 tag가 없다. 응답 폭도 카테고리마다 다르다: motor는 uint64(8바이트 전체), 나머지는 uint32(4바이트). 따라서 호출자(`Gim6010Motor`)가 "어떤 카테고리를 요청했는지"를 큐로 직접 추적해야 응답을 올바르게 해석할 수 있다.

현재 heartbeat decoder는 firmware `0.5.13+` 형식(axis error, state, 통합 flags, reserved, life)을 기준으로 한다. `0.5.11` 이하에서는 motor/encoder/controller flag가 서로 다른 byte에 있고 life가 없으므로 호환되지 않는다. 실제 12개 장치의 firmware version을 확인하기 전에는 실기 검증 완료로 간주하지 않는다.

### 제어 방식 선택

| `hardware_control_method` | GDS68 mode | ros2_control command | GDS68 command 단위 |
|---|---|---|---|
| `direct_position` | control 3, input 1 | `position` rad | rotor rev |
| `direct_velocity` | control 2, input 1 | `velocity` rad/s | rotor rev/s |
| `direct_torque` | control 1, input 1 | `effort` output N·m | motor N·m |
| `mit` | control 3, input 9 | `position/velocity/kp/kd/effort` | output rad, rad/s, N·m |

Direct position/velocity는 `8 / 2π`로 output SI 값을 rotor rev 계열로 변환한다. Direct torque는 이상적인 8:1 감속을 기준으로 output torque를 8로 나눠 motor torque command로 변환한다. 실제 효율과 마찰은 포함하지 않는다.

bringup 기본값은 `mit`이며 전용 `quattro_controllers::MitTrajectoryController`가 다섯 command interface를 모두 claim한다. 일반 `JointTrajectoryController`로 MIT position만 claim하는 mode switch는 거부한다. `direct_position`/`direct_velocity`/`direct_torque`는 `hardware_control_method`와 그에 맞는 `controller_file`/`command_controller_name`을 함께 명시해 선택한다 (`docs/hardware_bringup.md` 참고). `MitTrajectoryController`의 `kp`/`kd`는 launch 시점에 `calibration.yaml`의 joint별 값에서 채워지므로, 활성화 gain-ramp에 쓰이는 값과 정상 주행 중 값이 같은 소스를 공유한다.

### Position controller gain

일반 Position Control은 cascade controller의 다음 gain을 사용한다.

- Position gain: `Set_Pos_Gain`, command `0x1A`, float32 little-endian, `(rev/s)/rev`
- Velocity gain: `Set_Vel_Gains`, command `0x1B`, byte 0 float32, `N·m/(rev/s)`
- Velocity integrator gain: `0x1B`, byte 4 float32, `N·m/rev`

이는 MIT의 Kp/Kd와 다른 타입이다. 매뉴얼의 `20.0 / 0.16 / 0.32`는 PID 튜닝 절차의 예시이며 factory default라는 근거가 없다. 따라서 `apply_position_gains=false`가 기본이고 장치의 현재 runtime 값을 보존한다. 명시적으로 true일 때만 configure 단계에서 CAN runtime gain을 쓴다. `Save_Configuration (0x1F)`은 자동 호출하지 않으므로 flash/nonvolatile 설정은 변경되지 않는다.

Position input mode는 Direct `1`, Position Filter `3`, Trapezoidal Trajectory `5`를 타입으로 지원한다. Quattro 기본은 Direct이다. Trapezoidal mode의 velocity/acceleration/deceleration command `0x11/0x12`도 저수준 API에 구현했다. Position Filter bandwidth는 rev2.2의 고정 CAN Simple command가 아니라 firmware별 endpoint 접근이 필요하므로 추측 구현하지 않았다.

Direct mode에서 `0x009`는 position/velocity만 제공하므로 effort state는 측정값을 확보하지 못한 동안 `NaN`이다. 명령값을 실제 feedback처럼 게시하지 않는다.

## 4. `quattro_hardware`

`hardware_interface::SystemInterface`를 구현한다.

state interface:

- position
- velocity
- effort

기본 command interface는 `position` 하나이며 GDS68 Direct Position `Set_Input_Pos`로 연결된다. 선택적 Direct Velocity/Torque는 각각 표준 `velocity`/`effort` 하나를 쓴다. MIT는 전용 controller가 다섯 interface를 모두 claim해야 한다.

현재 joint 좌표 변환:

```text
joint_position = direction * motor_position - offset
motor_position = direction * (joint_command + offset)
```

offset 단위는 rad이다.

`Set_Limits (0x00F)`의 velocity는 ROS joint rad/s가 아니라 motor rotor rev/s이고 current는 A이다. 현재 보수적 초기값은 각각 `5.0 rev/s`, `5.0 A`이며 최종 운용값으로 확정된 수치가 아니다. 출력축 속도 환산은 `rotor_rev_s * 2π / 8`이다.

## 5. CAN ID와 bus 매핑

| Joint | CAN ID | Bus |
|---|---:|---|
| `front_left_hip_joint` | 0 | `can0` |
| `front_left_upper_leg_joint` | 1 | `can0` |
| `front_left_lower_leg_joint` | 2 | `can0` |
| `front_right_hip_joint` | 3 | `can0` |
| `front_right_upper_leg_joint` | 4 | `can0` |
| `front_right_lower_leg_joint` | 5 | `can0` |
| `back_left_hip_joint` | 6 | `can1` |
| `back_left_upper_leg_joint` | 7 | `can1` |
| `back_left_lower_leg_joint` | 8 | `can1` |
| `back_right_hip_joint` | 9 | `can1` |
| `back_right_upper_leg_joint` | 10 | `can1` |
| `back_right_lower_leg_joint` | 11 | `can1` |

기본 direction:

```text
[-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]
```

실제 offset은 머신별 `src/quattro_bringup/config/calibration.yaml`에서 관리한다.

## 6. 설정 파일

### 실제 머신

```text
src/quattro_bringup/config/calibration.yaml
```

Git에서 제외한다.

### 버전 관리 템플릿

```text
src/quattro_bringup/config/calibration.yaml.example
```

### description 기본값

Xacro/URDF 검사에 사용하는 기본 calibration과 실제 머신별 calibration을 혼동하지 않는다.

## 7. Safe Start

실제 하드웨어 활성화 시 기본 원칙:

1. CAN socket과 motor mapping 생성
2. heartbeat와 encoder communication 확인
3. 기존 fault와 상세 error를 오류 삭제 없이 capture
4. 현재 output position 확보
5. Position control/input mode와 limit 설정
6. 요청된 경우에만 runtime Position gain 설정
7. 현재 위치를 Direct Position target으로 준비
8. closed-loop enable
9. 현재 위치 target 재송신과 feedback/heartbeat 확인
10. 상위 controller command 허용

부팅 직후 0 rad 같은 임의 목표를 먼저 보내지 않는다.

## 8. 안전 정책

하드웨어 제어에서 최소한 다음을 검출한다.

- invalid node ID
- invalid command range
- command timeout
- stale motor feedback
- stale heartbeat
- axis/controller/system fault
- CAN socket 오류

오류 시 안전 상태로 전환하고 원인을 로그/diagnostics에 남긴다.

startup은 원인 보존을 위해 자동으로 `Clear_Errors`를 호출하지 않는다. 기존 fault가 있으면 활성화를 거부한다. 오류 삭제는 원인을 확인한 뒤 별도의 명시적 정비 절차에서만 수행한다.

일시적 CAN frame 누락과 실제 GDS68 fault는 가능한 한 구분하여 진단한다.

한 모터의 통신 문제 때문에 전체를 정지시키는 정책은 로봇 안전상 필요할 수 있지만, 원인 분석을 위해 어떤 조건이 전체 safe stop을 발생시켰는지 반드시 기록한다.

## 9. CAN 진단

호스트에서 CAN 상태:

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

raw frame:

```bash
candump -e -tz can0
candump -e -tz can1
```

확인 대상:

- `ERROR-ACTIVE`
- `ERROR-WARNING`
- `ERROR-PASSIVE`
- `BUS-OFF`
- RX/TX error count
- dropped frame

`CanSocket`은 `CAN_RAW_ERR_FILTER`와 `SO_RXQ_OVFL`을 활성화해 error-warning, error-passive, bus-off, ACK/protocol/transceiver error, TX/RX error counter와 socket RX drop을 수집한다. `QuattroSystem`은 passive와 bus-off를 safe stop 원인으로 취급하고 warning은 진단 경고로 유지한다. 커널 error frame만으로 모든 누적 통계를 대체할 수는 없으므로 host 상태도 함께 확인한다.

## 10. gain / current 설정 주의

Position gain/velocity PI, MIT Kp/Kd와 current limit은 서로 분리하며 로봇 하중, firmware, 전압 버전에 따라 실제 시험으로 결정한다.

높은 Kp 상태에서 joint offset 또는 direction이 잘못되면 position error가 즉시 큰 torque/current command로 변환될 수 있다.

따라서 최초 시험은:

- 로봇 지지
- 단일 모터
- 낮은 gain
- 낮은 current limit
- 작은 각도 이동

순서로 진행한다.

하드웨어 사양을 확인하지 않고 12축 전체에 큰 current limit을 일괄 적용하지 않는다.

## 11. Encoder: 물리적으로 하나뿐이다 (secondary encoder 없음)

2026-08-19 rev2.2 매뉴얼 원본(96페이지 전체)을 직접 확인해 다음을 확정했다. 이전 버전의 이 절은 "매뉴얼만으로 확정할 수 없다"고 hedge했으나, 전체 CAN Simple 명령 목록(`0x001`~`0x01F`)과 BOM을 대조한 결과 아래는 확정 사실이다.

- BOM(2.5절): 인코더 칩은 `MA732, 14비트 절대값` 1개뿐이다. 별도의 두 번째 인코더 칩은 없다.
- CAN Simple 전체 명령 목록에 "두 번째 인코더 값을 읽는" 명령이 없다. 인코더 관련 명령은 `Get_Encoder_Estimates (0x009)`, `Get_Encoder_Count (0x00A)`, `Set_Linear_Count (0x019)` 셋뿐이며 전부 **같은** `odrv0.axis0.encoder` 객체를 가리킨다.
- `SEC_ENC_COM_FAIL (0x00000400)`은 인코더 오류 카테고리에 존재하는 플래그이지만, 이 값을 발생시키거나 읽는 CAN 명령/하드웨어 인터페이스는 매뉴얼 어디에도 문서화되어 있지 않다. GDS68 펌웨어가 물려받은 ODrive 계열 오류 enum에 남아있는 값으로 보이며, 이 보드에서 실제로 기능하는 secondary encoder 입력이 있다는 근거가 아니다.
- 하드웨어 인터페이스 절(2.4절)에는 전원/CAN/RS485/PWM, Type-C 디버그, 확장 슬롯(v3.9부터 미지원), 브레이크 저항, 리미트 스위치만 있고 별도 인코더 입력 핀은 없다.

**결론: GIM6010-8에는 물리적으로 분리된 "secondary encoder"가 없다.** `encoder.pos_estimate`(=`Get_Encoder_Estimates`/`0x009`)와 `controller.input_pos`(=마지막으로 host가 `Set_Input_Pos`로 써넣은 command 값, 센서 아님)는 흔히 헷갈리기 쉬우나 어느 쪽도 두 번째 센서가 아니다.

### 그럼 절대 위치는 어떻게 확인하는가

`0x009`가 참조하는 그 하나의 encoder가 14-bit **single-turn absolute** 타입이므로, 전원 인가 즉시 **rotor 1회전 이내의 절대각**은 바로 얻는다 (움직여서 원점을 찾는 과정이 필요 없다). 이게 이 프로젝트에서 얻을 수 있는 "startup absolute position"의 전부다.

다만 기어비가 8:1이므로, rotor 절대각 하나만으로 알 수 있는 출력축 위치는 `360° / 8 = 45°` 간격으로 반복(wrap)된다. 실제 `quattro.urdf.xacro`의 관절 가동범위:

| 관절 | 가동범위 |
|---|---|
| hip | 약 119° (`-1.04`~`1.04` rad) |
| upper_leg | 약 239° (`-1.5708`~`2.59` rad) |
| lower_leg | 약 256° (`-2.9`~`1.5708` rad) |

전부 45°보다 훨씬 넓으므로, **로터의 단일회전 절대각만으로는 관절이 가동범위 내 몇 번째 45° 구간에 있는지 원리적으로 구분되지 않는다** (hip 기준 최대 약 3가지, upper/lower_leg 기준 최대 약 6가지 후보). 이 모호성이 풀리려면 GDS68 펌웨어가 전원이 꺼져 있는 동안에도 로터 회전수(멀티턴 카운트)를 비휘발성으로 보존해서 재부팅 후 이어받아야 하는데, 매뉴얼에는 이 동작이 명시되어 있지 않다.

`Get_Encoder_Count (0x00A)`의 `Shadow_Count`(멀티턴)도 같은 센서의 소프트웨어 누적 카운트이므로 이 문제를 해결해주지 않는다 — 펌웨어가 계속 켜져 있는 동안의 회전만 누적되며, 전원이 꺼졌다 켜진 뒤에도 그 값이 유지된다는 근거가 없다. `Set_Linear_Count (0x019)`는 그 카운트를 host가 원하는 값으로 강제 설정하는 명령이지만, 그 값을 어떤 근거로 정할지는(리미트 스위치, 수동 확인 등) 이 프로젝트에 없다.

**따라서 이 코드베이스는 "전원 재인가 후 관절의 정확한 절대 위치를 CAN만으로 자동 검증"하는 문제를 풀지 않는다.** 대신 다음 원칙으로 안전을 확보한다.

1. `waitForInitialFeedback()`이 활성화 전 `0x009`/`0x008`의 fresh feedback을 반드시 확인하고, 확인되지 않으면 startup을 실패시킨다(임의 값으로 진행하지 않는다).
2. 활성화 직후 명령 target은 항상 그 순간 읽은 실제 위치로 초기화한다(8절, Safe Start) — 즉 "45° 중 어느 구간인지"가 틀리더라도 최소한 활성화 순간에 급격한 이동은 만들지 않는다.
3. `docs/hardware_bringup.md`의 실기 안전 절차(로봇 지지, 단일 모터, 낮은 gain부터 확대)를 그대로 유지한다 — 만약 관절이 예상과 다른 45° 구간에서 재부팅됐다면 이 절차에서 곧바로 드러난다.
4. `calibration.yaml`의 `offset`은 어디까지나 "encoder zero 대비 ROS joint zero의 위치"를 정의할 뿐, 전원 재인가 시 멀티턴 모호성을 해소해주지 않는다는 점을 캘리브레이션 수행자가 인지해야 한다(13절).

이 멀티턴 모호성은 "실기 검증 필요" 항목 중 하나이며(`docs/gim6010_audit.md` 참고), 실제 GDS68 펌웨어가 인코더 카운트를 비휘발성으로 보존하는지 확인되기 전까지는 미해결로 취급한다.

## 12. Watchdog 정책

- encoder periodic 기본 10 ms 대비 feedback fault 150 ms
- heartbeat periodic 기본 100 ms 대비 heartbeat fault 400 ms
- controller scheduling delay 50 ms부터 warning, 250 ms에서 fault
- CAN warning은 경고, error-passive와 bus-off는 별도 fault

이는 hard realtime 보장이 아니라 단일 지연 frame과 지속 장애를 구분하기 위한 보수적 초기 정책이다. Raspberry Pi 5 실측 후 조정해야 하며 단순히 timeout을 늘려 문제를 숨기지 않는다.

## 13. 캘리브레이션

상세 절차는 `docs/calibration.md`를 따른다.

offset 계산식:

```text
offset = direction * current_motor_position
```

캘리브레이션 중에는 `controller_manager`와 다른 CAN sender를 동시에 실행하지 않는다.

## 14. 실물 시험 확대 순서

1. CAN interface 상태 확인
2. feedback-only 단일 모터 시험
3. enable/disable 단일 모터 시험
4. 현재 위치 hold
5. 작은 각도 이동
6. timeout / safe stop 검증
7. 한 다리 3축
8. 두 CAN bus 부하 확인
9. 12축 hold
10. 초기 자세 transition
11. 지지대 위 gait
12. 실제 지면 보행
