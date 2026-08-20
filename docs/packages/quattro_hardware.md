# `quattro_hardware`

## 상태: 구현됨

`src/quattro_hardware/`는 라이브러리(`libquattro_hardware.so`, pluginlib으로 `quattro_hardware/QuattroSystem` 노출)와 `calibration_gui` 실행 파일로 빌드된다. `gim6010_driver`를 실제로 링크한다. 소비 측 계약(파라미터 이름, command/state interface 목록)은 `quattro_description/urdf/quattro.urdf.xacro`와 `quattro_bringup`의 launch/config가 고정하고 있고(`docs/packages/quattro_description.md`, `docs/packages/quattro_bringup.md`), 이 구현이 그 계약을 그대로 만족한다. `docs/packages/gim6010_driver.md`(사용하는 드라이버)를 먼저 읽는다.

**미검증 범위**: `on_init`/`on_configure`와 pluginlib 로딩은 실제로 빌드·테스트했지만(6절), 실제 CAN bus·모터와의 `on_activate`/`read`/`write` 연동은 이 개발 환경에 `vcan`/실물 CAN이 없어 검증하지 못했다. 실기 검증 절차는 `docs/hardware_bringup.md`(6절, 단일 모터 시험)를 따른다.

## 역할

Quattro joint와 실제 GIM6010-8을 연결하는 `ros2_control` 계층(`ament_cmake`, C++). `hardware_interface::SystemInterface`를 구현하는 `QuattroSystem` 플러그인이 핵심이다. `gim6010_driver`를 라이브러리로 직접 링크해서 쓰며, joint별 방향/오프셋/한계 변환과 모터 활성화 시 안전 절차를 담당한다. raw CAN payload는 만들지 않는다(`gim6010_driver`의 책임).

이 워크스페이스의 `hardware_interface`는 2025년에 리팩터링된 `HardwareComponentInterface` API(`on_init(HardwareComponentInterfaceParams)`, `get_state<T>`/`set_state<T>`/`get_command<T>` 이름 기반 접근자)를 쓴다 — `export_state_interfaces()`/`export_command_interfaces()`를 직접 구현하는 옛 방식(예: `docs/ros_odrive`의 `odrive_hardware_interface`)과 다르다. URDF에 선언된 interface가 그대로 자동 export되므로 이 구현은 둘 다 오버라이드하지 않는다.

## 파일 구조

```text
src/quattro_hardware/
├── CMakeLists.txt
├── package.xml
├── quattro_hardware.xml              # pluginlib 플러그인 설명 (hardware_interface::SystemInterface)
├── include/quattro_hardware/
│   ├── joint_transform.hpp           # direction/offset/gear_ratio 변환(+ calibration_gui가 쓰는 MIT 출력축 변환), 순수 함수
│   └── quattro_system.hpp
├── src/
│   ├── joint_transform.cpp
│   ├── quattro_system.cpp
│   └── calibration_gui.cpp           # 실행 파일: 관절 영점 캘리브레이션 GUI (Qt5 + yaml-cpp)
└── test/
    ├── test_joint_transform.cpp      # 변환 round-trip, 도메인 경계값 검증
    └── test_pluginlib_export.cpp     # pluginlib이 실제로 QuattroSystem을 로드하는지 검증
```

## 0. 인코더 물리적 구성(상충하는 증거)과 CAN ID 매핑

### 매뉴얼 근거: 인코더 칩은 하나뿐인 것으로 보인다

GIM6010-8 매뉴얼(rev2.2, 96페이지 전체)을 직접 확인한 결과:

- BOM(2.5절): 인코더 칩은 `MA732, 14비트 절대값` 1개뿐이다.
- CAN Simple 전체 명령 목록(4.1.2절)에 "두 번째 인코더 값을 읽는" 명령이 없다. 인코더 관련 명령은 `Get_Encoder_Estimates(0x009)`, `Get_Encoder_Count(0x00A)`, `Set_Linear_Count(0x019)` 셋뿐이며 모두 같은 encoder 객체를 가리키는 것처럼 문서화되어 있다.
- 오류 카테고리에 `SEC_ENC_COM_FAIL` 플래그가 존재한다(오류 코드 표) — 아래 반박 증거 이전에는 ODrive 계열 오류 enum에서 물려받은 미사용 값으로 잠정 판단했었다.
- 하드웨어 인터페이스 절(2.4절)에는 전원/CAN/RS485/PWM, Type-C 디버그, 확장 슬롯(v3.9부터 미지원), 브레이크 저항, 리미트 스위치만 있고 별도 인코더 입력 핀이 없다.

### 반박 증거: 실제 분해 사진은 출력축 쪽 2차 encoder 보드를 보여준다

제조사 문서는 아니지만, GIM6010-8을 실제로 분해한 제3자 블로그(varofla, ["바퀴 달린 두 다리 로봇" 2편](https://varofla.com/blog/2-wheel-leg-robot-2/))가 다음을 사진과 함께 보고한다:

- 메인 드라이버 보드 위 uC 옆 정사각형 칩은 로터(모터측) 회전을 측정하는 마그네틱 인코더로 보인다(매뉴얼의 `MA732`와 위치가 일치).
- **별도의 초록색 기판**이 하나 더 있고, 그 위 "출력축(기어를 거친 뒤) 절대 위치를 알기 위한" 자석 + 3핀 IC 4개(홀 센서로 추정) + serializer IC 구성이 확인된다 — 로터측 인코더와 물리적으로 분리된 보드다.
- 저자는 "전원을 껐다 켜도 같은 회전수를 유지하는 것을 보아 샤프트 엔코더가 잘 작동하는 것 같다"고 실측으로 보고한다(정확한 유지 메커니즘·CAN 읽기 명령은 명시하지 않음).

이 보고가 정확하다면 `SEC_ENC_COM_FAIL`은 미사용 값이 아니라 실제 하드웨어를 가리키는 플래그일 가능성이 높고, 이 2차 encoder가 출력축을 직접(기어비 45° wrap 없이) 측정한다면 아래 멀티턴 모호성 문제 자체가 없을 수 있다.

### 결론: 확정하지 않는다 — 실기에서 검증한다

매뉴얼(공식 문서, 그러나 이 특정 기판을 명시적으로 다루지 않음)과 제3자 분해 보고(비공식이지만 실물 사진 근거)가 서로 다른 결론을 가리킨다. 어느 쪽도 다음을 확정하지 못한다: 이 2차 encoder를 CAN으로 읽는 명령이 무엇인지, `Get_Encoder_Estimates(0x009)`/`Get_Encoder_Count(0x00A)`가 로터측·출력축측 중 어느 쪽 값을 반환하는지, 유지 메커니즘이 비휘발성 저장인지 단순 배터리/커패시터 백업인지. **이 설계는 이 질문을 문서만으로 풀지 않는다** — 6절 단일 모터 시험 절차 3단계에서 실측으로 확인하고, 그 결과에 따라 `on_activate`의 "활성화 직전 실측 위치를 target으로 사용" 정책(아래)을 유지할지 저장된 절대 위치를 신뢰해 단순화할지 결정한다. 결과가 나오기 전까지는 아래 보수적 정책을 기본값으로 유지한다.

이 encoder(들)는 14-bit 단위 기준이라, 전원 인가 즉시 로터 1회전 이내 절대각은 바로 얻는다. 로터측 인코더만 있고 2차 encoder가 없다고 가정할 경우의 문제는 기어비 8:1이다 — 로터 절대각 하나로 알 수 있는 출력축 위치는 `360°/8 = 45°` 간격으로 반복(wrap)된다. `quattro.urdf.xacro`의 실제 관절 가동범위(hip 약 119°, upper/lower_leg 약 239~256°)는 모두 45°보다 훨씬 넓으므로, 이 최악의 경우 재부팅 직후 로터 절대각만으로는 관절이 가동범위 내 몇 번째 45° 구간에 있는지 원리적으로 구분되지 않는다.

**이 설계는 "전원 재인가 후 관절의 정확한 절대 위치를 CAN만으로 자동 검증"됨을 전제하지 않는다.** 위 실기 검증 전까지는 최악의 경우(로터측 단일 encoder뿐)를 가정하고 `on_activate`(2절)에서:

- 활성화 전 fresh feedback(`0x009` 또는 `0x008`)을 반드시 확인하고, 확인되지 않으면 startup을 실패시킨다.
- 활성화 직후 명령 target은 항상 그 순간 읽은 실제 위치로 초기화한다 — "45° 중 어느 구간인지"가 틀리더라도 활성화 순간 급격한 이동은 만들지 않는다.
- 실기 절차(6절 단일 모터 시험)는 낮은 gain부터 확대해, 예상과 다른 45° 구간에서 재부팅됐을 경우 초기 단계에서 드러나게 한다.
- `calibration.yaml`의 `offset`은 "encoder zero 대비 ROS joint zero의 위치"만 정의할 뿐 이 멀티턴 모호성을 해소하지 않는다.

미해결 상태이며 실기 검증 항목은 `docs/development_status.md`에 기록한다.

### CAN ID와 bus 매핑

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

기본 direction(`calibration.yaml.example`): `[-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]`(위 표 순서). 실제 offset은 머신별 `src/quattro_bringup/config/calibration.yaml`에서 관리하며 `src/quattro_description/config/calibration.yaml`(Xacro 검사용 기본값)과는 다른 파일이다. `calibration_gui`(5절)는 저장 시 이 매핑과 다른 `can_interface`/`can_id`/`direction`을 거부한다.

## 1. `joint_transform.hpp` — 좌표 변환

순수 함수(하드웨어·ROS 의존 없음, `gim6010_driver`도 몰라도 됨 — `double`만 주고받는다):

```cpp
struct JointCalibration { double direction; double offset; double gear_ratio; };  // gear_ratio 기본 8.0

double motor_rev_to_joint_rad(double motor_rev, const JointCalibration &);
// joint_rad = direction * (motor_rev * 2π / gear_ratio) - offset
double joint_rad_to_motor_rev(double joint_rad, const JointCalibration &);
// motor_rev = direction * (joint_rad + offset) * gear_ratio / (2π)

double motor_rev_s_to_joint_rad_s(double motor_rev_s, const JointCalibration &);
double joint_rad_s_to_motor_rev_s(double joint_rad_s, const JointCalibration &);

double motor_Nm_to_joint_Nm(double motor_Nm, const JointCalibration &);   // joint_Nm = direction * motor_Nm * gear_ratio
double joint_Nm_to_motor_Nm(double joint_Nm, const JointCalibration &);   // motor_Nm = direction * joint_Nm / gear_ratio

// MIT(0x08)는 이미 출력축(gear_ratio 반영 완료) 값이라 이 세 쌍은 gear_ratio를 쓰지
// 않는다 -- direction/offset만 적용한다. QuattroSystem은 Direct Position만
// 쓰므로 이 쌍은 호출하지 않고, calibration_gui의 관절 영점 조깅 절차(5절)가
// gim6010_driver의 MIT 프레임을 통해 사용한다.
double mit_output_rad_to_joint_rad(double mit_output_rad, const JointCalibration &);
double joint_rad_to_mit_output_rad(double joint_rad, const JointCalibration &);
double mit_output_rad_s_to_joint_rad_s(double mit_output_rad_s, const JointCalibration &);
double joint_rad_s_to_mit_output_rad_s(double joint_rad_s, const JointCalibration &);
double mit_output_Nm_to_joint_Nm(double mit_output_Nm, const JointCalibration &);
double joint_Nm_to_mit_output_Nm(double joint_Nm, const JointCalibration &);
```

`Set_Limits`의 velocity/current는 모터 rotor 단위(rev/s, A)이므로 `rotor_velocity_limit_rev_s`/`motor_current_limit_a` 파라미터는 변환 없이 그대로 `gim6010_driver`에 전달한다(joint 단위가 아니다).

## 2. `QuattroSystem` — `hardware_interface::SystemInterface`

### `on_init(const hardware_interface::HardwareComponentInterfaceParams & params)`

이 워크스페이스의 `hardware_interface`(2025년 리팩터링된 `HardwareComponentInterface` API)는 `on_init`이 `HardwareInfo`가 아니라 `HardwareComponentInterfaceParams`(내부에 `hardware_info` 포함)를 받는다. 반드시 `SystemInterface::on_init(params)`(기반 클래스, `info_` 멤버를 채운다)를 먼저 호출한 뒤 자체 파싱을 진행한다.

- `info_.hardware_parameters`에서 하드웨어 전역 파라미터를 읽는다: `apply_position_gains`, `position_gain`/`velocity_gain`/`velocity_integrator_gain`, `feedback_timeout_ms`, `feedback_request_period_ms`, `heartbeat_timeout_ms`, `startup_timeout_ms`, `motor_activation_interval_ms`, `command_timeout_ms`, `scheduling_warning_ms`, `rotor_velocity_limit_rev_s`, `motor_current_limit_a`, `telemetry_period_ms`(전체 목록과 값은 `quattro.urdf.xacro`의 `<ros2_control name="QuattroSystem">` 블록, `docs/packages/quattro_description.md`). 파싱 실패는 각각 로그를 남기고 `on_init`을 실패시킨다(누락된 키, 숫자로 파싱 안 되는 값, 음수 timeout 등).
- `info_.joints`를 순회하며 관절마다 `can_interface`/`can_id`/`direction`/`offset`/`gear_ratio`/`current_limit`을 읽어 `JointCalibration`과 `MotorRoute`(node_id=`can_id`, bus=`can_interface`)를 구성한다. `direction`은 정확히 `1.0`/`-1.0`만 허용, `can_id`는 `[0, kMaxNodeId]`(`gim6010_driver`), 중복 `can_id`는 거부한다.
- 관절당 command interface는 `position` 1개, state interface는 `position`/`velocity`/`effort` 3개를 기대한다. `info_.joints[i]`의 실제 개수/이름이 이와 다르면 `on_init`을 실패시킨다(URDF-하드웨어 파라미터 불일치를 조용히 넘기지 않는다).
- **현재 구현은 joint 개수를 정확히 12개로, 또는 `can_id`/`direction`을 0절의 기준 매핑과 일치하도록 강제하지 않는다**(최소 1개 이상만 요구) — 그 엄격한 검증은 `calibration_gui`(5절)에서만 한다. `QuattroSystem` 자체는 구조적 유효성(중복 없음, 범위 안, interface 계약 일치)만 검증한다.

`on_export_state_interfaces()`/`on_export_command_interfaces()`는 오버라이드하지 않는다 — URDF에 선언된 state/command interface가 기본 구현으로 자동 export되고, `read()`/`write()`는 `set_state<double>("<joint>/<interface>", value)`/`get_command<double>("<joint>/<interface>")` 이름 기반 접근자를 그때그때 호출한다(위 "역할" 절 참고). `docs/ros_odrive`의 `odrive_hardware_interface`(구버전 API, 포인터 기반 `export_state_interfaces()` 직접 구현)와는 이 지점에서 구조가 다르다.

### `on_configure`

- 각 joint의 `can_interface`에서 고유한 bus 이름 집합을 뽑아 `gim6010_driver::MotorManager`를 생성하고 연다(bus 이름은 코드에 하드코딩하지 않는다 — 몇 개든, 이름이 무엇이든 동작). 실패하면 `CallbackReturn::ERROR`.
- `apply_position_gains`가 `true`면 `Set_Pos_Gain`/`Set_Vel_Gains`를 전송한다. 기본(`false`)은 장치 값을 보존한다. 매뉴얼 예시값(`20.0/0.16/0.32`)은 튜닝 절차의 예시일 뿐 factory default라는 근거가 없다.
- `Set_Limits(rotor_velocity_limit_rev_s, motor_current_limit_a)`를 모든 모터에 전송한다.
- **`Clear_Errors`를 자동으로 호출하지 않는다**(4절) — 기존 fault가 있으면 이후 `on_activate`가 거부해야 한다.

### `on_activate` — 순차 활성화

1. 전체 모터에 대해 `Get_Encoder_Estimates(0x09)`로 fresh feedback을 먼저 확인한다. `startup_timeout_ms` 안에 전체 모터가 응답하지 않으면 `CallbackReturn::ERROR`. 이어서 `Get_Error`로 기존 fault 여부를 확인하고, 하나라도 fault가 있으면(또는 응답이 없으면) 활성화를 거부한다.
2. `info_.joints` 순서(= calibration.yaml에 나열된 순서)대로 모터를 하나씩 활성화한다:
   - 그 순간 실제로 읽은 위치를 target으로 준비(급격한 이동 방지, 0절 — 인코더 45° 모호성 때문에 임의 위치로 시작하지 않는다).
   - `Set_Axis_State(closed-loop)` 후 그 위치로 즉시 hold.
   - `motor_activation_interval_ms` 동안 안정 상태(fault 없음, feedback 정상)를 확인한 뒤 다음 모터로 진행.
3. 어느 모터든 활성화 실패(feedback 없음, fault)가 나오면, **이미 활성화된 모터를 포함해 전체를 safe stop으로 되돌린다**(부분 활성화 상태로 남기지 않는다).

`startup_timeout_ms`는 1단계(전체 fresh feedback + fault 확인)만 제한한다 — 12관절 전체를 `motor_activation_interval_ms`(기본 100ms)씩 순차 활성화하는 2단계는 관절 수에 비례해 수 초가 걸릴 수 있고 이를 정상 동작으로 본다(`hardware_spawner`의 `--controller-manager-timeout 30`이 이를 감안한 값이다, `docs/packages/quattro_bringup.md`). `startup_timeout_ms`를 전체 활성화 절차에 적용하면 관절 수가 늘어날 때 항상 실패하므로 그렇게 하지 않는다.

### `on_deactivate`

모든 모터를 idle(`Set_Axis_State(1)`)로 전환하고 소켓을 유지한 채 대기(재활성화 가능 상태로 남긴다). `on_cleanup`에서만 소켓을 닫는다.

### `read()`

1. `MotorManager::poll()`로 두 버스를 non-blocking 드레인한다(추가 스레드 없음 — `docs/packages/gim6010_driver.md` 0/3절).
2. 각 모터의 최신 feedback(`0x09` 폴링)을 `motor_rev_to_joint_rad`/`motor_rev_s_to_joint_rad_s`로 joint 단위(rad, rad/s)로 변환해 state 버퍼에 쓴다.
3. `feedback_timeout_ms`/`heartbeat_timeout_ms`를 넘긴 모터가 있으면 stale로 표시하고 안전 정책(4절)을 트리거한다.
4. Direct Position은 실측 토크 경로가 없으므로 effort state는 `NaN`을 반환한다 — 임의 값을 대신 채우지 않는다.

### `write()`

1. `active_`가 아니면(비활성 상태) 아무 것도 보내지 않고 즉시 반환한다.
2. `joint_rad_to_motor_rev`로 motor 단위로 변환한 뒤 `gim6010_driver::MotorManager::send_set_input_pos`를 호출한다.
3. encode가 범위 초과로 실패(`std::nullopt`, `docs/packages/gim6010_driver.md` 2절)를 반환하면 그 관절 명령을 거부하고 전체를 safe stop한다 — clamp해서 대신 보내지 않는다(`AGENTS.md` 9번 원칙).
4. 매 호출 시작 시 `last_write_time_`을 현재 시각으로 갱신한다.

**command watchdog은 `write()`가 아니라 `read()`에서 판정한다.** 컨트롤러가 매 주기 값을 다시 쓰는지("새 값" 여부)를 비교하는 방식은 채택하지 않았다 — 목표에 도달한 뒤에도 완전히 동일한 값으로 계속 hold command를 쓰는 정상적인 정지 상태를 "명령이 끊겼다"고 오판할 수 있기 때문이다. 대신 `read()`가 `now - last_write_time_ > command_timeout_ms`를 확인한다 — `write()`가 실제로 호출되지 않는 상황(예: `controller_manager` write 루프 자체가 멈춤)만 잡아낸다.

## 3. 제어 방식과 단위 변환

| GDS68 mode(`Set_Controller_Mode`) | command interface | 명령 단위 변환(`joint_transform.hpp`) | feedback 소스 |
|---|---|---|---|
| control 3, input 1 (Direct Position) | `position` (rad) | `joint_rad_to_motor_rev`: `motor_rev = direction * (joint_rad+offset) * gear_ratio / (2π)` | `0x09` 폴링, `motor_rev_to_joint_rad`로 역변환 |

`joint_trajectory_controller`가 관절당 `position` command interface 1개를 claim하면 활성화된다(`docs/packages/quattro_bringup.md`).

## 4. 활성화 조건 요약표 / 안전 정책

| 조건 | 감지 위치 | 결과 |
|---|---|---|
| command watchdog(`command_timeout_ms`) | `write()` | 해당 관절 hold 또는 전체 safe stop(정책은 구현 시 확정) |
| feedback stale(`feedback_timeout_ms`) | `read()` | fault 상태, safe stop |
| heartbeat stale(`heartbeat_timeout_ms`) | `read()` | fault 상태, safe stop |
| axis/controller/system fault(`Get_Error`) | `read()` | safe stop, 원인 로그 |
| CAN bus error-passive/bus-off | `read()`(`gim6010_driver::CanBusState`) | safe stop, bus 단위로 원인 구분 |
| encode 범위 초과 명령 | `write()` | 해당 관절만 거부 + fault 보고(clamp 금지) |

원칙:

- **startup에서 자동으로 `Clear_Errors`를 호출하지 않는다.** 기존 fault가 있으면 활성화를 거부해 원인 증거를 보존한다. 오류 삭제는 원인을 확인한 뒤 별도의 명시적 절차에서만 수행한다.
- **일시적 CAN frame 누락과 실제 GDS68 fault를 구분해서 진단한다.** 단발성 프레임 손실만으로 즉시 fault 처리하면 실제 하드웨어 이상과 정상적인 버스 지연을 혼동한다 — 반대로 무한정 재시도하면 실제 이상을 놓친다. "일시적 지연 경고"와 "지속 장애로 인한 safe stop"을 서로 다른 임계값으로 나눈다(7절).
- **한 모터의 문제로 전체를 정지시키는 정책은 안전상 정당화될 수 있지만, 어떤 조건이 전체 safe stop을 유발했는지 반드시 로그/diagnostics에 남긴다.**

## 5. `calibration_gui`

`docs/calibration.md`에 이미 기술된 두 활성화 모드(선택 모터만 활성화 / 12개 전체 hold)를 그대로 구현한다. `ros2_control`/`controller_manager`를 거치지 않고 `gim6010_driver::MotorManager`를 직접 사용하는 독립 실행 파일이다(캘리브레이션은 `controller_manager`와 동시에 실행하지 않음 — `docs/calibration.md` 안전 조건).

- X11 GUI(Qt 계열, `docs/development_environment.md`의 X11 forwarding 절차로 원격 실행).
- 활성화 시 GDS68 rotor velocity/current limit을 캘리브레이션 전용 보수적 값으로 설정한다(최종 보행용 gain/limit과 무관 — `docs/calibration.md`).
- 저장 시 화면 target이 아니라 최신 encoder feedback을 다시 읽어 `offset = direction * current_motor_position`을 계산한다.
- 저장 대상 `calibration.yaml`의 `can_interface`/`can_id`/`direction`이 기준 매핑(0절)과 다르면 저장을 거부한다.
- enable 전 fault를 자동으로 지우지 않는다(`quattro_hardware`의 나머지 코드와 동일한 원칙).

## 6. 단일 모터부터 확대하는 시험 순서

자동 movement 도구를 전제하지 않고, 매 단계 결과를 사람이 확인한다. 12축 시험으로 범위를 넓히기 전에 이 순서를 항상 거친다.

1. `ip -details -statistics link show can0`으로 버스 상태 확인.
2. 대상 모터 하나만 연결한 상태에서 heartbeat/encoder feedback이 들어오는지 확인 — mode/gain/limit을 바꾸지 않는 read-only 확인부터, `gim6010_driver::gim6010_diagnostic` CLI(`docs/packages/gim6010_driver.md` 4절) 사용.
3. **인코더 전원 유지 특성 확인(0절 상충 증거 검증)**: 모터를 활성화하지 않은 상태에서 출력축을 손으로 여러 바퀴(기어 통과 후 360° 이상) 돌리고 `0x009`/`0x00A` 값을 기록한 뒤, 전원을 완전히 껐다 켜고 같은 값을 다시 읽는다. 값이 유지되면 그 범위(±1회전/±수회전/무제한)를 반복 확인한다. 이 결과를 문서(0절)에 기록하고 이후 단계의 "45° 모호성" 가정 여부를 갱신한다.
4. Direct Position mode와 검증된 낮은 current/velocity limit을 설정.
5. 현재 output position을 읽어 같은 위치를 target으로 준비.
6. closed-loop 진입 후 현재 위치 hold가 유지되는지 확인.
7. 사용자가 명시한 작은 각도만 이동시켜 방향·부호를 확인.
8. feedback 방향, current, fault를 확인.
9. idle/disable.

3단계에서 인코더가 멀티턴을 유지하지 않는 것으로 확인되면(또는 확인 전까지는 보수적으로), 전원 재인가 직후에는 단일회전 절대각만 즉시 확인되고 관절이 가동범위 내 어느 45° 구간에 있는지는 자동으로 검증되지 않으므로, 재부팅 후 첫 시험에서는 반드시 관절의 실제 육안 위치와 대조한다. 단일 모터 hold와 작은 step이 검증되기 전에는 12축 시험으로 넘어가지 않는다. 전체 bringup 실행 절차(`hardware.launch.py`)는 `docs/packages/quattro_bringup.md`를 따른다.

## 7. Watchdog 값 설계 원칙

정확한 timeout 수치를 이 문서에서 미리 확정하지 않는다. 대신 각 watchdog을 정할 때 지켜야 할 관계만 명시한다.

- feedback fault 임계값은 feedback을 요청하는 주기보다 충분히 커야 하되, 상위 `ros2_control` 컨트롤러의 `update_rate`(현재 100 Hz)에서 발생하는 정상적인 스케줄링 지연(Linux/Docker jitter)보다는 훨씬 커야 한다.
- heartbeat fault 임계값은 GDS68의 기본 heartbeat 주기(매뉴얼 4.1.5절, 기본 100 ms)의 배수로 잡아 단일 프레임 손실을 오탐하지 않게 한다.
- controller scheduling 지연은 "경고"와 "fault"를 별도 임계값으로 나눠, 일시적 지연과 지속적 문제를 구분한다.
- 위 세 종류(feedback, heartbeat, scheduling)는 서로 다른 원인을 가리키므로 하나의 timeout으로 합치지 않는다.

Raspberry Pi 5 + Docker의 실제 jitter를 측정한 뒤 구체적 ms 값을 정하고, 이 문서에 실측 근거와 함께 기록한다. 근거 없이 timeout을 늘려 문제를 숨기지 않는다.

## 8. 테스트 전략

- `test_joint_transform.cpp`: 변환 함수의 round-trip과 부호(특히 토크의 감속비 방향)를 실제 하드웨어 없이 검증.
- `QuattroSystem`의 `on_init`/interface export/watchdog 판정 로직은 `gim6010_driver::MotorManager`를 mock/fake로 대체해 단위 테스트할 수 있도록 `MotorManager`에 대한 의존을 인터페이스 뒤에 두는 것을 권장한다(구현 시 결정).
- 실제 모터 활성화(`on_activate`)는 unit test 대상이 아니다 — 6절의 단일 모터 시험 절차로 실기 검증한다.

## 관련 문서

- 사용하는 드라이버(CAN Simple/MIT 프로토콜 상세): `docs/packages/gim6010_driver.md`
- 인터페이스 계약(Xacro 파라미터): `docs/packages/quattro_description.md`
- 실행 절차: `docs/packages/quattro_bringup.md`, `docs/calibration.md`
- 실기 검증 미해결 항목: `docs/development_status.md`
