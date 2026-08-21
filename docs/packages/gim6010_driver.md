# `gim6010_driver`

## 상태: 구현됨

`src/gim6010_driver/`는 라이브러리(`libgim6010_driver.so`)와 진단 CLI(`gim6010_diagnostic`)로 빌드된다. `colcon build --packages-select gim6010_driver`와 유닛 테스트(6절)로 검증했다. `quattro_hardware`가 이 라이브러리를 `find_package(gim6010_driver)`로 직접 링크한다(`docs/packages/quattro_hardware.md`).

## 역할

GIM6010-8 + GDS68 전용 저수준 C++17 드라이버(`ament_cmake`, `rclcpp` 등 ROS 의존성 없음). SocketCAN 송수신, CAN Simple 명령 전체 세트(축 상태 제어, Direct Position/Velocity/Torque, 리밋/게인, 텔레메트리, 에러 조회) + MIT motion control encode·decode, 여러 CAN bus에 걸친 다중 모터 라우팅을 책임진다.

**재사용성**: `MotorManager`는 CAN bus 이름 목록과 `{node_id, bus}` 라우팅 목록을 생성자 인자로 받을 뿐, Quattro의 12관절·`can0`/`can1` 구성을 코드에 하드코딩하지 않는다. Quattro joint 이름, URDF, IK/FK를 전혀 알지 못하므로 GDS68/GIM6010-8(또는 같은 CAN Simple 명령 집합을 쓰는 다른 ODrive 계열 드라이버)을 SocketCAN으로 구동하는 다른 프로젝트에서도 이 패키지를 그대로 `find_package(gim6010_driver)`로 가져다 쓸 수 있다.

## 0. 대상 하드웨어와 설계 근거

- SteadyWin GIM6010-8 × 12, GDS68 driver, Linux SocketCAN, 기본 bitrate `500000 bit/s`. 11-bit standard CAN frame, `arbitration_id = (node_id << 5) | cmd_id`, payload little-endian(단, MIT는 예외 — 2절).
- GIM6010-8 매뉴얼(rev2.2)을 직접 대조한 결과 **GDS68 펌웨어는 [ODrive](https://github.com/odriverobotics/ODrive) 계열 펌웨어를 기반**으로 한다 — CAN Simple 명령 표(`cmd_id` `0x01~0x1F`), heartbeat/오류 코드 구조, `odrivetool` 호환성이 ODrive와 동일한 형식이다(매뉴얼 3.1.2절, 4.1절). 매뉴얼 4.6절(SDK)은 ROS 2 연동으로 `odrive_can`(`docs/ros_odrive`와 동일 계열)을 명시적으로 지정한다.
- 이 사실에 근거해 실제로 SocketCAN 위에서 ODrive/Steadywin 계열 모터를 구동하는 두 오픈소스 프로젝트(`docs/ros_odrive/`, `docs/Steadywin-RS485-CAN-Connector/`)를 clone해 계층 분리와 `ros2_control` 통합 방식만 참고했다 — 코드는 그대로 가져오지 않는다.
  - **`docs/ros_odrive/`**: `odrive_ros2_control`은 별도 ROS 노드나 토픽을 거치지 않고 SocketCAN 래퍼와 메시지 encode/decode 소스를 하드웨어 플러그인에 직접 컴파일해 넣는다(두 프로세스가 소켓을 공유하지 않음) → 이 패키지를 `quattro_hardware`가 라이브러리로 직접 링크하는 구조의 근거. CAN 수신도 별도 스레드 없이 컨트롤러 업데이트 주기 안에서 non-blocking 소켓을 동기적으로 드레인한다 → `MotorManager::poll()`의 근거(3절). 반면 `odrive_node`가 필드별 byte offset을 수기로 반복 정의해 정의가 흩어지는 문제를 보인 것은 반면교사로 삼아, 필드 encode/decode를 명령별로 한 곳(`can_simple_messages.cpp`)에만 선언한다.
  - **`docs/Steadywin-RS485-CAN-Connector/`**: GDS68/GIM6010-8이라는 모델명이 없고 CAN 백엔드도 Windows 전용이라 코드는 재사용하지 않지만, `hal/`(전송)/`protocol/`(encode/decode)/`core/`(모터 상태) 3-계층 분리가 이 도메인에서 자연스러운 경계라는 근거로 삼았다(1~3절 구조와 동일). 참조 구현 저자가 MIT 모드의 TX arbitration ID(`0x400|addr` vs `0x500|addr`)를 스펙에서 확정 못해 추측으로 처리했다고 밝힌 점은 **채택하지 않는다** — GIM6010-8 매뉴얼 4.1.2절 `Mit_Control`(`cmd_id 0x08`)은 CAN Simple과 동일하게 `(node_id << 5) | cmd_id`로 arbitration ID를 만들며 MIT 여부와 무관하다.
- **구현 중 확정한 사항 하나**: 설계 단계에서는 `Get_Error(0x03)`가 카테고리별(모터/인코더/컨트롤러/시스템)로 응답 폭이 다르고 요청 큐로 짝지어야 한다고 추정했었다. 실제 구현은 ODrive CAN Simple의 표준 axis-level 형식(`active_errors`/`disarm_reason`, 각각 uint32, 요청 없이도 자기완결적으로 decode 가능)을 채택했다. 실기에서 이 가정과 다른 응답이 관찰되면 이 절과 `can_simple_messages.cpp`를 함께 수정한다.
- **실기 확인 결과: `Get_Error(0x03)`는 응답 자체가 오지 않는다(2026-08-21)**: 위 응답 형식 가정을 실기로 검증하려던 중, `can0`의 node 0/1/2에 대해 `Get_Error` 요청을 RTR 프레임(`cansend can0 003#R`)과 payload 없는 일반 프레임(`cansend can0 003#`) 양쪽으로 여러 차례 직접 보내고 `candump`로 확인했으나 **한 번도 응답 프레임이 관측되지 않았다** — 같은 방식(RTR)의 `Get_Encoder_Estimates(0x09)` 요청은 동일 조건에서 매번 정상 응답했고, `Heartbeat(0x01)`도 모든 12개 노드에서 요청 없이 100ms 주기로 정상 브로드캐스트됨을 함께 확인했다. 즉 문제는 응답 페이로드 형식이 아니라 **이 펌웨어가 `Get_Error` 자체를 지원하지 않거나 무시한다는 것**이다. `Get_Error`에 대한 응답을 기다려 값을 얻는 코드는(요청 큐 유무와 무관하게) 실기에서 영원히 타임아웃되므로 쓸 수 없다 — 축 오류 감지는 `Heartbeat`의 `axis_error` 필드(이미 요청 없이 주기적으로 도착함, 위 표)로 대체해야 한다. `MotorManager::request_get_error`/`Gim6010Motor::last_error` API 자체는 남겨두되(향후 펌웨어가 응답하게 되거나 `gim6010_diagnostic`처럼 참고용으로 찍어보는 용도), **활성화/런타임 안전 판정처럼 응답을 기다려야 하는 로직에서는 절대 의존하지 않는다** — `quattro_hardware`의 `on_activate`/`read()`와 `direct_position_tuning_gui`가 이 문제로 실제로 막혀 있었고 모두 `axis_error` 기반으로 수정했다(`docs/packages/quattro_hardware.md` 4절/5절).
- **모든 모터를 `Get_Encoder_Estimates(0x09)` 자동 브로드캐스트로 설정했다(2026-08-21) — 이제 이 드라이버를 쓰는 어떤 코드도 `0x09`를 요청하지 않는다**: 원래는 node 3/5/11만 요청 없이 `0x09`를 자발적으로 내보냈고(원인 미확정), 나머지 9개는 `Heartbeat(0x01)` 100ms 브로드캐스트만 있었다. `RxSdo`가 이 펌웨어에서 응답하지 않아 CAN으로는 이 설정을 바꿀 수 없었으나(아래 항목), 모터 설정 도구로 직접 각 모터에 자동 송신을 켜서 해결했다. 그에 따라 `QuattroSystem::read()`의 주기 요청(`feedback_request_period_ms`)과 "이미 fresh하면 요청 생략" 규칙, `on_activate`의 재요청 루프, `calibration_gui`/`direct_position_tuning_gui`의 요청 호출을 **전부 제거**했다 — 이제 모든 소비자는 `poll()`로 드레인하고 `has_fresh_feedback()`로 신선도만 판정한다.
  - **적용 상태는 `candump`로 반드시 확인하고 쓴다.** 2026-08-21 실측 기준 `0x09`를 브로드캐스트하는 노드는 **0/1/3(can0), 6/9/11(can1)** 6개뿐이고 주기는 **10ms(100Hz)**였다. node 2/7/8/10은 `Heartbeat`만 오고 `0x09`가 없으며, node 4/5는 `Heartbeat`조차 오지 않았다(버스에서 이탈). **브로드캐스트가 설정되지 않은 모터는 이제 아무도 대신 요청해 주지 않으므로 `read()`에서 곧바로 stale feedback으로 잡혀 safe stop된다** — 12개 전부에 설정이 들어갔는지 확인하는 것이 bringup의 전제 조건이다. 확인 명령:
    ```bash
    candump -L can0 > /tmp/b0.log & candump -L can1 > /tmp/b1.log &   # 3초쯤 뒤 종료
    # 노드별 0x09 도착 여부/주기를 세어 12개 전부 나오는지 본다
    ```
- **실기 확인 결과: `RxSdo/TxSdo(0x04/0x05)` 자체가 응답하지 않는다(2026-08-21)**: 위 자발적 브로드캐스트를 끌 방법을 찾으려고, node 3(자발적 브로드캐스트 중)과 node 0(안 함) 양쪽에 endpoint `0`~`1999`(node 3) / `0`~`299`(node 0) 전체를 `RxSdo` Read로 순서대로 요청했으나 **`TxSdo` 응답이 단 한 건도 오지 않았다**(직접 raw SocketCAN으로 요청/수신 구현, 같은 소켓으로 보낸 `Get_Encoder_Estimates` RTR 요청은 즉시 정상 응답해 소켓/코드 자체는 정상 동작 확인됨). Write opcode로도 시험 삼아 한 번 보내봤으나 마찬가지로 무응답이었다. 즉 `Get_Error`뿐 아니라 **RxSdo/TxSdo 메커니즘 전체가 이 펌웨어에서 구현되어 있지 않거나 무시된다** — 이름 붙지 않은 파라미터(극쌍수, 토크상수, `config.can.encoder_rate_ms` 같은 ODrive 표준 설정 등)에 CAN으로 접근할 방법이 이 펌웨어에는 없다는 뜻이다. `MotorManager::request_sdo_read`/`send_sdo_write` API 자체는 재사용성을 위해(다른 GDS68/ODrive 개체는 지원할 수도 있으므로) 남겨두되, 이 로봇의 12개 모터에는 쓸 수 없다고 간주한다.
- **Quattro 전용이 아니라는 요구사항**: 이 패키지는 Quattro가 실제로 쓰지 않는 명령도 포함한다 — 다른 프로젝트가 같은 GDS68/ODrive 계열 CAN Simple 드라이버를 이 패키지로 그대로 재사용할 수 있어야 하기 때문이다. `MotorManager`가 bus 이름과 라우팅을 생성자 인자로만 받고 12관절/`can0`·`can1` 구성을 코드에 하드코딩하지 않는 것도 같은 이유다(위 "재사용성" 참고).
  - Trap-trajectory 설정(`Set_Traj_Vel_Limit`/`Set_Traj_Accel_Limits`/`Set_Traj_Inertia`, `0x11`/`0x12`/`0x13`)과 `Disable_Can`(`0x1E`)까지 구현했다 — Quattro 자신은 이 중 어느 것도 쓰지 않는다(`quattro_bringup.md`가 trap-traj를 명시적으로 배제).
  - RxSdo/TxSdo(`0x04`/`0x05`) 범용 파라미터 read/write도 구현했다 — 극쌍수, 토크상수, 열 리밋처럼 이름 붙은 명령이 없는 파라미터에 접근하는 유일한 경로다. endpoint ID 표는 매뉴얼에 없고(펌웨어 버전별 JSON 파일 URL만 안내) 이 문서에도 하드코딩하지 않는다 — 호출자가 자신의 펌웨어 버전에 맞는 endpoint ID를 알아내 넘겨야 한다.
  - **의도적으로 넣지 않은 것**: `ControlMode::kVoltageControl`을 `Set_Controller_Mode`로 선택하는 것 자체는 가능하지만, 이 모드에 값을 직접 지시하는 CAN 명령(`Set_Input_Voltage`류)은 매뉴얼에서 확인되지 않아 구현하지 않았다. 모터를 직접 회전시킬 수 있는 명령을 추측으로 구현하지 않는다는 원칙(`AGENTS.md` 9번)에 따른 것이다 — 실기에서 이 경로를 확인하면 추가한다.

## 파일 구조

```text
src/gim6010_driver/
├── CMakeLists.txt
├── package.xml
├── include/gim6010_driver/
│   ├── can_frame.hpp             # CanFrame 값 타입(id/dlc/data/rtr, 비교 가능)
│   ├── byte_utils.hpp            # little-endian read_le<T>/write_le<T> (내부용)
│   ├── types.hpp                 # AxisState/ControlMode/InputMode/CanBusState/CanBusError/
│   │                              #   MotorRoute, arbitration ID 헬퍼(make/split)
│   ├── can_socket.hpp            # transport: SocketCAN RAII wrapper
│   ├── can_error.hpp             # CAN error frame 파싱 (warning/passive/bus-off)
│   ├── can_simple_messages.hpp   # CommandId + CAN Simple 전체 명령 encode/decode
│   ├── mit_protocol.hpp          # MIT 8-byte bit-packing encode/decode
│   ├── gim6010_motor.hpp         # 단일 모터 상태 추적(freshness 포함)
│   └── motor_manager.hpp         # 다중 bus/모터 소유, 라우팅, send_*/request_* API
├── src/
│   ├── can_socket.cpp
│   ├── can_error.cpp
│   ├── can_simple_messages.cpp
│   ├── mit_protocol.cpp
│   ├── gim6010_motor.cpp
│   ├── motor_manager.cpp
│   └── gim6010_diagnostic.cpp    # 실행 파일: 단일 모터 read-only 진단 CLI
└── test/
    ├── test_can_simple_messages.cpp  # 명령별 encode 바이트 배치, decode 값
    ├── test_mit_protocol.cpp         # MIT 경계값/중간값, 범위 밖 입력 거부
    ├── test_can_error.cpp            # error frame 파싱(warning/passive/bus-off 판정)
    └── test_motor_manager.cpp        # 생성자 검증, dispatch 라우팅(소켓 없이)
```

## 1. 전송 계층 — `can_socket.hpp`/`.cpp`

```cpp
class CanSocket {
public:
  explicit CanSocket(std::string interface_name);
  ~CanSocket();
  CanSocket(const CanSocket &) = delete;
  CanSocket(CanSocket &&) noexcept;              // move-only, RAII로 fd 소유권 이전

  bool open();                                    // PF_CAN/SOCK_RAW, CAN_RAW_ERR_FILTER, O_NONBLOCK, bind
  void close();
  bool is_open() const noexcept;

  bool send(const CanFrame & frame);              // 실패 시 false, 예외 없음
  std::optional<CanFrame> receive_nonblocking();  // EAGAIN까지 반복 드레인, error frame은 내부 상태로 소비
  std::optional<CanBusError> poll_error_frame();  // 마지막 error frame을 반환하고 소비

  CanBusState bus_state() const noexcept;
  const std::string & interface_name() const noexcept;
};
```

- non-blocking 소켓 하나, bus 이름(예: `can0`)마다 `MotorManager`가 하나씩 소유한다(개수 제한 없음).
- CAN 필터는 설정하지 않는다 — 인터페이스 전체를 받고 `MotorManager`가 arbitration ID로 라우팅한다(`docs/ros_odrive` 패턴). `CAN_RAW_ERR_FILTER`는 설정해 error frame을 같은 fd로 함께 받는다.
- `receive_nonblocking()`은 error frame(`CAN_ERR_FLAG`)을 만나면 `bus_state()`/`poll_error_frame()`용 내부 상태만 갱신하고 계속 드레인한다 — 데이터 프레임으로 반환하지 않는다.
- 송신 실패는 예외를 던지지 않고 반환값으로 알린다. `CanFrame`은 `{ uint32_t id; uint8_t dlc; std::array<uint8_t,8> data; bool rtr; }` 값 타입이다(`rtr`은 payload 없는 Get_* 요청을 표준에 맞게 Remote Transmission Request로 보내기 위한 필드).

## 2. 프로토콜 계층 — `can_simple_messages.hpp`/`mit_protocol.hpp`

모든 encode/decode 함수는 순수 함수(값 타입만 주고받고, 소켓·ROS 의존 없음)라 `test/`에서 실제 버스 없이 gtest로 검증한다.

`can_simple_messages.hpp`가 구현하는 명령(`CommandId` enum, `arbitration_id = (node_id << 5) | cmd_id`):

| 명령 | `cmd_id` | 방향 | 페이로드 |
|---|---|---|---|
| Heartbeat | `0x01` | motor→host, decode만 | axis_error(u32), axis_state(u8), flags(u8, bit0 motor/1 encoder/2 controller/3 system/7 traj_done), reserved, life(u8) |
| Estop | `0x02` | host→motor, encode만 | 없음 |
| Get_Error | `0x03` | 요청(RTR)+decode, **실기에서 응답 없음 확인(0절)** | active_errors(u32), disarm_reason(u32) |
| RxSdo | `0x04` | host→motor, encode만 | opcode(u8, 0=read/1=write), endpoint_id(u16), reserved, value(4byte raw) |
| TxSdo | `0x05` | motor→host, decode만(자기완결적, endpoint_id 포함) | reserved, endpoint_id(u16), reserved, value(4byte raw) |
| Set_Axis_Node_Id | `0x06` | host→motor, encode만 | new_node_id(u32) |
| Set_Axis_State | `0x07` | host→motor, encode만 | requested_state(u32) |
| Mit_Control | `0x08` | 별도(`mit_protocol.hpp`) | 아래 참고 |
| Get_Encoder_Estimates | `0x09` | 모터가 자동 브로드캐스트(요청 API는 남아있으나 런타임에서 안 씀) | pos_rev(f32), vel_rev_s(f32) |
| Get_Encoder_Count | `0x0A` | 요청(RTR)+decode, 진단 전용 | shadow_count(i32), count_in_cpr(i32) |
| Set_Controller_Mode | `0x0B` | host→motor, encode만 | control_mode(u32), input_mode(u32) |
| Set_Input_Pos | `0x0C` | host→motor, encode만 | pos_rev(f32), vel_ff(i16, ×0.001 rev/s), torque_ff(i16, ×0.001 N·m) |
| Set_Input_Vel | `0x0D` | host→motor, encode만 | vel_rev_s(f32), torque_ff_Nm(f32) |
| Set_Input_Torque | `0x0E` | host→motor, encode만 | torque_Nm(f32) |
| Set_Limits | `0x0F` | host→motor, encode만 | velocity_limit_rev_s(f32), current_limit_A(f32) |
| Set_Traj_Vel_Limit | `0x11` | host→motor, encode만 | traj_vel_limit_rev_s(f32) |
| Set_Traj_Accel_Limits | `0x12` | host→motor, encode만 | traj_accel_limit_rev_s2(f32), traj_decel_limit_rev_s2(f32) |
| Set_Traj_Inertia | `0x13` | host→motor, encode만 | traj_inertia(f32) |
| Get_Bus_Voltage_Current | `0x17` | 요청(RTR)+decode | bus_voltage_V(f32), bus_current_A(f32) |
| Clear_Errors | `0x18` | host→motor, encode만 | 없음 |
| Set_Pos_Gain | `0x1A` | host→motor, encode만 | pos_gain(f32) |
| Set_Vel_Gains | `0x1B` | host→motor, encode만 | vel_gain(f32), vel_integrator_gain(f32) |
| Get_Torques | `0x1C` | 요청(RTR)+decode | torque_target_Nm(f32), torque_estimate_Nm(f32) |
| Disable_Can | `0x1E` | host→motor, encode만 | 없음(대부분 펌웨어에서 소프트웨어로 되돌릴 수 없음 — 아래 경고) |
| Save_Configuration | `0x1F` | host→motor, encode만 | 없음 |

전부 little-endian(매뉴얼 4.1.1절). `Set_Input_Pos`의 `vel_ff`/`torque_ff`는 0.001 스케일 int16이라 대략 ±32.767 범위를 벗어나면 `encode_set_input_pos`가 `std::nullopt`를 반환한다(clamp하지 않는다). `Set_Pos_Gain`/`Set_Vel_Gains`는 Direct Position/Velocity 전용이며 MIT의 Kp/Kd(아래)와는 완전히 다른 타입(단위·의미 모두 다름)이므로 이름도 분리했다. `Set_Limits`의 velocity/current는 모터 rotor 단위(rev/s, A)이지 ROS joint 단위가 아니다. `Set_Traj_*`는 `Set_Controller_Mode`의 `InputMode::kTrapTraj`가 실제로 사용하는 shaping 파라미터다 — 이 세 명령을 호출하지 않고 `kTrapTraj`만 선택하면 장치에 이미 저장된 값으로 동작한다. `Disable_Can`은 호출 즉시 해당 축이 이후 CAN Simple 명령에 반응하지 않게 될 수 있고 펌웨어에 따라 전원 재인가 전까지 복구가 안 될 수 있다 — 실제로 이 상태를 의도할 때만 호출한다.

RxSdo/TxSdo는 명령별로 이름 붙지 않은 임의 파라미터(극쌍수, 토크상수, 열 리밋 등)에 접근하는 범용 경로다. `SdoValue`는 4바이트 raw 값이고 `make_sdo_value`/`sdo_value_as_*`(float/int32/uint32/uint8/bool)로 해석한다 — endpoint별 실제 타입은 호출자가 알아야 한다(매뉴얼은 고정된 표 대신 펌웨어 버전별 JSON URL만 안내한다). `TxSdo`는 응답에 `endpoint_id`를 그대로 포함해 자기완결적이므로(설계 단계에 우려했던 것과 달리) `Get_Error`처럼 요청 큐를 둘 필요가 없다.

### MIT(`0x08`) 정확한 비트 레이아웃 — `mit_protocol.hpp`

매뉴얼 4.1.2절 `Mit_Control` 원문 기준(출력축 rad/rad·s⁻¹/N·m). 호스트→모터 8바이트, MSB부터 채운다(CAN Simple의 little-endian 규칙과 다름):

| 필드 | 위치 | 비트폭 | 범위 | 변환식(값→정수) |
|---|---|---|---|---|
| Position (rad) | B0(상위8) + B1(하위8) | 16 | ±12.5 rad | `pos_int = (pos + 12.5) * 65535 / 25` |
| Velocity (rad/s) | B2(상위8) + B3[7:4] | 12 | ±65 rad/s | `vel_int = (vel + 65) * 4095 / 130` |
| Kp | B3[3:0](상위4) + B4(하위8) | 12 | [0, 500] | `kp_int = kp * 4095 / 500` |
| Kd | B5(상위8) + B6[7:4] | 12 | [0, 5] | `kd_int = kd * 4095 / 5` |
| Torque (N·m) | B6[3:0](상위4) + B7(하위8) | 12 | ±50 N·m | `t_int = (t + 50) * 4095 / 100` |

모터→호스트 feedback은 **다른(더 짧은) 레이아웃**이다 — Kp/Kd가 없으므로 position/velocity/torque만 6바이트에 담긴다. 명령 프레임 바이트를 그대로 재해석하면 안 된다(테스트에서 실제로 이 오류를 냈다가 고쳤다 — 6절):

| 필드 | 위치 | 변환식(정수→값) |
|---|---|---|
| Node ID | B0 | — |
| Position | B1 + B2 (16bit) | `pos = pos_int*25/65535 - 12.5` |
| Velocity | B3 + B4[7:4] (12bit) | `vel = vel_int*130/4095 - 65` |
| Torque | B4[3:0] + B5 (12bit) | `t = t_int*100/4095 - 50` |

이 6개 상수(`12.5`/`65`/`500`/`5`/`50`/`100`)는 GDS68 프로토콜 상수이며 모터별 튜닝값이 아니다 — `kMitPositionRangeRad` 등으로 코드에 하드코딩했다. `kp ∈ [0,500]`/`kd ∈ [0,5]` 범위는 `calibration_gui`가 관절 영점 조깅용 MIT hold gain을 검증할 때도 동일하게 쓴다(`docs/packages/quattro_hardware.md` 5절).

```cpp
struct MitCommand { double position_rad, velocity_rad_s, kp, kd, torque_Nm; };
struct MitFeedback { uint8_t node_id; double position_rad, velocity_rad_s, torque_Nm; };

std::optional<CanFrame> encode_mit_command(uint8_t node_id, const MitCommand & command);
// 범위를 벗어난 필드가 하나라도 있으면 clamp하지 않고 nullopt를 반환한다.
MitFeedback decode_mit_feedback(const CanFrame & frame);
```

`AGENTS.md` 9번 원칙에 따라 `Steadywin-RS485-CAN-Connector`의 `float_to_uint`처럼 범위를 넘는 값을 조용히 clamp하지 않는다 — 범위 밖 명령은 encode 단계에서 거부하고 호출자가 이를 fault로 취급해야 한다.

## 3. 모터 상태·라우팅 계층 — `gim6010_motor.hpp`/`motor_manager.hpp`

`Gim6010Motor`는 명령별 마지막 수신 값과 수신 시각(`std::chrono::steady_clock`)만 들고 있다. position/velocity feedback은 `Get_Encoder_Estimates` 프레임과 MIT 응답 둘 다 같은 freshness 시계(`has_fresh_feedback`)를 갱신한다 — 어느 쪽을 실제 feedback 소스로 쓸지는 호출자가 정한다: `quattro_hardware/QuattroSystem`과 `calibration_gui`는 Direct Position 경로이므로 모터가 브로드캐스트하는 `Get_Encoder_Estimates`를 읽는다. 시각은 **모터가 보낸 시각이 아니라 `dispatch()`가 프레임을 처리한 시각**이라, `poll()`을 오래 부르지 않으면 브로드캐스트가 계속 오고 있어도 feedback은 stale로 판정된다(`docs/packages/quattro_hardware.md` 2절 `on_activate` 4단계).

```cpp
class MotorManager {
public:
  // bus_interfaces: 예 {"can0", "can1"} (개수 제한 없음).
  // routes: {node_id, bus} 목록. 잘못된 정적 구성(중복 node_id, kMaxNodeId 초과,
  // routes가 가리키는 bus가 bus_interfaces에 없음, 빈 인터페이스 이름)은
  // std::invalid_argument를 던진다 -- 프로그래밍/설정 오류이지 런타임 I/O 실패가 아니기 때문.
  MotorManager(std::vector<std::string> bus_interfaces, std::vector<MotorRoute> routes);

  bool open();   // 모든 bus 오픈. 하나라도 실패하면 이미 연 것들도 닫고 false.
  void close();
  void poll();   // 모든 bus를 non-blocking 드레인, dispatch()로 라우팅. 스레드 없음.

  bool send_to(uint8_t node_id, const CanFrame & frame);
  bool send_estop(uint8_t node_id);
  bool send_set_axis_node_id(uint8_t node_id, uint8_t new_node_id);
  bool send_set_axis_state(uint8_t node_id, AxisState state);
  bool send_set_controller_mode(uint8_t node_id, ControlMode control_mode, InputMode input_mode);
  bool send_set_input_pos(uint8_t node_id, const SetInputPosCommand & command);
  bool send_set_input_vel(uint8_t node_id, float velocity_rev_s, float torque_ff_Nm);
  bool send_set_input_torque(uint8_t node_id, float torque_Nm);
  bool send_mit_command(uint8_t node_id, const MitCommand & command);
  bool send_set_limits(uint8_t node_id, float velocity_limit_rev_s, float current_limit_A);
  bool send_set_pos_gain(uint8_t node_id, float pos_gain);
  bool send_set_vel_gains(uint8_t node_id, float vel_gain, float vel_integrator_gain);
  bool send_clear_errors(uint8_t node_id);
  bool send_save_configuration(uint8_t node_id);
  bool send_set_traj_vel_limit(uint8_t node_id, float traj_vel_limit_rev_s);
  bool send_set_traj_accel_limits(uint8_t node_id, float accel_rev_s2, float decel_rev_s2);
  bool send_set_traj_inertia(uint8_t node_id, float traj_inertia);
  bool send_disable_can(uint8_t node_id);      // 대부분 펌웨어에서 되돌릴 수 없음
  bool send_sdo_write(uint8_t node_id, uint16_t endpoint_id, SdoValue value);

  bool request_get_error(uint8_t node_id);
  bool request_encoder_estimate(uint8_t node_id);
  bool request_encoder_count(uint8_t node_id);
  bool request_bus_voltage_current(uint8_t node_id);
  bool request_torques(uint8_t node_id);
  bool request_sdo_read(uint8_t node_id, uint16_t endpoint_id);  // 응답은 motor(node_id)->last_sdo_response()

  Gim6010Motor * motor(uint8_t node_id);          // 없으면 nullptr
  std::vector<uint8_t> node_ids() const;          // 정렬됨
  CanBusState bus_state(const std::string & bus) const;

  void dispatch(const CanFrame & frame);          // poll()이 내부적으로 호출; 테스트에서 소켓 없이 직접 호출 가능
};
```

- `poll()`은 `quattro_hardware::QuattroSystem::read()`가 매 제어 주기(100 Hz) 호출하는 것을 전제로 한다. 별도 CAN 수신 스레드를 두지 않는다(`docs/ros_odrive` 패턴, 0절) — `ros2_control`의 주기 스케줄링이 이미 폴링 주기를 보장한다.
- `dispatch()`는 arbitration ID의 node_id로 `motors_` 맵을 조회하고, cmd_id로 어떤 `decode_*`를 호출할지 정하는 단순 스위치문이다. 요청 큐나 응답 태그 매칭은 없다 — `Get_Error` 응답이 자기완결적 형식(0절)이기 때문이다.
- 한 버스에 여러 모터가 있어도 `MotorManager`가 순차적으로 명령을 보낸다(공유 mutex 불필요 — `read()`/`write()` 모두 같은 `ros2_control` 업데이트 스레드에서만 호출됨을 전제로 둔다. 다중 스레드에서 호출하는 다른 프로젝트는 자체적으로 직렬화해야 한다).

## 4. 진단 CLI — `gim6010_diagnostic`

`docs/packages/quattro_hardware.md`의 "단일 모터부터 확대하는 시험 순서" 2단계("mode/gain/limit을 바꾸지 않는 read-only 확인")를 위한 최소 실행 파일. `MotorManager`를 직접 사용하고 `ros2_control`/`rclcpp`에 의존하지 않는다.

```bash
./install/gim6010_driver/lib/gim6010_driver/gim6010_diagnostic --interface can0 --node-id 0
```

heartbeat, encoder estimate(`0x09`), bus voltage/current(`0x17`), 현재 error(`0x03`)를 200ms 주기로 요청하고 표준출력에 찍는다. `Set_Axis_State`/`Set_Input_*`/`Set_Controller_Mode` 등 모터를 움직이거나 모드를 바꾸는 명령은 전혀 보내지 않는다 — 배선·CAN ID·bus 매핑 확인 전용이다.

## 5. 에러 처리 원칙

- 프로토콜 encode 함수는 범위 밖 입력을 clamp하지 않고 실패(`std::nullopt`)를 반환한다(2절).
- `CanSocket::send`/`receive_nonblocking`과 `MotorManager`의 `send_*`/`request_*`는 예외를 던지지 않고 `bool`로 실패를 알린다 — `quattro_hardware::write()`가 실시간 제어 루프 안에서 호출될 것이기 때문이다. 예외는 생성자의 정적 설정 오류(중복 node_id 등, 3절)에만 쓴다.
- CAN bus 상태(`CanBusState`)는 `MotorManager::poll()` 이후 언제나 `bus_state(bus)`로 조회할 수 있다 — 호출자가 bus 자체의 error-passive/bus-off를 개별 모터 fault와 구분해서 진단하는 근거(`docs/packages/quattro_hardware.md` 안전 정책).

## 6. 테스트

`colcon test`(ament_lint_auto 경유)는 이 워크스페이스 Docker 이미지의 `ament_cmake_test` 파이썬 모듈 누락으로 현재 실행되지 않는다 — 이는 이미지 문제이지 이 패키지의 결함이 아니다. 대신 gtest 바이너리를 직접 실행해 확인했다.

```bash
cd /ws
colcon build --symlink-install --packages-select gim6010_driver
./build/gim6010_driver/test_can_simple_messages
./build/gim6010_driver/test_mit_protocol
./build/gim6010_driver/test_can_error
./build/gim6010_driver/test_motor_manager
```

36개 테스트 전부 통과(2026-08-20 기준, RxSdo/TxSdo·trajectory limit·Disable_Can 추가분 포함). 구현 중 실제로 하나 잡은 버그: MIT 커맨드 프레임(8바이트, Kp/Kd 포함)과 MIT 피드백 프레임(6바이트, Kp/Kd 없음)은 서로 다른 바이트 레이아웃인데, 초기 테스트가 커맨드 바이트를 그대로 피드백으로 재해석해 실패했다 — `decode_mit_feedback` 구현이 아니라 테스트의 프레임 구성이 잘못되어 있었다(2절 표 참고).

- `test_can_simple_messages.cpp`/`test_mit_protocol.cpp`: 실제 CAN 없이 encode 바이트 배치와 decode 값을 검증. MIT는 경계값(최댓값/최솟값)과 범위 밖 입력 거부를 확인.
- `test_can_error.cpp`: `linux/can/error.h` 상수로 만든 합성 error frame으로 warning/passive/bus-off 판정과 우선순위(bus-off가 최우선)를 검증.
- `test_motor_manager.cpp`: 실제 소켓을 열지 않고(`open()` 미호출) `dispatch()`를 직접 호출해 라우팅을 검증하고, 생성자 유효성 검사(중복 node_id, 알 수 없는 bus, node_id 초과, 빈 bus 목록)를 확인.
- `CanSocket` 자체(실제 SocketCAN 호출)는 unit test 대상이 아니다 — `vcan0` 가상 인터페이스를 이용한 통합 테스트는 필요 시 별도로 추가한다.

## 7. 빌드

`ament_cmake` 패키지지만 `rclcpp`에 의존하지 않는다. `gim6010_driver` 타깃은 `ament_export_targets`로 내보내므로 다른 ament_cmake 패키지는 `find_package(gim6010_driver REQUIRED)` 후 `target_link_libraries(... gim6010_driver)`로 링크할 수 있다(`quattro_hardware`가 이렇게 사용할 예정). 진단 CLI(`gim6010_diagnostic`)는 `lib/gim6010_driver/`에 설치된다.

```bash
cd /ws
colcon build --symlink-install --packages-select gim6010_driver --event-handlers console_direct+
```

## 관련 문서

- 이 드라이버를 사용하는 쪽(joint 변환, 안전 정책, CAN ID 매핑, MIT 조깅을 쓰는 `calibration_gui`): `docs/packages/quattro_hardware.md`
- 제조사 번역 매뉴얼: `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf`
- 참고 프로젝트: `docs/ros_odrive/`, `docs/Steadywin-RS485-CAN-Connector/`
