# `gim6010_driver`

## 역할

`gim6010_driver`는 GIM6010-8 / GDS68용 저수준 C++ 드라이버다. ROS 2와 Quattro 구조를 알지 않고 다음만 담당한다.

- Linux SocketCAN 송수신
- CAN Simple encode/decode
- MIT protocol encode/decode
- 다중 CAN bus / 다중 node routing
- 모터별 최신 Heartbeat/Encoder/telemetry 상태 캐시
- CAN error frame 해석

Quattro joint 이름, URDF, gait, ros2_control lifecycle은 이 패키지의 책임이 아니다.

## 1. CAN 기본 규칙

- bitrate: `500000 bit/s`
- standard 11-bit CAN ID
- arbitration ID:

```text
(node_id << 5) | cmd_id
```

Quattro runtime에서 중요한 CAN Simple command:

| cmd_id | 이름 | 용도 |
|---:|---|---|
| `0x001` | Heartbeat | axis state/error 자동 수신 |
| `0x007` | Set_Axis_State | Idle / Closed Loop |
| `0x009` | Get_Encoder_Estimates | position/velocity feedback |
| `0x00B` | Set_Controller_Mode | Position + Pos Filter 설정 |
| `0x00C` | Set_Input_Pos | 정상 position command |
| `0x00F` | Set_Limits | velocity/current limit |
| `0x01A` | Set_Pos_Gain | position gain |
| `0x01B` | Set_Vel_Gains | velocity + integrator gain |

## 2. Quattro에서 사용하는 control mode

실기 runtime은 다음 조합을 사용한다.

```text
ControlMode::kPositionControl
InputMode::kPosFilter
```

`kDirect`는 Quattro 실기 startup/runtime의 기본 input mode가 아니다.

## 3. EncoderEstimate 운용 계약

Quattro에 연결된 모터는 `Get_Encoder_Estimates(0x009)`를 약 10 ms 주기로 자동 송신하도록 운용한다.

따라서 Quattro runtime, `quattro_hardware`, bringup, calibration/tuning GUI는 정상 경로에서 encoder request를 보내지 않고 `MotorManager::poll()`로 들어오는 frame을 수신한다.

`MotorManager::request_encoder_estimate()` API는 드라이버 재사용성과 진단 목적으로 남겨둘 수 있지만 Quattro 정상 제어 경로에서 사용하지 않는다.

## 4. Closed Loop 이전 encoder 주의

실기에서 중요한 제약이 확인되었다.

> Closed Loop Control에 들어가기 전 EncoderEstimate 위치값은 startup 위치로 신뢰할 수 없다.

따라서 이 드라이버는 최신 encoder를 캐시하더라도 **그 값의 사용 가능 여부를 결정하지 않는다.** Closed Loop 전/후의 의미 판단은 `quattro_hardware` 같은 상위 lifecycle 계층의 책임이다.

상위 계층이 새 frame 여부를 확실하게 판정할 수 있도록 `Gim6010Motor`에 encoder sequence/generation counter를 제공하는 것을 권장한다.

예:

```cpp
std::uint64_t encoder_sequence() const noexcept;
```

`on_encoder_estimate()`가 호출될 때마다 증가한다.

## 5. Heartbeat

Heartbeat는 startup과 runtime 안전 판정의 핵심 feedback이다.

상위 계층은 최소 다음을 사용한다.

```text
axis_state
axis_error
```

Quattro 실기에서는 `Get_Error(0x03)` 요청에 응답이 없는 현상이 확인되었으므로 startup/fault safety는 `Heartbeat.axis_error`를 사용한다.

`request_get_error()` API는 범용 드라이버 기능으로 남겨도 되지만 Quattro의 안전 경로에서 응답을 기다리지 않는다.

## 6. MotorManager

`MotorManager`는:

- bus별 `CanSocket` 소유
- `{node_id, bus}` route 관리
- `poll()`을 통해 non-blocking receive drain
- 수신 frame을 각 `Gim6010Motor`로 dispatch
- 각 command 전송 함수 제공

을 담당한다.

background thread를 만들지 않고 `quattro_hardware::read()` 또는 GUI timer가 `poll()`을 호출한다.

## 7. GIM6010 startup에서 필요한 send API

`quattro_hardware`가 다음 순서로 사용할 수 있어야 한다.

```text
send_set_limits()
send_set_pos_gain()
send_set_vel_gains()
send_set_controller_mode(PositionControl, PosFilter)
send_set_axis_state(ClosedLoopControl)
```

startup에서는 현재 위치 hold를 만들 목적으로 `send_set_input_pos(current)`를 사용하지 않는다.

정상 controller command가 시작된 뒤에만 `send_set_input_pos()`가 position setpoint 전송에 사용된다.

## 8. CAN bus health

SocketCAN error frame을 이용해 최소 다음 상태를 구분한다.

```text
ERROR-ACTIVE
WARNING
ERROR-PASSIVE
BUS-OFF
```

실기 control 중 ERROR-PASSIVE 또는 BUS-OFF는 `quattro_hardware`에서 safe stop 판단에 사용한다.

## 9. 테스트

드라이버 unit test는 ROS 없이 수행 가능해야 한다.

최소 검증:

- arbitration ID encode/decode
- CAN Simple payload encode/decode
- Position/PosFilter enum 값
- MotorManager routing
- encoder frame dispatch
- encoder sequence 증가
- Heartbeat decode
- CAN error state

실제 lifecycle 순서는 `docs/packages/quattro_hardware.md`에서 테스트한다.

## 10. 개발 이력과 현재 계약 분리

날짜별 실험 결과, 특정 node가 일시적으로 offline이었던 기록, TX queue 문제 등의 개발 이력은 `docs/development_status.md`에 기록한다.

이 문서는 현재 드라이버 API와 하드웨어 계약만 유지한다.
