# `quattro_hardware`

## 역할

`quattro_hardware`는 Quattro의 12개 ROS joint와 실제 GIM6010-8 액추에이터를 연결하는 `ros2_control` 하드웨어 계층이다.

핵심 클래스는 `quattro_hardware/QuattroSystem`이며 다음을 담당한다.

- joint ↔ motor 좌표 변환
- CAN bus/node mapping 적용
- GIM6010 startup sequence
- Closed Loop 이후 encoder 동기화
- `read()` / `write()`
- heartbeat, encoder, CAN bus watchdog
- fault 발생 시 전체 safe stop

raw CAN frame encode/decode와 SocketCAN I/O는 `gim6010_driver`가 담당한다.

## 1. 실기 확인된 GIM6010-8 startup 특성

다음 항목을 이 패키지의 최우선 하드웨어 계약으로 사용한다.

1. 모터 설정 순서는 다음과 같다.

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Set_Controller_Mode(Position Control, Pos Filter)
→ Set_Axis_State(Closed Loop Control)
```

2. **Closed Loop 이전 EncoderEstimate 위치값은 유효한 관절 위치로 사용하지 않는다.** 실기에서 이 상태의 값이 쓰레기 값으로 관측되었다.

3. `Get_Encoder_Estimates(0x009)`는 모터가 약 10 ms 주기로 자동 송신한다. 정상 runtime과 startup은 별도 encoder request를 보내지 않는다.

4. `Position Control + Pos Filter`를 설정한 뒤 Closed Loop에 진입하면, 별도의 `Set_Input_Pos` 명령 없이도 모터 축이 현재 위치에서 고정된다.

5. 따라서 Safe Start를 위해 Closed Loop 전에 encoder를 읽고 `Set_Input_Pos(current)`를 보내는 과거 구현은 사용하지 않는다.

6. 초기 ROS 위치는 **Closed Loop가 확인된 이후 도착한 새 EncoderEstimate**로 결정한다.

## 2. CAN ID / bus mapping

| Joint | Node ID | Bus |
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

`direction`, `offset`은 calibration YAML에서 관리하고 gear ratio는 현재 8.0을 사용한다.

## 3. 좌표 변환

대표 변환식:

```text
joint_rad = direction * (motor_rev * 2π / gear_ratio) - offset
motor_rev = direction * (joint_rad + offset) * gear_ratio / 2π
```

ROS 경계는 rad/rad/s/N·m를 사용한다.

## 4. `on_configure()` 동작

`on_configure()`에서는 CAN 연결과 모터 설정을 완료하지만 Closed Loop 전 encoder 위치는 초기 state로 사용하지 않는다.

```text
MotorManager 생성
→ can0/can1 open
→ wait_for_all_heartbeats()
→ check_pre_activation_faults()
→ 각 모터 Set_Limits
→ 각 모터 Set_Pos_Gain
→ 각 모터 Set_Vel_Gains
→ 각 모터 Set_Controller_Mode(Position Control, Pos Filter)
→ CONFIGURED
```

존재 확인에 Heartbeat만 사용하고 encoder는 보지 않는다. Closed Loop 이전 EncoderEstimate는 도착 자체가 liveness를 뜻하더라도 위치값이 무효이므로, 사용 가능한 값처럼 보이는 경로를 아예 만들지 않는다.

설정 burst가 SocketCAN TX queue를 넘지 않도록 startup 전송에는 bounded retry/pacing(`send_with_retry()`)을 유지한다.

`Clear_Errors`는 자동으로 보내지 않는다. 기존 fault는 운영자가 판단할 때까지 보이는 상태로 남는다.

## 5. `on_activate()` 동작

모터별 activation(`activate_joint()`)은 다음 순서를 따른다.

```text
Set_Axis_State(Closed Loop Control)
→ wait_for_closed_loop()
     Heartbeat.axis_state == Closed Loop 확인
     Heartbeat.axis_error == 0 확인
→ 이 시점의 encoder_sequence()를 baseline으로 저장
→ wait_for_post_closed_loop_encoder()
     baseline 이후 encoder_sync_frames개 EncoderEstimate 대기
→ 마지막 frame의 motor_rev -> joint_rad 변환
→ ROS state.position 동기화
→ position command interface도 같은 joint_rad로 동기화
→ 다음 모터
```

baseline을 `Set_Axis_State` 전이 아니라 **Closed Loop Heartbeat 확인 후**에 잡는다. Heartbeat 자체가 최대 한 주기만큼 늦게 도착할 수 있으므로, 그 이전 frame은 Closed Loop 이후에 sampling되었다고 보장할 수 없다.

핵심 규칙:

- Closed Loop 전에 캐시된 encoder는 초기 위치로 사용하지 않는다.
- startup에서 `Set_Input_Pos`를 전혀 보내지 않는다.
- 첫 유효 encoder는 Closed Loop 확인 이후 수신된 프레임이어야 한다.
- 한 모터라도 timeout/fault가 발생하면 `safe_stop_all()`로 전체 모터를 Idle로 전환하고 activation을 실패시킨다.

12축 모두 동기화된 뒤에만 `on_activate()`가 SUCCESS를 반환한다.

12축 순차 activation은 `feedback_timeout_ms`보다 오래 걸리지만, 각 대기 루프의 `poll()`이 모든 bus를 drain하므로 다른 모터의 freshness도 함께 갱신된다. 그래도 `read()`에 제어를 넘기기 전에 `wait_for_all_fresh_feedback()`으로 전체 freshness를 한 번 더 확인한다.

### lifecycle helper

| 함수 | 책임 |
|---|---|
| `wait_for_all_heartbeats()` | 전 모터 Heartbeat 존재/freshness |
| `wait_for_all_fresh_feedback()` | Heartbeat + encoder freshness (Closed Loop 이후에만 유효) |
| `check_pre_activation_faults()` | 전 모터 `Heartbeat.axis_error == 0` |
| `wait_for_closed_loop()` | 단일 모터 Closed Loop 전환 확인 |
| `wait_for_post_closed_loop_encoder()` | baseline 이후 encoder frame 대기 |
| `activate_joint()` | 위 단계 + state/command 동기화 |
| `safe_stop_all()` | 전 모터 Idle |

### activation timeout parameter

고정 지연(`motor_activation_interval_ms`)은 제거되었고 상태 기반 timeout으로 대체되었다.

| parameter | 기본값 | 의미 |
|---|---:|---|
| `closed_loop_timeout_ms` | 500 | Closed Loop Heartbeat 대기 한도 |
| `encoder_sync_timeout_ms` | 200 | post-Closed-Loop encoder 대기 한도 |
| `encoder_sync_frames` | 2 | 초기 위치 확정에 필요한 post-Closed-Loop frame 수 |

`encoder_sync_frames`는 1 이상이어야 하며 `on_init()`에서 검증한다.

## 6. Encoder sequence와 `encoder_sync_frames`

`Gim6010Motor::encoder_sequence()`는 0x009 수신마다 1씩 증가하는 counter다. activation은 이것으로 Closed Loop 전/후 프레임을 구분한다.

```text
Set_Axis_State(ClosedLoop)
wait heartbeat closed-loop
baseline = encoder_sequence()
wait encoder_sequence() >= baseline + encoder_sync_frames
```

### `encoder_sync_frames`를 2로 두는 이유

Heartbeat(약 100 ms)와 EncoderEstimate(약 10 ms)는 서로 독립적인 broadcast 스트림이다. Closed Loop Heartbeat를 수신한 시점에서:

- 그 Heartbeat는 최대 한 주기만큼 과거의 상태일 수 있다.
- 같은 poll 배치에 함께 들어온 0x009은 전환 전에 sampling되었을 가능성이 있다.

`encoder_sync_frames = 1`이면 이 경계선상의 frame을 초기 위치로 쓸 위험이 남는다. 2로 두면 Closed Loop가 이미 보고된 뒤에 sampling된 frame이 최소 한 개는 포함되는 것이 보장된다. 비용은 축당 약 10 ms다.

3 이상은 두 스트림의 skew가 한 frame 주기를 넘는 하드웨어에 대한 추가 마진이며, 현재 실기에서는 관측되지 않았다.

## 7. `read()`

`read()`는 `MotorManager::poll()`로 자동 broadcast frame을 드레인한다.

각 joint에 대해:

- EncoderEstimate → position/velocity state
- Heartbeat → axis state/fault 판단
- CAN bus state → ERROR-ACTIVE 여부 판단

`effort` state는 NaN이다. Position Control + Pos Filter는 측정 토크를 보고하지 않으며 별도 토크 feedback 경로를 연결하지 않았다.

runtime 중 stale encoder, stale heartbeat, non-zero `axis_error`, CAN passive/bus-off 등은 안전 fault로 처리한다.

Encoder feedback을 얻기 위해 0x009 request를 주기적으로 보내지 않는다.

## 8. `write()`

`write()`는 상위 `position` command interface를 motor revolution으로 변환하여 `Set_Input_Pos(0x00C)`로 전송한다.

중요한 구분:

```text
startup / Closed Loop 진입
    -> Set_Input_Pos를 Safe Start 수단으로 사용하지 않음

정상 command 제어 시작 이후
    -> Set_Input_Pos 사용
```

첫 command가 현재 encoder 위치와 불연속이 되지 않도록 activation 완료 시 command 기준을 현재 joint position으로 동기화해야 한다.

일시적인 SocketCAN TX queue rejection 한 번으로 즉시 전체 정지하지 않고, 기존의 연속 write failure 정책을 유지한다.

## 9. Safe stop

다음 조건에서 전체 모터를 Idle로 전환한다.

- activation timeout
- Closed Loop 전환 실패
- post-Closed-Loop encoder 미수신
- heartbeat timeout
- encoder stale
- `axis_error != 0`
- CAN bus unhealthy
- 연속 position command 전송 실패
- controller/hardware lifecycle 종료

부분적으로 활성화된 상태를 정상 상태로 간주하지 않는다.

## 10. Calibration / tuning GUI

`calibration_gui` 하나가 offset 캘리브레이션과 position control 게인 튜닝을 모두 담당한다(과거의 별도 `position_control_tuning_gui`는 삭제했다). runtime과 동일한 activation 원칙을 따른다.

Enable 동작:

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position + Pos Filter
→ Closed Loop
→ Heartbeat Closed Loop 확인
→ 그 이후 도착한 encoder 수신
→ session current position 설정
```

Enable 순간에는 `Set_Input_Pos`를 보내지 않는다. 첫 position command는 `+/- jog`, `Go to Target`(절대 목표각), `Move to Saved Zero` 등 사용자가 실제 이동을 요청했을 때 전송한다.

축마다 "사용자가 target을 요청했는가"를 별도로 기록하고, 주기 timer는 그 플래그가 설정된 축에만 명령을 반복 전송한다. Enable만 된 축은 모터 자체 Hold 상태로 둔다.

`Save Current Position as Zero`는 화면에 남은 값이나 Closed Loop 이전 cache가 아니라, 해당 축의 Closed Loop 동기화 시점 이후에 도착한 encoder frame을 다시 읽어 사용한다. 저장 후에도 해당 축은 Enable 상태를 유지한다(명시적으로 Disable해야 Idle된다).

position control 게인(`current_limit`/`position_gain`/`velocity_gain`/`velocity_integrator_gain`)은 축별이 아니라 `position_control` 키 하나로 전역 적용된다. `Apply to Enabled Motors`는 입력값을 즉시 활성 축 전체에 재전송하고(순서는 `Set_Limits → Set_Pos_Gain → Set_Vel_Gains`, hold target은 유지), `Save Gains to YAML`은 파일에 기록만 한다.

GUI는 12축을 항상 표로 보여준다(축 상태/fault, saved·session·target 각도, 추종 오차, 속도, raw motor rev, offset). `Absolute target`은 saved(ROS joint) 좌표계이며 URDF `<limit>` 범위를 벗어나면 거부된다. jog는 캘리브레이션 도중 offset이 아직 부정확한 경우가 많아 이 한계를 적용하지 않는다.

## 11. 테스트

`test/test_quattro_system_startup.cpp`가 실제 `QuattroSystem`을 in-memory CAN bus(`test/fake_can_network.hpp`) 위에서 구동한다. transport만 교체하므로 `gim6010_driver`의 encode/decode, routing, dispatch, encoder sequence는 실제 코드 그대로 동작하며, "startup에서 `Set_Input_Pos` frame이 0개"와 같은 주장이 mock 호출 횟수가 아니라 실제 wire traffic으로 검증된다.

seam은 `QuattroSystem::create_motor_manager()` 하나뿐이다. 테스트 subclass가 이것만 override한다.

검증 항목:

1. Closed Loop 이전 encoder는 startup 위치로 사용되지 않는다.
2. controller mode가 Position Control + Pos Filter로 설정되고, 순서가 limits → gains → mode이며 configure 단계에서는 `Set_Axis_State`를 보내지 않는다.
3. configure + activate 전체에서 `Set_Input_Pos` 전송 횟수가 0이다.
4. Closed Loop 확인 이후의 frame만 state 초기화에 사용된다(`encoder_sync_frames` 동작 포함).
5. post-Closed-Loop encoder timeout 시 activation 실패 및 all-idle.
6. enable 시 axis_error 발생 시 activation 실패 및 all-idle. 기존 axis_error는 configure 자체를 거부한다.
7. N번째 모터 activation 실패 시 앞서 활성화된 모터도 모두 Idle.
8. Heartbeat 미수신 시 configure 실패.
9. activation 직후 command interface == state, 첫 `write()`가 그 위치를 그대로 전송한다.
10. `read()`가 encoder request를 보내지 않는다.
11. `direction`/`offset`이 동기화 위치에 반영된다.

encoder sequence counter 자체의 동작은 `gim6010_driver`의 `test_motor_manager.cpp`에서 검증한다.

## 12. 관련 문서

- 저수준 CAN: `docs/packages/gim6010_driver.md`
- 실기 실행: `docs/packages/quattro_bringup.md`
- calibration: `docs/calibration.md`
- 현재 실기 상태: `docs/development_status.md`
