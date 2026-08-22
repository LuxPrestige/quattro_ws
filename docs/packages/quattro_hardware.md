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

## 4. `on_configure()` 목표 동작

`on_configure()`에서는 CAN 연결과 모터 설정을 완료하지만 Closed Loop 전 encoder 위치는 초기 state로 사용하지 않는다.

권장 순서:

```text
MotorManager 생성
→ can0/can1 open
→ heartbeat 기반 모터 존재/axis_error 확인
→ 각 모터 Set_Limits
→ 각 모터 Set_Pos_Gain
→ 각 모터 Set_Vel_Gains
→ 각 모터 Set_Controller_Mode(Position Control, Pos Filter)
→ CONFIGURED
```

설정 burst가 SocketCAN TX queue를 넘지 않도록 startup 전송에는 bounded retry/pacing을 유지한다.

## 5. `on_activate()` 목표 동작

모터별 activation은 다음 순서를 따른다.

```text
activation 기준 encoder generation/timestamp 저장
→ Set_Axis_State(Closed Loop Control)
→ Heartbeat.axis_state == Closed Loop 확인
→ Heartbeat.axis_error == 0 확인
→ Closed Loop 확인 이후 도착한 새 EncoderEstimate 대기
→ motor_rev -> joint_rad 변환
→ ROS state.position 동기화
→ 첫 command 기준도 같은 joint_rad로 동기화
→ 다음 모터
```

핵심 규칙:

- Closed Loop 전에 캐시된 encoder는 초기 위치로 사용하지 않는다.
- startup에서 `Set_Input_Pos(current)`를 보내지 않는다.
- 첫 유효 encoder는 Closed Loop 이후 수신된 프레임이어야 한다.
- 필요하면 연속 2~3개 encoder frame을 확인하는 안전 마진을 둘 수 있다.
- 한 모터라도 timeout/fault가 발생하면 전체 모터를 Idle로 전환하고 activation을 실패시킨다.

12축 모두 동기화된 뒤에만 `on_activate()`가 SUCCESS를 반환한다.

## 6. Encoder freshness 구현 권장안

현재 `Gim6010Motor`는 최신 encoder와 timestamp를 캐시한다. Closed Loop 전/후 프레임을 명확히 구분하기 위해 encoder generation counter를 추가하는 것을 권장한다.

예:

```cpp
std::uint64_t encoder_sequence_{0};
```

새 0x009 수신 시 증가시키고 activation에서:

```text
sequence_before = encoder_sequence
Set_Axis_State(ClosedLoop)
wait heartbeat closed-loop
wait encoder_sequence > sequence_before
```

로 판단한다.

더 엄격하게 하려면 Closed Loop heartbeat를 확인한 시점의 sequence를 다시 기준으로 잡고 그 이후 프레임만 사용한다.

## 7. `read()`

`read()`는 `MotorManager::poll()`로 자동 broadcast frame을 드레인한다.

각 joint에 대해:

- EncoderEstimate → position/velocity state
- Heartbeat → axis state/fault 판단
- CAN bus state → ERROR-ACTIVE 여부 판단

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

`calibration_gui`와 position control tuning GUI도 runtime과 동일한 activation 원칙을 따라야 한다.

Enable 동작:

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position + Pos Filter
→ Closed Loop
→ Closed Loop 이후 encoder 수신
→ session current position 설정
```

Enable 순간에는 `Set_Input_Pos`를 보내지 않는다. 첫 position command는 `+/- jog`, `Move to Saved Zero`, relative target 등 사용자가 실제 이동을 요청했을 때 전송한다.

## 11. 테스트 요구사항

기존 좌표 변환/pluginlib 테스트에 startup sequence 테스트를 추가한다.

최소 검증 항목:

1. Closed Loop 이전 encoder는 startup 위치로 사용되지 않는다.
2. controller mode가 Position + PosFilter로 설정된다.
3. startup에서 `Set_Input_Pos(current)`가 전송되지 않는다.
4. Closed Loop 이후 새 encoder만 state 초기화에 사용된다.
5. encoder timeout 시 activation 실패 및 all-idle.
6. non-zero axis_error 시 activation 실패.
7. N번째 모터 activation 실패 시 앞서 활성화된 모터도 모두 Idle.
8. 첫 정상 command가 encoder 동기화 위치에서 연속적으로 시작한다.

## 12. 관련 문서

- 저수준 CAN: `docs/packages/gim6010_driver.md`
- 실기 실행: `docs/packages/quattro_bringup.md`
- calibration: `docs/calibration.md`
- 현재 실기 상태: `docs/development_status.md`
