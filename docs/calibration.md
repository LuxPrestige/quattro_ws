# Quattro 관절 캘리브레이션

## 목적

GIM6010-8 encoder 좌표를 ROS joint zero와 맞추기 위한 `offset`을 설정한다. 같은 GUI에서 position control 게인(전역 1세트) 튜닝도 함께 수행한다.

실제 설정 파일:

```text
/quattro_ws/src/quattro_bringup/config/calibration.yaml
```

## 실기 activation 원칙

캘리브레이션 GUI도 runtime과 동일한 GIM6010 startup 계약을 따른다.

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position Control + Pos Filter
→ Closed Loop Control
→ 모터 자체 현재 위치 Hold
→ Closed Loop 이후 새 EncoderEstimate 수신
→ session position 설정
```

중요:

- Closed Loop 이전 encoder 위치는 캘리브레이션 기준으로 사용하지 않는다.
- `0x009`는 약 10 ms 주기로 자동 수신하므로 GUI가 encoder request를 보내지 않는다.
- Enable 순간 현재 위치를 고정하기 위한 `Set_Input_Pos(current)`를 보내지 않는다.
- 첫 `Set_Input_Pos`는 사용자가 실제 이동을 요청하는 Jog/Go to Target/Move to Saved Zero 명령에서만 발생한다.

## 좌표 변환

```text
joint_rad = direction * (motor_rev * 2π / gear_ratio) - offset
motor_rev = direction * (joint_rad + offset) * gear_ratio / 2π
```

현재 위치를 ROS zero로 저장할 때는 Closed Loop 이후의 유효 encoder 값을 사용한다.

## 안전 조건

1. 로봇을 지지대에 고정한다.
2. `controller_manager`와 다른 CAN 송신 프로그램을 종료한다.
3. `can0`, `can1`이 ERROR-ACTIVE인지 확인한다.
4. 대상 모터의 Heartbeat와 0x009 자동 broadcast를 확인한다.
5. 낮은 current/gain에서 시작한다.
6. 즉시 전원을 차단할 수 있어야 한다.

캘리브레이션 GUI와 hardware bringup을 동시에 실행하지 않는다.

## 실행

```bash
cd /quattro_ws
colcon build --symlink-install --packages-select gim6010_driver quattro_hardware
source /quattro_ws/install/setup.bash

ros2 run quattro_hardware calibration_gui \
  --calibration-file /quattro_ws/src/quattro_bringup/config/calibration.yaml
```

## 선택 모터 Enable

1. 관절 선택
2. `Enable Selected Motor`
3. limits/gains 적용
4. Position + PosFilter 설정
5. Closed Loop 요청
6. Heartbeat에서 Closed Loop 및 `axis_error == 0` 확인
7. Closed Loop 이후 새 EncoderEstimate 수신
8. 그 위치를 session 기준으로 저장
9. 모터는 command 없이 자체 Hold 상태 유지

이 단계에서 `Set_Input_Pos`를 보내지 않는다.

## Jog

`- step` / `+ step` 버튼을 눌렀을 때만 목표 위치를 계산해 `Set_Input_Pos`를 보낸다. 스텝 크기는 `Jog step (deg)` 입력란으로 조절한다(기본 1도).

GUI는 축별로 "사용자가 target을 요청했는가"를 기록하고, 주기 timer는 그 축에만 명령을 반복 전송한다. Enable만 된 축은 모터 자체 Hold 상태로 두고 아무것도 보내지 않는다. 축을 Disable하면 이 기록도 초기화되므로, 다시 Enable한 축은 이전 target을 이어서 명령하지 않고 새로 동기화한다.

Jog는 session 좌표계(enable 시점 기준 상대 각도)로 동작하며 **URDF 관절 한계를 검사하지 않는다**. 캘리브레이션 도중에는 offset이 아직 부정확하거나 0인 경우가 많아 saved 좌표계 한계가 무의미하고, jog 자체가 바로 그 offset을 찾기 위한 수단이기 때문이다.

```text
current synchronized position
→ requested relative joint target
→ joint_rad_to_motor_rev
→ Set_Input_Pos
```

## Go to Target (절대 목표각)

`Absolute target (deg, saved frame)` 입력 후 `Go to Target`을 누르면 **saved(ROS joint) 좌표계**의 절대 각도로 이동한다.

- Enable 완료 후 사용한다.
- 좌표 변환: `session_rad = saved_rad + offset` (`Go to Target(0)`은 `Move to Saved Zero`와 동일 위치로 이동한다).
- **URDF `<limit>` 범위를 벗어나면 명령을 보내지 않고 경고만 표시한다.** jog와 달리 절대 이동은 관절이 현재 어디 있든 멀리 떨어진 위치로 곧장 보낼 수 있기 때문이다.

## Move to Saved Zero

기존 offset으로 계산한 ROS zero 위치(`Go to Target(0)`과 동일)까지 실제 모터를 이동시키는 단축 기능이다.

- Enable 완료 후 사용한다.
- 저장된 offset을 motor target으로 변환한다.
- 이 기능은 실제 이동이므로 `Set_Input_Pos`를 사용한다.

## Save Current Position as Zero

저장 버튼을 누를 때는 최신 **Closed Loop 이후 유효 EncoderEstimate**를 다시 읽어 사용한다.

구현상 축별로 Closed Loop 동기화 시점의 `encoder_sequence()`를 기록해 두고, 저장 시 그보다 나중에 도착한 frame만 받아들인다. 화면에 남아 있는 stale 값이나 Closed Loop 이전 cache는 저장되지 않는다.

저장 후에도 해당 축은 Enable 상태를 유지한다. Disable은 항상 별도 버튼으로 명시적으로 눌러야 한다.

## Position control 게인 튜닝

게인은 축별이 아니라 `position_control` 키 하나로 **전역 1세트**를 사용한다 (Current limit / Position gain / Velocity gain / Velocity integrator gain).

- `Apply to Enabled Motors`: 입력값을 검증한 뒤, 현재 Enable된 모든 축에 `Set_Limits → Set_Pos_Gain → Set_Vel_Gains` 순서로 즉시 재전송한다. Disable/Enable을 거치지 않으므로 hold target이나 세션 상태가 초기화되지 않는다. 이후 새로 Enable하는 축도 자동으로 이 값을 받는다.
- `Save Gains to YAML`: `position_control` 키만 파일에 기록한다. offset(`joints` 키)에는 영향을 주지 않는다.
- `Reload Calibration from File`을 누르면 게인 입력란과 12축 테이블의 Offset 열이 모두 파일 값으로 갱신되고, Enable된 축에는 새 게인이 즉시 재전송된다.

낮은 current/gain에서 시작해 단계적으로 올린다(안전 조건 참고).

## 12축 상태 테이블

GUI는 항상 12행짜리 테이블로 모든 축을 동시에 보여준다. 행 선택이 곧 축 선택이며, 다른 축의 Enable 상태에는 영향을 주지 않는다.

| 열 | 내용 |
|---|---|
| Joint | 관절 이름 |
| CAN | `can버스:can_id` |
| State | Heartbeat 기반 `CLOSED LOOP` / `IDLE` / `AXIS STATE n` / `NO HEARTBEAT`, fault 시 `fault 0x…` 병기 |
| Saved [deg] | offset이 적용된 ROS joint 각도 |
| Session [deg] | enable 시점 기준 상대 각도 (jog가 사용하는 좌표계) |
| Target [deg] | 현재 명령 target을 saved 좌표계로 환산한 값 (Saved 열과 직접 비교 가능) |
| Error [deg] | `idle`(비활성) / `holding`(target 미요청) / 수치(추종 오차) |
| Vel [deg/s] | session 좌표계 속도 |
| Motor [rev] | 원시 회전자 값 (candump 대조용) |
| Offset [rad] | `calibration.yaml`에 저장된 현재 offset |

feedback이 없는 축은 State 외의 열이 `--`로 표시된다. `CLOSED LOOP`(초록) 표시는 실제 Heartbeat 기반이며, `IDLE`(빨강)과 `NO HEARTBEAT`(회색)는 서로 다른 문제이므로 구분해서 표시한다.

## 설정 파일 명칭

제어 설정 키는 `position_control`이다.

```yaml
position_control:
  current_limit: 10.0
  position_gain: 20.0
  velocity_gain: 0.11
  velocity_integrator_gain: 0.32

joints:
  front_left_hip_joint:
    can_interface: can0
    can_id: 0
    direction: -1
    offset: 0.0
```

runtime, calibration GUI, Xacro가 모두 이 키를 사용한다.

## 완료 후 검증

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  calibration_file:=/quattro_ws/src/quattro_bringup/config/calibration.yaml \
  > /tmp/quattro_calibrated.urdf
check_urdf /tmp/quattro_calibrated.urdf
```

실제 전체 bringup은 `docs/packages/quattro_bringup.md`를 따른다.
