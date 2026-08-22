# Quattro 관절 캘리브레이션

## 목적

GIM6010-8 encoder 좌표를 ROS joint zero와 맞추기 위한 `offset`을 설정한다.

실제 설정 파일:

```text
/ws/src/quattro_bringup/config/calibration.yaml
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
- 첫 `Set_Input_Pos`는 사용자가 실제 이동을 요청하는 Jog/Move 명령에서만 발생한다.

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
cd /ws
colcon build --symlink-install --packages-select gim6010_driver quattro_hardware
source /ws/install/setup.bash

ros2 run quattro_hardware calibration_gui \
  --calibration-file /ws/src/quattro_bringup/config/calibration.yaml
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

`-1 deg`, `+1 deg` 등 실제 이동 버튼을 눌렀을 때만 목표 위치를 계산해 `Set_Input_Pos`를 보낸다.

```text
current synchronized position
→ requested relative joint target
→ joint_rad_to_motor_rev
→ Set_Input_Pos
```

## Move to Saved Zero

기존 offset으로 계산한 ROS zero 위치까지 실제 모터를 이동시키는 기능이다.

- Enable 완료 후 사용한다.
- 저장된 offset을 motor target으로 변환한다.
- 이 기능은 실제 이동이므로 `Set_Input_Pos`를 사용한다.

## Save Current Position as Zero

저장 버튼을 누를 때는 최신 **Closed Loop 이후 유효 EncoderEstimate**를 다시 사용한다.

화면에 남아 있는 stale 값이나 Closed Loop 이전 cache를 저장하지 않는다.

## Live feedback

최소 표시 항목:

- Heartbeat axis state
- Heartbeat axis error
- 최신 encoder position/velocity
- encoder freshness
- saved joint position
- session position
- command target

`CLOSED LOOP` 표시가 실제 Heartbeat 기반이어야 한다.

## 설정 파일 명칭

이번 position-control 리팩터링에서는 기존 `direct_position` 키를 `position_control`로 변경하는 것을 권장한다.

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

runtime, calibration GUI, tuning GUI, Xacro가 같은 키를 사용해야 한다.

## 완료 후 검증

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  > /tmp/quattro_calibrated.urdf
check_urdf /tmp/quattro_calibrated.urdf
```

실제 전체 bringup은 `docs/packages/quattro_bringup.md`를 따른다.
