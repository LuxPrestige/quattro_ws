# `quattro_description`

## 역할

Quattro의 URDF/Xacro, mesh, TF, joint/link, inertial/collision, 실제/시뮬레이션 `ros2_control` description을 소유한다.

제어 알고리즘이나 GIM6010 protocol 구현은 포함하지 않는다.

## `quattro.urdf.xacro`

실기 `ros2_control`에서는 `quattro_hardware/QuattroSystem`을 사용한다.

각 joint는:

- `position` command interface
- `position`, `velocity`, `effort` state interface
- `can_interface`
- `can_id`
- `direction`
- `offset`
- `gear_ratio`

를 `QuattroSystem`에 전달한다.

## Position Control 설정

기존 `direct_position` YAML 명칭은 실제 제어 방식과 일치하지 않으므로 리팩터링 시 `position_control`로 통일하는 것을 권장한다.

예:

```yaml
position_control:
  current_limit: 10.0
  position_gain: 20.0
  velocity_gain: 0.11
  velocity_integrator_gain: 0.32
```

Xacro도 동일하게:

```text
calibration['position_control']
```

을 읽도록 수정한다.

이 값은 `QuattroSystem::on_configure()`에서 다음 startup 설정에 사용된다.

```text
Set_Limits
Set_Pos_Gain
Set_Vel_Gains
Set_Controller_Mode(Position Control, Pos Filter)
```

## Hardware parameters

`<ros2_control name="QuattroSystem" type="system">`은 최소 다음 timeout/safety parameter를 전달한다.

권장 항목:

```text
feedback_timeout_ms
heartbeat_timeout_ms
startup_timeout_ms
closed_loop_timeout_ms
encoder_sync_timeout_ms
encoder_sync_frames
command_timeout_ms
scheduling_warning_ms
rotor_velocity_limit_rev_s
telemetry_period_ms
```

기존 `motor_activation_interval_ms` 같은 고정 지연보다는 실제 Heartbeat Closed Loop 전환과 post-Closed-Loop encoder 수신을 기준으로 activation 완료를 판단하는 것을 우선한다.

## 실기/시뮬레이션 분기

```text
simulation:=false
  -> quattro_hardware/QuattroSystem

simulation:=true
  -> gz_ros2_control/GazeboSimSystem
```

실기 Position + PosFilter startup 특성은 simulation controller에 억지로 복제하지 않는다. simulation은 ROS joint position interface 계약만 동일하게 유지한다.

## 검증

```bash
cd /ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

`simulation:=true`와 `false` 양쪽을 확인한다.

## 관련 문서

- `docs/architecture.md`
- `docs/packages/quattro_hardware.md`
- `docs/packages/quattro_bringup.md`
- `docs/calibration.md`
