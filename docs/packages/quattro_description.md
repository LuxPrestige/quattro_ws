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

calibration YAML의 제어 설정 키는 `position_control`이다.

```yaml
position_control:
  current_limit: 10.0
  position_gain: 20.0
  velocity_gain: 0.11
  velocity_integrator_gain: 0.32
```

Xacro는 이를 다음으로 읽는다.

```text
calibration['position_control']
```

이 값은 `QuattroSystem::on_configure()`에서 다음 startup 설정에 사용된다.

```text
Set_Limits
Set_Pos_Gain
Set_Vel_Gains
Set_Controller_Mode(Position Control, Pos Filter)
```

## Hardware parameters

`<ros2_control name="QuattroSystem" type="system">`은 다음 timeout/safety parameter를 전달한다. 모두 필수이며 누락되면 `on_init()`이 실패한다.

| parameter | 값 | 의미 |
|---|---:|---|
| `feedback_timeout_ms` | 150 | encoder freshness 한도 |
| `heartbeat_timeout_ms` | 400 | heartbeat freshness 한도 |
| `startup_timeout_ms` | 1000 | 전 모터 heartbeat/feedback 대기 한도 |
| `closed_loop_timeout_ms` | 500 | Closed Loop Heartbeat 대기 한도 |
| `encoder_sync_timeout_ms` | 200 | post-Closed-Loop encoder 대기 한도 |
| `encoder_sync_frames` | 2 | 초기 위치 확정에 필요한 post-Closed-Loop frame 수 |
| `command_timeout_ms` | 250 | `write()` watchdog |
| `scheduling_warning_ms` | 50 | `read()` 주기 경고 임계값 |
| `rotor_velocity_limit_rev_s` | 5.0 | `Set_Limits` 속도 제한 |
| `telemetry_period_ms` | 500 | telemetry 주기 |

`current_limit` / `position_gain` / `velocity_gain` / `velocity_integrator_gain`은 calibration YAML의 `position_control`에서 온다.

고정 지연 parameter(`motor_activation_interval_ms`)는 제거되었다. activation 완료는 실제 Heartbeat Closed Loop 전환과 post-Closed-Loop encoder 수신으로 판단한다. `encoder_sync_frames`를 2로 두는 근거는 `docs/packages/quattro_hardware.md` 6절과 Xacro 주석에 있다.

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
cd /quattro_ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

`simulation:=true`와 `false` 양쪽을 확인한다.

## 관련 문서

- `docs/architecture.md`
- `docs/packages/quattro_hardware.md`
- `docs/packages/quattro_bringup.md`
- `docs/calibration.md`
