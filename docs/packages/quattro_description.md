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

`quattro_hardware_joint`(실기)/`quattro_simulation_joint`(시뮬레이션) 매크로 호출부의 `command_interface`(`position`) `min`/`max`는 아래 관절 한계와 반드시 같은 값을 유지한다 — 실제 command 경로가 참조하는 값은 `<joint><limit>`이 아니라 이 `min`/`max`이기 때문이다.

## 관절 한계와 액추에이터 사양

액추에이터: SteadyWin GIM6010-8(모델 표기 `6010-8`, gear ratio 8:1 내장). Xacro의 `gear_ratio` 파라미터는 이 값을 그대로 `8.0`으로 하드코딩한다(calibration YAML이 아님).

전기적 특성(제조사 자료 기준, 감속기 출력 기준 값):

| 항목 | 값 |
|---|---:|
| 정격 회전수 | 120 rpm |
| 최대 회전수 | 420 rpm |
| 정격 토크 | 5 N·m |
| 정지 토크 | 11 N·m |
| 정격 전류 | 10.5 A |
| 정지 전류 | 25 A |
| 무부하 전류 | 0.4 A |
| 토크 상수 | 0.47 N·m/A |

각 관절 `<joint><limit>`(및 이를 mirror하는 `command_interface` min/max)은 다음 범위를 사용한다.

| 관절 | 각도 범위(deg) | 각도 범위(rad) |
|---|---:|---:|
| hip | -45 ~ 45 | -0.7853981634 ~ 0.7853981634 |
| upper | -90 ~ 140 | -1.5707963268 ~ 2.4434609528 |
| lower | -135 ~ 135 | -2.3561944902 ~ 2.3561944902 |

`effort`/`velocity`/`dynamics`는 12관절 모두 동일하게 다음 근거로 설정한다.

| 필드 | 값 | 근거 |
|---|---:|---|
| `effort` | 11.0 N·m | 정지 토크(순간 최대치). 연속 열보호는 URDF가 아니라 하드웨어 `current_limit` 파라미터가 담당한다. |
| `velocity` | 12.5663706144 rad/s | 정격 회전수 120 rpm(연속 안전 속도 기준; 최대 회전수 420 rpm은 사용하지 않는다) |
| `dynamics damping` | 0.0 | 제조사 자료에 점성 감쇠 계수가 없어 추정하지 않았다 |
| `dynamics friction` | 0.188 N·m | 무부하 전류 0.4 A × 토크 상수 0.47 N·m/A (Coulomb friction 근사치) |

## Position Control 설정

calibration YAML의 제어 설정 키는 `position_control`이다. 아래 값은 버전관리되는 `calibration.yaml.example` 기준이며, 실제 기체별 `calibration.yaml`(git 미추적)은 PID 튜닝 결과에 따라 다를 수 있다.

```yaml
position_control:
  current_limit: 5.0
  position_gain: 20.0
  velocity_gain: 0.16
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
| `rotor_velocity_limit_rev_s` | 10.0 | `Set_Limits` 속도 제한 |
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
- `docs/packages/quattro_hardware.md` (calibration/tuning GUI 포함)
- `docs/packages/quattro_bringup.md`
