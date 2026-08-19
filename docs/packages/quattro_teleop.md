# `quattro_teleop`

## 역할

Switch Pro Controller(joystick)와 keyboard 입력을 표준 ROS 2 메시지 상위 명령으로 변환한다(`ament_python`). 모터를 직접 제어하지 않는다 — 발행 대상은 항상 `quattro/gait_controller`가 소비하는 표준 토픽/서비스다.

```text
src/quattro_teleop/quattro_teleop/
├── teleop_node.py       # teleop_node 실행 파일 (joystick, 실기 기본값)
└── keyboard_teleop.py   # keyboard_teleop 실행 파일 (데스크톱 개발용)
```

## `teleop_node.py` (`TeleopNode`, 노드 이름 `quattro_teleop`)

`sensor_msgs/Joy`(`/joy`)를 받아 stepping 모드와 pose(정지-시야) 모드를 전환하며 명령을 발행한다.

**발행**

| 토픽/서비스 | 타입 | 조건 |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/Twist` | stepping 모드 + `/joy` 신선 + estop 아님, 아니면 zero Twist |
| `/body_pose` | `geometry_msgs/PoseStamped` | pose 모드 + 신선 + estop 아님일 때만 발행(재사용 없음) |
| `/estop` | `std_msgs/Bool` | 매 주기 현재 상태 발행 |
| `/imu_auto` | `std_msgs/Bool` | 매 주기 현재 상태 발행 |
| `/gait/clearance_height`, `/gait/penetration_depth`, `/gait/swing_duration` | `std_msgs/Float64` | 매 주기 현재 값 발행 |
| `/gait/enable` | `std_srvs/SetBool` (클라이언트) | `button_switch_mode` 상승 edge에서만 비동기 호출 |

**입력 매핑**(기본값, 전부 파라미터로 재설정 가능): 왼쪽 스틱 = 전진/횡이동(stepping) 또는 pitch/roll(pose), 오른쪽 스틱 X = yaw, 오른쪽 스틱 Y = 높이, D-pad = gait clearance/penetration 실시간 조정, `button_switch_mode` = stepping↔pose 전환, `button_estop` = E-stop 토글, `button_imu_auto` = IMU balance 토글, 범퍼 두 개 = gait 조정값을 launch 기본값으로 리셋.

**안전 원칙**

- `apply_deadzone`으로 축 입력에 연속 데드존을 적용해 중립 근처 떨림을 제거한다.
- `/joy`가 `joy_timeout_sec`(기본 0.5초) 이상 갱신되지 않으면 "신선하지 않음"으로 판단해 `/cmd_vel`/`/body_pose`를 더 이상 새로 발행하지 않는다(zero Twist만 계속 발행) — 컨트롤러 연결이 끊겨도 마지막 명령이 무한히 유지되지 않는다.
- 버튼 rising-edge만 감지해(`_rising_edge`) 길게 누르고 있어도 반복 트리거되지 않는다.
- `gait/enable` 서비스가 아직 준비되지 않았으면 모드 전환 시도를 경고만 남기고 무시한다(예외로 죽지 않음).

## `keyboard_teleop.py` (`KeyboardTeleop`, 노드 이름 `quattro_keyboard_teleop`)

터미널 raw 모드(`termios`/`tty`)로 WASD+QE 키를 읽어 `/cmd_vel`만 발행하는 데스크톱 전용 대안. 인터랙티브 터미널이 아니면(`sys.stdin.isatty()`가 거짓) 생성자에서 즉시 실패한다. 키 입력이 `key_timeout`(기본 0.25초) 이상 없으면 zero `Twist`로 되돌아간다. 종료 시 `restore_terminal()`로 터미널 설정을 복원한다.

## Launch

`launch/teleop.launch.py`가 `config/switch_pro.yaml` 파라미터로 `joy/game_controller_node` + `teleop_node`를 함께 실행한다. `quattro_bringup/hardware.launch.py`는 `use_teleop:=true`(기본값)일 때 동일 파라미터로 두 노드를 시작하며 `start_stepping:=true`를 추가로 지정한다.
