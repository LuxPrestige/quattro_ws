# Quattro 문서 색인

이 저장소는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어 워크스페이스다.

## 먼저 읽을 문서

| 문서 | 내용 |
|---|---|
| [`architecture.md`](architecture.md) | 패키지 책임, 의존 방향, 실기 startup 경계 |
| [`development_status.md`](development_status.md) | 실기에서 확인된 GIM6010 특성과 리팩터링 상태 |
| [`packages/gim6010_driver.md`](packages/gim6010_driver.md) | CAN Simple / SocketCAN 저수준 계약 |
| [`packages/quattro_hardware.md`](packages/quattro_hardware.md) | `QuattroSystem` lifecycle, Position + PosFilter startup |
| [`packages/quattro_bringup.md`](packages/quattro_bringup.md) | 실제 robot bringup 순서, `bringup_manager` 상태 머신 |
| [`calibration.md`](calibration.md) | 관절 영점 캘리브레이션 |

## 현재 실기 제어 원칙

Quattro 실기 GIM6010-8 제어의 공식 startup sequence는 다음과 같다.

```text
Set_Limits
→ Set_Pos_Gain
→ Set_Vel_Gains
→ Position Control + Pos Filter
→ Closed Loop Control
→ 모터 자체 Hold
→ Closed Loop 이후 새 EncoderEstimate
→ ROS joint state 동기화
```

중요:

- Closed Loop 이전 encoder position은 startup 위치로 사용하지 않는다.
- EncoderEstimate(0x009)는 약 10 ms 주기로 자동 송신하므로 정상 경로에서 별도 요청하지 않는다.
- Closed Loop 진입 후 position command 없이도 축이 현재 위치에서 고정된다.
- startup Safe Start 목적으로 `Set_Input_Pos(current)`를 보내지 않는다.

## 패키지별 문서

| 패키지 | 문서 |
|---|---|
| `quattro` | [`packages/quattro.md`](packages/quattro.md) |
| `quattro_description` | [`packages/quattro_description.md`](packages/quattro_description.md) |
| `quattro_bringup` | [`packages/quattro_bringup.md`](packages/quattro_bringup.md) |
| `quattro_gazebo` | [`packages/quattro_gazebo.md`](packages/quattro_gazebo.md) |
| `quattro_sensors` | [`packages/quattro_sensors.md`](packages/quattro_sensors.md) |
| `quattro_teleop` | [`packages/quattro_teleop.md`](packages/quattro_teleop.md) |
| `gim6010_driver` | [`packages/gim6010_driver.md`](packages/gim6010_driver.md) |
| `quattro_hardware` | [`packages/quattro_hardware.md`](packages/quattro_hardware.md) |

## 실행 환경

- Docker/X11/GPU/Git: [`development_environment.md`](development_environment.md)
- Gazebo: [`gazebo.md`](gazebo.md)
- 제조사 매뉴얼: `GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf`

## 실기 bringup

```bash
ros2 launch quattro_bringup hardware.launch.py
```

실행 결과는 다음 상태여야 한다.

```text
QuattroSystem           ACTIVE
GIM6010-8 x12           CLOSED LOOP
Encoder                 post-Closed-Loop synchronized
joint_state_broadcaster ACTIVE
joint_trajectory_controller ACTIVE
Gait                    OFF
Robot                   READY / HOLD
```

Gait와 초기 자세 이동은 hardware bringup과 분리되어 있다. gait controller 프로세스는 `use_gait:=true`일 때만 시작된다.

`READY` 판정은 `bringup_manager`가 로그로 명시한다. launch 파일이 끝까지 진행되는 것 자체는 READY를 뜻하지 않는다.

## 기본 검증

```bash
ros2 control list_hardware_components
ros2 control list_controllers
ros2 topic hz /joint_states
ip -details -statistics link show can0
ip -details -statistics link show can1
```

## 하드웨어 도구

```bash
ros2 run quattro_hardware calibration_gui --calibration-file <path>
ros2 run quattro_hardware position_control_tuning_gui --calibration-file <path>
```

두 GUI와 `hardware.launch.py`는 같은 SocketCAN 인터페이스를 소유하므로 동시에 실행하지 않는다.
