# AGENTS.md

## 프로젝트 범위

이 저장소는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어 워크스페이스이다.

- 워크스페이스: `quattro_ws`
- OS: Ubuntu 24.04 계열
- SBC: Raspberry Pi 5
- 액추에이터: SteadyWin GIM6010-8 × 12
- 드라이버: GDS68
- 통신: Linux SocketCAN / CAN Simple / 500000 bit/s
- 실기 위치 제어: `Position Control + Pos Filter`
- CAN 매핑: `0~5 -> can0`, `6~11 -> can1`
- IMU: BNO085
- 시뮬레이션: Gazebo Harmonic + `gz_ros2_control`

세부 설계와 실행 절차는 관련 `docs/` 문서를 1차 출처로 사용한다.

## 필수 개발 원칙

1. 기존 구현과 문서를 먼저 읽고 수정한다.
2. ROS 2 표준 메시지와 `ros2_control` 표준 인터페이스를 우선한다.
3. CAN 프로토콜, `ros2_control` 하드웨어 계층, 상위 보행 로직의 책임을 섞지 않는다.
4. ROS 외부 인터페이스는 SI 단위와 REP-103을 따른다.
5. 하드웨어 코드는 timeout, stale feedback, CAN fault, axis fault, safe stop을 우선한다.
6. 잘못된 명령이나 설정을 무조건 clamp해서 숨기지 않는다.
7. 소스 코드 주석에는 이모지를 사용하지 않는다.

## GIM6010-8 실기 확인 하드웨어 계약

다음 항목은 Quattro의 startup 로직에서 반드시 지켜야 하는 실기 확인 사항이다.

1. 설정 순서는 `Set_Limits -> Set_Pos_Gain -> Set_Vel_Gains -> Position Control + Pos Filter -> Closed Loop Control`이다.
2. **Closed Loop 이전의 EncoderEstimate 위치값은 신뢰하지 않는다.** startup 초기 위치나 hold target으로 사용하지 않는다.
3. `Get_Encoder_Estimates(0x009)`는 모터가 약 10 ms 주기로 자동 송신한다. Quattro runtime, bringup, calibration GUI는 정상 경로에서 encoder request를 보내지 않는다.
4. 위치값은 **Closed Loop가 확인된 이후 도착한 새 EncoderEstimate**만 유효한 startup 위치로 사용한다.
5. `Position Control + Pos Filter` 상태에서 Closed Loop에 들어가면 별도 `Set_Input_Pos` 없이도 모터 축이 현재 위치를 유지한다.
6. 따라서 startup 과정에서 현재 위치를 얻기 위한 `Set_Input_Pos(current)` 명령을 보내지 않는다.
7. Closed Loop 이후 encoder를 ROS joint 좌표로 변환하여 `state.position`과 첫 command 기준을 동기화한 뒤 상위 controller에 제어권을 넘긴다.
8. `Heartbeat.axis_error`와 `axis_state`를 활성화/런타임 안전 판정에 사용한다. 실기에서 응답하지 않는 `Get_Error(0x03)`에 startup 안전 판정을 의존하지 않는다.

이 계약과 충돌하던 과거의 Direct Position safe-start 구현, Closed Loop 전 encoder 사용, startup `Set_Input_Pos(current)` 로직은 모두 제거되었다. 다시 도입하지 않는다.

실기 제어 설정 키는 `position_control`이며 `direct_position`은 존재하지 않는다.

## 패키지 경계

| 패키지 | 책임 |
|---|---|
| `quattro` | FK/IK, gait, balance, 상위 trajectory 생성 |
| `quattro_description` | URDF/Xacro, mesh, TF, `ros2_control` description |
| `quattro_bringup` | launch, controller 구성, startup orchestration |
| `quattro_gazebo` | Gazebo simulation |
| `quattro_hardware` | `QuattroSystem`, joint 변환, GIM6010 lifecycle 및 안전 제어, calibration/tuning GUI |
| `gim6010_driver` | SocketCAN, CAN Simple/MIT encode/decode, 모터 상태 캐시 |
| `quattro_sensors` | 센서 취득 및 표준 ROS 메시지 발행 |
| `quattro_teleop` | joystick/keyboard 입력을 상위 명령으로 변환 |

의존 방향:

```text
quattro
  -> ros2_control controller
  -> quattro_hardware
  -> gim6010_driver
  -> SocketCAN
  -> GIM6010-8 / GDS68
```

`gim6010_driver`는 Quattro joint 이름, URDF, gait를 알지 않는다.

## 변경 후 최소 검증

```bash
cd /quattro_ws
colcon build --symlink-install --event-handlers console_direct+
colcon test --event-handlers console_direct+
```

URDF/Xacro 변경 시:

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

실기 startup 변경 시 단일 모터 시험을 먼저 수행하고, CAN 로그에서 반드시 다음을 확인한다.

```text
Set_Limits
Set_Pos_Gain
Set_Vel_Gains
Set_Controller_Mode(Position, PosFilter)
Set_Axis_State(ClosedLoop)
post-Closed-Loop EncoderEstimate
```

Closed Loop 이전 EncoderEstimate가 startup 위치로 사용되거나, startup 과정에서 `Set_Input_Pos(current)`가 전송되면 실패로 본다.

## 문서 라우팅

- 전체 구조: `docs/architecture.md`
- GIM6010 드라이버: `docs/packages/gim6010_driver.md`
- ros2_control 하드웨어: `docs/packages/quattro_hardware.md`
- 실기 bringup: `docs/packages/quattro_bringup.md`
- 캘리브레이션: `docs/calibration.md`
- 현재 구현/실기 상태: `docs/development_status.md`
- 제조사 매뉴얼: `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf`

문서와 코드가 충돌하면 실기에서 확인된 하드웨어 동작을 우선하고 코드와 문서를 함께 갱신한다.
