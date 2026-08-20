# AGENTS.md

## 프로젝트 범위

이 저장소는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어 워크스페이스이다.

고정 개발 환경과 하드웨어 기준은 다음과 같다.

- 워크스페이스: `quattro_ws`
- ROS 2: Jazzy
- OS: Ubuntu 24.04 계열
- 기본 실행 환경: Docker, 컨테이너 내부 `/ws`
- SBC: Raspberry Pi 5
- 액추에이터: SteadyWin GIM6010-8 × 12
- 드라이버: GDS68
- 모터 통신: Linux SocketCAN / CAN Simple / Direct Position
- 기본 CAN bitrate: `500000`
- IMU: BNO085
- 시뮬레이션: Gazebo Harmonic + `gz_ros2_control`

세부 설계와 실행 절차는 이 파일에 중복해서 기록하지 않는다. 작업 전 관련 `docs/` 문서를 읽는다.

## 필수 개발 원칙

1. 기존 파일을 먼저 읽고 현재 구현을 확인한 뒤 수정한다.
2. 사용자의 명시적 요청 없이 기존 파일을 전면 재작성하지 않는다.
3. ROS 2 개발, 빌드, 테스트는 기본적으로 Docker 컨테이너 내부에서 수행한다.
4. ROS 2 표준 메시지와 `ros2_control` 표준 인터페이스를 우선 사용한다.
5. 커스텀 메시지는 표준 인터페이스로 표현할 수 없는 경우에만 추가한다.
6. ROS 외부 인터페이스는 SI 단위를 사용하고 REP-103 좌표계를 따른다.
7. 패키지 책임을 넘는 코드를 넣지 않는다. 특히 CAN 프로토콜과 상위 보행 로직을 분리한다.
8. 하드웨어 제어 코드는 안전 동작, 입력 검증, timeout, stale feedback, fault 처리를 우선한다.
9. 잘못된 설정이나 명령을 무조건 clamp해서 숨기지 말고 거부하거나 명확히 보고한다.
10. 소스 코드 주석에는 이모지를 사용하지 않는다.

## 패키지 경계

| 패키지 | 책임 | 넣지 않는 내용 |
|---|---|---|
| `quattro` | FK/IK, gait, 자세·balance 제어, 상위 상태 | raw CAN, GDS68 패킷 |
| `quattro_description` | URDF/Xacro, mesh, TF, inertial/collision, `ros2_control` description | 제어 알고리즘, 드라이버 구현 |
| `quattro_bringup` | launch, controller 구성, 실제 시스템 실행 조합 | 드라이버·알고리즘 구현 |
| `quattro_gazebo` | Gazebo world, simulation launch, simulation controller | 실제 하드웨어 제어 |
| `quattro_hardware` | `hardware_interface::SystemInterface` 구현(`QuattroSystem`, Direct Position 제어), joint direction/offset/limit 변환, 활성화 안전 절차 | raw CAN payload 직접 생성, gait/IK |
| `gim6010_driver` | SocketCAN 송수신, CAN Simple/MIT encode·decode, GIM6010 모터 상태 추상화. MIT encode/decode는 `calibration_gui`의 관절 영점 조깅 절차가 사용한다 | Quattro joint 이름, URDF, ROS 의존 |
| `quattro_sensors` | BNO085 등 센서 취득과 표준 ROS 메시지 발행 | balance 제어 |
| `quattro_teleop` | joystick/keyboard 입력과 상위 명령 생성 | 모터 직접 제어 |

`gim6010_driver`, `quattro_hardware` 모두 구현됨(`docs/packages/gim6010_driver.md`, `docs/packages/quattro_hardware.md`). `quattro_hardware`는 `gim6010_driver`를 라이브러리로 직접 링크하는 `QuattroSystem`(`hardware_interface::SystemInterface`)과 `calibration_gui`로 구성된다.

의존 방향은 다음을 유지한다.

```text
quattro
  -> ROS 2 controller interface
  -> quattro_hardware
  -> gim6010_driver
  -> SocketCAN
  -> GIM6010-8 / GDS68
```

`gim6010_driver`가 `quattro_hardware` 또는 `quattro`에 의존해서는 안 된다.


CAN ID 기준은 `0~5 -> can0`, `6~11 -> can1`이다. 상세 매핑은 `docs/packages/quattro_hardware.md`(0절)를 따른다.

## 코딩 규칙

### Python

- Python 3
- ROS 노드는 class 기반으로 작성한다.
- 가능한 경우 type hint를 사용한다.
- PEP 8을 따른다.

### C++

- 기본 C++17
- RAII를 사용한다.
- 소유권을 가진 raw pointer를 지양한다.
- ROS 계층과 CAN/프로토콜 계층을 분리한다.
- 패킷 encode/decode 함수는 ROS 없이 unit test 가능하게 작성한다.

### ROS 2

- parameter는 사용 전에 선언한다.
- 로봇 설정값은 가능한 YAML/Xacro parameter로 분리한다.
- 머신별 절대 경로를 코드에 하드코딩하지 않는다.
- package resource는 package share 경로를 사용한다.

## 변경 후 최소 검증

변경 범위에 맞는 검증을 수행하고 실패를 숨기지 않는다.

- C++/Python 패키지: 관련 package build 및 test
- URDF/Xacro: `xacro` 변환 후 `check_urdf`
- Gazebo: Xacro 검사, SDF 검사, headless launch
- 하드웨어 코드: unit test와 정적 검증을 우선하고, 실제 모터 enable은 사용자가 요청한 시험 범위에서만 수행한다.

기본 빌드 예시는 다음과 같다.

```bash
cd /ws
colcon build --symlink-install --event-handlers console_direct+
```

## 문서 라우팅

작업 주제에 따라 다음 문서를 먼저 읽는다.

| 주제 | 문서 |
|---|---|
| 문서 전체 색인 | `docs/README.md` |
| 패키지 구조, ROS 인터페이스, 좌표계·이름 규칙 | `docs/architecture.md` |
| 패키지별 세부 구현(8개 패키지 모두 구현됨) | `docs/packages/*.md` |
| Docker, X11, GPU, 빌드, Git 개발 환경 | `docs/development_environment.md` |
| Gazebo 시뮬레이션 | `docs/gazebo.md` |
| GIM6010-8 / GDS68 / CAN / ros2_control 하드웨어 구조 | `docs/packages/gim6010_driver.md`, `docs/packages/quattro_hardware.md`(모두 구현됨, 실기 미검증) |
| 관절 영점 캘리브레이션 | `docs/calibration.md` |
| 실제 로봇 실행 | `docs/packages/quattro_bringup.md` |
| 구현 상태와 다음 작업 | `docs/development_status.md` |
| 제조사 번역 매뉴얼 | `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` |
| gim6010_driver/quattro_hardware 재설계 시 참고한 레퍼런스 프로젝트 | `docs/ros_odrive/`, `docs/Steadywin-RS485-CAN-Connector/` (의존성 아님, 아키텍처 참고용) |

문서 내용과 코드가 충돌하면 **현재 코드와 실제 하드웨어 사양을 확인한 뒤 문서도 함께 수정**한다.
