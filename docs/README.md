# Quattro 문서 색인

이 저장소(`lgh_ws`)는 ROS 2 Jazzy 기반 4족 보행 로봇 **Quattro**의 제어 소프트웨어 워크스페이스다. 작업 전 `AGENTS.md`의 문서 라우팅 표를 먼저 확인하고, 아래에서 관련 문서를 읽는다.

## 전체 구조

| 문서 | 내용 |
|---|---|
| [`architecture.md`](architecture.md) | 패키지 책임, 의존 방향, ROS 인터페이스, 좌표계·이름 규칙, 12관절 표준 순서 |
| [`development_status.md`](development_status.md) | 현재 구현된 기능, 미구현/재작성 중인 패키지, 실기 검증이 필요한 이슈 |

## 패키지별 세부 구현

`docs/packages/`에 패키지마다 파일 구조, ROS 토픽/서비스/파라미터, 동작 원칙을 정리했다. `quattro_hardware`는 코드가 없어(재작성 중) 설계 명세 형태다.

| 패키지 | 문서 |
|---|---|
| `quattro` (FK/IK, gait, gait_controller) | [`packages/quattro.md`](packages/quattro.md) |
| `quattro_description` (URDF/Xacro, mesh, RViz) | [`packages/quattro_description.md`](packages/quattro_description.md) |
| `quattro_bringup` (실기 launch, controller 구성) | [`packages/quattro_bringup.md`](packages/quattro_bringup.md) |
| `quattro_controllers` (`MitTrajectoryController`) | [`packages/quattro_controllers.md`](packages/quattro_controllers.md) |
| `quattro_gazebo` (Gazebo Harmonic 시뮬레이션) | [`packages/quattro_gazebo.md`](packages/quattro_gazebo.md) |
| `quattro_sensors` (BNO085 IMU) | [`packages/quattro_sensors.md`](packages/quattro_sensors.md) |
| `quattro_teleop` (joystick/keyboard teleop) | [`packages/quattro_teleop.md`](packages/quattro_teleop.md) |
| `gim6010_driver` (CAN 드라이버, CAN Simple + MIT 전체) | [`packages/gim6010_driver.md`](packages/gim6010_driver.md) |
| `quattro_hardware` (`ros2_control` 하드웨어 계층) — **설계 명세, 코드 없음** | [`packages/quattro_hardware.md`](packages/quattro_hardware.md) |

## 실행 환경

| 문서 | 내용 |
|---|---|
| [`development_environment.md`](development_environment.md) | Docker, X11, GPU, 빌드, Git 개발 환경 |
| [`gazebo.md`](gazebo.md) | Gazebo Harmonic 시뮬레이션 실행 |
| [`calibration.md`](calibration.md) | 관절 영점 캘리브레이션 |

## 하드웨어 참고 자료 (GIM6010-8 / GDS68)

CAN/MIT 프로토콜(구현됨)은 `packages/gim6010_driver.md`, 안전 정책·활성화 절차(설계 명세)는 `packages/quattro_hardware.md`, 실행 절차는 `packages/quattro_bringup.md`가 1차 출처다. 아래는 그 문서들의 근거 자료.

| 자료 | 내용 |
|---|---|
| `GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` | 제조사 번역 매뉴얼 (CAN Simple/MIT/오류 코드 원본) |
| `ros_odrive/`, `Steadywin-RS485-CAN-Connector/` | `gim6010_driver`/`quattro_hardware` 재설계를 위해 아키텍처만 참고한 외부 레퍼런스 프로젝트. 워크스페이스 의존성이 아니다 |

## 문서 규칙

문서 내용과 코드가 충돌하면 현재 코드와 실제 하드웨어 사양을 확인한 뒤 문서도 함께 수정한다(`AGENTS.md`).
