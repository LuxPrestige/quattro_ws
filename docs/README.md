# Quattro 문서 색인

이 디렉터리는 `AGENTS.md`에서 분리한 세부 설계와 운영 문서를 관리한다.

`AGENTS.md`에는 에이전트가 항상 지켜야 하는 최소 규칙만 두고, 변경 가능성이 높거나 주제별로 긴 내용은 아래 문서에서 관리한다.

## 문서 목록

| 문서 | 내용 |
|---|---|
| `architecture.md` | 패키지 책임, 의존 방향, ROS 인터페이스, SI 단위, REP-103, 이름 규칙 |
| `development_environment.md` | Docker, X11, Raspberry Pi, NVIDIA GPU, 빌드, Git 작업 방식 |
| `gazebo.md` | Gazebo Harmonic, `gz_ros2_control`, launch 흐름, 시뮬레이션 검증 |
| `gim6010_hardware.md` | GIM6010-8 + GDS68, CAN 매핑, CAN Simple/MIT 계층, `ros2_control`, 안전 규칙 |
| `calibration.md` | 실제 관절 zero offset 캘리브레이션 절차 |
| `hardware_bringup.md` | 실제 로봇 bringup과 실행 전 안전 확인 |
| `development_status.md` | 현재 구현 상태, 미검증 항목, 다음 개발 순서 |
| `GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` | 제조사 매뉴얼 한국어 번역본 |

## 문서 작성 원칙

- 현재 동작을 설명하는 내용은 실제 코드와 일치시킨다.
- 구현 예정 사항과 현재 구현 사항을 구분한다.
- 실행 절차는 한 문서에만 authoritative하게 유지하고 다른 문서에서는 링크한다.
- 장치별 캘리브레이션 값, `.env`, 로그 등 머신별 데이터는 문서에 실제 값으로 고정하지 않는다.
- 명령 예시는 기본적으로 Docker 컨테이너 내부 `/ws` 기준으로 작성한다.
- 하드웨어 수치와 CAN 프로토콜은 제조사 매뉴얼 또는 실제 검증된 구현을 기준으로 한다.

## 코드와 문서가 다를 때

문서가 코드보다 오래된 경우가 있을 수 있다. 작업할 때는 다음 순서로 확인한다.

1. 관련 소스 코드와 설정 파일 확인
2. 실제 하드웨어 사양 또는 제조사 매뉴얼 확인
3. 동작 또는 시험 결과 확인
4. 코드 변경과 함께 관련 문서 갱신

과거 개발 계획을 현재 사실처럼 유지하지 않는다.
