# `quattro_controllers`

## 역할

Quattro 전용 `ros2_control` 컨트롤러 플러그인 패키지(`ament_cmake`, C++). GIM6010 MIT command mode를 위한 `MitTrajectoryController` 하나를 제공한다. `quattro_hardware`(하드웨어 인터페이스)와 `quattro`(gait/trajectory 생성)의 사이에 위치하며, 두 계층 어디에도 속하지 않는 "MIT 5-interface 동시 claim + 보간" 책임만 가진다.

```text
src/quattro_controllers/
├── include/quattro_controllers/mit_trajectory_controller.hpp
├── src/mit_trajectory_controller.cpp
├── quattro_controllers.xml   # pluginlib 플러그인 설명
└── CMakeLists.txt / package.xml
```

## 왜 표준 `JointTrajectoryController`를 쓰지 않는가

표준 `joint_trajectory_controller/JointTrajectoryController`는 관절당 `position` 하나만 command interface로 다루도록 설계되어 있어, MIT 모드가 요구하는 `position`/`velocity`/`kp`/`kd`/`effort` 5개 interface를 동시에 claim할 수 없다. `MitTrajectoryController`는 `command_interface_configuration()`에서 관절마다 `<joint>/position`, `<joint>/velocity`, `<joint>/kp`, `<joint>/kd`, `<joint>/effort` 5개를 모두 요구한다. `ros2_control`은 컨트롤러가 요구한 interface를 모두 확보하지 못하면 활성화를 거부하므로, 결과적으로 `position` interface 하나만 가진 표준 컨트롤러로 MIT 축을 부분적으로만 claim하는 구성은 애초에 성립하지 않는다(`docs/packages/quattro_hardware.md` 3절).

## `MitTrajectoryController`

**State interface**: 관절당 `position`, `velocity`.

**Command interface**: 관절당 `position`, `velocity`, `kp`, `kd`, `effort`(feed-forward torque).

**파라미터**

| 파라미터 | 설명 |
|---|---|
| `joints` | 제어할 관절 이름 목록(순서가 곧 command 순서) |
| `kp`, `kd` | 관절별 MIT hold gain. `kp ∈ [0, 500]`, `kd ∈ [0, 5]` 범위를 벗어나면 `on_configure`가 즉시 실패한다(GIM6010 MIT 프레임의 12-bit 필드 범위, `docs/GIM6010-8 메뉴얼...pdf` 4.1.2절 `Mit_Control` 참고) |
| `command_timeout` | 마지막 trajectory point 이후 이 시간(초)이 지나면 현재 목표 궤적을 버리고 마지막 위치를 유지 |

**동작**

- `~/joint_trajectory` (`trajectory_msgs/JointTrajectory`)를 구독한다. 수신 즉시 실시간 스레드가 아닌 콜백에서 `normalizeTrajectory`로 검증·재정렬(관절 이름 집합 일치, 포인트 시간 단조증가, 위치 유한값)하고 실패하면 통째로 거부·로그만 남긴다. 통과한 궤적만 `RealtimeBuffer`로 실시간 `update()` 스레드에 넘긴다.
- `update()`는 활성 궤적의 인접 두 포인트 사이를 선형 보간해 매 제어 주기 목표 `position`을 만들고, `velocity`는 항상 `0.0`으로 command한다(속도 feed-forward를 아직 사용하지 않음). `kp`/`kd`는 파라미터 값을 그대로 매 주기 다시 쓴다. `effort`(torque feed-forward)는 항상 `0.0`.
- `on_activate`: 현재 `position` state를 읽어 그 자리에서 hold하는 command를 먼저 한 번 써서 활성화 순간 급격한 이동을 방지한다.
- `on_deactivate`: `kp`를 일시적으로 0으로 낮춰 마지막 위치를 write한 뒤 원래 `kp`를 복원한다 — 비활성화 순간 모터가 급하게 그 자리에서 뻣뻣하게 버티지 않도록 하는 안전 조치.
- 활성 궤적이 없거나(초기 상태) 실행이 끝난 뒤 `command_timeout`을 넘기면 마지막 위치를 계속 hold command로 유지한다(느슨하게 풀리거나 정지 명령이 없어 방치되지 않도록).

## 등록

`pluginlib` + `quattro_controllers.xml`로 `controller_interface::ControllerInterface`를 구현하는 `quattro_controllers/MitTrajectoryController`를 노출한다. `quattro_bringup/config/hardware_controllers_mit.yaml`이 `controller_manager` 설정에서 이 타입 이름으로 로드한다.

## 관련 문서

- 실제 command interface 계약의 반대편(하드웨어 인터페이스): `docs/packages/quattro_hardware.md`
- CAN/MIT 프로토콜: `docs/packages/gim6010_driver.md`
- launch/파라미터 사용법: `docs/packages/quattro_bringup.md`
