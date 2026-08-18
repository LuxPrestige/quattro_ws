# Quattro 실기 Bringup

실기 bringup은 모터, BNO085 IMU, Switch Pro Controller, gait controller와
`ros2_control` controller를 함께 실행한다.

## 안전 확인

실행 전에 다음 조건을 모두 확인한다.

1. 로봇을 지지대에 올려 다리에 하중이 걸리지 않게 한다.
2. 캘리브레이션 GUI와 다른 CAN 송신 프로그램을 모두 종료한다.
3. `can0`과 `can1`이 모두 `500000 bit/s`, `ERROR-ACTIVE`인지 확인한다.
4. 머신별 `calibration.yaml`에 12개 관절의 실제 offset이 저장되어 있는지 확인한다.
5. 비상 전원 차단 수단을 준비한다.

CAN 상태 확인:

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

## 실행

Docker 컨테이너 안에서 실행한다.

```bash
cd /ws
source /opt/ros/jazzy/setup.bash
source /ws/install/setup.bash

ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml
```

기본값으로 IMU와 joystick을 실행한다. 실기 launch에는 RViz2 노드가 포함되지
않는다. 선택적으로 다음 인자를 사용할 수 있다.

```bash
ros2 launch quattro_bringup hardware.launch.py \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  use_imu:=true \
  use_teleop:=true
```

## 시작 동작

`controller_manager`는 `QuattroSystem`을 inactive 상태로 로드한다. 이어서
`hardware_spawner --activate QuattroSystem`이 실제 encoder 위치를 읽는다.
12개 모터를 각자의 최신 위치에서 Kp=0으로 활성화한 뒤 위치를 다시 읽는다.
현재 위치를 유지하면서 Kp를 1초 동안 설정값까지 올리고, 12개 관절을 동시에
최대 0.5 rad/s로 nominal 초기 위치까지 이동한다. 실제 위치가 목표의 ±0.03 rad
안에 들어왔는지 확인하며, 한 관절이라도 30초 안에 도달하지 못하면 전체
모터를 비활성화한다. 12축 동시 초기화가 끝난 다음
`joint_state_broadcaster`와 `joint_trajectory_controller`가 차례로 활성화된다.

실기 launch에서는 gait 출력을 활성 상태로 시작한다. controller가 활성화되면
gait controller는 현재 관절 위치에서 nominal stance까지 2초 궤적을 한 번
전송한다. 초기 자세 전환이 끝나면 100 Hz 명령을 시작한다. teleop stepping
모드는 비활성 상태로 시작하며 Switch Pro Controller의 모드 전환 버튼으로
활성화한다.

## 100 Hz 제어 주기

다음 주기는 100 Hz로 설정된다.

- `controller_manager.update_rate`: 100 Hz
- `gait_controller.control_frequency`: 100 Hz
- `joint_trajectory_controller.state_publish_rate`: 100 Hz
- 하드웨어 `read()` / `write()`: controller manager update마다 한 번
- 각 모터의 MIT position command: hardware `write()`에서 100 Hz

설정 파일은
`src/quattro_bringup/config/hardware_controllers.yaml`이다. 실기 launch도
`update_rate=100`을 명시적으로 전달하므로 다른 controller YAML을 지정해도
controller manager 주기는 100 Hz로 유지된다.

실행 후 다음을 확인한다.

```bash
ros2 control list_controllers
ros2 topic hz /joint_states
ros2 topic hz /joint_trajectory_controller/joint_trajectory
ros2 topic hz /imu/data
```

정상 상태:

- `joint_state_broadcaster`: `active`
- `joint_trajectory_controller`: `active`
- `/joint_states`: 약 100 Hz
- 초기 자세 전환 후 joint trajectory: 약 100 Hz
- BNO085 사용 시 `/imu/data`: 약 100 Hz

## 종료

`Ctrl+C`로 launch를 종료한다. `controller_manager`가 종료되면
`QuattroSystem`은 모든 모터에 safe stop을 적용한다. CAN 오류, stale feedback,
command watchdog 또는 잘못된 관절 명령이 감지되어도 전체 모터를 비활성화한다.
