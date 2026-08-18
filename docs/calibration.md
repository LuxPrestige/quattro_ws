# Quattro 관절 캘리브레이션

## 목적

실제 GIM6010-8의 encoder 좌표를 ROS joint zero와 맞추기 위한 머신별 offset을 설정한다.

실제 설정 파일:

```text
/ws/src/quattro_bringup/config/calibration.yaml
```

이 파일은 Git에서 제외한다. 최초에는 다음 템플릿을 복사한다.

```text
src/quattro_bringup/config/calibration.yaml.example
```

하드웨어 구조와 CAN 매핑은 `docs/gim6010_hardware.md`를 먼저 확인한다.

## 좌표 변환

`quattro_hardware`는 다음 식을 사용한다.

```text
joint_position = direction * motor_position - offset
motor_command  = direction * (joint_command + offset)
```

현재 자세를 ROS joint zero로 저장할 때:

```text
offset = direction * current_motor_position
```

단위는 rad이다.

## 안전 조건

캘리브레이션 전에 반드시:

1. 로봇을 지지대에 고정하여 다리에 하중이 걸리지 않게 한다.
2. `controller_manager`를 종료한다.
3. 다른 CAN 송신 프로그램을 모두 종료한다.
4. `can0`, `can1` 상태와 12개 encoder feedback을 확인한다.
5. 즉시 전원을 차단할 수 있는 수단을 준비한다.

캘리브레이션 GUI와 실제 bringup을 동시에 실행하지 않는다.

## 빌드

Docker 컨테이너 내부에서:

```bash
cd /ws
colcon build \
  --symlink-install \
  --packages-select gim6010_driver quattro_hardware
source /ws/install/setup.bash
```

## 실행

```bash
ros2 run quattro_hardware calibration_gui \
  --calibration-file /ws/src/quattro_bringup/config/calibration.yaml
```

원격 GUI/X11 설정은 `docs/development_environment.md`를 따른다.

## GUI 동작 방식

현재 GUI는 두 가지 활성화 방식을 지원한다.

### 선택 모터만 활성화

1. 관절을 선택한다.
2. `Enable Selected Motor`를 누른다.
3. 현재 encoder 위치를 읽고 해당 위치를 hold한다.
4. `-1 deg`, `+1 deg`로 joint zero를 맞춘다.
5. `Save Current Position as Zero`를 누른다.
6. 저장 후 선택 모터는 비활성화된다.

다른 관절을 선택하면 기존 단일 활성 모터는 먼저 비활성화된다.

### 12개 모터 전체 hold

1. `Enable All Motors`를 누른다.
2. 12개 모터의 현재 encoder 위치를 읽는다.
3. 각 모터를 현재 위치에서 hold한다.
4. 관절 하나를 선택한다.
5. 선택한 관절만 `-1 deg`, `+1 deg`로 조정한다.
6. offset을 저장하고 다음 관절을 선택한다.
7. 모든 작업 후 `Disable All Motors`를 누른다.

전체 활성 모드에서는 나머지 11개 모터가 기존 목표 위치를 유지한다.

## 현재 GUI 내부 안전값

현재 `calibration_gui.cpp`는 캘리브레이션용으로 GDS68의 rotor/current limits를 다음 값으로 설정한다.

```text
rotor velocity limit: 5.0 rev/s
current limit : 10.0
```

이 값은 캘리브레이션 툴의 현재 구현값이며 실제 보행용 최종 gain/current 설정을 의미하지 않는다. GUI는 기존 fault 원인을 보존하기 위해 enable 전에 자동으로 오류를 삭제하지 않는다.

## 저장 내용

GUI는 화면의 target만 사용하지 않고 최신 encoder feedback을 다시 읽어 offset을 계산한다.

저장 예:

```yaml
joints:
  front_left_hip_joint:
    can_interface: can0
    can_id: 0
    direction: -1
    offset: 0.123456
    kp: 20.0
    kd: 0.5
```

CAN ID, bus, direction은 기준 매핑과 달라지면 GUI가 설정을 거부한다.

`kp`와 `kd` 필드는 calibration GUI가 명시적으로 사용하는 MIT hold gain이며 일반 Position Control의 position/velocity PI gain이 아니다. runtime 기본 Position Control은 GUI와 동일한 `0x009` rotor estimate를 `2π/8`로 변환한 output position과 같은 direction/offset 식을 사용한다. Secondary encoder 전용 feedback이라고 가정하지 않는다.

## 완료 후 확인

1. 모든 모터를 disable한다.
2. calibration 파일을 다시 연다.
3. 저장한 offset이 반영되었는지 확인한다.
4. Xacro 변환을 검사한다.

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml \
  > /tmp/quattro_calibrated.urdf
check_urdf /tmp/quattro_calibrated.urdf
```

실제 모터 bringup은 `docs/hardware_bringup.md`를 따른다.
