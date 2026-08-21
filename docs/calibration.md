# Quattro 관절 캘리브레이션

## 상태

`calibration_gui`는 `quattro_hardware`에 구현되어 있다(`docs/packages/quattro_hardware.md` 5절). 실기 CAN bus·모터 연동은 이 개발 환경에 실물 CAN이 없어 아직 검증되지 않았으므로, 이 문서의 실행 명령은 실기 검증 전 목표 절차로 읽는다 — 좌표 변환 식과 안전 절차는 코드와 함께 이미 구현되어 있다.

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

하드웨어 구조와 CAN 매핑은 `docs/packages/quattro_hardware.md`(0절)를 먼저 확인한다.

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

## GUI 제어 설정

GUI는 runtime bringup과 동일하게 Direct Position 모드를 사용한다. 활성화 전에 YAML의 공통 current limit과 position/velocity gain을 해당 모터에 적용한다. rotor velocity limit은 캘리브레이션 전용으로 다음 값을 사용한다.

```text
rotor velocity limit: 5.0 rev/s
```

GUI는 설정을 장치 flash에 저장하지 않는다. 기존 fault 원인을 보존하기 위해 enable 전에 자동으로 오류를 삭제하지도 않는다.

## 저장 내용

GUI는 화면의 target만 사용하지 않고 최신 encoder feedback을 다시 읽어 offset을 계산한다.

저장 예:

```yaml
direct_position:
  current_limit: 5.0
  position_gain: 20.0
  velocity_gain: 0.16
  velocity_integrator_gain: 0.32

joints:
  front_left_hip_joint:
    can_interface: can0
    can_id: 0
    direction: -1
    offset: 0.123456
```

CAN ID, bus, direction은 기준 매핑과 달라지면 GUI가 설정을 거부한다.

`direct_position` 값은 12개 모터에 공통 적용된다. `calibration_gui`와 runtime 모두 `0x009` rotor estimate, 같은 direction/offset 변환 및 `Set_Input_Pos` 명령을 사용한다.

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

실제 모터 bringup은 `docs/packages/quattro_bringup.md`를 따른다.
