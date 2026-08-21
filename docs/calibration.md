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
3. 브로드캐스트로 도착한 최신 encoder 위치(100ms 이내)를 읽고 해당 위치를 hold한다.
4. `-1 deg`, `+1 deg`로 joint zero를 맞춘다.
5. `Save Current Position as Zero`를 누른다.
6. 저장 후 선택 모터는 비활성화된다.

**다른 관절을 선택해도 기존에 활성화된 모터는 계속 hold를 유지한다** — 드롭다운 선택은 `-1 deg`/`+1 deg`/`Enable`/`Disable Selected Motor`가 어느 관절에 적용될지만 바꾸고, 다른 관절의 활성 상태에는 영향을 주지 않는다. 인접 관절을 기준점으로 고정해두고 다른 관절을 조정하는 등, 여러 관절을 하나씩 켜서 동시에 hold하고 싶을 때 이 방식을 그대로 반복해 쓰면 된다. 전부 끄고 싶으면 `Disable All Motors`를 누른다(어떤 방식으로 켰든 전부 idle로 전환한다).

### 12개 모터 전체 hold

1. `Enable All Motors`를 누른다.
2. 12개 모터의 현재 encoder 위치를 읽는다(각 모터의 최신 브로드캐스트 값).
3. 각 모터를 현재 위치에서 hold한다.
4. 관절 하나를 선택한다.
5. 선택한 관절만 `-1 deg`, `+1 deg`로 조정한다.
6. offset을 저장하고 다음 관절을 선택한다.
7. 모든 작업 후 `Disable All Motors`를 누른다.

전체 활성 모드에서는 나머지 11개 모터가 기존 목표 위치를 유지한다.

### `Move to Saved Zero` — 기존 캘리브레이션 검증/미세조정

위 두 방식은 "관절을 손으로 원하는 위치에 놓고 그 자리를 새 zero로 저장"하는, 아직 zero를 모르는 상태에서 처음 잡을 때 쓰는 흐름이다. 이미 `calibration.yaml`에 저장된 zero가 있고, 그 위치가 맞는지 눈으로 확인하거나 살짝만 미세조정하고 싶을 때는 이 버튼을 쓴다.

1. 관절을 선택하고 `Enable Selected Motor`(또는 `Enable All Motors`)로 활성화한다 — 이 시점엔 현재 위치를 그대로 hold한다.
2. `Move to Saved Zero`를 누른다 — 현재 화면이 들고 있는(파일에서 로드됐거나 `Reload Calibration from File`로 새로고침한) 그 관절의 저장된 offset을 이용해, **모터를 실제로 그 저장된 zero 위치까지 이동시킨다**(제자리 hold가 아니라 실제 이동).
3. 도착한 위치에서 `-1 deg`, `+1 deg`로 필요한 만큼만 미세조정한다.
4. 결과가 맞으면 `Save Current Position as Zero`로 새 offset을 저장한다(안 맞으면 그냥 `Disable Selected Motor`로 끄면 파일은 바뀌지 않는다).

내부적으로 offset은 CAN으로 전송되지 않고 GUI가 목표각을 계산할 때만 쓰인다 — `Move to Saved Zero`는 저장된 offset을 이번 활성화 세션의 목표각으로 변환해 `Set_Input_Pos`를 보내는 것뿐이다.

### `Live feedback` — 현재 위치 실시간 표시

창 아래 `Live feedback` 그룹은 **선택된 관절의 axis state와 현재 위치를 50ms마다 갱신해 보여준다.**

맨 위 색상 표시줄은 axis state다 — **`CLOSED LOOP`이면 초록 배경, `IDLE`이면 빨강 배경**(둘 다 흰 글자). 색은 Qt 스타일시트로만 칠하며 별도 이미지 리소스를 쓰지 않는다. 이 색은 **모터가 보내는 `Heartbeat(0x01)`의 `axis_state` 실측값**을 따르지, GUI가 "Enable을 눌렀다"고 기억하는 상태를 따르지 않는다 — fault로 closed loop에서 떨어져 나간 모터는 GUI 입장에선 여전히 enabled지만 실제로는 힘이 빠져 있고, 그 경우를 잡아내는 것이 이 표시의 목적이기 때문이다.

- 초록 `CLOSED LOOP` — 모터가 실제로 closed-loop 제어 중(위치를 잡고 있다).
- 빨강 `IDLE` — 모터가 idle. 손으로 돌릴 수 있는 상태다.
- 회색 `NO HEARTBEAT` — 400ms 이상 heartbeat가 없다. "모터가 idle이라고 말한 것"과 "모터가 아예 응답하지 않는 것"은 다른 문제라서 빨강으로 합치지 않는다.
- 회색 `AXIS STATE n` — calibration 등 위 둘 중 어느 쪽도 아닌 상태.
- `axis_error`가 0이 아니면 상태 뒤에 `fault 0x........`가 붙는다.

그 아래는 위치 표시다. 모터가 `Get_Encoder_Estimates(0x09)`를 자동으로 브로드캐스트하므로(`docs/packages/gim6010_driver.md` 0절) GUI는 요청 없이 받은 값을 그대로 표시한다. 세 가지 좌표계를 함께 보여주는데, 관절이 캘리브레이션되기 전에는 이 셋이 서로 다르기 때문이다.

```text
CLOSED LOOP
front_left_hip_joint  (node 0 on can0)
saved     -1.984 deg
session    0.016 deg   target     0.000 deg   error     0.016 deg
motor   -0.038712 rev      0.00 deg/s
```

- `saved` — `calibration.yaml`에 저장된 offset을 적용한 각도. **저장된 zero에서 0이 되는 값**이므로, 기존 영점을 확인하거나 `Move to Saved Zero` 결과를 검증할 때 보는 숫자다.
- `session` — 모터를 활성화한 순간을 0으로 잡은 각도. `-1 deg`/`+1 deg` 조깅의 `target`이 사는 좌표계와 같다. `error`는 `session - target`으로, 모터가 지시한 목표를 실제로 따라갔는지 본다(GUI가 이 관절을 켜지 않은 상태에서는 목표가 의미 없으므로 `idle`로 표시된다).
- `motor` — 버스에서 온 로터 원값(rev)과 속도(관절 환산 deg/s). `candump`로 직접 본 값과 대조할 때 쓴다.

feedback이 100ms 이상 끊기면 `no feedback`으로 표시된다 — 해당 모터에 `0x09` 자동 송신 설정이 안 들어갔거나 버스에서 이탈했다는 뜻이다.

### `Reload Calibration from File`

`calibration.yaml`을 다시 읽어 공통 `current_limit`/`position_gain`/`velocity_gain`/`velocity_integrator_gain`과 12관절의 저장된 offset을 화면(GUI 메모리) 상태에 반영한다.

- 이미 활성화(hold)된 모터가 있으면, 그 모터들에는 새로 읽은 `current_limit`/gain 값을 즉시 다시 전송한다(`Set_Limits`/`Set_Pos_Gain`/`Set_Vel_Gains`) — idle로 껐다가 다시 켤 필요 없이, hold 중인 위치를 그대로 유지한 채로 새 게인이 적용된다.
- offset은 CAN으로 전송하지 않는다(GUI는 활성화된 모터를 항상 "활성화된 순간" 기준 상대 위치로 제어하므로, session 중에는 offset이 관여하지 않는다) — 화면에 새로 반영된 offset은 이후 `Save Current Position as Zero`를 눌렀을 때의 계산과 상태 라벨 표시에만 쓰인다.
- 주 용도: `direct_position_tuning_gui`로 게인을 튜닝해 YAML에 저장한 뒤, `calibration_gui`를 재시작하지 않고 이미 켜둔 모터에 새 게인을 바로 적용해 확인할 때. 또는 다른 프로세스/사람이 파일을 수정했을 때 이 GUI가 들고 있는 값을 동기화할 때.
- 파일 로드가 실패(YAML 형식 오류, 기준 매핑 불일치 등)하면 기존 화면 값은 그대로 유지되고 오류 메시지만 뜬다.

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
