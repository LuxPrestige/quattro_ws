# Quattro 관절 캘리브레이션

캘리브레이션은 `quattro_hardware`에서 적용한다. 보행 생성기와 역기구학은
캘리브레이션이 적용되지 않은 ROS 관절 좌표를 rad 단위로 사용한다.

머신별 캘리브레이션 파일은 다음 위치에 있다.

```text
/ws/src/quattro_bringup/config/calibration.yaml
```

이 파일은 Git에서 제외된다. 버전 관리되는
`calibration.yaml.example`을 초기 템플릿으로 사용한다. 관절 방향은
`lgh_ws-old`의 GIM 캘리브레이션 설정과 동일하다.

CAN 인터페이스는 다음과 같이 구분한다.

```text
CAN ID 0~5  → can0
CAN ID 6~11 → can1
```

GUI는 이 규칙을 위반하는 캘리브레이션 파일을 거부한다.

## 안전 요구사항

캘리브레이션 전에 다음 사항을 확인한다.

1. 로봇을 작업대 등에 안전하게 지지한다.
2. 선택할 관절에 하중이 걸리지 않게 한다.
3. `controller_manager`를 종료한다.
4. CAN 명령을 전송하는 다른 프로그램을 모두 종료한다.
5. 전체 활성화 전에 12개 모터 모두에서 encoder feedback을 받을 수 있는지 확인한다.
6. 전체 활성화 후에도 한 번에 선택한 관절 하나만 조정한다.
7. 오류가 발생하면 `Disable All Motors`를 누르고 전원을 차단한다.

## 빌드 및 실행

Docker 개발 컨테이너 안에서 실행한다.

Raspberry Pi에 WSL 또는 Linux PC에서 원격 접속해 GUI를 표시하려면 VS Code의
Remote SSH 터미널이 아니라 로컬 터미널에서 X11 forwarding으로 접속한다.

```bash
ssh -Y <사용자>@<라즈베리파이_IP>
echo "$DISPLAY"
```

`DISPLAY`에 `localhost:10.0`과 같은 값이 출력되면 X11 override를 적용해
컨테이너를 생성한다. 이 override는 SSH가 만든 Xauthority 쿠키를 컨테이너에
읽기 전용으로 전달한다.

```bash
cd ~/lgh_ws
docker compose -f compose.yaml -f compose.x11.yaml up -d --force-recreate dev
docker compose -f compose.yaml -f compose.x11.yaml exec dev bash
```

Raspberry Pi에서는 기본 `compose.yaml`에 NVIDIA GPU 설정을 적용하지 않는다.
NVIDIA GPU가 장착된 데스크톱에서만 다음과 같이 NVIDIA override를 추가한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.nvidia.yaml \
  up -d --force-recreate dev
```

컨테이너 안에서는 다음 명령으로 빌드하고 실행한다.

```bash
cd /ws

colcon build \
  --symlink-install \
  --packages-select gim6010_driver quattro_hardware

source /ws/install/setup.bash

ros2 run quattro_hardware calibration_gui \
  --calibration-file /ws/src/quattro_bringup/config/calibration.yaml
```

GUI가 열리지 않고 `could not connect to display` 오류가 발생하면 Docker의
`DISPLAY`와 Xauthority 전달 상태를 확인한다.

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
ls -l "$XAUTHORITY"
```

원격 GUI 컨테이너를 VS Code Remote SSH 터미널에서 재생성하면 `DISPLAY`가
비어 있으므로, 반드시 `ssh -Y`로 연결한 로컬 터미널에서 재생성한다.

한글이 네모 또는 깨진 문자로 표시되면 이전 Docker 이미지가 실행 중인
것이다. `fonts-noto-cjk`가 포함된 이미지를 다시 빌드한 후 컨테이너를
재생성한다.

```bash
cd ~/lgh_ws
docker compose build dev
docker compose -f compose.yaml -f compose.x11.yaml up -d --force-recreate dev
```

## GUI 사용 순서

관절 버튼에는 관절 이름과 번호만 표시된다. 관절을 선택하면 별도 정보
영역에 `calibration.yaml`에서 불러온 기존 offset을 rad와 degree로 표시한다.

1. `Enable All Motors` 버튼을 누른다.
2. 안전 확인 창에서 전체 활성화를 승인한다.
3. 12개 모터가 현재 위치를 유지하는지 확인한다.
4. 관절 선택 버튼을 누른다.
5. `-1 deg` 또는 `+1 deg` 버튼으로 선택한 관절의 영점을 맞춘다.
6. `Save Current Position as Zero` 버튼을 누른다.
7. 다음 관절을 선택하여 같은 작업을 반복하고, 완료 후 `Disable All Motors`를 누른다.

`Enable All Motors`는 각 모터의 현재 encoder 위치를 먼저 읽고 hold 명령을
준비한 다음 모터를 순차적으로 활성화한다. 관절을 선택해도 나머지 11개 모터는
현재 목표 위치를 유지한다. `-1 deg` 또는 `+1 deg`를 누를 때는 선택한 모터의
목표만 1도씩 변경된다. 저장할 때는 화면의 목표값만 사용하지 않고 마지막 실제
encoder feedback으로 offset을 다시 계산하며, 전체 활성 상태는 다음 관절을
조정할 수 있도록 유지된다.

관절 선택 버튼에는 다음과 같이 관절 이름과 번호가 함께 표시된다.

```text
front_left_hip_joint (joint 0)
front_left_upper_leg_joint (joint 1)
front_left_lower_leg_joint (joint 2)
...
back_right_lower_leg_joint (joint 11)
```

관절 번호는 CAN ID와 같다. 전체 활성 모드에서는 다른 관절을 선택해도 기존
모터를 비활성화하지 않는다. `Disable All Motors`, 창 닫기 또는 오류 발생
시에는 두 CAN 버스의 모든 모터를 비활성화한다.

## Offset 저장

`현재 자세를 0도로 저장`을 누르면 다음 식으로 offset을 계산한다.

```text
offset = direction × 현재 motor position
```

offset은 rad 단위로 머신별 `calibration.yaml`에 저장된다. 저장된 파일을
사용하여 Xacro를 변환하려면 다음과 같이 지정한다.

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  calibration_file:=/ws/src/quattro_bringup/config/calibration.yaml
```

`quattro_description`에 포함된 zero-offset 파일은 시각화 및 URDF 검사용
기본값이다. 캘리브레이션된 실제 로봇에는 머신별 파일을 사용해야 한다.
