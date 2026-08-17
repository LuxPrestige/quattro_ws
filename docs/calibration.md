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
5. 한 번에 모터 하나만 캘리브레이션한다.

## 빌드 및 실행

Docker 개발 컨테이너 안에서 실행한다.

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
`DISPLAY`, X11 전달 및 Xauthority 설정을 먼저 확인한다.

한글이 네모 또는 깨진 문자로 표시되면 이전 Docker 이미지가 실행 중인
것이다. `fonts-noto-cjk`가 포함된 이미지를 다시 빌드한 후 컨테이너를
재생성한다.

```bash
cd ~/lgh_ws
docker compose build dev
docker compose up -d --force-recreate dev
```

## GUI 사용 순서

관절 버튼에는 관절 이름과 번호만 표시된다. 관절을 선택하면 별도 정보
영역에 `calibration.yaml`에서 불러온 기존 offset을 rad와 degree로 표시한다.

1. 관절 선택 버튼을 누른다.
2. `선택한 모터 활성화` 버튼을 누른다.
3. 안전 확인 창에서 활성화를 승인한다.
4. `-1도` 또는 `+1도` 버튼으로 관절 영점을 맞춘다.
5. `현재 자세를 0도로 저장` 버튼을 누른다.
6. 다음 관절을 선택하여 같은 작업을 반복한다.

모터를 활성화하면 현재 엔코더 위치를 기준으로 `저장 예정 offset`을
계산한다. `-1도` 또는 `+1도`를 누를 때 모터 목표와 저장 예정 offset이 함께
1도씩 변경된다. 저장할 때는 화면의 목표값만 사용하지 않고 마지막 실제
엔코더 피드백으로 offset을 다시 계산한다.

관절 선택 버튼에는 다음과 같이 관절 이름과 번호가 함께 표시된다.

```text
front_left_hip_joint (0번)
front_left_upper_leg_joint (1번)
front_left_lower_leg_joint (2번)
...
back_right_lower_leg_joint (11번)
```

관절 번호는 CAN ID와 같다. 다른 관절을 선택하면 현재 활성화된 모터를 먼저
비활성화한다. 저장, 창 닫기 또는 오류 발생 시에도 모터를 비활성화한다.

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
