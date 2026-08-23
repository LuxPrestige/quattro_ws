# Quattro 개발 환경

## 1. 기본 원칙

ROS 2 개발, 빌드, 테스트는 기본적으로 Docker 컨테이너 내부 `/quattro_ws`에서 수행한다.

```bash
cd ~/quattro_ws
docker compose up -d dev
docker compose exec dev bash
```

컨테이너 내부:

```bash
source /opt/ros/jazzy/setup.bash
cd /quattro_ws
```

빌드 후:

```bash
source /quattro_ws/install/setup.bash
```

## 2. 기본 빌드

전체 빌드:

```bash
cd /quattro_ws
colcon build \
  --symlink-install \
  --event-handlers console_direct+
```

특정 패키지:

```bash
colcon build \
  --symlink-install \
  --packages-select <package_name> \
  --event-handlers console_direct+
```

## 3. Raspberry Pi 하드웨어 장치 전달

BNO085 I2C와 joystick을 사용할 때 `compose.hardware.yaml`을 함께 사용한다.

```bash
cd ~/quattro_ws
INPUT_GID=$(getent group input | cut -d: -f3) \
docker compose \
  -f compose.yaml \
  -f compose.hardware.yaml \
  up -d --force-recreate dev
```

진입:

```bash
docker compose \
  -f compose.yaml \
  -f compose.hardware.yaml \
  exec dev bash
```

장치 확인:

```bash
ls -l /dev/i2c-1
ls -l /dev/input
```

SocketCAN의 `can0`, `can1`은 Linux network interface이므로 host networking을 사용한다.

## 4. X11 GUI

Raspberry Pi에 원격 접속하여 calibration GUI 등을 띄울 때는 X11 forwarding을 사용한다.

로컬 PC에서:

```bash
ssh -Y <user>@<raspberry-pi-ip>
echo "$DISPLAY"
```

`DISPLAY`가 설정된 터미널에서 X11 override를 적용하여 컨테이너를 재생성한다.

```bash
cd ~/quattro_ws
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  up -d --force-recreate dev
```

GUI 오류 확인:

```bash
echo "$DISPLAY"
echo "$XAUTHORITY"
ls -l "$XAUTHORITY"
```

VS Code Remote SSH 터미널은 SSH X11 forwarding 환경과 다를 수 있으므로 GUI 컨테이너 재생성은 `ssh -Y` 세션에서 수행한다.

## 5. NVIDIA 데스크톱

NVIDIA GPU가 있는 Linux 데스크톱에서 Gazebo/RViz 하드웨어 가속을 사용할 경우 `compose.nvidia.yaml`을 추가한다.

```bash
docker compose \
  -f compose.yaml \
  -f compose.x11.yaml \
  -f compose.nvidia.yaml \
  up -d --force-recreate dev
```

컨테이너에서 GPU 확인:

```bash
docker compose exec dev nvidia-smi
```

Raspberry Pi에서는 NVIDIA override를 사용하지 않는다.

## 6. URDF 검사

```bash
cd /quattro_ws
xacro src/quattro_description/urdf/quattro.urdf.xacro > /tmp/quattro.urdf
check_urdf /tmp/quattro.urdf
```

시뮬레이션용:

```bash
xacro src/quattro_description/urdf/quattro.urdf.xacro \
  simulation:=true > /tmp/quattro_sim.urdf
check_urdf /tmp/quattro_sim.urdf
```

## 7. Git 작업 방식

GitHub 저장소를 여러 개발 PC와 Raspberry Pi 사이의 기준 저장소로 사용한다.

작업 전:

```bash
cd ~/quattro_ws
git status
git pull origin main
```

작업 후:

```bash
git status
git add -A
git commit -m "<type>: <summary>"
git push
```

로컬 수정이 있는 상태에서 무조건 `git pull`하지 않는다. 먼저 commit, stash 또는 변경 내용을 확인한다.

## 8. Git에 저장하지 않는 머신별 데이터

- `.env`
- 실제 장치별 calibration offset
- 머신별 임시 CAN 설정
- 로그
- `build/`
- `install/`
- `log/`

예제 파일만 버전 관리한다.

```text
.env.example
calibration.yaml.example
```

## 9. GUI와 렌더링 문제

`qt.qpa.xcb: could not connect to display`가 나오면 URDF 문제로 간주하지 않고 먼저 X11/DISPLAY/Xauthority를 확인한다.

한글 폰트가 깨지면 Docker 이미지에 `fonts-noto-cjk`가 포함되었는지 확인하고 이미지 및 컨테이너를 다시 빌드한다.
