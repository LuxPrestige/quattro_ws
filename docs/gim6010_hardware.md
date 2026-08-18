# GIM6010-8 / GDS68 하드웨어 구조

## 1. 대상 하드웨어

Quattro는 다음 조합을 사용한다.

- SteadyWin GIM6010-8 × 12
- GDS68 driver
- secondary encoder 옵션
- Linux SocketCAN
- CAN Simple
- MIT Control
- 기본 bitrate: `500000 bit/s`

세부 CAN 명령과 하드웨어 사양은 `docs/GIM6010-8 메뉴얼_한국어(번역)_rev2.2.pdf` 및 실제 검증된 firmware를 기준으로 한다.

## 2. 소프트웨어 계층

```text
ros2_control controller
        ↓
quattro_hardware::QuattroSystem
        ↓
joint direction / offset / limits / safety
        ↓
gim6010_driver::Gim6010Motor
        ↓
CAN Simple command routing
        ↓
MIT Control encode/decode
        ↓
SocketCAN
        ↓
can0 / can1
        ↓
GDS68
```

`quattro_hardware`는 raw CAN payload를 직접 만들지 않는다.

`gim6010_driver`는 Quattro joint 이름이나 URDF를 알지 않는다.

## 3. `gim6010_driver`

주요 구성:

```text
include/gim6010_driver/
├── can_socket.hpp
├── can_diagnostics.hpp
├── mit_protocol.hpp
├── gim6010_motor.hpp
└── motor_manager.hpp
```

책임:

- standard CAN frame 송수신
- CAN Simple arbitration ID 생성
- MIT command encode/decode
- heartbeat / encoder / bus telemetry decode
- GIM6010 motor abstraction
- 한 CAN bus의 다중 motor routing

현재 arbitration ID 형식:

```text
(node_id << 5) | command_id
```

## 4. `quattro_hardware`

`hardware_interface::SystemInterface`를 구현한다.

state interface:

- position
- velocity
- effort

command interface:

- position

현재 joint 좌표 변환:

```text
joint_position = direction * motor_position - offset
motor_position = direction * (joint_command + offset)
```

offset 단위는 rad이다.

## 5. CAN ID와 bus 매핑

| Joint | CAN ID | Bus |
|---|---:|---|
| `front_left_hip_joint` | 0 | `can0` |
| `front_left_upper_leg_joint` | 1 | `can0` |
| `front_left_lower_leg_joint` | 2 | `can0` |
| `front_right_hip_joint` | 3 | `can0` |
| `front_right_upper_leg_joint` | 4 | `can0` |
| `front_right_lower_leg_joint` | 5 | `can0` |
| `back_left_hip_joint` | 6 | `can1` |
| `back_left_upper_leg_joint` | 7 | `can1` |
| `back_left_lower_leg_joint` | 8 | `can1` |
| `back_right_hip_joint` | 9 | `can1` |
| `back_right_upper_leg_joint` | 10 | `can1` |
| `back_right_lower_leg_joint` | 11 | `can1` |

기본 direction:

```text
[-1, -1, -1, -1, 1, 1, 1, -1, -1, 1, 1, 1]
```

실제 offset은 머신별 `src/quattro_bringup/config/calibration.yaml`에서 관리한다.

## 6. 설정 파일

### 실제 머신

```text
src/quattro_bringup/config/calibration.yaml
```

Git에서 제외한다.

### 버전 관리 템플릿

```text
src/quattro_bringup/config/calibration.yaml.example
```

### description 기본값

Xacro/URDF 검사에 사용하는 기본 calibration과 실제 머신별 calibration을 혼동하지 않는다.

## 7. Safe Start

실제 하드웨어 활성화 시 기본 원칙:

1. CAN socket과 motor mapping 생성
2. encoder feedback 확인
3. 현재 motor position 획득
4. 현재 위치를 목표로 준비
5. closed-loop enable
6. 현재 위치 hold 확인
7. gain을 안전하게 적용
8. 상위 controller command를 받기 시작

부팅 직후 0 rad 같은 임의 목표를 먼저 보내지 않는다.

## 8. 안전 정책

하드웨어 제어에서 최소한 다음을 검출한다.

- invalid node ID
- invalid command range
- command timeout
- stale motor feedback
- stale heartbeat
- axis/controller/system fault
- CAN socket 오류

오류 시 안전 상태로 전환하고 원인을 로그/diagnostics에 남긴다.

일시적 CAN frame 누락과 실제 GDS68 fault는 가능한 한 구분하여 진단한다.

한 모터의 통신 문제 때문에 전체를 정지시키는 정책은 로봇 안전상 필요할 수 있지만, 원인 분석을 위해 어떤 조건이 전체 safe stop을 발생시켰는지 반드시 기록한다.

## 9. CAN 진단

호스트에서 CAN 상태:

```bash
ip -details -statistics link show can0
ip -details -statistics link show can1
```

raw frame:

```bash
candump -e -tz can0
candump -e -tz can1
```

확인 대상:

- `ERROR-ACTIVE`
- `ERROR-WARNING`
- `ERROR-PASSIVE`
- `BUS-OFF`
- RX/TX error count
- dropped frame

`CanSocket`에서 CAN error frame을 무시한다면 상위 계층에서는 단순 stale feedback으로만 보일 수 있으므로 CAN physical-layer 문제를 조사할 때는 host 상태도 함께 확인한다.

## 10. 현재 gain / current 설정 주의

Kp/Kd와 current limit은 로봇의 하중, firmware, 전압 버전에 따라 실제 시험으로 결정한다.

높은 Kp 상태에서 joint offset 또는 direction이 잘못되면 position error가 즉시 큰 torque/current command로 변환될 수 있다.

따라서 최초 시험은:

- 로봇 지지
- 단일 모터
- 낮은 gain
- 낮은 current limit
- 작은 각도 이동

순서로 진행한다.

하드웨어 사양을 확인하지 않고 12축 전체에 큰 current limit을 일괄 적용하지 않는다.

## 11. Secondary encoder

secondary encoder가 있더라도 로봇 기구학상의 joint zero와 절대 encoder zero는 별개의 개념이다.

조립 후 반드시 joint offset calibration을 수행한다.

상위 ROS 좌표계에서는 calibration 적용 후의 joint angle을 사용한다.

## 12. 캘리브레이션

상세 절차는 `docs/calibration.md`를 따른다.

offset 계산식:

```text
offset = direction * current_motor_position
```

캘리브레이션 중에는 `controller_manager`와 다른 CAN sender를 동시에 실행하지 않는다.

## 13. 실물 시험 확대 순서

1. CAN interface 상태 확인
2. feedback-only 단일 모터 시험
3. enable/disable 단일 모터 시험
4. 현재 위치 hold
5. 작은 각도 이동
6. timeout / safe stop 검증
7. 한 다리 3축
8. 두 CAN bus 부하 확인
9. 12축 hold
10. 초기 자세 transition
11. 지지대 위 gait
12. 실제 지면 보행
