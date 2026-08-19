# `quattro_sensors`

## 역할

BNO085 IMU 취득 계층(`ament_python`). 센서 데이터를 표준 `sensor_msgs/Imu`로 발행하며 balance 제어는 수행하지 않는다(balance PID는 `quattro/gait_controller`가 소유).

```text
src/quattro_sensors/quattro_sensors/
├── bno085_driver.py   # ROS 비의존 하드웨어 어댑터
└── bno085_node.py      # bno085_node 실행 파일 (ROS 노드)
```

## `bno085_driver.py`

`Bno085Driver`: Adafruit `adafruit_bno08x`/`adafruit_extended_bus` CircuitPython API를 감싸는 ROS 비의존 클래스.

- I2C 주소는 `0x4A` 또는 `0x4B`만 허용(그 외는 `ValueError`).
- 생성 시 `ROTATION_VECTOR`(쿼터니언), `GYROSCOPE`(rad/s), `ACCELEROMETER`(m/s²) 세 feature를 요청 `report_rate_hz`에 맞춘 interval(최소 1000 µs)로 활성화한다.
- `read()`는 세 값이 모두 준비되지 않으면(`None`) `RuntimeError`를 던진다 — 부분 데이터로 `ImuSample`을 만들지 않는다.
- `close()`는 I2C 버스를 해제한다.

## `bno085_node.py`

`Bno085Node`: 하드웨어 연결 실패를 노드 죽음으로 만들지 않고 재시도하는 패턴.

**파라미터**: `frame_id`(기본 `imu_link`), `i2c_address`(기본 `0x4A`), `publish_rate_hz`(기본 `100.0`), `reconnect_interval_sec`(기본 `2.0`), `orientation_stddev`/`angular_velocity_stddev`/`linear_acceleration_stddev`(대각 공분산 계산용 표준편차).

**발행**

| 토픽 | 타입 | QoS |
|---|---|---|
| `/imu/data` | `sensor_msgs/Imu` | `qos_profile_sensor_data` |
| `/diagnostics` | `diagnostic_msgs/DiagnosticArray` | 기본(1 Hz) |

**동작**

- `_connect()`: 이미 연결되어 있으면 아무것도 하지 않고, 실패하면 예외를 잡아 `_last_error`에 기록만 하고 `reconnect_interval_sec`마다 재시도한다(타이머 기반, 블로킹 없음).
- `_publish_imu()`: 드라이버가 없으면 스킵. 값 중 하나라도 `NaN`/`Inf`이면 예외로 취급해 **연결을 끊고** 다음 재연결 사이클을 기다린다 — 잘못된 값을 그대로 publish하지 않는다.
- `_publish_diagnostic()`: 연결 상태를 `DiagnosticStatus.OK`/`ERROR`로, 마지막 오류 메시지와 누적 publish 샘플 수를 `KeyValue`로 보고한다.
- `destroy_node()`를 오버라이드해 종료 시 I2C를 반드시 해제한다.

## 실기 검증 상태

Raspberry Pi 5의 실제 I2C 조건(버스 번호, 전압, 배선)에서 계속 검증이 필요하다(`docs/development_status.md` 참고). `ExtendedI2C(1)`로 I2C 버스 1을 고정 사용한다.

## Launch

`launch/bno085.launch.py`가 `config/bno085.yaml` 파라미터로 `bno085_node`를 단독 실행한다. `quattro_bringup/hardware.launch.py`는 `use_imu:=true`(기본값)일 때 같은 파라미터 파일로 동일 노드를 함께 실행한다.
