# Battery Package 가이드

<br>

## 1) Battery 패키지가 하는 일

`battery` 패키지는 INA219 센서(I2C)로 전압/전류를 읽어 배터리 잔량(%)을 계산하고 ROS 토픽으로 publish 합니다.

핵심 포인트:
- 입력: INA219 센서 값 (`bus_voltage`, `shunt_voltage`, `current`)
- 계산: 전압 기반 배터리 퍼센트(0~100%) 
- 출력: `battery_msgs/msg/Battery` 메시지 (`battery_status` 필드)

<br>

## 2) 동작 방식

`battery_node`는 주기적으로 타이머 콜백을 실행합니다.

1. INA219에서 전압/전류 읽기
2. `use_load_voltage` 설정에 따라 계산 전압 선택
- `True`: `load_voltage = bus_voltage + shunt_voltage`
- `False`: `load_voltage = bus_voltage`
3. `min_voltage ~ max_voltage` 범위를 0~100%로 정규화
4. `battery_status`로 publish

퍼센트 변환식:
```text
((voltage - min_voltage) / (max_voltage - min_voltage)) * 100
```
결과는 0~100 범위로 clamp 됩니다.
- 현재 지급된 배터리(18650) 배터리 기반으로 계산되었으니 퍼센트 변환은 별도로 수정하지 않으셔도 됩니다.

<br>

## 3) battery_node 구동 방법
### 3-1. 빌드

```bash
cd /home/topst/D-Racer
colcon build --packages-select battery
source install/setup.bash
```
### 3-2. 기본 실행

```bash
ros2 run battery battery_node
```
### 3-3. 파라미터 오버라이드 실행 예시

```bash
ros2 run battery battery_node --ros-args \
  -p publish_topic:=battery_status \
  -p publish_hz:=10.0 \
  -p min_voltage:=6.4 \
  -p max_voltage:=8.4 \
  -p debug_log:=false
```
### 3-4. 모니터 패키지 결과 확인
```bash
ros2 run monitor monitor_node
```
대쉬보드에 배터리 상태량을 출력한다. (Figure1) 
<p align="center">  
  ![Figure 1. Battery Status](topst-development url 첨부예정)
</p>

<br>

## 4) 출력 토픽/메시지

- 기본 출력 토픽: `battery_status`
- 메시지 타입: `battery_msgs/msg/Battery`
- 주요 필드:
  - `battery_status` (float): 배터리 잔량(%)

확인 명령:
```bash
ros2 topic echo /battery_status
ros2 topic hz /battery_status
```

<br>

## 5) ROS 파라미터 설명

| 파라미터 | 타입 | 기본값 | 설명 |
|---|---|---|---|
| `publish_topic` | string | `battery_status` | 배터리 퍼센트 publish 토픽 |
| `publish_hz` | float | `10.0` | publish 주기(Hz), 0보다 커야 함 |
| `i2c_bus` | int | `I2C_BUS` | INA219가 연결된 I2C 버스 번호 |
| `ina_addr` | int | `INA_ADDR` | INA219 I2C 주소 |
| `r_shunt_ohm` | float | `0.1` | 션트 저항값(ohm) |
| `max_current_a` | float | `2.0` | INA219 보정 시 최대 전류(A) |
| `min_voltage` | float | `6.4` | 0% 기준 전압 |
| `max_voltage` | float | `8.4` | 100% 기준 전압 (`min_voltage`보다 커야 함) |
| `use_load_voltage` | bool | `True` | `bus+shunt` 전압 사용 여부 |
| `debug_log` | bool | `True` | 센서/계산 로그 출력 여부 |

참고:
- `i2c_bus`, `ina_addr` 기본값은 `topst_utils.ina219` 상수(`I2C_BUS`, `INA_ADDR`)를 따릅니다.

<br>

## 6) 트러블슈팅
### 6-1. 토픽이 안 나올 때

1. 노드가 실행 중인지 확인
```bash
ros2 node list | grep battery_node
```
2. 토픽 존재 확인
```bash
ros2 topic list | grep battery
```
3. 퍼블리시 주기 확인
```bash
ros2 topic hz /battery_status
```
### 6-2. I2C 관련 에러가 날 때

- 보드와 D-Racer Interface Board 배선(GND/SCL/SDA/VCC) 확인
- `i2c_bus`, `ina_addr` 파라미터 확인
- 보드에서 I2C 디바이스 인식 여부 확인
```bash
sudo i2cdetect -y 3
```
### 6-3. 배터리 퍼센트가 비정상일 때

- `min_voltage`, `max_voltage` 값을 실제 배터리 스펙에 맞게 조정
- `use_load_voltage`를 `True/False`로 바꿔 비교

<br>

## 7) Monitor 패키지 연동

`monitor` 패키지는 기본적으로 `battery_status` 토픽을 subscribe해 대시보드 배터리 상태 화면을 갱신합니다.

따라서 Battery를 기본값으로 쓰면 별도 설정 없이 바로 연동됩니다.
